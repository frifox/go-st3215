// Command example is a demo for the st3215 package: a WebSocket server that
// streams servo telemetry and accepts control commands, plus a web page
// (web/index.html, embedded) to monitor and drive the servos.
//
// The page walks through three steps: pick the driver board (serial port),
// scan it for servos, then monitor/control the servos that were found.
//
//	go run .                               # choose the port in the browser
//	go run . -port /dev/ttyACM0            # connect on startup (Linux)
//	go run . -port /dev/cu.usbmodem1101    # connect on startup (macOS)
//	go run . -sim 1,2,3                    # connect to simulated servos on startup
//
// Then open http://localhost:8080.
package main

import (
	"context"
	_ "embed"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"runtime"
	"slices"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	st3215 "github.com/frifox/go-st3215"
	"github.com/gorilla/websocket"
	"go.bug.st/serial/enumerator"
)

//go:embed web/index.html
var indexHTML []byte

// simPortName is the pseudo port that selects the simulated driver board.
const simPortName = "sim"

func main() {
	port := flag.String("port", os.Getenv("ST3215_PORT"), "serial device to connect to on startup (optional)")
	baud := flag.Int("baud", st3215.DefaultBaudRate, "bus baud rate used with -port")
	addr := flag.String("addr", "localhost:8080", "HTTP listen address")
	sim := flag.String("sim", "", "connect to simulated servos with these IDs on startup, e.g. 1,2,3")
	poll := flag.Duration("poll", 50*time.Millisecond, "telemetry polling interval")
	noSync := flag.Bool("nosync", false, "poll servos one by one instead of SYNC READ")
	cfgPath := flag.String("config", "config.toml", "per-servo settings file (names, mirrored), created on first change")
	flag.Parse()

	cfg, warnings, err := loadConfig(*cfgPath)
	if err != nil {
		log.Fatal(err)
	}
	for _, w := range warnings {
		log.Print(w)
	}
	log.Printf("config: %s (%d servo entries)", *cfgPath, len(cfg.all()))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	srv := &server{clients: map[*client]struct{}{}, cfg: cfg, poll: *poll, noSync: *noSync, simIDs: []uint8{1, 2, 3}}
	if *sim != "" {
		ids, err := parseIDs(*sim)
		if err != nil {
			log.Fatal(err)
		}
		srv.simIDs = ids
		*port = simPortName
	}
	if *port != "" {
		if err := srv.connect(*port, *baud); err != nil {
			log.Fatal(err)
		}
		go srv.scan(ctx, 0, st3215.MaxID)
	}
	defer srv.disconnect()
	go srv.pollLoop(ctx)

	mux := http.NewServeMux()
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Write(indexHTML)
	})
	mux.HandleFunc("/ws", srv.handleWS)
	hs := &http.Server{Addr: *addr, Handler: mux}
	go func() {
		<-ctx.Done()
		hs.Close()
	}()
	log.Printf("open http://%s", *addr)
	if err := hs.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
		log.Fatal(err)
	}
}

func parseIDs(s string) ([]uint8, error) {
	var ids []uint8
	for _, f := range strings.Split(s, ",") {
		v, err := strconv.Atoi(strings.TrimSpace(f))
		if err != nil || v < 0 || v > int(st3215.MaxID) {
			return nil, fmt.Errorf("invalid servo ID %q", f)
		}
		ids = append(ids, uint8(v))
	}
	return ids, nil
}

// ---------------------------------------------------------------------------
// Server state

type server struct {
	poll   time.Duration
	noSync bool
	simIDs []uint8

	// busMu guards bus: users hold it for reading, connect/disconnect swap it.
	busMu sync.RWMutex
	bus   *st3215.Bus

	mu         sync.Mutex
	port       string
	baud       int
	ids        []uint8 // servos found by the last scan
	scanned    bool    // a scan has completed on this connection
	scanning   bool
	scanCancel context.CancelCauseFunc
	cfg        *config // config.toml: names and mirrored state per ID
	refresh    map[uint8]*pendingRefresh
	tried      map[uint8]map[string]int // values applied with "Try" but not saved, per servo
	fights     map[string]*fightState   // fight protection per group key
	at         *autotuneRun             // current or last auto-tune run
	clients    map[*client]struct{}
}

type client struct {
	id   int // sent to the browser so it can tell its own changes from others'
	conn *websocket.Conn
	send chan any
}

var lastClientID atomic.Int64

// push queues a message without blocking; it is dropped if the client is too slow.
func (c *client) push(msg any) {
	select {
	case c.send <- msg:
	default:
	}
}

func (s *server) broadcast(msg any) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for c := range s.clients {
		c.push(msg)
	}
}

func (s *server) stateMsg() stateMsg {
	s.mu.Lock()
	defer s.mu.Unlock()
	mirrored, names, colors, zeros := []int{}, map[string]string{}, map[string]string{}, map[string]float64{}
	for id, sc := range s.cfg.all() {
		if sc.Mirrored {
			mirrored = append(mirrored, int(id))
		}
		if sc.Name != "" {
			names[strconv.Itoa(int(id))] = sc.Name
		}
		if sc.Color != "" {
			colors[strconv.Itoa(int(id))] = sc.Color
		}
		if sc.Zero != 0 {
			zeros[strconv.Itoa(int(id))] = sc.Zero
		}
	}
	slices.Sort(mirrored)
	return stateMsg{Type: "state", Connected: s.port != "", Port: s.port, Baud: s.baud,
		Scanning: s.scanning, Scanned: s.scanned, IDs: toInts(s.ids), Mirrored: mirrored, Names: names,
		Colors: colors, Zeros: zeros, Groups: groupInfos(s.cfg.allGroups())}
}

func (s *server) broadcastState() { s.broadcast(s.stateMsg()) }

func (s *server) servoIDs() []uint8 {
	s.mu.Lock()
	defer s.mu.Unlock()
	return slices.Clone(s.ids)
}

func (s *server) logf(level, format string, args ...any) {
	msg := fmt.Sprintf(format, args...)
	log.Print(msg)
	s.broadcast(logMsg{Type: "log", Level: level, Message: msg})
}

// withBus runs f with the connected bus, or fails if there is none.
func (s *server) withBus(f func(*st3215.Bus) error) error {
	s.busMu.RLock()
	defer s.busMu.RUnlock()
	if s.bus == nil {
		return errors.New("no driver board connected")
	}
	return f(s.bus)
}

func toInts(ids []uint8) []int {
	out := make([]int, len(ids))
	for i, id := range ids {
		out[i] = int(id)
	}
	return out
}

// ---------------------------------------------------------------------------
// Ports, connection and scanning

type portInfo struct {
	Name        string `json:"name"`
	Description string `json:"description"`
	USB         bool   `json:"usb"`
	VID         string `json:"vid,omitempty"`
	PID         string `json:"pid,omitempty"`
	Serial      string `json:"serial,omitempty"`
	Likely      bool   `json:"likely"` // looks like a Bus Servo Adapter (USB-UART bridge)
}

// usbBridges maps USB vendor IDs of common USB-UART bridges to a name.
// The Bus Servo Adapter (A) uses a WCH CH34x chip.
var usbBridges = map[string]string{
	"1A86": "WCH CH34x",
	"0403": "FTDI",
	"10C4": "Silicon Labs CP210x",
	"067B": "Prolific PL2303",
}

func listPorts() ([]portInfo, error) {
	details, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return nil, err
	}
	out := []portInfo{}
	for _, d := range details {
		// On macOS every device appears as /dev/tty.* and /dev/cu.*;
		// the cu device is the one to use for outgoing connections.
		if runtime.GOOS == "darwin" && strings.HasPrefix(d.Name, "/dev/tty.") {
			continue
		}
		p := portInfo{Name: d.Name, USB: d.IsUSB, VID: strings.ToUpper(d.VID), PID: strings.ToUpper(d.PID), Serial: d.SerialNumber}
		bridge, known := usbBridges[p.VID]
		p.Likely = d.IsUSB && known
		var desc []string
		if d.Product != "" {
			desc = append(desc, d.Product)
		}
		if known {
			desc = append(desc, bridge)
		}
		if d.IsUSB {
			desc = append(desc, fmt.Sprintf("USB %s:%s", p.VID, p.PID))
		}
		p.Description = strings.Join(desc, " · ")
		out = append(out, p)
	}
	rank := func(p portInfo) int {
		switch {
		case p.Likely:
			return 0
		case p.USB:
			return 1
		}
		return 2
	}
	slices.SortStableFunc(out, func(a, b portInfo) int { return rank(a) - rank(b) })
	return out, nil
}

func (s *server) connect(port string, baud int) error {
	if port == "" {
		return errors.New("no port selected")
	}
	s.disconnect()
	if baud <= 0 {
		baud = st3215.DefaultBaudRate
	}
	var bus *st3215.Bus
	var err error
	if port == simPortName {
		bus, err = st3215.NewBus(newSimPort(s.simIDs...))
	} else {
		bus, err = st3215.Open(port, baud)
	}
	if err != nil {
		return err
	}
	for id, sc := range s.cfg.all() {
		bus.SetMirrored(id, sc.Mirrored)
	}
	s.busMu.Lock()
	s.bus = bus
	s.busMu.Unlock()
	s.mu.Lock()
	s.port, s.baud, s.ids, s.scanned = port, baud, nil, false
	s.mu.Unlock()
	s.logf("info", "connected to %s at %d baud", port, baud)
	s.broadcastState()
	return nil
}

func (s *server) disconnect() {
	s.stopAutotune()
	s.mu.Lock()
	if s.scanCancel != nil {
		s.scanCancel(nil)
	}
	wasConnected := s.port != ""
	s.mu.Unlock()

	s.busMu.Lock() // waits for an in-flight scan or command to finish
	if s.bus != nil {
		s.bus.Close()
		s.bus = nil
	}
	s.busMu.Unlock()

	s.mu.Lock()
	s.port, s.baud, s.ids, s.scanned = "", 0, nil, false
	s.mu.Unlock()
	if wasConnected {
		s.logf("info", "disconnected")
		s.broadcastState()
	}
}

// errScanFinished is the cancel cause used by "Done": stop scanning but keep
// the servos found so far.
var errScanFinished = errors.New("scan finished early")

func (s *server) scan(ctx context.Context, first, last uint8) error {
	s.mu.Lock()
	if s.scanning {
		s.mu.Unlock()
		return errors.New("a scan is already running")
	}
	ctx, cancel := context.WithCancelCause(ctx)
	s.scanning, s.scanCancel = true, cancel
	s.mu.Unlock()
	defer cancel(nil)
	s.broadcastState()

	total := int(last) - int(first) + 1
	var found []uint8
	err := s.withBus(func(bus *st3215.Bus) error {
		_, err := bus.ScanRange(ctx, first, last, 15*time.Millisecond, func(id uint8, ok bool) {
			if ok {
				found = append(found, id)
			}
			s.broadcast(scanProgressMsg{Type: "scanProgress", Done: int(id) - int(first) + 1,
				Total: total, Current: int(id), Found: toInts(found)})
		})
		return err
	})
	early := err != nil && errors.Is(context.Cause(ctx), errScanFinished)
	if early {
		err = nil // "Done": keep the servos found so far
	}

	s.mu.Lock()
	s.scanning, s.scanCancel = false, nil
	if err == nil {
		s.ids, s.scanned = found, true
	}
	s.mu.Unlock()
	switch {
	case early:
		s.logf("info", "scan stopped early, using %d servo(s) found so far: %v", len(found), found)
	case errors.Is(err, context.Canceled):
		s.logf("info", "scan cancelled")
	case err != nil:
		s.logf("error", "scan stopped: %v", err)
	default:
		s.logf("info", "scan found %d servo(s): %v", len(found), found)
	}
	s.broadcastState()
	return err
}

// ---------------------------------------------------------------------------
// Telemetry

type servoState struct {
	st3215.Feedback
	StatusText string `json:"statusText"`
	Error      string `json:"error,omitempty"`
}

func (s *server) pollLoop(ctx context.Context) {
	t := time.NewTicker(s.poll)
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
		}
		s.mu.Lock()
		ids, scanning := slices.Clone(s.ids), s.scanning
		s.mu.Unlock()
		if len(ids) == 0 || scanning {
			continue
		}
		states := map[string]servoState{}
		var health map[string]groupHealth
		err := s.withBus(func(bus *st3215.Bus) error {
			defer func() { health = s.checkGroups(bus, states) }()
			if s.noSync {
				for _, id := range ids {
					f, err := bus.Servo(id).Feedback()
					states[strconv.Itoa(int(id))] = toState(f, err)
				}
				return nil
			}
			res, err := bus.SyncFeedback(ids...)
			for id, r := range res {
				states[strconv.Itoa(int(id))] = toState(r.Feedback, r.Err)
			}
			return err
		})
		if err != nil {
			continue
		}
		s.broadcast(feedbackMsg{Type: "feedback", Time: time.Now().UnixMilli(), Servos: states, Groups: health})
	}
}

func toState(f st3215.Feedback, err error) servoState {
	st := servoState{Feedback: f, StatusText: f.Status.String()}
	if err != nil {
		st.Error = err.Error()
	}
	return st
}

// ---------------------------------------------------------------------------
// Messages

type stateMsg struct {
	Type      string             `json:"type"`
	Connected bool               `json:"connected"`
	Port      string             `json:"port"`
	Baud      int                `json:"baud"`
	Scanning  bool               `json:"scanning"`
	Scanned   bool               `json:"scanned"`
	IDs       []int              `json:"ids"` // not []uint8: encoding/json would emit base64
	Mirrored  []int              `json:"mirrored"`
	Names     map[string]string  `json:"names"`  // servo ID -> name from config.toml
	Colors    map[string]string  `json:"colors"` // servo ID -> color override from config.toml
	Zeros     map[string]float64 `json:"zeros"`  // servo ID -> virtual 0° in degrees
	Groups    []groupInfo        `json:"groups"`
}

type portsMsg struct {
	Type  string     `json:"type"`
	Ports []portInfo `json:"ports"`
	Sim   string     `json:"sim"` // description of the simulated board
}

type scanProgressMsg struct {
	Type    string `json:"type"`
	Done    int    `json:"done"`
	Total   int    `json:"total"`
	Current int    `json:"current"`
	Found   []int  `json:"found"`
}

type feedbackMsg struct {
	Type   string                 `json:"type"`
	Time   int64                  `json:"time"`
	Servos map[string]servoState  `json:"servos"`
	Groups map[string]groupHealth `json:"groups"`
}

type logMsg struct {
	Type    string `json:"type"`
	Level   string `json:"level"`
	Message string `json:"message"`
}

type resultMsg struct {
	Type  string `json:"type"`
	Seq   int    `json:"seq"`
	OK    bool   `json:"ok"`
	Error string `json:"error,omitempty"`
	Goal  *int   `json:"goal,omitempty"` // for "angle" and "jog": the goal the server chose
}

type registerInfo struct {
	Name     string `json:"name"`
	Addr     uint8  `json:"addr"`
	Size     uint8  `json:"size"`
	Area     string `json:"area"`
	ReadOnly bool   `json:"readOnly"`
	Min      int    `json:"min"`
	Max      int    `json:"max"`
	Unit     string `json:"unit"`
	Value    int    `json:"value"`
}

type helloMsg struct {
	Type     string `json:"type"`
	ClientID int    `json:"clientId"`
}

type configMsg struct {
	Type      string         `json:"type"`
	ID        uint8          `json:"id"`
	Origin    int            `json:"origin"` // client whose change triggered this; 0 = plain request, -1 = several
	Config    st3215.Config  `json:"config"`
	Registers []registerInfo `json:"registers"`
	Tried     map[string]int `json:"tried"` // tuning values tried but not saved
}

// request is a command from the browser. Only the fields relevant to Type
// are set.
type request struct {
	Seq      int        `json:"seq"`
	Type     string     `json:"type"`
	ID       uint8      `json:"id"`
	Port     string     `json:"port"`
	Baud     int        `json:"baud"`
	First    uint8      `json:"first"`
	Last     uint8      `json:"last"`
	Position int        `json:"position"`
	Speed    int        `json:"speed"`
	Acc      uint8      `json:"acc"`
	On       bool       `json:"on"`
	Mode     int        `json:"mode"`
	Duty     int        `json:"duty"`
	NewID    uint8      `json:"newId"`
	Register string     `json:"register"`
	Value    int        `json:"value"`
	Percent  float64    `json:"percent"`
	Name     string     `json:"name"`    // for "rename"
	Values   []regValue `json:"values"`  // for "tune"
	Save     bool       `json:"save"`    // for "tune"/"copyTuning": persist instead of until power-off
	Color    string     `json:"color"`   // for "color" and "servoEdit"
	Zero     float64    `json:"zero"`    // for "servoEdit": virtual 0° in degrees
	Degrees  float64    `json:"degrees"` // "positionAs": what the current position should read; "zeroAt": the angle (as shown) that becomes 0°
	// Groups: Group targets a command at a group; the rest is for "groupSave".
	Group        string  `json:"group"`
	Members      []int   `json:"members"`
	MaxSpread    int     `json:"maxSpread"`
	MaxFightLoad float64 `json:"maxFightLoad"`
	OnFight      string  `json:"onFight"`
	// Auto-tune
	Amplitude float64 `json:"amplitude"` // degrees either side of the start
	Tolerance int     `json:"tolerance"` // steps
}

type regValue struct {
	Register string `json:"register"`
	Value    int    `json:"value"`
}

// ---------------------------------------------------------------------------
// WebSocket

var upgrader = websocket.Upgrader{
	// The demo is meant for localhost; accept any origin.
	CheckOrigin: func(*http.Request) bool { return true },
}

func (s *server) handleWS(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		return
	}
	c := &client{id: int(lastClientID.Add(1)), conn: conn, send: make(chan any, 256)}
	s.mu.Lock()
	s.clients[c] = struct{}{}
	s.mu.Unlock()
	c.push(helloMsg{Type: "hello", ClientID: c.id})
	c.push(s.stateMsg())
	if m := s.autotuneState(); m != nil {
		c.push(*m)
	}

	done := make(chan struct{})
	go func() { // writer
		defer conn.Close()
		for {
			select {
			case msg := <-c.send:
				conn.SetWriteDeadline(time.Now().Add(5 * time.Second))
				if err := conn.WriteJSON(msg); err != nil {
					return
				}
			case <-done:
				return
			}
		}
	}()

	defer func() {
		s.mu.Lock()
		delete(s.clients, c)
		s.mu.Unlock()
		close(done)
	}()

	// Commands of one client run in order, so rapid moves can't overtake each
	// other. Scans run in the background so they can be cancelled.
	for {
		_, data, err := conn.ReadMessage()
		if err != nil {
			return
		}
		var req request
		if err := json.Unmarshal(data, &req); err != nil {
			c.push(resultMsg{Type: "result", Seq: req.Seq, Error: "bad request: " + err.Error()})
			continue
		}
		if req.Type == "scan" || req.Type == "autotune" { // long-running
			go s.handle(c, req)
			continue
		}
		s.handle(c, req)
	}
}

func (s *server) handle(c *client, req request) {
	var goal *int
	var err error
	switch req.Type {
	case "angle", "jog":
		var g int
		if g, err = s.computedMove(req); err == nil {
			goal = &g
		}
	default:
		err = s.exec(c, req)
	}
	res := resultMsg{Type: "result", Seq: req.Seq, OK: err == nil, Goal: goal}
	if err != nil {
		res.Error = err.Error()
		log.Printf("%s: %v", req.Type, err)
	}
	c.push(res)
	if err == nil {
		s.afterChange(c, req)
	}
}

// turnWindow keeps angle moves within one turn either side of the center
// (multi-turn mode), so repeated moves can't wind up a cable.
var turnWindow = [2]int{st3215.CenterPosition - st3215.StepsPerRev, st3215.CenterPosition + st3215.StepsPerRev}

// computedMove handles moves whose goal depends on where the servo is, read
// fresh here: "angle" goes to req.Position's angle the short way round
// (multi-turn mode), "jog" moves req.Position steps from the present position.
// Both use the servo's absolute position: in multi-turn mode the reported
// position wraps every turn, but goals use the servo's turn count.
func (s *server) computedMove(req request) (int, error) {
	ids := []uint8{req.ID}
	if req.Group != "" {
		g, ok := s.cfg.group(req.Group)
		if !ok {
			return 0, fmt.Errorf("unknown group %q", req.Group)
		}
		ids = g.Members
	}
	found := s.servoIDs()
	for _, id := range ids {
		if !slices.Contains(found, id) {
			return 0, fmt.Errorf("servo %d was not found by the last scan", id)
		}
		if s.tuning(id) {
			return 0, fmt.Errorf("servo %d is being auto-tuned; stop the auto-tune first", id)
		}
	}
	var goal int
	err := s.withBus(func(bus *st3215.Bus) error {
		lead := bus.Servo(ids[0]) // the group's leader decides the goal
		var err error
		if req.Type == "jog" {
			var cur int
			if cur, err = lead.AbsolutePosition(); err != nil {
				return err
			}
			goal = cur + req.Position
		} else if goal, err = lead.ShortestGoal(req.Position, turnWindow[0], turnWindow[1]); err != nil {
			return err
		}
		if req.Group != "" {
			return bus.Group(ids...).MoveTo(goal, req.Speed, req.Acc)
		}
		return lead.MoveTo(goal, req.Speed, req.Acc)
	})
	return goal, err
}

// ---------------------------------------------------------------------------
// Keeping several browser windows in sync
//
// State, telemetry, scan progress and logs are broadcast to every window.
// Servo settings are per-servo snapshots (configMsg): after any command that
// changes a servo, a fresh snapshot is broadcast to all windows, and the other
// windows get a short note in their log.

// changeNote describes a servo-changing command for the other windows' logs;
// "" means the command doesn't change servo settings.
func changeNote(req request) string {
	switch req.Type {
	case "torque":
		return fmt.Sprintf("torque %s", onOff(req.On))
	case "move":
		return "" // frequent while dragging; visible in telemetry anyway
	case "stop":
		return "stop"
	case "wheel":
		return fmt.Sprintf("wheel speed %d", req.Speed)
	case "pwm":
		return fmt.Sprintf("pwm %d", req.Duty)
	case "mode":
		return fmt.Sprintf("mode %s", st3215.Mode(req.Mode))
	case "multiturn":
		return fmt.Sprintf("multi-turn %s", onOff(req.On))
	case "torqueLimit":
		return fmt.Sprintf("torque limit %.0f%%", req.Percent)
	case "calibrate":
		return "center calibrated"
	case "write":
		return fmt.Sprintf("%s ← %d", req.Register, req.Value)
	case "tune":
		how := "until power-off"
		if req.Save {
			how = "saved"
		}
		return fmt.Sprintf("tuning changed (%d value(s), %s)", len(req.Values), how)
	case "mirror":
		return fmt.Sprintf("mirrored %s", onOff(req.On))
	case "servoEdit":
		return "settings edited"
	case "step":
		return fmt.Sprintf("step %d", req.Position)
	case "align":
		return "aligned to leader"
	case "copyTuning":
		return "tuning copied from leader"
	}
	return ""
}

func onOff(on bool) string {
	if on {
		return "on"
	}
	return "off"
}

// changesServo lists the commands after which a fresh config is broadcast.
var changesServo = map[string]bool{
	"torque": true, "move": true, "stop": true, "wheel": true, "pwm": true, "mode": true,
	"multiturn": true, "torqueLimit": true, "calibrate": true, "write": true, "tune": true,
	"mirror": true, "setid": true, "servoEdit": true, "positionAs": true, "zeroAt": true, "zeroReset": true, "angle": true, "jog": true, "step": true, "align": true, "copyTuning": true,
}

func (s *server) afterChange(c *client, req request) {
	if !changesServo[req.Type] {
		return
	}
	if req.Group != "" {
		g, _ := s.cfg.group(req.Group)
		if note := changeNote(req); note != "" {
			s.broadcastExcept(c, logMsg{Type: "log", Level: "info", Message: fmt.Sprintf("another window: group %s %s", g.Name, note)})
		}
		for _, id := range g.Members {
			s.scheduleRefresh(id, c.id)
		}
		return
	}
	id := req.ID
	if req.Type == "setid" {
		id = req.NewID
	}
	if note := changeNote(req); note != "" {
		s.broadcastExcept(c, logMsg{Type: "log", Level: "info", Message: fmt.Sprintf("another window: servo %d %s", req.ID, note)})
	}
	s.scheduleRefresh(id, c.id)
}

func (s *server) broadcastExcept(skip *client, msg any) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for c := range s.clients {
		if c != skip {
			c.push(msg)
		}
	}
}

type pendingRefresh struct {
	timer  *time.Timer
	origin int
}

// refreshDelay coalesces bursts of changes (e.g. dragging a slider) into one
// config read and broadcast per servo.
const refreshDelay = 150 * time.Millisecond

func (s *server) scheduleRefresh(id uint8, origin int) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.refresh == nil {
		s.refresh = map[uint8]*pendingRefresh{}
	}
	if p, ok := s.refresh[id]; ok {
		if p.origin != origin {
			p.origin = -1 // changes from several windows: everyone resyncs
		}
		p.timer.Reset(refreshDelay)
		return
	}
	p := &pendingRefresh{origin: origin}
	p.timer = time.AfterFunc(refreshDelay, func() {
		s.mu.Lock()
		delete(s.refresh, id)
		origin := p.origin
		s.mu.Unlock()
		var msg configMsg
		err := s.withBus(func(bus *st3215.Bus) error {
			var err error
			msg, err = s.readConfigMsg(bus, id)
			return err
		})
		if err != nil {
			log.Printf("refresh servo %d: %v", id, err)
			return
		}
		msg.Origin = origin
		s.broadcast(msg)
	})
	s.refresh[id] = p
}

func (s *server) readConfigMsg(bus *st3215.Bus, id uint8) (configMsg, error) {
	sv := bus.Servo(id)
	cfg, err := sv.ReadConfig()
	if err != nil {
		return configMsg{}, err
	}
	mem, err := sv.ReadMemory()
	if err != nil {
		return configMsg{}, err
	}
	regs := make([]registerInfo, len(st3215.Registers))
	for i, r := range st3215.Registers {
		regs[i] = registerInfo{Name: r.Name, Addr: r.Addr, Size: r.Size, Area: r.Area.String(),
			ReadOnly: r.ReadOnly, Min: r.Min, Max: r.Max, Unit: r.Unit, Value: r.Value(mem)}
	}
	return configMsg{Type: "config", ID: id, Config: cfg, Registers: regs, Tried: s.triedValues(id, regs)}, nil
}

// ---------------------------------------------------------------------------
// Tried-but-not-saved tuning values
//
// "Try" writes EEPROM settings with the lock closed, so the servo uses them
// until it is power-cycled. Reading the servo back can't tell tried values
// from saved ones, so the server remembers them; "Save" can then persist them.

func (s *server) setTried(id uint8, values []regValue, save bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.tried == nil {
		s.tried = map[uint8]map[string]int{}
	}
	t := s.tried[id]
	if t == nil {
		t = map[string]int{}
		s.tried[id] = t
	}
	for _, v := range values {
		if save {
			delete(t, v.Register)
		} else {
			t[v.Register] = v.Value
		}
	}
	if len(t) == 0 {
		delete(s.tried, id)
	}
}

// triedValues returns the tried values still active on the servo. Entries
// whose register no longer holds the tried value (the servo was power-cycled
// and reverted) are forgotten.
func (s *server) triedValues(id uint8, regs []registerInfo) map[string]int {
	s.mu.Lock()
	defer s.mu.Unlock()
	t := s.tried[id]
	out := map[string]int{}
	for _, r := range regs {
		if v, ok := t[r.Name]; ok {
			if r.Value == v {
				out[r.Name] = v
			} else {
				delete(t, r.Name)
			}
		}
	}
	if len(t) == 0 {
		delete(s.tried, id)
	}
	return out
}

func (s *server) exec(c *client, req request) error {
	switch req.Type {
	case "ports":
		ports, err := listPorts()
		if err != nil {
			return err
		}
		c.push(portsMsg{Type: "ports", Ports: ports, Sim: fmt.Sprintf("Simulated board with servos %v", s.simIDs)})
		return nil
	case "connect":
		return s.connect(req.Port, req.Baud)
	case "disconnect":
		s.disconnect()
		return nil
	case "scan":
		last := req.Last
		if last == 0 || last > st3215.MaxID {
			last = st3215.MaxID
		}
		if req.First > last {
			return fmt.Errorf("invalid ID range %d..%d", req.First, last)
		}
		return s.scan(context.Background(), req.First, last)
	case "cancelScan", "finishScan":
		cause := error(nil) // cancel: discard results
		if req.Type == "finishScan" {
			cause = errScanFinished
		}
		s.mu.Lock()
		if s.scanCancel != nil {
			s.scanCancel(cause)
		}
		s.mu.Unlock()
		return nil
	case "color":
		color, err := validColor(req.Color)
		if err != nil {
			return err
		}
		if err := s.cfg.update(req.ID, func(c *servoConfig) { c.Color = color }); err != nil {
			return err
		}
		s.broadcastState()
		return nil
	case "groupSave":
		return s.groupSave(req)
	case "autotune":
		return s.startAutotune(req)
	case "autotuneStop":
		s.stopAutotune()
		return nil
	case "autotuneSave", "autotuneRevert":
		return s.finishAutotune(req.Type == "autotuneSave")
	case "groupDelete":
		g, _ := s.cfg.group(req.Group)
		if err := s.cfg.deleteGroup(req.Group); err != nil {
			return err
		}
		s.logf("info", "group %s deleted", g.Name)
		s.broadcastState()
		return nil
	}
	// Don't let other commands move servos while auto-tune is testing them.
	if changesServo[req.Type] {
		ids := []uint8{req.ID}
		if req.Group != "" {
			g, _ := s.cfg.group(req.Group)
			ids = g.Members
		}
		for _, id := range ids {
			if s.tuning(id) {
				return fmt.Errorf("servo %d is being auto-tuned; stop the auto-tune first", id)
			}
		}
	}
	if req.Group != "" {
		return s.withBus(func(bus *st3215.Bus) error { return s.groupCommand(bus, req) })
	}
	return s.withBus(func(bus *st3215.Bus) error { return s.servoCommand(c, bus, req) })
}

func (s *server) servoCommand(c *client, bus *st3215.Bus, req request) error {
	if !slices.Contains(s.servoIDs(), req.ID) {
		return fmt.Errorf("servo %d was not found by the last scan", req.ID)
	}
	sv := bus.Servo(req.ID)
	switch req.Type {
	case "torque":
		return sv.EnableTorque(req.On)
	case "move":
		return sv.MoveTo(req.Position, req.Speed, req.Acc)
	case "step":
		return sv.StepBy(req.Position, req.Speed, req.Acc)
	case "stop":
		return sv.Stop()
	case "wheel":
		return sv.SetWheelSpeed(req.Speed, req.Acc)
	case "pwm":
		return sv.SetPWM(req.Duty)
	case "mode":
		return sv.SetMode(st3215.Mode(req.Mode))
	case "mirror":
		bus.SetMirrored(req.ID, req.On)
		err := s.cfg.update(req.ID, func(c *servoConfig) { c.Mirrored = req.On })
		s.broadcastState()
		if err != nil {
			return fmt.Errorf("mirrored is active but not saved: %w", err)
		}
		return nil
	case "servoEdit": // the Edit servo dialog: name, color, mirrored and virtual zero at once
		name, err := validName(req.Name)
		if err != nil {
			return err
		}
		color, err := validColor(req.Color)
		if err != nil {
			return err
		}
		if req.Zero < 0 || req.Zero >= 360 {
			return fmt.Errorf("virtual zero must be between 0 and 360 degrees")
		}
		bus.SetMirrored(req.ID, req.On)
		if err := s.cfg.update(req.ID, func(c *servoConfig) {
			c.Name, c.Color, c.Mirrored, c.Zero = name, color, req.On, req.Zero
		}); err != nil {
			return err
		}
		s.broadcastState()
		return nil
	case "rename":
		name, err := validName(req.Name)
		if err != nil {
			return err
		}
		if err := s.cfg.update(req.ID, func(c *servoConfig) { c.Name = name }); err != nil {
			return err
		}
		s.broadcastState()
		return nil
	case "multiturn":
		return sv.SetMultiTurn(req.On)
	case "torqueLimit":
		return sv.SetTorqueLimit(req.Percent)
	case "calibrate":
		return sv.CalibrateMiddle()
	case "positionAs":
		d := math.Mod(math.Mod(req.Degrees, 360)+360, 360)
		steps := int(math.Round(d/st3215.DegreesPerStep)) % st3215.StepsPerRev
		if err := sv.SetPositionAs(steps); err != nil {
			return err
		}
		s.logf("info", "servo %d: current position now reads %.1f° (offset saved on the servo)", req.ID, float64(steps)*st3215.DegreesPerStep)
		return nil
	case "zeroReset":
		if err := sv.ResetZero(); err != nil {
			return err
		}
		s.logf("info", "servo %d: position offset reset to 0 (factory zero)", req.ID)
		return nil
	case "zeroAt":
		// The angle is as the console shows it, i.e. after any virtual 0°;
		// the servo's own zero replaces the virtual one, so fold it in and clear it.
		vz := s.cfg.get(req.ID).Zero
		d := math.Mod(math.Mod(req.Degrees+vz, 360)+360, 360)
		steps := int(math.Round(d/st3215.DegreesPerStep)) % st3215.StepsPerRev
		if err := sv.SetZeroAt(steps); err != nil {
			return err
		}
		if vz != 0 {
			if err := s.cfg.update(req.ID, func(c *servoConfig) { c.Zero = 0 }); err != nil {
				return err
			}
			s.broadcastState()
		}
		s.logf("info", "servo %d: the %.1f° mark is now 0° (offset saved on the servo)", req.ID, req.Degrees)
		return nil
	case "setid":
		if slices.Contains(s.servoIDs(), req.NewID) {
			return fmt.Errorf("ID %d is already used on the bus", req.NewID)
		}
		if err := sv.SetID(req.NewID); err != nil {
			return err
		}
		// The bus moved the mirrored flag to the new ID; keep config.toml in step.
		if err := s.cfg.move(req.ID, req.NewID); err != nil {
			s.logf("error", "config: %v", err)
		}
		s.mu.Lock()
		if t, ok := s.tried[req.ID]; ok {
			delete(s.tried, req.ID)
			s.tried[req.NewID] = t
		}
		s.ids = slices.DeleteFunc(s.ids, func(id uint8) bool { return id == req.ID })
		s.ids = append(s.ids, req.NewID)
		slices.Sort(s.ids)
		s.mu.Unlock()
		s.broadcastState()
		return nil
	case "write":
		reg, ok := st3215.RegisterByName(req.Register)
		if !ok {
			return fmt.Errorf("unknown register %q", req.Register)
		}
		if err := sv.Write(reg, req.Value); err != nil {
			return err
		}
		s.setTried(req.ID, []regValue{{Register: reg.Name, Value: req.Value}}, true) // now saved
		return nil
	case "tune":
		for _, rv := range req.Values {
			reg, ok := st3215.RegisterByName(rv.Register)
			if !ok {
				return fmt.Errorf("unknown register %q", rv.Register)
			}
			write := sv.WriteTemporary
			if req.Save {
				write = sv.Write
			}
			if err := write(reg, rv.Value); err != nil {
				return fmt.Errorf("%s: %w", reg.Name, err)
			}
		}
		s.setTried(req.ID, req.Values, req.Save)
		how := "until power-off"
		if req.Save {
			how = "saved"
		}
		log.Printf("servo %d: tuned %d register(s), %s", req.ID, len(req.Values), how)
		return nil
	case "config":
		msg, err := s.readConfigMsg(bus, req.ID)
		if err != nil {
			return err
		}
		c.push(msg)
		return nil
	}
	return fmt.Errorf("unknown command %q", req.Type)
}
