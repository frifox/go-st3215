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
	"net/http"
	"os"
	"os/signal"
	"runtime"
	"slices"
	"strconv"
	"strings"
	"sync"
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
	flag.Parse()

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	srv := &server{clients: map[*client]struct{}{}, poll: *poll, noSync: *noSync, simIDs: []uint8{1, 2, 3}}
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
	scanCancel context.CancelFunc
	clients    map[*client]struct{}
}

type client struct {
	conn *websocket.Conn
	send chan any
}

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
	return stateMsg{Type: "state", Connected: s.port != "", Port: s.port, Baud: s.baud,
		Scanning: s.scanning, Scanned: s.scanned, IDs: toInts(s.ids)}
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

// usbBridges maps USB vendor IDs of common USB-UART bridges to a name. The
// Bus Servo Adapter (A) uses a WCH CH34x chip.
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
		// On macOS every device appears as /dev/tty.* and /dev/cu.*; the cu
		// device is the one to use for outgoing connections.
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
	s.mu.Lock()
	if s.scanCancel != nil {
		s.scanCancel()
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

func (s *server) scan(ctx context.Context, first, last uint8) error {
	s.mu.Lock()
	if s.scanning {
		s.mu.Unlock()
		return errors.New("a scan is already running")
	}
	ctx, cancel := context.WithCancel(ctx)
	s.scanning, s.scanCancel = true, cancel
	s.mu.Unlock()
	defer cancel()
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

	s.mu.Lock()
	s.scanning, s.scanCancel = false, nil
	if err == nil {
		s.ids, s.scanned = found, true
	}
	s.mu.Unlock()
	switch {
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
		err := s.withBus(func(bus *st3215.Bus) error {
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
		s.broadcast(feedbackMsg{Type: "feedback", Time: time.Now().UnixMilli(), Servos: states})
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
	Type      string `json:"type"`
	Connected bool   `json:"connected"`
	Port      string `json:"port"`
	Baud      int    `json:"baud"`
	Scanning  bool   `json:"scanning"`
	Scanned   bool   `json:"scanned"`
	IDs       []int  `json:"ids"` // not []uint8: encoding/json would emit base64
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
	Type   string                `json:"type"`
	Time   int64                 `json:"time"`
	Servos map[string]servoState `json:"servos"`
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

type configMsg struct {
	Type      string         `json:"type"`
	ID        uint8          `json:"id"`
	Config    st3215.Config  `json:"config"`
	Registers []registerInfo `json:"registers"`
}

// request is a command from the browser. Only the fields relevant to Type
// are set.
type request struct {
	Seq      int     `json:"seq"`
	Type     string  `json:"type"`
	ID       uint8   `json:"id"`
	Port     string  `json:"port"`
	Baud     int     `json:"baud"`
	First    uint8   `json:"first"`
	Last     uint8   `json:"last"`
	Position int     `json:"position"`
	Speed    int     `json:"speed"`
	Acc      uint8   `json:"acc"`
	On       bool    `json:"on"`
	Mode     int     `json:"mode"`
	Duty     int     `json:"duty"`
	NewID    uint8   `json:"newId"`
	Register string  `json:"register"`
	Value    int     `json:"value"`
	Percent  float64 `json:"percent"`
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
	c := &client{conn: conn, send: make(chan any, 256)}
	s.mu.Lock()
	s.clients[c] = struct{}{}
	s.mu.Unlock()
	c.push(s.stateMsg())

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
		if req.Type == "scan" {
			go s.handle(c, req)
			continue
		}
		s.handle(c, req)
	}
}

func (s *server) handle(c *client, req request) {
	err := s.exec(c, req)
	res := resultMsg{Type: "result", Seq: req.Seq, OK: err == nil}
	if err != nil {
		res.Error = err.Error()
		log.Printf("%s: %v", req.Type, err)
	}
	c.push(res)
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
	case "cancelScan":
		s.mu.Lock()
		if s.scanCancel != nil {
			s.scanCancel()
		}
		s.mu.Unlock()
		return nil
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
	case "stop":
		return sv.Stop()
	case "wheel":
		return sv.SetWheelSpeed(req.Speed, req.Acc)
	case "pwm":
		return sv.SetPWM(req.Duty)
	case "mode":
		return sv.SetMode(st3215.Mode(req.Mode))
	case "multiturn":
		return sv.SetMultiTurn(req.On)
	case "torqueLimit":
		return sv.SetTorqueLimit(req.Percent)
	case "calibrate":
		return sv.CalibrateMiddle()
	case "setid":
		if slices.Contains(s.servoIDs(), req.NewID) {
			return fmt.Errorf("ID %d is already used on the bus", req.NewID)
		}
		if err := sv.SetID(req.NewID); err != nil {
			return err
		}
		s.mu.Lock()
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
		return sv.Write(reg, req.Value)
	case "config":
		cfg, err := sv.ReadConfig()
		if err != nil {
			return err
		}
		mem, err := sv.ReadMemory()
		if err != nil {
			return err
		}
		regs := make([]registerInfo, len(st3215.Registers))
		for i, r := range st3215.Registers {
			regs[i] = registerInfo{Name: r.Name, Addr: r.Addr, Size: r.Size, Area: r.Area.String(),
				ReadOnly: r.ReadOnly, Min: r.Min, Max: r.Max, Unit: r.Unit, Value: r.Value(mem)}
		}
		c.push(configMsg{Type: "config", ID: req.ID, Config: cfg, Registers: regs})
		return nil
	}
	return fmt.Errorf("unknown command %q", req.Type)
}
