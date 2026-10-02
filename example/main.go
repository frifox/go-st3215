// Command example is a demo for the st3215 package: a WebSocket server that
// streams servo telemetry and accepts control commands, plus a web page
// (web/index.html, embedded) to monitor and drive the servos.
//
//	go run . -port /dev/ttyACM0          # real hardware (Linux)
//	go run . -port /dev/cu.usbmodem1101  # real hardware (macOS)
//	go run . -sim 1,2                    # simulated servos, no hardware
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
	"slices"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	st3215 "github.com/frifox/go-st3215"
	"github.com/gorilla/websocket"
)

//go:embed web/index.html
var indexHTML []byte

func main() {
	port := flag.String("port", os.Getenv("ST3215_PORT"), "serial device of the Bus Servo Adapter")
	baud := flag.Int("baud", st3215.DefaultBaudRate, "bus baud rate")
	addr := flag.String("addr", "localhost:8080", "HTTP listen address")
	sim := flag.String("sim", "", "simulate servos with these IDs instead of using hardware, e.g. 1,2")
	poll := flag.Duration("poll", 50*time.Millisecond, "telemetry polling interval")
	noSync := flag.Bool("nosync", false, "poll servos one by one instead of SYNC READ")
	flag.Parse()

	var bus *st3215.Bus
	var err error
	switch {
	case *sim != "":
		ids, perr := parseIDs(*sim)
		if perr != nil {
			log.Fatal(perr)
		}
		bus, err = st3215.NewBus(newSimPort(ids...))
		log.Printf("simulating servos %v", ids)
	case *port != "":
		bus, err = st3215.Open(*port, *baud)
		log.Printf("opened %s at %d baud", *port, *baud)
	default:
		ports, _ := st3215.ListPorts()
		log.Fatalf("pass -port <device> (available: %v) or -sim 1,2", ports)
	}
	if err != nil {
		log.Fatal(err)
	}
	defer bus.Close()

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	srv := &server{bus: bus, clients: map[*client]struct{}{}, poll: *poll, noSync: *noSync}
	srv.scan(ctx)
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
// Server state and broadcasting

type server struct {
	bus      *st3215.Bus
	poll     time.Duration
	noSync   bool
	scanning atomic.Bool

	mu      sync.Mutex
	ids     []uint8
	clients map[*client]struct{}
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

func (s *server) servoIDs() []uint8 {
	s.mu.Lock()
	defer s.mu.Unlock()
	return slices.Clone(s.ids)
}

func (s *server) setIDs(ids []uint8) {
	s.mu.Lock()
	s.ids = ids
	s.mu.Unlock()
	s.broadcast(newServosMsg(ids))
}

func (s *server) broadcast(msg any) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for c := range s.clients {
		c.push(msg)
	}
}

func (s *server) scan(ctx context.Context) {
	s.scanning.Store(true)
	defer s.scanning.Store(false)
	s.broadcast(logMsg{Type: "log", Level: "info", Message: "scanning bus..."})
	found, err := s.bus.Scan(ctx, 15*time.Millisecond)
	ids := []uint8{}
	for _, f := range found {
		ids = append(ids, f.ID)
	}
	if err != nil {
		log.Printf("scan: %v", err)
	}
	log.Printf("servos found: %v", ids)
	s.setIDs(ids)
	s.broadcast(logMsg{Type: "log", Level: "info", Message: fmt.Sprintf("found %d servo(s): %v", len(ids), ids)})
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
		ids := s.servoIDs()
		if len(ids) == 0 || s.scanning.Load() {
			continue
		}
		states := map[string]servoState{}
		if s.noSync {
			for _, id := range ids {
				f, err := s.bus.Servo(id).Feedback()
				states[strconv.Itoa(int(id))] = toState(f, err)
			}
		} else {
			res, err := s.bus.SyncFeedback(ids...)
			if err != nil {
				log.Printf("poll: %v", err)
				continue
			}
			for id, r := range res {
				states[strconv.Itoa(int(id))] = toState(r.Feedback, r.Err)
			}
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

type servosMsg struct {
	Type string `json:"type"`
	IDs  []int  `json:"ids"` // not []uint8: encoding/json would emit base64
}

func newServosMsg(ids []uint8) servosMsg {
	out := make([]int, len(ids))
	for i, id := range ids {
		out[i] = int(id)
	}
	return servosMsg{Type: "servos", IDs: out}
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
	c := &client{conn: conn, send: make(chan any, 64)}
	s.mu.Lock()
	s.clients[c] = struct{}{}
	ids := slices.Clone(s.ids)
	s.mu.Unlock()
	c.send <- newServosMsg(ids)

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
		// Commands of one client run in order, so rapid moves can't overtake each other.
		s.handle(r.Context(), c, req)
	}
}

func (s *server) handle(ctx context.Context, c *client, req request) {
	err := s.exec(ctx, c, req)
	res := resultMsg{Type: "result", Seq: req.Seq, OK: err == nil}
	if err != nil {
		res.Error = err.Error()
		log.Printf("%s servo %d: %v", req.Type, req.ID, err)
	}
	c.push(res)
}

func (s *server) exec(ctx context.Context, c *client, req request) error {
	sv := s.bus.Servo(req.ID)
	switch req.Type {
	case "scan":
		s.scan(ctx)
		return nil
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
		ids := slices.DeleteFunc(s.servoIDs(), func(id uint8) bool { return id == req.ID })
		ids = append(ids, req.NewID)
		slices.Sort(ids)
		s.setIDs(ids)
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
