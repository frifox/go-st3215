package main

import (
	"context"
	_ "embed"
	"errors"
	"fmt"
	"log"
	"slices"
	"strconv"
	"sync"
	"sync/atomic"
	"time"

	st3215 "github.com/frifox/go-st3215"
	"github.com/gorilla/websocket"
)

// The server: state shared by all browser windows, and broadcasting to them.

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
	tried      map[uint8]map[string]triedValue // values applied with "Try" but not saved, per servo
	fights     map[string]*fightState          // fight protection per group key
	at         *autotuneRun                    // current or last auto-tune run
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
	mirrored, signed, names, colors, zeros, dialUps := []int{}, []int{}, map[string]string{}, map[string]string{}, map[string]float64{}, map[string]float64{}
	ranges := map[string][]int{}
	for id, sc := range s.cfg.all() {
		if sc.Mirrored {
			mirrored = append(mirrored, int(id))
		}
		if sc.Signed {
			signed = append(signed, int(id))
		}
		if len(sc.Range) == 2 {
			ranges[strconv.Itoa(int(id))] = sc.Range
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
		if sc.DialUp != 0 {
			dialUps[strconv.Itoa(int(id))] = sc.DialUp
		}

	}
	slices.Sort(mirrored)
	slices.Sort(signed)
	return stateMsg{Type: "state", Connected: s.port != "", Port: s.port, Baud: s.baud,
		Scanning: s.scanning, Scanned: s.scanned, IDs: toInts(s.ids), Mirrored: mirrored, Signed: signed, Names: names,
		Colors: colors, Zeros: zeros, DialUps: dialUps, Ranges: ranges, Groups: groupInfos(s.cfg.allGroups())}
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

func (s *server) broadcastExcept(skip *client, msg any) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for c := range s.clients {
		if c != skip {
			c.push(msg)
		}
	}
}
