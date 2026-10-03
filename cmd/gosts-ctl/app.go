package main

import (
	"context"
	"fmt"
	"log"
	"slices"
	"strconv"
	"sync"
	"time"

	"github.com/frifox/gosts"
	"github.com/frifox/gosts/cmd/gosts-ctl/board"
	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
	"github.com/frifox/gosts/cmd/gosts-ctl/servo"
	"github.com/frifox/gosts/cmd/gosts-ctl/web"
)

// app ties gosts-ctl together: it builds the web server, the driver board and
// the servo controller, runs the browser's requests on them (as the web
// package's Handler), polls telemetry, keeps every window in sync, and is the
// Notifier the other packages report through.
type app struct {
	cfg    *internal.Config
	srv    *web.Server
	board  *board.Board
	ctl    *servo.Controller
	poll   time.Duration
	noSync bool

	mu      sync.Mutex
	refresh map[uint8]*pendingRefresh
	at      *autotuneRun // current or last auto-tune run
}

// newApp builds the app. simIDs are the servos of the simulated board; poll is
// the telemetry interval; noSync polls servos one by one instead of SYNC READ.
func newApp(cfg *internal.Config, simIDs []uint8, poll time.Duration, noSync bool) *app {
	a := &app{cfg: cfg, poll: poll, noSync: noSync}
	a.srv = web.New(a)
	a.board = board.New(cfg, a, simIDs)
	a.ctl = servo.New(cfg, a)
	return a
}

// Logf logs a message and shows it in every window's log.
func (a *app) Logf(level, format string, args ...any) {
	msg := fmt.Sprintf(format, args...)
	log.Print(msg)
	a.srv.Broadcast(internal.LogMsg{Type: "log", Level: level, Message: msg})
}

// Broadcast sends a message to every window.
func (a *app) Broadcast(msg any) { a.srv.Broadcast(msg) }

// BroadcastState sends the current state to every window.
func (a *app) BroadcastState() { a.srv.Broadcast(a.stateMsg()) }

// Refresh re-reads servo id's settings soon and sends them to every window.
func (a *app) Refresh(id uint8) { a.scheduleRefresh(id, -1) }

func (a *app) stateMsg() internal.StateMsg {
	st := a.board.Status()
	mirrored, signed, names, colors, zeros, dialUps := []int{}, []int{}, map[string]string{}, map[string]string{}, map[string]float64{}, map[string]float64{}
	ranges := map[string][]int{}
	for id, sc := range a.cfg.All() {
		key := strconv.Itoa(int(id))
		if sc.Mirrored {
			mirrored = append(mirrored, int(id))
		}
		if sc.Signed {
			signed = append(signed, int(id))
		}
		if len(sc.Range) == 2 {
			ranges[key] = sc.Range
		}
		if sc.Name != "" {
			names[key] = sc.Name
		}
		if sc.Color != "" {
			colors[key] = sc.Color
		}
		if sc.Zero != 0 {
			zeros[key] = sc.Zero
		}
		if sc.DialUp != 0 {
			dialUps[key] = sc.DialUp
		}
	}
	slices.Sort(mirrored)
	slices.Sort(signed)
	return internal.StateMsg{Type: "state", Connected: st.Port != "", Port: st.Port, Baud: st.Baud,
		Scanning: st.Scanning, Scanned: st.Scanned, IDs: internal.ToInts(st.IDs), Mirrored: mirrored, Signed: signed, Names: names,
		Colors: colors, Zeros: zeros, DialUps: dialUps, Ranges: ranges, Groups: groupInfos(a.cfg.AllGroups())}
}

// run serves the console on addr until ctx is done. With port set it first
// connects to that driver board (board.SimPort for the simulator) and scans
// it for servos.
func (a *app) run(ctx context.Context, addr, port string, baud int) error {
	if port != "" {
		if err := a.board.Connect(port, baud); err != nil {
			return err
		}
		go a.board.Scan(ctx, 0, gosts.MaxID)
	}
	defer func() {
		a.stopAutotune()
		a.board.Disconnect()
	}()
	go a.pollLoop(ctx)
	return a.srv.ListenAndServe(ctx, addr)
}
