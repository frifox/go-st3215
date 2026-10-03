package main

import (
	"context"
	"fmt"
	"log"
	"slices"

	st3215 "github.com/frifox/go-st3215"
	"github.com/frifox/go-st3215/cmd/servo-ctl/board"
	"github.com/frifox/go-st3215/cmd/servo-ctl/internal"
	"github.com/frifox/go-st3215/cmd/servo-ctl/server"
)

// Requests from the browser windows.

// Connected sends a new window the current state.
func (a *app) Connected(c *server.Client) {
	c.Push(a.stateMsg())
	if m := a.autotuneState(); m != nil {
		c.Push(*m)
	}
}

// Handle runs one request. Scans and auto-tune runs are long, so they run in
// the background (and can be cancelled); the window's other requests run in
// order, so rapid moves can't overtake each other.
func (a *app) Handle(c *server.Client, req internal.Request) {
	if req.Type == "scan" || req.Type == "autotune" {
		go a.handle(c, req)
		return
	}
	a.handle(c, req)
}

func (a *app) handle(c *server.Client, req internal.Request) {
	var goal *int
	var err error
	switch req.Type {
	case "angle", "jog":
		var g int
		if g, err = a.move(req); err == nil {
			goal = &g
		}
	default:
		err = a.exec(c, req)
	}
	res := internal.ResultMsg{Type: "result", Seq: req.Seq, OK: err == nil, Goal: goal}
	if err != nil {
		res.Error = err.Error()
		log.Printf("%s: %v", req.Type, err)
	}
	c.Push(res)
	if err == nil {
		a.afterChange(c, req)
	}
}

func (a *app) exec(c *server.Client, req internal.Request) error {
	switch req.Type {
	case "ports":
		ports, err := board.ListPorts()
		if err != nil {
			return err
		}
		c.Push(internal.PortsMsg{Type: "ports", Ports: ports, Sim: a.board.SimDescription()})
		return nil
	case "connect":
		a.stopAutotune()
		return a.board.Connect(req.Port, req.Baud)
	case "disconnect":
		a.stopAutotune()
		a.board.Disconnect()
		return nil
	case "scan":
		last := req.Last
		if last == 0 || last > st3215.MaxID {
			last = st3215.MaxID
		}
		if req.First > last {
			return fmt.Errorf("invalid ID range %d..%d", req.First, last)
		}
		return a.board.Scan(context.Background(), req.First, last)
	case "cancelScan", "finishScan":
		a.board.StopScan(req.Type == "finishScan") // finish: keep the servos found so far
		return nil
	case "color":
		color, err := internal.ValidColor(req.Color)
		if err != nil {
			return err
		}
		if err := a.cfg.Update(req.ID, func(sc *internal.ServoConfig) { sc.Color = color }); err != nil {
			return err
		}
		a.BroadcastState()
		return nil
	case "groupSave":
		return a.groupSave(req)
	case "groupDelete":
		return a.groupDelete(req.Group)
	case "autotune":
		return a.startAutotune(req)
	case "autotuneStop":
		a.stopAutotune()
		return nil
	case "autotuneSave", "autotuneRevert":
		return a.finishAutotune(req.Type == "autotuneSave")
	}
	// Don't let other commands move servos while auto-tune is testing them.
	if changesServo[req.Type] {
		for _, id := range a.targets(req) {
			if a.tuning(id) {
				return fmt.Errorf("servo %d is being auto-tuned; stop the auto-tune first", id)
			}
		}
	}
	if req.Group != "" {
		return a.board.WithBus(func(bus *st3215.Bus) error { return a.groupCommand(bus, req) })
	}
	return a.board.WithBus(func(bus *st3215.Bus) error { return a.servoCommand(c, bus, req) })
}

// targets are the servos a request is for: req.ID, or the members of
// req.Group (leader first).
func (a *app) targets(req internal.Request) []uint8 {
	if req.Group != "" {
		g, _ := a.cfg.Group(req.Group)
		return g.Members
	}
	return []uint8{req.ID}
}

// servoCommand runs a command on one servo. ID changes and settings reads are
// handled here, as they involve the board's servo list or the requesting
// window; everything else goes to the servo controller.
func (a *app) servoCommand(c *server.Client, bus *st3215.Bus, req internal.Request) error {
	if !slices.Contains(a.board.IDs(), req.ID) {
		return fmt.Errorf("servo %d was not found by the last scan", req.ID)
	}
	switch req.Type {
	case "setid":
		if slices.Contains(a.board.IDs(), req.NewID) {
			return fmt.Errorf("ID %d is already used on the bus", req.NewID)
		}
		if err := a.ctl.SetID(bus, req.ID, req.NewID); err != nil {
			return err
		}
		a.board.ChangeID(req.ID, req.NewID)
		a.BroadcastState()
		return nil
	case "factoryReset":
		newID, err := a.ctl.FactoryReset(bus, req.ID)
		if err != nil {
			return err
		}
		if newID != req.ID {
			if a.board.ChangeID(req.ID, newID) {
				a.ctl.Renumber(bus, req.ID, newID) // keep its name, mirroring and groups with it
				a.Logf("info", "servo %d is now ID %d", req.ID, newID)
			} else {
				a.Logf("error", "servo %d now answers as ID %d, which another servo already uses: disconnect one of them and give it a new ID", req.ID, newID)
			}
			a.BroadcastState()
		}
		a.Refresh(newID)
		return nil
	case "config":
		msg, err := a.ctl.ConfigMsg(bus, req.ID)
		if err != nil {
			return err
		}
		c.Push(msg)
		return nil
	}
	return a.ctl.Exec(bus, req)
}

// groupCommand runs a command on all members of req.Group.
func (a *app) groupCommand(bus *st3215.Bus, req internal.Request) error {
	g, ok := a.cfg.Group(req.Group)
	if !ok {
		return fmt.Errorf("unknown group %q", req.Group)
	}
	found := a.board.IDs()
	for _, id := range g.Members {
		if !slices.Contains(found, id) {
			return fmt.Errorf("group %s: servo %d was not found by the last scan", g.Name, id)
		}
	}
	return a.ctl.GroupExec(bus, req)
}

// move runs an "angle" or "jog" move for a servo or group and returns the
// goal it chose.
func (a *app) move(req internal.Request) (int, error) {
	if req.Group != "" {
		if _, ok := a.cfg.Group(req.Group); !ok {
			return 0, fmt.Errorf("unknown group %q", req.Group)
		}
	}
	ids := a.targets(req)
	found := a.board.IDs()
	for _, id := range ids {
		if !slices.Contains(found, id) {
			return 0, fmt.Errorf("servo %d was not found by the last scan", id)
		}
		if a.tuning(id) {
			return 0, fmt.Errorf("servo %d is being auto-tuned; stop the auto-tune first", id)
		}
	}
	var goal int
	err := a.board.WithBus(func(bus *st3215.Bus) error {
		var err error
		goal, err = a.ctl.Move(bus, req, ids)
		return err
	})
	return goal, err
}
