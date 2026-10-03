package main

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"strconv"

	"github.com/frifox/gosts"
	"github.com/frifox/gosts/autotune"
	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
)

// One auto-tune run at a time; progress is broadcast to every window.

type autotuneRun struct {
	key     string  // "servo:1" or "group:pitch"
	label   string  // for messages
	members []uint8 // servos being moved
	group   string  // group key, if any
	cancel  context.CancelFunc
	result  *autotune.Result
}

// tunedNames are the registers autotune.Params maps to, in config/tried terms.
func tunedValues(p autotune.Params) []internal.RegValue {
	return []internal.RegValue{
		{Register: gosts.RegPositionP.Name, Value: p.P},
		{Register: gosts.RegPositionD.Name, Value: p.D},
		{Register: gosts.RegMinStartForce.Name, Value: p.MinStart},
		{Register: gosts.RegCWDeadZone.Name, Value: p.DeadZone},
		{Register: gosts.RegCCWDeadZone.Name, Value: p.DeadZone},
	}
}

// tuning reports whether an auto-tune run is moving servo id.
func (a *app) tuning(id uint8) bool {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.at != nil && a.at.cancel != nil && slices.Contains(a.at.members, id)
}

func (a *app) autotuneTarget(bus *gosts.Bus, req internal.Request) (autotune.Target, *autotuneRun, error) {
	found := a.board.IDs()
	if req.Group != "" {
		gc, ok := a.cfg.Group(req.Group)
		if !ok {
			return nil, nil, fmt.Errorf("unknown group %q", req.Group)
		}
		for _, id := range gc.Members {
			if !slices.Contains(found, id) {
				return nil, nil, fmt.Errorf("servo %d of group %s was not found by the last scan", id, gc.Name)
			}
		}
		return autotune.ForGroup(bus.Group(gc.Members...)),
			&autotuneRun{key: "group:" + req.Group, label: "group " + gc.Name, members: gc.Members, group: req.Group}, nil
	}
	if !slices.Contains(found, req.ID) {
		return nil, nil, fmt.Errorf("servo %d was not found by the last scan", req.ID)
	}
	if g := a.cfg.GroupOf(req.ID); g != "" {
		gc, _ := a.cfg.Group(g)
		return nil, nil, fmt.Errorf("servo %d belongs to group %s: auto-tune the group instead, so the members move together", req.ID, gc.Name)
	}
	return autotune.ForServo(bus.Servo(req.ID)),
		&autotuneRun{key: "servo:" + strconv.Itoa(int(req.ID)), label: fmt.Sprintf("servo %d", req.ID), members: []uint8{req.ID}}, nil
}

// startAutotune runs in its own goroutine (it takes minutes).
func (a *app) startAutotune(req internal.Request) error {
	return a.board.WithBus(func(bus *gosts.Bus) error {
		target, run, err := a.autotuneTarget(bus, req)
		if err != nil {
			return err
		}
		ctx, cancel := context.WithCancel(context.Background())
		defer cancel()
		a.mu.Lock()
		if a.at != nil && a.at.cancel != nil {
			a.mu.Unlock()
			return fmt.Errorf("auto-tune of %s is already running", a.at.label)
		}
		run.cancel = cancel
		a.at = run
		a.mu.Unlock()

		msg := func() internal.AutotuneMsg {
			return internal.AutotuneMsg{Type: "autotune", Key: run.key, Label: run.label, Members: internal.ToInts(run.members)}
		}
		start := msg()
		start.Running = true
		a.Broadcast(start)
		a.Logf("info", "auto-tune of %s started", run.label)

		opt := autotune.Options{
			Amplitude: int(req.Amplitude/gosts.DegreesPerStep + 0.5),
			Speed:     req.Speed,
			Acc:       req.Acc,
			Tolerance: req.Tolerance,
			Progress: func(p autotune.Progress) {
				m := msg()
				m.Running = true
				m.Progress = &p
				a.Broadcast(m)
			},
		}
		res, err := autotune.Run(ctx, target, opt)

		a.mu.Lock()
		run.cancel = nil
		run.result = &res
		a.mu.Unlock()
		end := msg()
		end.Result = &res
		switch {
		case errors.Is(err, context.Canceled):
			end.Error = "stopped; the original values were restored"
			a.Logf("info", "auto-tune of %s stopped", run.label)
		case err != nil:
			end.Error = err.Error() + " (the original values were restored)"
			a.Logf("error", "auto-tune of %s: %v", run.label, err)
		default:
			// Mark the values that changed as tried-but-unsaved (Tuning card).
			var changed []internal.RegValue
			before := tunedValues(res.Before.Params)
			for i, v := range tunedValues(res.Best.Params) {
				if v.Value != before[i].Value {
					changed = append(changed, v)
				}
			}
			for _, id := range run.members {
				saved := map[string]int{}
				for _, v := range tunedValues(res.Before.Params) {
					saved[v.Register] = v.Value
				}
				a.ctl.SetTried(id, changed, false, saved)
			}
			a.Logf("info", "auto-tune of %s done: %s (until power-off; Save to keep)", run.label, res.Best.Params)
		}
		a.Broadcast(end)
		for _, id := range run.members {
			a.Refresh(id)
		}
		return nil
	})
}

// finishAutotune saves or reverts the last result.
func (a *app) finishAutotune(save bool) error {
	a.mu.Lock()
	run := a.at
	a.mu.Unlock()
	if run == nil || run.result == nil || run.cancel != nil {
		return errors.New("no finished auto-tune result")
	}
	res := *run.result
	err := a.board.WithBus(func(bus *gosts.Bus) error {
		var errs []error
		for _, id := range run.members {
			sv := bus.Servo(id)
			p := res.Before.Params
			write := sv.WriteTemporary
			if save {
				p, write = res.Best.Params, sv.Write
			}
			for _, rv := range tunedValues(p) {
				reg, _ := gosts.RegisterByName(rv.Register)
				if err := write(reg, rv.Value); err != nil {
					errs = append(errs, fmt.Errorf("servo %d %s: %w", id, reg.Name, err))
				}
			}
			a.ctl.SetTried(id, tunedValues(p), true, nil) // saved, or back to the stored values
		}
		return errors.Join(errs...)
	})
	if err != nil {
		return err
	}
	m := internal.AutotuneMsg{Type: "autotune", Key: run.key, Label: run.label, Members: internal.ToInts(run.members), Result: &res, Saved: save, Reverted: !save}
	a.Broadcast(m)
	if save {
		a.Logf("info", "auto-tune of %s: saved %s", run.label, res.Best.Params)
	} else {
		a.Logf("info", "auto-tune of %s: reverted to %s", run.label, res.Before.Params)
	}
	a.mu.Lock()
	a.at = nil
	a.mu.Unlock()
	for _, id := range run.members {
		a.Refresh(id)
	}
	return nil
}

func (a *app) stopAutotune() {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.at != nil && a.at.cancel != nil {
		a.at.cancel()
	}
}

// autotuneState is sent to a window that connects while a run is active or
// has an unsaved result.
func (a *app) autotuneState() *internal.AutotuneMsg {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.at == nil {
		return nil
	}
	return &internal.AutotuneMsg{Type: "autotune", Key: a.at.key, Label: a.at.label, Members: internal.ToInts(a.at.members),
		Running: a.at.cancel != nil, Result: a.at.result}
}
