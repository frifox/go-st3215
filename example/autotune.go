package main

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"strconv"

	st3215 "github.com/frifox/go-st3215"
	"github.com/frifox/go-st3215/autotune"
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

type autotuneMsg struct {
	Type     string             `json:"type"` // "autotune"
	Key      string             `json:"key"`
	Label    string             `json:"label"`
	Members  []int              `json:"members"`
	Running  bool               `json:"running"`
	Progress *autotune.Progress `json:"progress,omitempty"`
	Result   *autotune.Result   `json:"result,omitempty"`
	Error    string             `json:"error,omitempty"`
	Saved    bool               `json:"saved,omitempty"`
	Reverted bool               `json:"reverted,omitempty"`
}

// tunedNames are the registers autotune.Params maps to, in config/tried terms.
func tunedValues(p autotune.Params) []regValue {
	return []regValue{
		{Register: st3215.RegPositionP.Name, Value: p.P},
		{Register: st3215.RegPositionD.Name, Value: p.D},
		{Register: st3215.RegMinStartForce.Name, Value: p.MinStart},
		{Register: st3215.RegCWDeadZone.Name, Value: p.DeadZone},
		{Register: st3215.RegCCWDeadZone.Name, Value: p.DeadZone},
	}
}

// tuning reports whether an auto-tune run is moving servo id.
func (s *server) tuning(id uint8) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.at != nil && s.at.cancel != nil && slices.Contains(s.at.members, id)
}

func (s *server) autotuneTarget(bus *st3215.Bus, req request) (autotune.Target, *autotuneRun, error) {
	found := s.servoIDs()
	if req.Group != "" {
		gc, ok := s.cfg.group(req.Group)
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
	if g := s.cfg.groupOf(req.ID); g != "" {
		gc, _ := s.cfg.group(g)
		return nil, nil, fmt.Errorf("servo %d belongs to group %s: auto-tune the group instead, so the members move together", req.ID, gc.Name)
	}
	return autotune.ForServo(bus.Servo(req.ID)),
		&autotuneRun{key: "servo:" + strconv.Itoa(int(req.ID)), label: fmt.Sprintf("servo %d", req.ID), members: []uint8{req.ID}}, nil
}

// startAutotune runs in its own goroutine (it takes minutes).
func (s *server) startAutotune(req request) error {
	return s.withBus(func(bus *st3215.Bus) error {
		target, run, err := s.autotuneTarget(bus, req)
		if err != nil {
			return err
		}
		ctx, cancel := context.WithCancel(context.Background())
		defer cancel()
		s.mu.Lock()
		if s.at != nil && s.at.cancel != nil {
			s.mu.Unlock()
			return fmt.Errorf("auto-tune of %s is already running", s.at.label)
		}
		run.cancel = cancel
		s.at = run
		s.mu.Unlock()

		msg := func() autotuneMsg {
			return autotuneMsg{Type: "autotune", Key: run.key, Label: run.label, Members: toInts(run.members)}
		}
		start := msg()
		start.Running = true
		s.broadcast(start)
		s.logf("info", "auto-tune of %s started", run.label)

		opt := autotune.Options{
			Amplitude: int(req.Amplitude/st3215.DegreesPerStep + 0.5),
			Speed:     req.Speed,
			Acc:       req.Acc,
			Tolerance: req.Tolerance,
			Progress: func(p autotune.Progress) {
				m := msg()
				m.Running = true
				m.Progress = &p
				s.broadcast(m)
			},
		}
		res, err := autotune.Run(ctx, target, opt)

		s.mu.Lock()
		run.cancel = nil
		run.result = &res
		s.mu.Unlock()
		end := msg()
		end.Result = &res
		switch {
		case errors.Is(err, context.Canceled):
			end.Error = "stopped; the original values were restored"
			s.logf("info", "auto-tune of %s stopped", run.label)
		case err != nil:
			end.Error = err.Error() + " (the original values were restored)"
			s.logf("error", "auto-tune of %s: %v", run.label, err)
		default:
			// Mark the values that changed as tried-but-unsaved (Tuning card).
			var changed []regValue
			before := tunedValues(res.Before.Params)
			for i, v := range tunedValues(res.Best.Params) {
				if v.Value != before[i].Value {
					changed = append(changed, v)
				}
			}
			for _, id := range run.members {
				s.setTried(id, changed, false)
			}
			s.logf("info", "auto-tune of %s done: %s (until power-off; Save to keep)", run.label, res.Best.Params)
		}
		s.broadcast(end)
		for _, id := range run.members {
			s.scheduleRefresh(id, -1)
		}
		return nil
	})
}

// finishAutotune saves or reverts the last result.
func (s *server) finishAutotune(save bool) error {
	s.mu.Lock()
	run := s.at
	s.mu.Unlock()
	if run == nil || run.result == nil || run.cancel != nil {
		return errors.New("no finished auto-tune result")
	}
	res := *run.result
	err := s.withBus(func(bus *st3215.Bus) error {
		var errs []error
		for _, id := range run.members {
			sv := bus.Servo(id)
			p := res.Before.Params
			write := sv.WriteTemporary
			if save {
				p, write = res.Best.Params, sv.Write
			}
			for _, rv := range tunedValues(p) {
				reg, _ := st3215.RegisterByName(rv.Register)
				if err := write(reg, rv.Value); err != nil {
					errs = append(errs, fmt.Errorf("servo %d %s: %w", id, reg.Name, err))
				}
			}
			s.setTried(id, tunedValues(p), true) // saved, or back to the stored values
		}
		return errors.Join(errs...)
	})
	if err != nil {
		return err
	}
	m := autotuneMsg{Type: "autotune", Key: run.key, Label: run.label, Members: toInts(run.members), Result: &res, Saved: save, Reverted: !save}
	s.broadcast(m)
	if save {
		s.logf("info", "auto-tune of %s: saved %s", run.label, res.Best.Params)
	} else {
		s.logf("info", "auto-tune of %s: reverted to %s", run.label, res.Before.Params)
	}
	s.mu.Lock()
	s.at = nil
	s.mu.Unlock()
	for _, id := range run.members {
		s.scheduleRefresh(id, -1)
	}
	return nil
}

func (s *server) stopAutotune() {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.at != nil && s.at.cancel != nil {
		s.at.cancel()
	}
}

// autotuneState is sent to a window that connects while a run is active or
// has an unsaved result.
func (s *server) autotuneState() *autotuneMsg {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.at == nil {
		return nil
	}
	return &autotuneMsg{Type: "autotune", Key: s.at.key, Label: s.at.label, Members: toInts(s.at.members),
		Running: s.at.cancel != nil, Result: s.at.result}
}
