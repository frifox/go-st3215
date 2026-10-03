package servo

import (
	"errors"
	"fmt"

	st3215 "github.com/frifox/go-st3215"
	"github.com/frifox/go-st3215/cmd/servo-ctl/internal"
)

// TuningRegisters are copied from a group's leader by "copyTuning".
var TuningRegisters = []st3215.Register{
	st3215.RegPositionP, st3215.RegPositionD, st3215.RegPositionI, st3215.RegMinStartForce,
	st3215.RegCWDeadZone, st3215.RegCCWDeadZone, st3215.RegMaxTorque, st3215.RegOverloadTorque,
	st3215.RegProtectionTime, st3215.RegProtectiveTorque, st3215.RegProtectionCurrent,
	st3215.RegOverCurrentTime, st3215.RegSpeedP, st3215.RegSpeedI,
}

// fightPolls is how many consecutive telemetry frames a fight must last
// before it is reported (filters out short load spikes when starting a move).
const fightPolls = 3

type fightState struct {
	count   int  // consecutive frames with a fight
	active  bool // reported, waiting for it to end
	tripped bool // torque cut; cleared when the group's torque is switched on
}

// CheckGroups computes the health of every group from one telemetry frame
// and applies fight protection. Called by the poll loop.
func (c *Controller) CheckGroups(bus *st3215.Bus, states map[string]internal.ServoState) map[string]internal.GroupHealth {
	out := map[string]internal.GroupHealth{}
	for key, g := range c.cfg.AllGroups() {
		var fb st3215.GroupFeedback
		fb.Members = map[uint8]st3215.FeedbackResult{}
		for _, id := range g.Members {
			if st, ok := states[fmt.Sprint(id)]; ok && st.Error == "" {
				fb.Members[id] = st3215.FeedbackResult{Feedback: st.Feedback}
			}
		}
		h := internal.GroupHealth{Spread: fb.Spread(), Fighting: fb.Fighting(g.FightLoadLimit())}
		if h.Spread > g.SpreadLimit() {
			h.Problem = fmt.Sprintf("spread %d steps (max %d)", h.Spread, g.SpreadLimit())
		}

		c.mu.Lock()
		if c.fights == nil {
			c.fights = map[string]*fightState{}
		}
		f := c.fights[key]
		if f == nil {
			f = &fightState{}
			c.fights[key] = f
		}
		if h.Fighting {
			f.count++
		} else {
			f.count, f.active = 0, false
		}
		start := f.count == fightPolls && !f.active
		if start {
			f.active = true
		}
		trip := start && g.OnFight == "torque-off" && !f.tripped
		if trip {
			f.tripped = true
		}
		h.Tripped = f.tripped
		count := f.count
		c.mu.Unlock()

		if count >= fightPolls {
			h.Problem = fmt.Sprintf("members are fighting (load beyond ±%.0f%%)", g.FightLoadLimit())
		}
		if start {
			c.n.Logf("error", "group %s: members are fighting each other (opposite load beyond ±%.0f%%, spread %d steps)", g.Name, g.FightLoadLimit(), h.Spread)
		}
		if trip {
			if err := bus.SyncTorque(false, g.Members...); err != nil {
				c.n.Logf("error", "group %s: torque off failed: %v", g.Name, err)
			} else {
				c.n.Logf("error", "group %s: torque switched off by fight protection; align or recalibrate, then turn Torque Lock on", g.Name)
			}
			for _, id := range g.Members {
				c.n.Refresh(id)
			}
		}
		out[key] = h
	}
	return out
}

// ClearTrip re-arms fight protection when the group's torque is switched on.
func (c *Controller) ClearTrip(key string) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if f := c.fights[key]; f != nil {
		f.tripped, f.count, f.active = false, 0, false
	}
}

// GroupExec runs a command on all members of req.Group, which the caller
// has checked are on the bus.
func (c *Controller) GroupExec(bus *st3215.Bus, req internal.Request) error {
	gc, ok := c.cfg.Group(req.Group)
	if !ok {
		return fmt.Errorf("unknown group %q", req.Group)
	}
	g := bus.Group(gc.Members...)
	switch req.Type {
	case "torque":
		if req.On {
			c.ClearTrip(req.Group)
		}
		return g.EnableTorque(req.On)
	case "move":
		return g.MoveTo(req.Position, req.Speed, req.Acc)
	case "step":
		return g.StepBy(req.Position, req.Speed, req.Acc)
	case "stop":
		return g.Stop()
	case "wheel":
		return g.SetWheelSpeed(req.Speed, req.Acc)
	case "pwm":
		return g.SetPWM(req.Duty)
	case "mode":
		return g.SetMode(st3215.Mode(req.Mode))
	case "torqueLimit":
		return g.SetTorqueLimit(req.Percent)
	case "multiturn":
		var errs []error
		for _, id := range gc.Members {
			if err := c.SetMultiTurn(bus, id, req.On); err != nil {
				errs = append(errs, fmt.Errorf("servo %d: %w", id, err))
			}
		}
		return errors.Join(errs...)
	case "zeroHere":
		// The pose the members are in now becomes 0° on each servo. The
		// servos' own zero replaces a virtual one, so those are cleared.
		if err := g.SetPositionAs(0); err != nil {
			return err
		}
		cleared := false
		for _, id := range gc.Members {
			if c.cfg.Get(id).Zero != 0 {
				if err := c.cfg.Update(id, func(sc *internal.ServoConfig) { sc.Zero = 0 }); err != nil {
					return err
				}
				cleared = true
			}
		}
		if cleared {
			c.n.BroadcastState()
		}
		c.n.Logf("info", "group %s: the current position is now 0° on every member (offsets saved on the servos)", gc.Name)
		return nil
	case "align":
		return g.Align(req.Speed, req.Acc)
	case "copyTuning":
		return g.CopyFromLeader(req.Save, TuningRegisters...)
	}
	return fmt.Errorf("command %q is not available for groups", req.Type)
}
