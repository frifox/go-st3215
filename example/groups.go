package main

import (
	"errors"
	"fmt"
	"slices"

	st3215 "github.com/frifox/go-st3215"
)

// tuningRegisters are copied from a group's leader by "copyTuning".
var tuningRegisters = []st3215.Register{
	st3215.RegPositionP, st3215.RegPositionD, st3215.RegPositionI, st3215.RegMinStartForce,
	st3215.RegCWDeadZone, st3215.RegCCWDeadZone, st3215.RegMaxTorque, st3215.RegOverloadTorque,
	st3215.RegProtectionTime, st3215.RegProtectiveTorque, st3215.RegProtectionCurrent,
	st3215.RegOverCurrentTime, st3215.RegSpeedP, st3215.RegSpeedI,
}

type groupInfo struct {
	Key          string  `json:"key"`
	Name         string  `json:"name"`
	Members      []int   `json:"members"` // leader first
	MaxSpread    int     `json:"maxSpread"`
	MaxFightLoad float64 `json:"maxFightLoad"`
	OnFight      string  `json:"onFight"`
}

func groupInfos(groups map[string]groupConfig) []groupInfo {
	out := []groupInfo{}
	for k, g := range groups {
		onFight := g.OnFight
		if onFight == "" {
			onFight = "warn"
		}
		out = append(out, groupInfo{Key: k, Name: g.Name, Members: toInts(g.Members),
			MaxSpread: g.maxSpread(), MaxFightLoad: g.maxFightLoad(), OnFight: onFight})
	}
	slices.SortFunc(out, func(a, b groupInfo) int { return a.Members[0] - b.Members[0] })
	return out
}

// groupHealth is sent with every telemetry frame.
type groupHealth struct {
	Spread   int    `json:"spread"`   // steps between the members' logical positions
	Fighting bool   `json:"fighting"` // members push in opposite directions
	Tripped  bool   `json:"tripped"`  // torque was cut by fight protection
	Problem  string `json:"problem,omitempty"`
}

// fightPolls is how many consecutive telemetry frames a fight must last
// before it is reported (filters out short load spikes when starting a move).
const fightPolls = 3

type fightState struct {
	count   int  // consecutive frames with a fight
	active  bool // reported, waiting for it to end
	tripped bool // torque cut; cleared when the group's torque is switched on
}

// checkGroups computes the health of every group from one telemetry frame
// and applies fight protection. Called by the poll loop.
func (s *server) checkGroups(bus *st3215.Bus, states map[string]servoState) map[string]groupHealth {
	out := map[string]groupHealth{}
	for key, g := range s.cfg.allGroups() {
		var fb st3215.GroupFeedback
		fb.Members = map[uint8]st3215.FeedbackResult{}
		for _, id := range g.Members {
			if st, ok := states[fmt.Sprint(id)]; ok && st.Error == "" {
				fb.Members[id] = st3215.FeedbackResult{Feedback: st.Feedback}
			}
		}
		h := groupHealth{Spread: fb.Spread(), Fighting: fb.Fighting(g.maxFightLoad())}
		if h.Spread > g.maxSpread() {
			h.Problem = fmt.Sprintf("spread %d steps (max %d)", h.Spread, g.maxSpread())
		}

		s.mu.Lock()
		if s.fights == nil {
			s.fights = map[string]*fightState{}
		}
		f := s.fights[key]
		if f == nil {
			f = &fightState{}
			s.fights[key] = f
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
		s.mu.Unlock()

		if count >= fightPolls {
			h.Problem = fmt.Sprintf("members are fighting (load beyond ±%.0f%%)", g.maxFightLoad())
		}
		if start {
			s.logf("error", "group %s: members are fighting each other (opposite load beyond ±%.0f%%, spread %d steps)", g.Name, g.maxFightLoad(), h.Spread)
		}
		if trip {
			if err := bus.SyncTorque(false, g.Members...); err != nil {
				s.logf("error", "group %s: torque off failed: %v", g.Name, err)
			} else {
				s.logf("error", "group %s: torque switched off by fight protection; align or recalibrate, then turn Torque Lock on", g.Name)
			}
			for _, id := range g.Members {
				s.scheduleRefresh(id, -1)
			}
		}
		out[key] = h
	}
	return out
}

// clearTrip re-arms fight protection when the group's torque is switched on.
func (s *server) clearTrip(key string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if f := s.fights[key]; f != nil {
		f.tripped, f.count, f.active = false, 0, false
	}
}

// groupCommand runs a control command on all members of req.Group.
func (s *server) groupCommand(bus *st3215.Bus, req request) error {
	gc, ok := s.cfg.group(req.Group)
	if !ok {
		return fmt.Errorf("unknown group %q", req.Group)
	}
	found := s.servoIDs()
	for _, id := range gc.Members {
		if !slices.Contains(found, id) {
			return fmt.Errorf("group %s: servo %d was not found by the last scan", gc.Name, id)
		}
	}
	g := bus.Group(gc.Members...)
	switch req.Type {
	case "torque":
		if req.On {
			s.clearTrip(req.Group)
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
			if err := s.setMultiTurn(bus.Servo(id), id, req.On); err != nil {
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
			if s.cfg.get(id).Zero != 0 {
				if err := s.cfg.update(id, func(c *servoConfig) { c.Zero = 0 }); err != nil {
					return err
				}
				cleared = true
			}
		}
		if cleared {
			s.broadcastState()
		}
		s.logf("info", "group %s: the current position is now 0° on every member (offsets saved on the servos)", gc.Name)
		return nil
	case "align":
		return g.Align(req.Speed, req.Acc)
	case "copyTuning":
		return g.CopyFromLeader(req.Save, tuningRegisters...)
	}
	return fmt.Errorf("command %q is not available for groups", req.Type)
}

// groupSave creates or updates a group from the browser.
func (s *server) groupSave(req request) error {
	name, err := validName(req.Name)
	if err != nil {
		return err
	}
	if name == "" {
		name = "Group" // the name is optional
	}
	members := make([]uint8, 0, len(req.Members))
	for _, id := range req.Members {
		if id < 0 || id > int(st3215.MaxID) {
			return fmt.Errorf("invalid servo ID %d", id)
		}
		members = append(members, uint8(id))
	}
	if req.MaxSpread < 0 || req.MaxFightLoad < 0 || req.MaxFightLoad > 100 {
		return fmt.Errorf("invalid fight protection limits")
	}
	onFight := req.OnFight
	if onFight == "warn" {
		onFight = "" // the default; keeps config.toml short
	}
	key, err := s.cfg.setGroup(req.Group, groupConfig{Name: name, Members: members,
		MaxSpread: req.MaxSpread, MaxFightLoad: req.MaxFightLoad, OnFight: onFight})
	if err != nil {
		return err
	}
	s.logf("info", "group %s saved: servos %v", name, members)
	s.clearTrip(key)
	s.broadcastState()
	return nil
}
