package st3215

import (
	"context"
	"errors"
	"fmt"
	"time"
)

// Group drives several servos as one: every command goes out as a single
// SYNC WRITE so all members act at the same instant, and feedback is read
// with one SYNC READ. Positions are logical, so mirrored members (see
// Bus.SetMirrored) move the same way as the others.
//
// The first member is the leader: the reference used by Align.
type Group struct {
	bus *Bus
	ids []uint8
}

// Group returns a handle for the given servos (at least one). It does not
// talk to the bus.
func (b *Bus) Group(ids ...uint8) *Group {
	return &Group{bus: b, ids: append([]uint8(nil), ids...)}
}

// IDs returns the member IDs, leader first.
func (g *Group) IDs() []uint8 { return append([]uint8(nil), g.ids...) }

// Leader returns the first member.
func (g *Group) Leader() *Servo { return g.bus.Servo(g.ids[0]) }

// Servo returns the handle of one member.
func (g *Group) Servo(id uint8) *Servo { return g.bus.Servo(id) }

// EnableTorque switches the motor output of all members on or off.
func (g *Group) EnableTorque(on bool) error { return g.bus.SyncTorque(on, g.ids...) }

// MoveTo sends all members to the same logical position at the same time.
func (g *Group) MoveTo(pos, speed int, acc uint8) error {
	targets := make([]Target, len(g.ids))
	for i, id := range g.ids {
		targets[i] = Target{ID: id, Position: pos, Speed: speed, Acc: acc}
	}
	return g.bus.SyncMove(targets...)
}

// MoveToShortest moves all members to the angle of pos the short way round,
// measured on the leader (see Servo.ShortestGoal), and returns the goal used.
func (g *Group) MoveToShortest(pos, speed int, acc uint8) (int, error) {
	goal, err := g.Leader().ShortestGoal(pos, -MultiTurnLimit, MultiTurnLimit)
	if err != nil {
		return 0, err
	}
	return goal, g.MoveTo(goal, speed, acc)
}

// StepBy moves all members a relative number of steps in ModeStep.
func (g *Group) StepBy(steps, speed int, acc uint8) error {
	entries := make([]SyncWriteEntry, len(g.ids))
	for i, id := range g.ids {
		data, err := moveData(mirrorSign(g.bus.Mirrored(id), steps), speed, acc)
		if err != nil {
			return err
		}
		entries[i] = SyncWriteEntry{ID: id, Data: data}
	}
	return g.bus.SyncWrite(RegAcceleration.Addr, entries)
}

// Stop holds every member at its present position.
func (g *Group) Stop() error {
	return g.each(func(s *Servo) error { return s.Stop() })
}

// SetMode changes the operating mode of all members (persisted).
func (g *Group) SetMode(m Mode) error {
	return g.each(func(s *Servo) error { return s.SetMode(m) })
}

// SetWheelSpeed sets the same logical rotation speed on all members (ModeWheel).
func (g *Group) SetWheelSpeed(speed int, acc uint8) error {
	if err := g.syncValue(RegAcceleration, func(uint8) int { return int(acc) }); err != nil {
		return err
	}
	return g.syncValue(RegGoalSpeed, func(id uint8) int { return mirrorSign(g.bus.Mirrored(id), speed) })
}

// SetPWM sets the same logical duty on all members (ModePWM).
func (g *Group) SetPWM(duty int) error {
	return g.syncValue(RegGoalTime, func(id uint8) int { return mirrorSign(g.bus.Mirrored(id), duty) })
}

// SetTorqueLimit sets the runtime torque limit of all members in % (0..100).
func (g *Group) SetTorqueLimit(percent float64) error {
	v := int(percent*10 + 0.5)
	return g.syncValue(RegTorqueLimit, func(uint8) int { return v })
}

// SetMultiTurn switches multi-turn mode (angle limits 0/0) on or off for every
// member. Persisted.
func (g *Group) SetMultiTurn(on bool) error {
	return g.each(func(s *Servo) error { return s.SetMultiTurn(on) })
}

// syncValue writes one SRAM register on every member with a single packet.
func (g *Group) syncValue(r Register, value func(id uint8) int) error {
	entries := make([]SyncWriteEntry, len(g.ids))
	for i, id := range g.ids {
		data, err := r.encode(value(id))
		if err != nil {
			return err
		}
		entries[i] = SyncWriteEntry{ID: id, Data: data}
	}
	return g.bus.SyncWrite(r.Addr, entries)
}

// each runs f on every member and joins the errors.
func (g *Group) each(f func(*Servo) error) error {
	var errs []error
	for _, id := range g.ids {
		if err := f(g.bus.Servo(id)); err != nil {
			errs = append(errs, fmt.Errorf("servo %d: %w", id, err))
		}
	}
	return errors.Join(errs...)
}

// GroupFeedback is the live state of all members.
type GroupFeedback struct {
	Members map[uint8]FeedbackResult
}

// Spread is the largest difference between the members' logical positions
// (steps), over the members that answered. Positions are compared the short
// way round, so members either side of the 0/4095 seam aren't far apart.
func (f GroupFeedback) Spread() int {
	first, ref, lo, hi := true, 0, 0, 0
	for _, m := range f.Members {
		if m.Err != nil {
			continue
		}
		if first {
			ref, first = m.Position, false
		}
		d := CircularDiff(m.Position, ref)
		lo, hi = min(lo, d), max(hi, d)
	}
	return hi - lo
}

// Fighting reports whether members push against each other: at least one
// member's logical load is above +threshold and another's below -threshold
// (percent). This happens when coupled servos disagree about the position.
func (f GroupFeedback) Fighting(threshold float64) bool {
	pos, neg := false, false
	for _, m := range f.Members {
		if m.Err != nil {
			continue
		}
		pos = pos || m.Load > threshold
		neg = neg || m.Load < -threshold
	}
	return pos && neg
}

// Err returns an error naming the members that did not answer, if any.
func (f GroupFeedback) Err() error {
	var errs []error
	for _, m := range f.Members {
		if m.Err != nil {
			errs = append(errs, m.Err)
		}
	}
	return errors.Join(errs...)
}

// Feedback reads the live state of all members with one SYNC READ.
func (g *Group) Feedback() (GroupFeedback, error) {
	res, err := g.bus.SyncFeedback(g.ids...)
	return GroupFeedback{Members: res}, err
}

// Align moves every member to the leader's present logical position, so
// coupled servos agree before torque is applied to the whole group.
func (g *Group) Align(speed int, acc uint8) error {
	pos, err := g.Leader().AbsolutePosition()
	if err != nil {
		return fmt.Errorf("leader: %w", err)
	}
	return g.MoveTo(pos, speed, acc)
}

// WaitForPosition polls until every member has stopped within Tolerance of
// target, ctx is done, or a member reports a FailOn fault.
func (g *Group) WaitForPosition(ctx context.Context, target int, opt WaitOptions) (GroupFeedback, error) {
	opt.defaults()
	t := time.NewTicker(opt.Poll)
	defer t.Stop()
	for {
		f, err := g.Feedback()
		if err != nil {
			return f, err
		}
		if err := f.Err(); err != nil {
			return f, err
		}
		done := true
		for id, m := range f.Members {
			if m.Status&opt.FailOn != 0 {
				return f, fmt.Errorf("%w: servo %d: %s", ErrFault, id, m.Status)
			}
			d := CircularDiff(m.Position, target)
			if d < 0 {
				d = -d
			}
			if m.Moving || d > opt.Tolerance {
				done = false
			}
		}
		if done {
			return f, nil
		}
		select {
		case <-ctx.Done():
			return f, ctx.Err()
		case <-t.C:
		}
	}
}

// CopyFromLeader copies register values (typically tuning: PID, dead zones,
// protection) from the leader to the other members. With save the values are
// persisted, otherwise they last until the members are power-cycled (see
// Servo.WriteTemporary). Raw values are copied, so don't use it for
// direction-dependent registers of mirrored servos (angle limits, offset).
func (g *Group) CopyFromLeader(save bool, regs ...Register) error {
	mem, err := g.Leader().ReadMemory()
	if err != nil {
		return fmt.Errorf("leader: %w", err)
	}
	return g.each(func(s *Servo) error {
		if s.ID() == g.ids[0] {
			return nil
		}
		for _, r := range regs {
			write := s.WriteTemporary
			if save {
				write = s.Write
			}
			if err := write(r, r.Value(mem)); err != nil {
				return fmt.Errorf("%s: %w", r.Name, err)
			}
		}
		return nil
	})
}
