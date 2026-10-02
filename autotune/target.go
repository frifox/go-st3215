package autotune

import (
	"errors"
	"fmt"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

// Reading is one member's telemetry sample.
type Reading struct {
	ID uint8
	st3215.Feedback
	Err error
}

// Target is what gets tuned: a single servo or a group moving together.
// Positions are logical (mirroring applied).
type Target interface {
	Params() (Params, error)
	Apply(Params) error // until power-off (lock closed)
	Save(Params) error  // persist in EEPROM
	Position() (int, error)
	// Limits returns the allowed position range; lo == hi means unlimited
	// (multi-turn).
	Limits() (lo, hi int, err error)
	MoveTo(pos, speed int, acc uint8) error
	Stop() error
	Read() ([]Reading, error)
	Now() time.Time
}

var tunedRegs = []st3215.Register{st3215.RegPositionP, st3215.RegPositionD, st3215.RegMinStartForce,
	st3215.RegCWDeadZone, st3215.RegCCWDeadZone}

func values(p Params) []int { return []int{p.P, p.D, p.MinStart, p.DeadZone, p.DeadZone} }

func readParams(s *st3215.Servo) (Params, error) {
	mem, err := s.ReadMemory()
	if err != nil {
		return Params{}, err
	}
	return Params{P: st3215.RegPositionP.Value(mem), D: st3215.RegPositionD.Value(mem),
		MinStart: st3215.RegMinStartForce.Value(mem), DeadZone: st3215.RegCWDeadZone.Value(mem)}, nil
}

func writeParams(s *st3215.Servo, p Params, save bool) error {
	write := s.WriteTemporary
	if save {
		write = s.Write
	}
	for i, r := range tunedRegs {
		if err := write(r, values(p)[i]); err != nil {
			return fmt.Errorf("servo %d %s: %w", s.ID(), r.Name, err)
		}
	}
	return nil
}

// ForServo tunes one servo.
func ForServo(s *st3215.Servo) Target { return servoTarget{s} }

type servoTarget struct{ s *st3215.Servo }

func (t servoTarget) Params() (Params, error)                { return readParams(t.s) }
func (t servoTarget) Apply(p Params) error                   { return writeParams(t.s, p, false) }
func (t servoTarget) Save(p Params) error                    { return writeParams(t.s, p, true) }
func (t servoTarget) Position() (int, error)                 { return t.s.Position() }
func (t servoTarget) Limits() (int, int, error)              { return t.s.AngleLimits() }
func (t servoTarget) MoveTo(pos, speed int, acc uint8) error { return t.s.MoveTo(pos, speed, acc) }
func (t servoTarget) Stop() error                            { return t.s.Stop() }
func (t servoTarget) Now() time.Time                         { return time.Now() }
func (t servoTarget) Read() ([]Reading, error) {
	f, err := t.s.Feedback()
	if err != nil {
		return nil, err
	}
	return []Reading{{ID: t.s.ID(), Feedback: f}}, nil
}

// ForGroup tunes a group: every member gets the same values and moves with
// the others, so coupled servos are never tuned against each other. Metrics
// are the worst over all members.
func ForGroup(g *st3215.Group) Target { return groupTarget{g} }

type groupTarget struct{ g *st3215.Group }

func (t groupTarget) Params() (Params, error) { return readParams(t.g.Leader()) }
func (t groupTarget) Apply(p Params) error {
	return t.each(func(s *st3215.Servo) error { return writeParams(s, p, false) })
}
func (t groupTarget) Save(p Params) error {
	return t.each(func(s *st3215.Servo) error { return writeParams(s, p, true) })
}
func (t groupTarget) Position() (int, error) { return t.g.Leader().Position() }
func (t groupTarget) Limits() (int, int, error) {
	lo, hi := 0, 0
	for i, id := range t.g.IDs() {
		l, h, err := t.g.Servo(id).AngleLimits()
		if err != nil {
			return 0, 0, err
		}
		if l == h { // a multi-turn member doesn't restrict the range
			continue
		}
		if i == 0 || lo == hi {
			lo, hi = l, h
		} else {
			lo, hi = max(lo, l), min(hi, h)
		}
	}
	return lo, hi, nil
}
func (t groupTarget) MoveTo(pos, speed int, acc uint8) error { return t.g.MoveTo(pos, speed, acc) }
func (t groupTarget) Stop() error                            { return t.g.Stop() }
func (t groupTarget) Now() time.Time                         { return time.Now() }
func (t groupTarget) Read() ([]Reading, error) {
	f, err := t.g.Feedback()
	if err != nil {
		return nil, err
	}
	out := make([]Reading, 0, len(f.Members))
	for _, id := range t.g.IDs() {
		m := f.Members[id]
		out = append(out, Reading{ID: id, Feedback: m.Feedback, Err: m.Err})
	}
	return out, nil
}

func (t groupTarget) each(f func(*st3215.Servo) error) error {
	var errs []error
	for _, id := range t.g.IDs() {
		if err := f(t.g.Servo(id)); err != nil {
			errs = append(errs, err)
		}
	}
	return errors.Join(errs...)
}

// SaveParams persists p on the target (e.g. Result.Best.Params).
func SaveParams(t Target, p Params) error { return t.Save(p) }
