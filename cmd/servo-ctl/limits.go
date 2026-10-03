package main

import (
	_ "embed"
	"fmt"
	"math"

	st3215 "github.com/frifox/go-st3215"
)

// Motion range: limits kept in config.toml, enforced by the library and,
// in single-turn mode, written to the servo as angle limits.

// setLimits restricts the servo to the arc from req.MinDeg clockwise to
// req.MaxDeg (console degrees: the servo reading minus the virtual 0°). The
// range is kept in config.toml on the encoder scale and enforced by the
// library for every move (multi-turn too); in single-turn mode it is also
// written to the servo as angle limits.
func (s *server) setLimits(bus *st3215.Bus, sv *st3215.Servo, req request) error {
	wrap := func(d float64) float64 { return math.Mod(math.Mod(d, 360)+360, 360) }
	toSteps := func(d float64) int { return int(math.Round(d / st3215.DegreesPerStep)) }
	span := toSteps(wrap(req.MaxDeg - req.MinDeg))
	if span < 2 || span > st3215.StepsPerRev-2 {
		return fmt.Errorf("the range must be more than 0° and less than a full turn")
	}
	zero, err := sv.Zero()
	if err != nil {
		return err
	}
	lo := toSteps(wrap(req.MinDeg+s.cfg.get(req.ID).Zero)) % st3215.StepsPerRev // reading
	r := st3215.Range{Lo: wrapSteps(lo + zero), Hi: wrapSteps(lo + zero + span)}
	if err := s.cfg.update(req.ID, func(c *servoConfig) { c.Range = []int{r.Lo, r.Hi} }); err != nil {
		return err
	}
	bus.SetRange(req.ID, r)
	s.broadcastState()
	s.logf("info", "servo %d: motion limited to %.1f°…%.1f° (saved in config.toml)", req.ID, req.MinDeg, req.MaxDeg)
	lim, err := multiTurn(sv)
	if err != nil || lim {
		return err
	}
	return s.applyAngleLimits(bus, req.ID)
}

// clearLimits removes the motion range; a single-turn servo may again use the
// whole turn.
func (s *server) clearLimits(bus *st3215.Bus, sv *st3215.Servo, id uint8) error {
	if err := s.cfg.update(id, func(c *servoConfig) { c.Range = nil }); err != nil {
		return err
	}
	bus.ClearRange(id)
	s.broadcastState()
	s.logf("info", "servo %d: motion range cleared", id)
	multi, err := multiTurn(sv)
	if err != nil || multi {
		return err
	}
	return sv.SetMultiTurn(false) // single-turn, limits 0..4095
}

// setMultiTurn switches multi-turn mode. Leaving it, a servo with a motion
// range gets that range as its angle limits (the library enforces the range
// either way).
func (s *server) setMultiTurn(bus *st3215.Bus, id uint8, on bool) error {
	if !on && len(s.cfg.get(id).Range) == 2 {
		return s.applyAngleLimits(bus, id)
	}
	return bus.Servo(id).SetMultiTurn(on)
}

func multiTurn(sv *st3215.Servo) (bool, error) {
	lo, hi, err := sv.AngleLimits()
	return lo == 0 && hi == 0, err
}

func wrapSteps(p int) int { return (p%st3215.StepsPerRev + st3215.StepsPerRev) % st3215.StepsPerRev }

// shiftZero moves the servo's 0 by shift steps (readings drop by shift) on
// the servo and, if it is in a group, on every member alike, adjusting the
// virtual 0° and dial orientation so the console angles stay the same. It
// returns the shift applied to servo id.
func (s *server) shiftZero(bus *st3215.Bus, id uint8, shift int) (int, error) {
	ids := []uint8{id}
	if g, ok := s.cfg.group(s.cfg.groupOf(id)); ok {
		ids = g.Members
	}
	wrap := func(d float64) float64 { return math.Mod(math.Mod(d, 360)+360, 360) }
	round := func(v float64) float64 { return math.Round(wrap(v)*10) / 10 }
	appliedTo := 0
	for _, m := range ids {
		sv := bus.Servo(m)
		z0, err := sv.Zero()
		if err != nil {
			return 0, err
		}
		if err := sv.SetZeroAt(shift); err != nil {
			return 0, fmt.Errorf("servo %d: %w", m, err)
		}
		z1, err := sv.Zero()
		if err != nil {
			return 0, err
		}
		applied := st3215.CircularDiff(z1, z0) // an offset of -2048 can't be stored: one step off
		d := float64(applied) * st3215.DegreesPerStep
		if err := s.cfg.update(m, func(c *servoConfig) {
			c.Zero, c.DialUp = round(c.Zero-d), round(c.DialUp-d)
		}); err != nil {
			return 0, err
		}
		if m == id {
			appliedTo = applied
		} else if err := s.refreshAngleLimits(sv, m); err != nil {
			return 0, err
		}
		s.logf("info", "servo %d: 0° moved by %.1f° (saved on the servo) so the range doesn't cross it; the console angles are unchanged", m, d)
	}
	s.broadcastState()
	return appliedTo, nil
}

// refreshAngleLimits rewrites a single-turn servo's angle limits from its
// motion range after its zero moved (the limits are in readings). If the
// range now crosses the servo's 0, the whole turn is allowed instead and the
// console keeps enforcing the range.
func (s *server) refreshAngleLimits(sv *st3215.Servo, id uint8) error {
	rg := s.cfg.get(id).Range
	if multi, err := multiTurn(sv); err != nil || multi || len(rg) != 2 {
		return err
	}
	zero, err := sv.Zero()
	if err != nil {
		return err
	}
	r := st3215.Range{Lo: rg[0], Hi: rg[1]}
	lo := wrapSteps(r.Lo - zero)
	if lo+r.Span() > st3215.StepsPerRev-1 {
		s.logf("info", "servo %d: its range now crosses its 0, so its angle limits allow the whole turn; this console still enforces the range", id)
		return sv.SetMultiTurn(false)
	}
	return sv.SetAngleLimits(lo, lo+r.Span())
}

// applyAngleLimits writes the servo's motion range as its angle limits
// (single-turn). Limits are a plain min < max range of readings, so an arc
// that crosses the servo's own 0 is first moved clear of it by shifting the
// servo's zero; the virtual 0° and dial orientation shift along, so the
// console shows the same angles. Group members share an axis, so all of
// them get the same shift (their readings must keep agreeing).
func (s *server) applyAngleLimits(bus *st3215.Bus, id uint8) error {
	sv := bus.Servo(id)
	rg := s.cfg.get(id).Range
	if len(rg) != 2 {
		return nil
	}
	r := st3215.Range{Lo: rg[0], Hi: rg[1]}
	span := r.Span()
	zero, err := sv.Zero()
	if err != nil {
		return err
	}
	lo := wrapSteps(r.Lo - zero) // reading
	if lo+span > st3215.StepsPerRev-1 {
		// Readings drop by shift. Shift by whole quarter turns where that fits
		// (any range up to 270°), so the virtual 0° moves by exactly 90° and
		// the console angles stay exact; otherwise center the arc on 2048.
		shift, best := st3215.CircularDiff(lo, st3215.StepsPerRev/2-span/2), st3215.StepsPerRev
		for k := 1; k < 4; k++ {
			q := k * st3215.StepsPerRev / 4
			nlo := wrapSteps(lo - q)
			if c := abs(nlo + span/2 - st3215.StepsPerRev/2); nlo+span <= st3215.StepsPerRev-1 && c < best {
				shift, best = st3215.CircularDiff(q, 0), c
			}
		}
		applied, err := s.shiftZero(bus, id, shift)
		if err != nil {
			return err
		}
		lo = wrapSteps(lo - applied)
	}
	if err := sv.SetAngleLimits(lo, lo+span); err != nil {
		return err
	}
	// A goal kept from multi-turn mode may carry a turn count: hold here instead.
	if err := sv.Stop(); err != nil {
		return err
	}
	s.logf("info", "servo %d: angle limits %d…%d written to the servo", id, lo, lo+span)
	return nil
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}
