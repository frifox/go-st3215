package gosts

import "fmt"

// Motion range
//
// A motion range keeps a servo on one arc of the circle: from Lo clockwise
// (increasing positions) to Hi. Unlike angle limits it also works in
// multi-turn mode, and moves never cross the gap: a goal is reached along the
// arc, even when the other way round is shorter. Goals outside the arc are
// clamped to its nearer end.
//
// Lo and Hi are on the encoder scale (what the position reads with the
// servo's zero at the encoder's 0, see Servo.Zero), logical for mirrored
// servos, so the range stays on the same physical arc when the zero changes.
//
// The range is enforced by this package for MoveTo, MoveToShortest,
// RegMoveTo, SyncMove and the Group moves (not StepBy in ModeStep, wheel or
// PWM). It lives on the Bus by ID (it follows SetID) and is not stored on the
// servo: set it each time your program starts, and use SetAngleLimits too if
// other programs drive the servo in single-turn mode.

// Range is an arc from Lo clockwise to Hi, in encoder-scale steps (0..4095).
type Range struct{ Lo, Hi int }

// Span is the length of the arc in steps.
func (r Range) Span() int { return wrapSteps(r.Hi - r.Lo) }

func wrapSteps(p int) int { return (p%StepsPerRev + StepsPerRev) % StepsPerRev }

// SetRange restricts the servo with this ID to the arc r (see Motion range).
func (b *Bus) SetRange(id uint8, r Range) {
	b.cfgMu.Lock()
	defer b.cfgMu.Unlock()
	if b.ranges == nil {
		b.ranges = map[uint8]Range{}
	}
	b.ranges[id] = Range{wrapSteps(r.Lo), wrapSteps(r.Hi)}
}

// ClearRange removes the servo's motion range.
func (b *Bus) ClearRange(id uint8) {
	b.cfgMu.Lock()
	defer b.cfgMu.Unlock()
	delete(b.ranges, id)
}

// Range returns the servo's motion range, if one is set.
func (b *Bus) Range(id uint8) (Range, bool) {
	b.cfgMu.RLock()
	defer b.cfgMu.RUnlock()
	r, ok := b.ranges[id]
	return r, ok
}

// Range returns this servo's motion range, if one is set.
func (s *Servo) Range() (Range, bool) { return s.bus.Range(s.id) }

// arc returns the range in present-reading coordinates: its start and span.
func (s *Servo) arc() (lo, span int, ok bool, err error) {
	r, ok := s.Range()
	if !ok {
		return 0, 0, false, nil
	}
	zero, err := s.Zero()
	if err != nil {
		return 0, 0, false, err
	}
	return wrapSteps(r.Lo - zero), r.Span(), true, nil
}

// rangeGoal turns a goal into one that stays on the motion range: clamped to
// the arc, and in multi-turn mode reached along it.
func (s *Servo) rangeGoal(pos int) (int, error) {
	lo, span, ok, err := s.arc()
	if err != nil || !ok {
		return pos, err
	}
	along := func(p int) int { return wrapSteps(p - lo) } // > span: in the gap
	t := wrapSteps(pos)
	if along(t) > span {
		if abs(CircularDiff(t, lo)) <= abs(CircularDiff(t, lo+span)) {
			t = lo
		} else {
			t = wrapSteps(lo + span)
		}
	}
	l, h, err := s.AngleLimits()
	if err != nil {
		return 0, err
	}
	cur, err := s.AbsolutePosition()
	if err != nil {
		return 0, err
	}
	c := wrapSteps(cur)
	if l != 0 || h != 0 {
		// Single-turn: the servo goes straight from c to t, which crosses
		// the gap if the arc wraps past 4095 and the gap lies between them.
		if gapLo, gapHi := wrapSteps(lo+span+1), lo-1; lo+span > StepsPerRev-1 && gapLo <= gapHi &&
			max(c, t) >= gapLo && min(c, t) <= gapHi {
			return 0, fmt.Errorf("gosts: servo %d: this move would leave the motion range (it crosses the servo's 0); use multi-turn mode or move the zero", s.id)
		}
		return t, nil
	}
	if along(c) <= span {
		return cur + along(t) - along(c), nil
	}
	return cur + CircularDiff(t, c), nil // outside the arc (moved by hand): back the short way
}

// MotionLimits returns the allowed goals around the present position, in goal
// coordinates: the angle limits combined with the motion range. lo == hi
// means unlimited. If the servo is outside its motion range, the range is
// empty (hi = lo+1).
func (s *Servo) MotionLimits() (lo, hi int, err error) {
	lo, hi, err = s.AngleLimits()
	if err != nil {
		return 0, 0, err
	}
	a, span, ok, err := s.arc()
	if err != nil || !ok {
		return lo, hi, err
	}
	cur, err := s.AbsolutePosition()
	if err != nil {
		return 0, 0, err
	}
	along := wrapSteps(wrapSteps(cur) - a)
	if along > span {
		return cur, cur + 1, nil
	}
	rlo, rhi := cur-along, cur-along+span
	if lo == hi {
		return rlo, rhi, nil
	}
	return max(lo, rlo), min(hi, rhi), nil
}
