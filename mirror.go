package gosts

// Mirroring
//
// Two servos mounted facing each other (e.g. both ends of one axis) turn in
// opposite directions for the same command. Marking one of them as mirrored
// makes the typed API work in "logical" coordinates for it, so the same
// command moves both the same way:
//
//	position  logical = 2*CenterPosition - physical   (reflected about 2048)
//	speed, load, current, wheel speed, PWM duty, step count: sign flipped
//	angle limits: reflected (min and max swap)
//
// Calibrate both servos so 2048 is the same mechanical pose (CalibrateMiddle
// or SetPositionOffset) and the mirrored servo will track the other exactly.
//
// The setting lives on the Bus, keyed by ID, so it applies to every Servo
// handle, SyncMove, RegMoveTo and SyncFeedback. It is not stored on the
// servo. Raw access (Servo.Read/Write/ReadMemory, Bus.Read/Write/SyncRead/
// SyncWrite) and calibration registers (PositionOffset) are never mirrored.
//
// In single-turn mode the logical range 0..4095 maps to physical 4096..1;
// physical 4096 is past the 4095 limit, so logical 0 stops one step short.
// In multi-turn mode the physical range ±30719 corresponds to the logical
// range -26623..+34815 (shifted by 4096).

// WithMirrored marks servos as mirrored when creating a Bus.
func WithMirrored(ids ...uint8) Option {
	return func(b *Bus) {
		for _, id := range ids {
			b.setMirrored(id, true)
		}
	}
}

// SetMirrored makes the servo with this ID move in the opposite direction:
// see the Mirroring section above.
func (b *Bus) SetMirrored(id uint8, on bool) { b.setMirrored(id, on) }

// Mirrored reports whether the servo with this ID is mirrored.
func (b *Bus) Mirrored(id uint8) bool {
	b.cfgMu.RLock()
	defer b.cfgMu.RUnlock()
	return b.mirror[id]
}

func (b *Bus) setMirrored(id uint8, on bool) {
	b.cfgMu.Lock()
	defer b.cfgMu.Unlock()
	if b.mirror == nil {
		b.mirror = map[uint8]bool{}
	}
	if on {
		b.mirror[id] = true
	} else {
		delete(b.mirror, id)
	}
}

// SetMirrored is a shortcut for Bus.SetMirrored with this servo's ID.
func (s *Servo) SetMirrored(on bool) { s.bus.SetMirrored(s.id, on) }

// Mirrored reports whether this servo is mirrored.
func (s *Servo) Mirrored() bool { return s.bus.Mirrored(s.id) }

// mirrorPos converts between logical and physical absolute positions (the
// reflection is its own inverse).
func mirrorPos(on bool, pos int) int {
	if on {
		return 2*CenterPosition - pos
	}
	return pos
}

// mirrorSign flips a signed quantity (speed, load, current, duty, steps).
func mirrorSign[T int | float64](on bool, v T) T {
	if on {
		return -v
	}
	return v
}

// mirrorLimits converts angle limits; 0/0 (multi-turn) is left as is and the
// result is clamped to the register range.
func mirrorLimits(on bool, lo, hi int) (int, int) {
	if !on || (lo == 0 && hi == 0) {
		return lo, hi
	}
	clamp := func(v int) int { return max(0, min(StepsPerRev-1, v)) }
	return clamp(mirrorPos(true, hi)), clamp(mirrorPos(true, lo))
}

func (f Feedback) mirrored(on bool) Feedback {
	if !on {
		return f
	}
	f.Position = mirrorPos(true, f.Position)
	f.Speed = -f.Speed
	f.Load = -f.Load
	f.Current = -f.Current
	return f
}
