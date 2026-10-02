package st3215

import "testing"

// rangeServo puts servo 1 at reading cur (holding there) in multi-turn or
// single-turn mode.
func rangeServo(t *testing.T, cur int, multi bool) (*Bus, *fakePort) {
	t.Helper()
	b, p := newTestBus(t, 1)
	m := p.servos[1].mem[:]
	putU16(m[RegPresentPosition.Addr:], uint16(cur))
	putU16(m[RegGoalPosition.Addr:], uint16(cur))
	m[RegTorqueEnable.Addr] = 1
	if multi {
		putU16(m[RegMaxAngleLimit.Addr:], 0)
	}
	return b, p
}

func TestRangeMultiTurnAlongArc(t *testing.T) {
	// Arc 300°..90° through 0 (span 150°), arm at 342.8°.
	b, p := rangeServo(t, 3900, true)
	b.SetRange(1, Range{Lo: 3413, Hi: 1024})
	s := b.Servo(1)
	for _, c := range []struct{ to, want int }{
		{500, 3900 + 696},           // across 0, forward
		{2000, 3900 + (1707 - 487)}, // in the gap: clamped to the nearer end (90°)
		{3000, 3900 - 487},          // in the gap, nearer 300°
	} {
		if err := s.MoveTo(c.to, 0, 0); err != nil {
			t.Fatal(err)
		}
		if g := physGoal(p, 1); g != c.want {
			t.Fatalf("MoveTo(%d): goal %d, want %d", c.to, g, c.want)
		}
	}
	lo, hi, err := s.MotionLimits()
	if err != nil || lo != 3900-487 || hi != 3900-487+1707 {
		t.Fatal("motion limits", lo, hi, err)
	}
}

func TestRangeNeverCrossesGap(t *testing.T) {
	// A 264° arc 0..3000: from 2900 to 100 the short way (+1296) crosses
	// the gap, so the move goes back along the arc instead.
	b, p := rangeServo(t, 2900, true)
	b.SetRange(1, Range{Lo: 0, Hi: 3000})
	goal, err := b.Servo(1).MoveToShortest(100, 0, 0)
	if err != nil {
		t.Fatal(err)
	}
	if goal != 100 || physGoal(p, 1) != 100 {
		t.Fatal("goal", goal, physGoal(p, 1))
	}
	// SyncMove follows the same rule (the fake servo doesn't move: hold at 2900 again).
	putU16(p.servos[1].mem[RegGoalPosition.Addr:], 2900)
	if err := b.SyncMove(Target{ID: 1, Position: 100 + 4096}); err != nil || physGoal(p, 1) != 100 {
		t.Fatal("sync move", physGoal(p, 1), err)
	}
}

func TestRangeSingleTurn(t *testing.T) {
	b, p := rangeServo(t, 3900, false)
	b.SetRange(1, Range{Lo: 3413, Hi: 1024})
	s := b.Servo(1)
	if err := s.MoveTo(500, 0, 0); err == nil {
		t.Fatal("single-turn move across the servo's 0 must be refused")
	}
	if err := s.MoveTo(3600, 0, 0); err != nil || physGoal(p, 1) != 3600 {
		t.Fatal("move within the arc", physGoal(p, 1), err)
	}
}

func TestRangeFollowsZero(t *testing.T) {
	// The range is on the encoder scale: with the servo's 0 at encoder 1024
	// the arc 3413..1024 reads 2389..0.
	b, p := rangeServo(t, 2500, true)
	putU16(p.servos[1].mem[RegPositionOffset.Addr:], encodeSignMag(1024, 11))
	b.SetRange(1, Range{Lo: 3413, Hi: 1024})
	if err := b.Servo(1).MoveTo(1500, 0, 0); err != nil { // in the gap 1..2388, nearer its start
		t.Fatal(err)
	}
	if g := physGoal(p, 1); g != 2389 {
		t.Fatal("goal", g)
	}
	b.ClearRange(1)
	if _, ok := b.Range(1); ok {
		t.Fatal("range not cleared")
	}
}
