package gosts

import "testing"

func physGoal(p *fakePort, id uint8) int {
	return RegGoalPosition.decode(p.servos[id].mem[RegGoalPosition.Addr:])
}

func TestMirroredMoves(t *testing.T) {
	p := newFakePort(1, 2)
	b, err := NewBus(p, WithMirrored(2))
	if err != nil {
		t.Fatal(err)
	}
	if b.Mirrored(1) || !b.Mirrored(2) {
		t.Fatal("WithMirrored")
	}

	// The same logical command reflects about 2048 on the mirrored servo.
	for _, pos := range []int{2048, 1000, 3500, 0} {
		if err := b.Servo(1).MoveTo(pos, 500, 10); err != nil {
			t.Fatal(err)
		}
		if err := b.Servo(2).MoveTo(pos, 500, 10); err != nil {
			t.Fatal(err)
		}
		if g1, g2 := physGoal(p, 1), physGoal(p, 2); g1 != pos || g2 != 4096-pos {
			t.Fatalf("pos %d: physical goals %d / %d", pos, g1, g2)
		}
	}

	if err := b.SyncMove(Target{ID: 1, Position: 1500}, Target{ID: 2, Position: 1500}); err != nil {
		t.Fatal(err)
	}
	if physGoal(p, 1) != 1500 || physGoal(p, 2) != 2596 {
		t.Fatal("SyncMove not mirrored")
	}

	if err := b.Servo(2).RegMoveTo(1200, 0, 0); err != nil {
		t.Fatal(err)
	}
	b.Action()
	if physGoal(p, 2) != 2896 {
		t.Fatal("RegMoveTo not mirrored", physGoal(p, 2))
	}

	// Signed quantities flip.
	if err := b.Servo(2).SetWheelSpeed(300, 0); err != nil {
		t.Fatal(err)
	}
	if v := RegGoalSpeed.decode(p.servos[2].mem[RegGoalSpeed.Addr:]); v != -300 {
		t.Fatal("wheel speed", v)
	}
	if err := b.Servo(2).SetPWM(-400); err != nil {
		t.Fatal(err)
	}
	if v := RegGoalTime.decode(p.servos[2].mem[RegGoalTime.Addr:]); v != 400 {
		t.Fatal("pwm", v)
	}
	if err := b.Servo(2).StepBy(100, 0, 0); err != nil {
		t.Fatal(err)
	}
	if physGoal(p, 2) != -100 {
		t.Fatal("step", physGoal(p, 2))
	}
}

func TestMirroredFeedback(t *testing.T) {
	p := newFakePort(1, 2)
	b, _ := NewBus(p)
	b.Servo(2).SetMirrored(true)
	for id, pos := range map[uint8]int{1: 1000, 2: 3096} { // same pose, physically reflected
		m := p.servos[id].mem[:]
		putU16(m[RegPresentPosition.Addr:], uint16(pos))
		putU16(m[RegPresentLoad.Addr:], encodeSignMag(100, 10))
		putU16(m[RegPresentSpeed.Addr:], encodeSignMag(-50, 15))
		putU16(m[RegPresentCurrent.Addr:], encodeSignMag(10, 15))
	}
	f1, _ := b.Servo(1).Feedback()
	f2, _ := b.Servo(2).Feedback()
	if f1.Position != 1000 || f2.Position != 1000 || f2.Load != -10 || f2.Speed != 50 || f2.Current != -65 {
		t.Fatalf("feedback %+v / %+v", f1, f2)
	}
	if pos, _ := b.Servo(2).Position(); pos != 1000 {
		t.Fatal("Position", pos)
	}
	res, _ := b.SyncFeedback(1, 2)
	if res[1].Position != res[2].Position {
		t.Fatal("SyncFeedback not mirrored")
	}
	// Raw register access is never mirrored.
	if v, _ := b.Servo(2).Read(RegPresentPosition); v != 3096 {
		t.Fatal("raw read mirrored", v)
	}

	// Stop holds the physical position (of a servo holding torque).
	p.servos[2].mem[RegTorqueEnable.Addr] = 1
	if err := b.Servo(2).Stop(); err != nil {
		t.Fatal(err)
	}
	if physGoal(p, 2) != 3096 {
		t.Fatal("Stop", physGoal(p, 2))
	}
}

func TestMirroredLimitsConfigAndID(t *testing.T) {
	p := newFakePort(3)
	b, _ := NewBus(p, WithMirrored(3))
	s := b.Servo(3)
	if err := s.SetAngleLimits(1000, 3000); err != nil {
		t.Fatal(err)
	}
	m := p.servos[3].mem[:]
	if getU16(m[RegMinAngleLimit.Addr:]) != 1096 || getU16(m[RegMaxAngleLimit.Addr:]) != 3096 {
		t.Fatal("physical limits")
	}
	if lo, hi, _ := s.AngleLimits(); lo != 1000 || hi != 3000 {
		t.Fatal("logical limits", lo, hi)
	}
	if err := s.SetMultiTurn(false); err != nil { // must not fail on the 4096 edge
		t.Fatal(err)
	}
	if getU16(m[RegMaxAngleLimit.Addr:]) != 4095 {
		t.Fatal("single-turn limits")
	}

	s.MoveTo(500, 0, 0)
	c, _ := s.ReadConfig()
	if c.GoalPosition != 500 {
		t.Fatal("config goal", c.GoalPosition)
	}

	if err := s.SetID(4); err != nil {
		t.Fatal(err)
	}
	if b.Mirrored(3) || !b.Mirrored(4) {
		t.Fatal("mirror flag should follow the ID")
	}
}
