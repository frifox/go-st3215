package st3215

import (
	"context"
	"testing"
	"time"
)

func TestGroupMoveMirrored(t *testing.T) {
	p := newFakePort(1, 2)
	b, _ := NewBus(p, WithMirrored(2))
	g := b.Group(1, 2)
	if err := g.EnableTorque(true); err != nil {
		t.Fatal(err)
	}
	if err := g.MoveTo(1000, 500, 20); err != nil {
		t.Fatal(err)
	}
	if len(p.written) != 2 || p.written[1][4] != InstSyncWrite {
		t.Fatal("expected one sync write per command")
	}
	if physGoal(p, 1) != 1000 || physGoal(p, 2) != 3096 {
		t.Fatal("goals", physGoal(p, 1), physGoal(p, 2))
	}
	if p.servos[2].mem[RegTorqueEnable.Addr] != 1 {
		t.Fatal("torque")
	}

	if err := g.SetWheelSpeed(300, 5); err != nil {
		t.Fatal(err)
	}
	if v := RegGoalSpeed.decode(p.servos[2].mem[RegGoalSpeed.Addr:]); v != -300 {
		t.Fatal("wheel speed mirrored", v)
	}
	if err := g.SetTorqueLimit(50); err != nil {
		t.Fatal(err)
	}
	if getU16(p.servos[1].mem[RegTorqueLimit.Addr:]) != 500 {
		t.Fatal("torque limit")
	}
}

func TestGroupFeedbackSpreadFight(t *testing.T) {
	p := newFakePort(1, 2)
	b, _ := NewBus(p, WithMirrored(2))
	putU16(p.servos[1].mem[RegPresentPosition.Addr:], 1000)
	putU16(p.servos[2].mem[RegPresentPosition.Addr:], 3090) // logical 1006
	putU16(p.servos[1].mem[RegPresentLoad.Addr:], encodeSignMag(300, 10))
	putU16(p.servos[2].mem[RegPresentLoad.Addr:], encodeSignMag(250, 10)) // logical -25%
	f, err := b.Group(1, 2).Feedback()
	if err != nil {
		t.Fatal(err)
	}
	if f.Spread() != 6 {
		t.Fatal("spread", f.Spread())
	}
	if !f.Fighting(20) || f.Fighting(28) {
		t.Fatal("fighting")
	}
}

func TestGroupAlignWaitCopy(t *testing.T) {
	p := newFakePort(1, 2)
	b, _ := NewBus(p, WithMirrored(2))
	g := b.Group(1, 2)
	putU16(p.servos[1].mem[RegPresentPosition.Addr:], 1500)
	if err := g.Align(200, 10); err != nil {
		t.Fatal(err)
	}
	if physGoal(p, 2) != 2596 {
		t.Fatal("align", physGoal(p, 2))
	}

	putU16(p.servos[2].mem[RegPresentPosition.Addr:], 2596) // both at logical 1500
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	if _, err := g.WaitForPosition(ctx, 1500, WaitOptions{Poll: time.Millisecond}); err != nil {
		t.Fatal(err)
	}

	p.servos[1].mem[RegPositionP.Addr] = 48
	if err := g.CopyFromLeader(true, RegPositionP); err != nil {
		t.Fatal(err)
	}
	if p.servos[2].mem[RegPositionP.Addr] != 48 {
		t.Fatal("copy")
	}
}

func TestGroupSetPositionAs(t *testing.T) {
	b, p := newTestBus(t, 1, 2)
	b.SetMirrored(2, true)
	for id, pos := range map[uint8]int{1: 1564, 2: 2600} {
		putU16(p.servos[id].mem[RegPresentPosition.Addr:], uint16(pos))
	}
	if err := b.Group(1, 2).SetPositionAs(0); err != nil {
		t.Fatal(err)
	}
	for id, raw := range map[uint8]int{1: 1564, 2: 2600} {
		off := RegPositionOffset.decode(p.servos[id].mem[RegPositionOffset.Addr:])
		if CircularDiff(raw-off, 0) != 0 {
			t.Fatalf("servo %d: offset %d doesn't make here read 0", id, off)
		}
	}
}
