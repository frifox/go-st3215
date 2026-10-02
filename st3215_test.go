package st3215

import (
	"bytes"
	"context"
	"errors"
	"testing"
	"time"
)

func hexEq(t *testing.T, got, want []byte) {
	t.Helper()
	if !bytes.Equal(got, want) {
		t.Fatalf("got % X\nwant % X", got, want)
	}
}

// Packets from the manufacturer's protocol manual and memory table workbook.
func TestEncodePacketManualExamples(t *testing.T) {
	cases := []struct {
		name   string
		id     byte
		inst   byte
		params []byte
		want   []byte
	}{
		{"ping", 1, InstPing, nil, []byte{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}},
		{"read pos", 1, InstRead, []byte{0x38, 0x02}, []byte{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}},
		{"set id", 0xFE, InstWrite, []byte{0x05, 0x01}, []byte{0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x05, 0x01, 0xF4}},
		{"change id", 1, InstWrite, []byte{0x05, 0x02}, []byte{0xFF, 0xFF, 0x01, 0x04, 0x03, 0x05, 0x02, 0xF0}},
		{"unlock", 0xFE, InstWrite, []byte{0x37, 0x00}, []byte{0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x37, 0x00, 0xC3}},
		{"action", 0xFE, InstAction, nil, []byte{0xFF, 0xFF, 0xFE, 0x02, 0x05, 0xFA}},
		{"move", 1, InstWrite, []byte{0x29, 0x32, 0xE8, 0x03, 0, 0, 0xE8, 0x03},
			[]byte{0xFF, 0xFF, 0x01, 0x0A, 0x03, 0x29, 0x32, 0xE8, 0x03, 0x00, 0x00, 0xE8, 0x03, 0xC0}},
		{"sync read", 0xFE, InstSyncRead, []byte{0x38, 0x08, 0x01, 0x02}, []byte{0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x38, 0x08, 0x01, 0x02, 0x36}},
	}
	for _, c := range cases {
		got, err := encodePacket(c.id, c.inst, c.params)
		if err != nil {
			t.Fatal(err)
		}
		t.Run(c.name, func(t *testing.T) { hexEq(t, got, c.want) })
	}
}

func TestSyncWriteManualExample(t *testing.T) {
	p := newFakePort()
	b, _ := NewBus(p)
	var entries []SyncWriteEntry
	for id := uint8(1); id <= 4; id++ {
		entries = append(entries, SyncWriteEntry{ID: id, Data: []byte{0x00, 0x08, 0x00, 0x00, 0xE8, 0x03}})
	}
	if err := b.SyncWrite(0x2A, entries); err != nil {
		t.Fatal(err)
	}
	hexEq(t, p.written[0], []byte{0xFF, 0xFF, 0xFE, 0x20, 0x83, 0x2A, 0x06,
		0x01, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03,
		0x02, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03,
		0x03, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03,
		0x04, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 0x58})
}

func TestParseReply(t *testing.T) {
	// Manual example: position 0x0518 = 1304, with noise around it.
	buf := []byte{0x00, 0xFF, 0x12, 0xFF, 0xFF, 0xFF, 0x01, 0x04, 0x00, 0x18, 0x05, 0xDD, 0xAA}
	r, used, ok := parseReply(buf)
	if !ok || r.ID != 1 || getU16(r.Params) != 1304 || used != 12 {
		t.Fatalf("got %+v used=%d ok=%v", r, used, ok)
	}
	// Incomplete packet: nothing parsed, header kept.
	_, used, ok = parseReply([]byte{0x33, 0xFF, 0xFF, 0x01, 0x04, 0x00})
	if ok || used != 1 {
		t.Fatalf("partial: used=%d ok=%v", used, ok)
	}
	// Bad checksum is skipped.
	if _, _, ok = parseReply([]byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0x00}); ok {
		t.Fatal("accepted bad checksum")
	}
}

func TestSignMagnitude(t *testing.T) {
	for _, v := range []int{0, 1, 2047, -1, -2047, 30719, -30719} {
		if got := decodeSignMag(encodeSignMag(v, 15), 15); got != v {
			t.Errorf("%d -> %d", v, got)
		}
	}
	if encodeSignMag(-5, 11) != 0x805 {
		t.Error("bit 11 sign")
	}
	if RegPresentLoad.decode([]byte{0x2C, 0x05}) != -300 { // 0x052C: bit10 + 300
		t.Error("load sign bit 10")
	}
}

func newTestBus(t *testing.T, ids ...uint8) (*Bus, *fakePort) {
	t.Helper()
	p := newFakePort(ids...)
	b, err := NewBus(p, WithTimeout(20*time.Millisecond), WithRetries(1))
	if err != nil {
		t.Fatal(err)
	}
	return b, p
}

func TestPingScanIdentify(t *testing.T) {
	b, _ := newTestBus(t, 1, 7)
	if _, err := b.Ping(1); err != nil {
		t.Fatal(err)
	}
	if _, err := b.Ping(2); !errors.Is(err, ErrTimeout) {
		t.Fatalf("want timeout, got %v", err)
	}
	found, err := b.Scan(context.Background(), 2*time.Millisecond)
	if err != nil || len(found) != 2 || found[0].ID != 1 || found[1].ID != 7 {
		t.Fatalf("scan: %v %v", found, err)
	}

	var probed []uint8
	found, err = b.ScanRange(context.Background(), 5, 10, 2*time.Millisecond, func(id uint8, ok bool) {
		probed = append(probed, id)
	})
	if err != nil || len(found) != 1 || found[0].ID != 7 || len(probed) != 6 {
		t.Fatalf("scan range: %v %v %v", found, probed, err)
	}

	b1, _ := newTestBus(t, 9)
	if id, err := b1.Identify(); err != nil || id != 9 {
		t.Fatalf("identify: %d %v", id, err)
	}
}

func TestFeedback(t *testing.T) {
	b, p := newTestBus(t, 1)
	m := &p.servos[1].mem
	putU16(m[RegPresentPosition.Addr:], 3000)
	putU16(m[RegPresentSpeed.Addr:], encodeSignMag(-120, 15))
	putU16(m[RegPresentLoad.Addr:], encodeSignMag(-250, 10))
	putU16(m[RegPresentCurrent.Addr:], 20)
	m[RegMoving.Addr] = 1
	m[RegStatus.Addr] = byte(StatusOverload | StatusTemperature)

	f, err := b.Servo(1).Feedback()
	if err != nil {
		t.Fatal(err)
	}
	want := Feedback{Position: 3000, Speed: -120, Load: -25, Voltage: 7.4, Temperature: 31,
		Status: StatusOverload | StatusTemperature, Moving: true, Current: 130}
	if f != want {
		t.Fatalf("got %+v\nwant %+v", f, want)
	}
	if f.Status.String() != "temperature|overload" {
		t.Error(f.Status.String())
	}
	if v, _ := b.Servo(1).Voltage(); v != 7.4 {
		t.Error("voltage", v)
	}
}

func TestMoveTo(t *testing.T) {
	b, p := newTestBus(t, 1)
	s := b.Servo(1)
	if err := s.MoveTo(-1000, 1500, 50); err != nil {
		t.Fatal(err)
	}
	hexEq(t, p.written[0], mustPacket(1, InstWrite, 41, 50, 0xE8, 0x83, 0, 0, 0xDC, 0x05))
	if v, _ := s.Read(RegGoalPosition); v != -1000 {
		t.Fatal(v)
	}
	if err := s.MoveTo(40000, 0, 0); err == nil {
		t.Fatal("expected range error")
	}
}

func mustPacket(id, inst byte, params ...byte) []byte {
	p, _ := encodePacket(id, inst, params)
	return p
}

func TestEEPROMWritesAreUnlocked(t *testing.T) {
	b, p := newTestBus(t, 1)
	s := b.Servo(1)
	if err := s.SetMode(ModeWheel); err != nil {
		t.Fatal(err)
	}
	if len(p.written) != 3 {
		t.Fatalf("want unlock/write/lock, got %d packets", len(p.written))
	}
	hexEq(t, p.written[0], mustPacket(1, InstWrite, 55, 0))
	hexEq(t, p.written[1], mustPacket(1, InstWrite, 33, 1))
	hexEq(t, p.written[2], mustPacket(1, InstWrite, 55, 1))
	if m, _ := s.Mode(); m != ModeWheel {
		t.Fatal(m)
	}
	if err := s.Write(RegPresentPosition, 1); err == nil {
		t.Fatal("wrote read-only register")
	}
}

func TestSetID(t *testing.T) {
	b, p := newTestBus(t, 1)
	s := b.Servo(1)
	if err := s.SetID(5); err != nil {
		t.Fatal(err)
	}
	if s.ID() != 5 || p.servos[1].mem[RegID.Addr] != 5 || p.servos[1].mem[RegLock.Addr] != 1 {
		t.Fatal("id not changed / not relocked")
	}
}

func TestConfig(t *testing.T) {
	b, p := newTestBus(t, 3)
	s := b.Servo(3)
	if err := s.SetPositionOffset(-100); err != nil {
		t.Fatal(err)
	}
	if err := s.SetMultiTurn(true); err != nil {
		t.Fatal(err)
	}
	c, err := s.ReadConfig()
	if err != nil {
		t.Fatal(err)
	}
	if c.ID != 3 || c.PositionOffset != -100 || !c.MultiTurn() || c.MaxVoltage != 80 ||
		c.Info.String() != "servo 9.3, firmware 3.6" || c.BaudRate.BitsPerSecond() != 1000000 {
		t.Fatalf("%+v", c)
	}
	if got := p.servos[3].mem[RegPositionOffset.Addr+1]; got != 0x08 { // 100 | bit11
		t.Fatalf("offset high byte %x", got)
	}
}

func TestCalibrateMiddle(t *testing.T) {
	b, p := newTestBus(t, 1)
	putU16(p.servos[1].mem[RegPresentPosition.Addr:], 2148)
	s := b.Servo(1)
	if err := s.CalibrateMiddle(); err != nil {
		t.Fatal(err)
	}
	pos, _ := s.Position()
	off, _ := s.Read(RegPositionOffset)
	if pos != 2048 || off != 100 {
		t.Fatal(pos, off)
	}
}

func TestSyncOps(t *testing.T) {
	b, p := newTestBus(t, 1, 2, 3)
	p.servos[3].noReply = true
	if err := b.SyncMove(Target{ID: 1, Position: 100, Speed: 500}, Target{ID: 2, Position: 4000, Acc: 10}); err != nil {
		t.Fatal(err)
	}
	for id, want := range map[uint8]int{1: 100, 2: 4000} {
		if v, _ := b.Servo(id).Read(RegGoalPosition); v != want {
			t.Errorf("servo %d goal %d", id, v)
		}
	}
	putU16(p.servos[2].mem[RegPresentPosition.Addr:], 1234)
	res, err := b.SyncFeedback(1, 2, 3)
	if err != nil {
		t.Fatal(err)
	}
	if res[1].Err != nil || res[2].Position != 1234 || res[3].Err == nil {
		t.Fatalf("%+v", res)
	}
	if err := b.SyncTorque(true, 1, 2); err != nil {
		t.Fatal(err)
	}
	if on, _ := b.Servo(2).TorqueEnabled(); !on {
		t.Fatal("torque")
	}
}

func TestRegWriteAction(t *testing.T) {
	b, p := newTestBus(t, 1, 2)
	if err := b.Servo(1).RegMoveTo(500, 0, 0); err != nil {
		t.Fatal(err)
	}
	if err := b.Servo(2).RegMoveTo(600, 0, 0); err != nil {
		t.Fatal(err)
	}
	if p.servos[1].mem[RegRegWriteFlag.Addr] != 1 || getU16(p.servos[1].mem[RegGoalPosition.Addr:]) != 0 {
		t.Fatal("reg write applied early")
	}
	if err := b.Action(); err != nil {
		t.Fatal(err)
	}
	if getU16(p.servos[1].mem[RegGoalPosition.Addr:]) != 500 || getU16(p.servos[2].mem[RegGoalPosition.Addr:]) != 600 {
		t.Fatal("action not applied")
	}
}

func TestNoiseAndRetry(t *testing.T) {
	b, p := newTestBus(t, 1)
	p.garbage = []byte{0x00, 0xFF, 0x13, 0xFF}
	if _, err := b.Ping(1); err != nil {
		t.Fatal("garbage:", err)
	}
	p.corrupt = 1 // first reply corrupt, retry succeeds
	if _, err := b.Servo(1).Position(); err != nil {
		t.Fatal("retry:", err)
	}
	p.corrupt = 5 // more than retries
	if _, err := b.Servo(1).Position(); !errors.Is(err, ErrTimeout) {
		t.Fatal("want timeout, got", err)
	}
}

func TestStatusHandler(t *testing.T) {
	var gotID uint8
	var got Status
	p := newFakePort(4)
	b, _ := NewBus(p, WithStatusHandler(func(id uint8, s Status) { gotID, got = id, s }))
	p.servos[4].status = StatusVoltage
	if _, err := b.Ping(4); err != nil {
		t.Fatal(err)
	}
	if gotID != 4 || got != StatusVoltage {
		t.Fatal(gotID, got)
	}
}

func TestWaitForPosition(t *testing.T) {
	b, p := newTestBus(t, 1)
	s := b.Servo(1)
	m := &p.servos[1].mem
	m[RegMoving.Addr] = 1
	go func() {
		time.Sleep(30 * time.Millisecond)
		p.mu.Lock()
		putU16(m[RegPresentPosition.Addr:], 999)
		m[RegMoving.Addr] = 0
		p.mu.Unlock()
	}()
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	f, err := s.WaitForPosition(ctx, 1000, WaitOptions{Poll: 5 * time.Millisecond})
	if err != nil || f.Position != 999 {
		t.Fatal(f, err)
	}

	m[RegStatus.Addr] = byte(StatusOverload)
	if _, err := s.WaitForPosition(ctx, 0, WaitOptions{FailOn: StatusOverload}); !errors.Is(err, ErrFault) {
		t.Fatal("want fault, got", err)
	}
}

func TestDegrees(t *testing.T) {
	if DegreesToSteps(180) != 2048 || DegreesToSteps(-90) != -1024 || StepsToDegrees(1024) != 90 {
		t.Fatal("conversion")
	}
}

func TestRegistersTable(t *testing.T) {
	seen := map[uint8]bool{}
	for _, r := range Registers {
		if seen[r.Addr] {
			t.Errorf("duplicate address %d", r.Addr)
		}
		seen[r.Addr] = true
		if int(r.Addr)+int(r.Size) > memoryTableSize {
			t.Errorf("%s beyond table", r.Name)
		}
	}
	if r, ok := RegisterByName("goalposition"); !ok || r.Addr != 42 {
		t.Fatal("lookup")
	}
}

func TestRegisterValueFromMemory(t *testing.T) {
	b, p := newTestBus(t, 1)
	putU16(p.servos[1].mem[RegPositionOffset.Addr:], encodeSignMag(-42, 11))
	mem, err := b.Servo(1).ReadMemory()
	if err != nil {
		t.Fatal(err)
	}
	if RegPositionOffset.Value(mem) != -42 || RegMaxVoltage.Value(mem) != 80 {
		t.Fatal("Value")
	}
}

func TestWriteTemporary(t *testing.T) {
	b, p := newTestBus(t, 1)
	s := b.Servo(1)
	p.servos[1].mem[RegLock.Addr] = 0 // left unlocked by someone else
	if err := s.WriteTemporary(RegPositionP, 48); err != nil {
		t.Fatal(err)
	}
	hexEq(t, p.written[0], mustPacket(1, InstWrite, 55, 1)) // lock first
	hexEq(t, p.written[1], mustPacket(1, InstWrite, 21, 48))
	if len(p.written) != 2 || p.servos[1].mem[RegPositionP.Addr] != 48 {
		t.Fatal("temporary write")
	}
}

func TestNearestEquivalent(t *testing.T) {
	cases := []struct{ cur, target, want int }{
		{4000, 100, 4196},    // 351.6° -> 8.8°: forward across the seam
		{100, 4000, -96},     // backward across the seam
		{1000, 3000, 3000},   // within a turn: unchanged
		{5000, 100, 4196},    // second turn
		{-3000, 2048, -2048}, // negative multi-turn positions
		{2048, 2048 + 4096, 2048},
	}
	for _, c := range cases {
		if got := NearestEquivalent(c.cur, c.target); got != c.want {
			t.Errorf("NearestEquivalent(%d, %d) = %d, want %d", c.cur, c.target, got, c.want)
		}
	}
}

func TestMoveToShortest(t *testing.T) {
	b, p := newTestBus(t, 1)
	s := b.Servo(1)
	putU16(p.servos[1].mem[RegPresentPosition.Addr:], 4000)
	if goal, err := s.MoveToShortest(100, 0, 0); err != nil || goal != 100 { // single-turn: plain MoveTo
		t.Fatal("single-turn", goal, err)
	}
	if err := s.SetMultiTurn(true); err != nil {
		t.Fatal(err)
	}
	putU16(p.servos[1].mem[RegGoalPosition.Addr:], 4000) // settled at 4000
	p.servos[1].mem[RegTorqueEnable.Addr] = 1
	if goal, err := s.MoveToShortest(100, 0, 0); err != nil || goal != 4196 || physGoal(p, 1) != 4196 {
		t.Fatal("multi-turn", goal, err)
	}
}

// The ST3215 reports its position within one turn even in multi-turn mode,
// while goals use its internal turn count (seen on hardware: a 20° move
// spun a full turn). The turn count comes from the goal register.
func TestMultiTurnReportedPositionWraps(t *testing.T) {
	for _, mirrored := range []bool{false, true} {
		p := newFakePort(1)
		b, _ := NewBus(p)
		b.SetMirrored(1, mirrored)
		s := b.Servo(1)
		if err := s.SetMultiTurn(true); err != nil {
			t.Fatal(err)
		}
		// Internally one turn up at logical 7100; the register shows 3004.
		phys := func(logical int) int { return mirrorPos(mirrored, logical) }
		putU16(p.servos[1].mem[RegGoalPosition.Addr:], encodeSignMag(phys(7100), 15))
		putU16(p.servos[1].mem[RegPresentPosition.Addr:], uint16(((phys(3004)%4096)+4096)%4096))
		p.servos[1].mem[RegTorqueEnable.Addr] = 1 // holding: the goal has the turn count

		if abs, err := s.AbsolutePosition(); err != nil || abs != 7100 {
			t.Fatalf("mirrored=%v: AbsolutePosition %d %v", mirrored, abs, err)
		}
		goal, err := s.MoveToShortest(3982, 0, 0) // 350°: +978 steps, not a turn back
		if err != nil || goal != 8078 {
			t.Fatalf("mirrored=%v: goal %d %v", mirrored, goal, err)
		}
		putU16(p.servos[1].mem[RegGoalPosition.Addr:], encodeSignMag(phys(7100), 15))
		if err := s.Stop(); err != nil {
			t.Fatal(err)
		}
		if g := mirrorPos(mirrored, physGoal(p, 1)); g != 7100 {
			t.Fatalf("mirrored=%v: Stop wrote %d, want 7100 (no extra turn)", mirrored, g)
		}
	}
}

func TestCircularDiff(t *testing.T) {
	for _, c := range []struct{ a, b, want int }{{10, 4090, 16}, {4090, 10, -16}, {100, 50, 50}, {8200, 4100, 4}} {
		if got := CircularDiff(c.a, c.b); got != c.want {
			t.Errorf("CircularDiff(%d, %d) = %d, want %d", c.a, c.b, got, c.want)
		}
	}
	f := GroupFeedback{Members: map[uint8]FeedbackResult{1: {Feedback: Feedback{Position: 4090}}, 2: {Feedback: Feedback{Position: 6}}}}
	if f.Spread() != 12 {
		t.Fatal("spread across the seam", f.Spread())
	}
}

func TestSetPositionAs(t *testing.T) {
	for _, mirrored := range []bool{false, true} {
		b, p := newTestBus(t, 1)
		b.SetMirrored(1, mirrored)
		m := p.servos[1].mem[:]
		putU16(m[RegPositionOffset.Addr:], encodeSignMag(85, 11))
		putU16(m[RegPresentPosition.Addr:], 1564) // physical reported
		putU16(m[RegGoalPosition.Addr:], 1564)    // holding here
		m[RegTorqueEnable.Addr] = 1
		if err := b.Servo(1).SetPositionAs(0); err != nil {
			t.Fatal(err)
		}
		off := RegPositionOffset.decode(m[RegPositionOffset.Addr:])
		raw := 1564 + 85
		// The physical reading for logical 0 is 0 (or 4096 mirrored, the same).
		if CircularDiff(raw-off, 0) != 0 {
			t.Fatalf("mirrored=%v: offset %d doesn't make here read 0", mirrored, off)
		}
		// The goal moved with the coordinates: still pointing at the same raw spot.
		goal := RegGoalPosition.decode(m[RegGoalPosition.Addr:])
		if CircularDiff(goal+off, raw) != 0 {
			t.Fatalf("mirrored=%v: goal %d no longer at the arm (raw %d)", mirrored, goal, raw)
		}
	}
}

func TestSetZeroAt(t *testing.T) {
	for _, mirrored := range []bool{false, true} {
		b, p := newTestBus(t, 1)
		b.SetMirrored(1, mirrored)
		m := p.servos[1].mem[:]
		putU16(m[RegPositionOffset.Addr:], encodeSignMag(85, 11))
		putU16(m[RegGoalPosition.Addr:], 1564)
		m[RegTorqueEnable.Addr] = 1                        // holding: the goal must follow
		if err := b.Servo(1).SetZeroAt(3072); err != nil { // 270° becomes 0°
			t.Fatal(err)
		}
		off := RegPositionOffset.decode(m[RegPositionOffset.Addr:])
		// Physical readings drop by 3072 (rise mirrored), modulo a turn.
		want := 85 + 3072
		if mirrored {
			want = 85 - 3072
		}
		if CircularDiff(off, want) != 0 {
			t.Fatalf("mirrored=%v: offset %d, want ≡ %d", mirrored, off, want)
		}
		goal := RegGoalPosition.decode(m[RegGoalPosition.Addr:])
		if CircularDiff(goal+off, 1564+85) != 0 { // still the same raw spot
			t.Fatalf("mirrored=%v: goal %d moved", mirrored, goal)
		}
	}
}

// With torque off the goal must not be written: on the ST3215 a goal write
// switches torque on and the arm would drive to a stale goal.
func TestSetZeroAtTorqueOffLeavesGoal(t *testing.T) {
	b, p := newTestBus(t, 1)
	m := p.servos[1].mem[:]
	putU16(m[RegPositionOffset.Addr:], encodeSignMag(85, 11))
	putU16(m[RegGoalPosition.Addr:], 0) // stale
	m[RegTorqueEnable.Addr] = 0
	n := len(p.written)
	if err := b.Servo(1).SetZeroAt(3072); err != nil {
		t.Fatal(err)
	}
	for _, pkt := range p.written[n:] {
		if pkt[4] == InstWrite && pkt[5] == RegGoalPosition.Addr {
			t.Fatal("goal written while torque was off")
		}
	}
	if physGoal(p, 1) != 0 {
		t.Fatal("goal changed")
	}
}

func TestResetZero(t *testing.T) {
	b, p := newTestBus(t, 1)
	m := p.servos[1].mem[:]
	putU16(m[RegPositionOffset.Addr:], encodeSignMag(1186, 11))
	putU16(m[RegGoalPosition.Addr:], 100)
	m[RegTorqueEnable.Addr] = 1
	if err := b.Servo(1).ResetZero(); err != nil {
		t.Fatal(err)
	}
	if off := RegPositionOffset.decode(m[RegPositionOffset.Addr:]); off != 0 {
		t.Fatal("offset", off)
	}
	if physGoal(p, 1) != 1286 { // same raw spot: 100 + 1186
		t.Fatal("goal", physGoal(p, 1))
	}
}

func TestSetZeroAbsolute(t *testing.T) {
	for _, mirrored := range []bool{false, true} {
		b, p := newTestBus(t, 1)
		b.SetMirrored(1, mirrored)
		sv := b.Servo(1)
		for i := 0; i < 2; i++ { // applying twice must not shift twice
			if err := sv.SetZero(3072); err != nil {
				t.Fatal(err)
			}
		}
		if z, _ := sv.Zero(); z != 3072 {
			t.Fatalf("mirrored=%v: Zero %d", mirrored, z)
		}
		off := RegPositionOffset.decode(p.servos[1].mem[RegPositionOffset.Addr:])
		want := -1024 // 3072 ≡ -1024
		if mirrored {
			want = 1024
		}
		if off != want {
			t.Fatalf("mirrored=%v: offset %d, want %d", mirrored, off, want)
		}
		if err := sv.SetZero(0); err != nil {
			t.Fatal(err)
		}
		if z, _ := sv.Zero(); z != 0 {
			t.Fatal("back to factory", z)
		}
	}
}

// Torque off: the goal register may be stale and must not decide the turn
// (on hardware a stale 4011 goal made a move to 0° spin a whole turn). If it
// disagrees with the reading, refuse instead of guessing.
func TestAbsolutePositionTorqueOff(t *testing.T) {
	b, p := newTestBus(t, 1)
	s := b.Servo(1)
	if err := s.SetMultiTurn(true); err != nil {
		t.Fatal(err)
	}
	m := p.servos[1].mem[:]
	putU16(m[RegGoalPosition.Addr:], 4011) // left from earlier
	putU16(m[RegPresentPosition.Addr:], 1)
	m[RegTorqueEnable.Addr] = 0
	if _, err := s.AbsolutePosition(); !errors.Is(err, ErrTurnUnknown) {
		t.Fatal("want ErrTurnUnknown, got", err)
	}
	n := len(p.written)
	if _, err := s.MoveToShortest(0, 0, 0); !errors.Is(err, ErrTurnUnknown) {
		t.Fatal("move must be refused, got", err)
	}
	for _, pkt := range p.written[n:] {
		if pkt[4] == InstWrite && pkt[5] == RegAcceleration.Addr {
			t.Fatal("a goal was written")
		}
	}
	// Goal and reading agree (hand-moved a little): fine.
	putU16(m[RegGoalPosition.Addr:], 20)
	if abs, err := s.AbsolutePosition(); err != nil || abs != 1 {
		t.Fatal(abs, err)
	}
	// Stop with torque off writes nothing.
	n = len(p.written)
	if err := s.Stop(); err != nil {
		t.Fatal(err)
	}
	for _, pkt := range p.written[n:] {
		if pkt[4] == InstWrite {
			t.Fatal("Stop wrote with torque off")
		}
	}
}
