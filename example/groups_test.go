package main

import (
	"path/filepath"
	"testing"

	st3215 "github.com/frifox/go-st3215"
)

func TestFightProtection(t *testing.T) {
	cfg, _, err := loadConfig(filepath.Join(t.TempDir(), "config.toml"))
	if err != nil {
		t.Fatal(err)
	}
	key, err := cfg.setGroup("", groupConfig{Name: "Pitch", Members: []uint8{1, 2}, OnFight: "torque-off"})
	if err != nil || key != "pitch" {
		t.Fatal(key, err)
	}
	if _, err := cfg.setGroup("", groupConfig{Name: "Other", Members: []uint8{2, 3}}); err == nil {
		t.Fatal("servo 2 must not join a second group")
	}

	bus, _ := st3215.NewBus(newSimPort(1, 2))
	bus.SyncTorque(true, 1, 2)
	s := &server{cfg: cfg, clients: map[*client]struct{}{}}

	fight := map[string]servoState{
		"1": {Feedback: st3215.Feedback{Position: 1000, Load: 45}},
		"2": {Feedback: st3215.Feedback{Position: 1030, Load: -40}},
	}
	for i := 1; i <= fightPolls; i++ {
		h := s.checkGroups(bus, fight)["pitch"]
		if !h.Fighting || h.Spread != 30 || h.Problem == "" {
			t.Fatalf("frame %d: %+v", i, h)
		}
		if h.Tripped != (i == fightPolls) {
			t.Fatalf("frame %d: tripped=%v", i, h.Tripped)
		}
	}
	if on, _ := bus.Servo(2).TorqueEnabled(); on {
		t.Fatal("torque should have been cut")
	}

	// Stays tripped until torque is switched on again for the group.
	calm := map[string]servoState{"1": {Feedback: st3215.Feedback{Position: 1000}}, "2": {Feedback: st3215.Feedback{Position: 1001}}}
	if h := s.checkGroups(bus, calm)["pitch"]; !h.Tripped || h.Fighting || h.Problem != "" {
		t.Fatalf("calm: %+v", h)
	}
	s.clearTrip("pitch")
	if h := s.checkGroups(bus, calm)["pitch"]; h.Tripped {
		t.Fatal("trip should be cleared")
	}
}

func TestConfigGroupsRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "config.toml")
	cfg, _, _ := loadConfig(path)
	cfg.update(2, func(c *servoConfig) { c.Mirrored, c.Color = true, "#ff0000" })
	if _, err := cfg.setGroup("", groupConfig{Name: "Pitch Axis", Members: []uint8{1, 2}, MaxSpread: 15}); err != nil {
		t.Fatal(err)
	}
	cfg.move(2, 9)
	again, warnings, err := loadConfig(path)
	if err != nil || len(warnings) != 0 {
		t.Fatal(err, warnings)
	}
	g, ok := again.group("pitch-axis")
	if !ok || g.Name != "Pitch Axis" || g.Members[1] != 9 || g.maxSpread() != 15 || g.maxFightLoad() != defaultMaxFightLoad {
		t.Fatalf("%+v", g)
	}
	if sc := again.get(9); !sc.Mirrored || sc.Color != "#ff0000" {
		t.Fatalf("%+v", sc)
	}
}

func TestGroupDefaultName(t *testing.T) {
	cfg, _, _ := loadConfig(filepath.Join(t.TempDir(), "config.toml"))
	s := &server{cfg: cfg, clients: map[*client]struct{}{}}
	if err := s.groupSave(request{Members: []int{1, 2}}); err != nil {
		t.Fatal(err)
	}
	if err := s.groupSave(request{Name: "  ", Members: []int{3, 4}}); err != nil {
		t.Fatal(err)
	}
	g1, ok1 := cfg.group("group")
	g2, ok2 := cfg.group("group-2")
	if !ok1 || !ok2 || g1.Name != "Group" || g2.Name != "Group" {
		t.Fatalf("%+v %+v", g1, g2)
	}
}
