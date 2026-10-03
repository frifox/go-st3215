package servo

import (
	"path/filepath"
	"testing"

	"github.com/frifox/gosts"
	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
	servosim "github.com/frifox/gosts/cmd/gosts-ctl/servo-sim"
)

type nopNotifier struct{}

func (nopNotifier) Logf(string, string, ...any) {}
func (nopNotifier) Broadcast(any)               {}
func (nopNotifier) BroadcastState()             {}
func (nopNotifier) Refresh(uint8)               {}

func TestFightProtection(t *testing.T) {
	cfg, _, err := internal.LoadConfig(filepath.Join(t.TempDir(), "config.toml"))
	if err != nil {
		t.Fatal(err)
	}
	key, err := cfg.SetGroup("", internal.GroupConfig{Name: "Pitch", Members: []uint8{1, 2}, OnFight: "torque-off"})
	if err != nil || key != "pitch" {
		t.Fatal(key, err)
	}
	if _, err := cfg.SetGroup("", internal.GroupConfig{Name: "Other", Members: []uint8{2, 3}}); err == nil {
		t.Fatal("servo 2 must not join a second group")
	}

	bus, _ := gosts.NewBus(servosim.NewPort(1, 2))
	bus.SyncTorque(true, 1, 2)
	c := New(cfg, nopNotifier{})

	fight := map[string]internal.ServoState{
		"1": {Feedback: gosts.Feedback{Position: 1000, Load: 45}},
		"2": {Feedback: gosts.Feedback{Position: 1030, Load: -40}},
	}
	for i := 1; i <= fightPolls; i++ {
		h := c.CheckGroups(bus, fight)["pitch"]
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
	calm := map[string]internal.ServoState{"1": {Feedback: gosts.Feedback{Position: 1000}}, "2": {Feedback: gosts.Feedback{Position: 1001}}}
	if h := c.CheckGroups(bus, calm)["pitch"]; !h.Tripped || h.Fighting || h.Problem != "" {
		t.Fatalf("calm: %+v", h)
	}
	c.ClearTrip("pitch")
	if h := c.CheckGroups(bus, calm)["pitch"]; h.Tripped {
		t.Fatal("trip should be cleared")
	}
}
