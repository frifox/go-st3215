package internal

import (
	"path/filepath"
	"testing"
)

func TestConfigGroupsRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "config.toml")
	cfg, _, _ := LoadConfig(path)
	cfg.Update(2, func(c *ServoConfig) { c.Mirrored, c.Color = true, "#ff0000" })
	if _, err := cfg.SetGroup("", GroupConfig{Name: "Pitch Axis", Members: []uint8{1, 2}, MaxSpread: 15}); err != nil {
		t.Fatal(err)
	}
	cfg.Move(2, 9)
	again, warnings, err := LoadConfig(path)
	if err != nil || len(warnings) != 0 {
		t.Fatal(err, warnings)
	}
	g, ok := again.Group("pitch-axis")
	if !ok || g.Name != "Pitch Axis" || g.Members[1] != 9 || g.SpreadLimit() != 15 || g.FightLoadLimit() != DefaultMaxFightLoad {
		t.Fatalf("%+v", g)
	}
	if sc := again.Get(9); !sc.Mirrored || sc.Color != "#ff0000" {
		t.Fatalf("%+v", sc)
	}
}
