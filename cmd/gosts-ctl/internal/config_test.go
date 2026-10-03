package internal

import (
	"os"
	"path/filepath"
	"strings"
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

func TestListenAddr(t *testing.T) {
	path := filepath.Join(t.TempDir(), "config.toml")
	cfg, _, err := LoadConfig(path) // missing file: created with the default
	if err != nil || cfg.ListenAddr() != DefaultListenAddr {
		t.Fatal(cfg.ListenAddr(), err)
	}
	if b, _ := os.ReadFile(path); !strings.Contains(string(b), `ListenAddr = ":8080"`) {
		t.Fatalf("not written:\n%s", b)
	}

	os.WriteFile(path, []byte("ListenAddr = \"127.0.0.1:9090\"\n\n[1]\nName = \"Left\"\n"), 0o644)
	cfg, warnings, err := LoadConfig(path)
	if err != nil || len(warnings) != 0 || cfg.ListenAddr() != "127.0.0.1:9090" || cfg.Get(1).Name != "Left" {
		t.Fatal(cfg.ListenAddr(), warnings, err)
	}
	cfg.Update(1, func(c *ServoConfig) { c.Name = "Right" }) // a save keeps it
	again, _, _ := LoadConfig(path)
	if again.ListenAddr() != "127.0.0.1:9090" || again.Get(1).Name != "Right" {
		t.Fatal(again.ListenAddr(), again.Get(1))
	}

	os.WriteFile(path, []byte("[1]\nName = \"Left\"\n"), 0o644) // older file: ListenAddr added
	cfg, _, _ = LoadConfig(path)
	if b, _ := os.ReadFile(path); cfg.ListenAddr() != DefaultListenAddr || !strings.Contains(string(b), "ListenAddr") || cfg.Get(1).Name != "Left" {
		t.Fatalf("%s\n%s", cfg.ListenAddr(), b)
	}
}
