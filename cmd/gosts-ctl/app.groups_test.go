package main

import (
	"path/filepath"
	"testing"

	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
)

func TestGroupDefaultName(t *testing.T) {
	cfg, _, _ := internal.LoadConfig(filepath.Join(t.TempDir(), "config.toml"))
	s := newApp(cfg, nil, 0, false)
	if err := s.groupSave(internal.Request{Members: []int{1, 2}}); err != nil {
		t.Fatal(err)
	}
	if err := s.groupSave(internal.Request{Name: "  ", Members: []int{3, 4}}); err != nil {
		t.Fatal(err)
	}
	g1, ok1 := cfg.Group("group")
	g2, ok2 := cfg.Group("group-2")
	if !ok1 || !ok2 || g1.Name != "Group" || g2.Name != "Group" {
		t.Fatalf("%+v %+v", g1, g2)
	}
}
