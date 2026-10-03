package main

import (
	"fmt"
	"slices"

	"github.com/frifox/gosts"
	"github.com/frifox/gosts/cmd/servo-ctl/internal"
)

// Groups: creating, editing and deleting them (their commands and fight
// protection are in the servo package).

// groupInfos describes the groups for the StateMsg, ordered by leader ID.
func groupInfos(groups map[string]internal.GroupConfig) []internal.GroupInfo {
	out := []internal.GroupInfo{}
	for k, g := range groups {
		onFight := g.OnFight
		if onFight == "" {
			onFight = "warn"
		}
		out = append(out, internal.GroupInfo{Key: k, Name: g.Name, Members: internal.ToInts(g.Members),
			MaxSpread: g.SpreadLimit(), MaxFightLoad: g.FightLoadLimit(), OnFight: onFight})
	}
	slices.SortFunc(out, func(a, b internal.GroupInfo) int { return a.Members[0] - b.Members[0] })
	return out
}

// groupSave creates or updates a group from the browser.
func (a *app) groupSave(req internal.Request) error {
	name, err := internal.ValidName(req.Name)
	if err != nil {
		return err
	}
	if name == "" {
		name = "Group" // the name is optional
	}
	members := make([]uint8, 0, len(req.Members))
	for _, id := range req.Members {
		if id < 0 || id > int(gosts.MaxID) {
			return fmt.Errorf("invalid servo ID %d", id)
		}
		members = append(members, uint8(id))
	}
	if req.MaxSpread < 0 || req.MaxFightLoad < 0 || req.MaxFightLoad > 100 {
		return fmt.Errorf("invalid fight protection limits")
	}
	onFight := req.OnFight
	if onFight == "warn" {
		onFight = "" // the default; keeps config.toml short
	}
	key, err := a.cfg.SetGroup(req.Group, internal.GroupConfig{Name: name, Members: members,
		MaxSpread: req.MaxSpread, MaxFightLoad: req.MaxFightLoad, OnFight: onFight})
	if err != nil {
		return err
	}
	a.Logf("info", "group %s saved: servos %v", name, members)
	a.ctl.ClearTrip(key)
	a.BroadcastState()
	return nil
}

// groupDelete removes a group from config.toml; its servos keep their settings.
func (a *app) groupDelete(key string) error {
	g, _ := a.cfg.Group(key)
	if err := a.cfg.DeleteGroup(key); err != nil {
		return err
	}
	a.Logf("info", "group %s deleted", g.Name)
	a.BroadcastState()
	return nil
}
