// Package internal holds what the gosts-ctl packages share: the settings
// file (config.toml), the messages exchanged with the browser, the Notifier
// they report through, and small helpers.
package internal

import (
	"bytes"
	"errors"
	"fmt"
	"io/fs"
	"os"
	"path/filepath"
	"regexp"
	"slices"
	"strconv"
	"strings"
	"sync"

	"github.com/BurntSushi/toml"
)

// ServoConfig holds the per-servo settings kept in config.toml. They live in
// gosts-ctl, not on the servo (which has no room for user data).
type ServoConfig struct {
	Name     string  `toml:"Name,omitempty"`
	Mirrored bool    `toml:"Mirrored,omitempty"`
	Signed   bool    `toml:"Signed,omitempty"` // show angles as -180..180 instead of 0..360
	Color    string  `toml:"Color,omitempty"`  // "#rrggbb"; empty = palette color by ID
	Zero     float64 `toml:"Zero,omitzero"`    // virtual 0° (degrees, 0..360): shown angle = physical - Zero
	DialUp   float64 `toml:"DialUp,omitzero"`  // encoder-scale angle at which the arm points physically up (dial orientation)
	Range    []int   `toml:"Range,omitempty"`  // motion range [lo, hi]: clockwise arc in encoder-scale steps (0..4095, logical)
}

func (c ServoConfig) empty() bool {
	return c.Name == "" && !c.Mirrored && !c.Signed && c.Color == "" && c.Zero == 0 && c.DialUp == 0 && len(c.Range) == 0
}

// GroupConfig is a set of servos driven as one (see gosts.Group).
type GroupConfig struct {
	Name    string  `toml:"Name"`
	Members []uint8 `toml:"Members"` // leader first
	// Fight detection: warn (or cut torque) when members disagree.
	MaxSpread    int     `toml:"MaxSpread,omitzero"`    // steps; 0 = default
	MaxFightLoad float64 `toml:"MaxFightLoad,omitzero"` // %; 0 = default
	OnFight      string  `toml:"OnFight,omitempty"`     // "warn" (default) or "torque-off"
}

const (
	DefaultMaxSpread    = 20 // steps (≈1.8°)
	DefaultMaxFightLoad = 30 // % load pushing in opposite directions
)

// SpreadLimit is MaxSpread, or its default.
func (g GroupConfig) SpreadLimit() int {
	if g.MaxSpread > 0 {
		return g.MaxSpread
	}
	return DefaultMaxSpread
}

// FightLoadLimit is MaxFightLoad, or its default.
func (g GroupConfig) FightLoadLimit() float64 {
	if g.MaxFightLoad > 0 {
		return g.MaxFightLoad
	}
	return DefaultMaxFightLoad
}

// Config is config.toml: settings for gosts-ctl itself (ListenAddr), one
// table per servo ID, plus [group.<key>] tables.
//
//	ListenAddr = ":8080"
//
//	[1]
//	Name = "Left"
//
//	[2]
//	Name = "Right"
//	Mirrored = true
//
//	[group.pitch]
//	Name = "Pitch"
//	Members = [1, 2]
type Config struct {
	path string

	mu         sync.Mutex
	listenAddr string
	servos     map[uint8]ServoConfig
	groups     map[string]GroupConfig
}

// DefaultListenAddr is the web console's address when the file doesn't set
// ListenAddr: port 8080 on every network interface.
const DefaultListenAddr = ":8080"

// ListenAddr is the address the web console listens on ("host:port"; an
// empty host means every interface).
func (c *Config) ListenAddr() string {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.listenAddr == "" {
		return DefaultListenAddr
	}
	return c.listenAddr
}

const configHeader = `# gosts-ctl: per-servo settings keyed by servo ID, and servo groups.
# Edited by the web UI; changes made here are read on startup.
`

// LoadConfig reads path. A missing file, or one without ListenAddr, is
// written with the default ListenAddr so the setting is easy to find.
func LoadConfig(path string) (*Config, []string, error) {
	c := &Config{path: path, servos: map[uint8]ServoConfig{}, groups: map[string]GroupConfig{}}
	var raw map[string]toml.Primitive
	md, err := toml.DecodeFile(path, &raw)
	if errors.Is(err, fs.ErrNotExist) {
		c.listenAddr = DefaultListenAddr
		return c, nil, c.saveLocked()
	}
	if err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
	}
	var warnings []string
	warn := func(format string, args ...any) {
		warnings = append(warnings, path+": "+fmt.Sprintf(format, args...))
	}
	for key, prim := range raw {
		if key == "ListenAddr" {
			if err := md.PrimitiveDecode(prim, &c.listenAddr); err != nil {
				return nil, nil, fmt.Errorf("%s: ListenAddr: %w", path, err)
			}
			continue
		}
		if key == "group" {
			var groups map[string]GroupConfig
			if err := md.PrimitiveDecode(prim, &groups); err != nil {
				return nil, nil, fmt.Errorf("%s: [group]: %w", path, err)
			}
			for gk, g := range groups {
				c.groups[gk] = g
			}
			continue
		}
		id, err := strconv.Atoi(key)
		if err != nil || id < 0 || id > 253 {
			warn("[%s] is not a servo ID (0-253), ignored", key)
			continue
		}
		var sc ServoConfig
		if err := md.PrimitiveDecode(prim, &sc); err != nil {
			return nil, nil, fmt.Errorf("%s: [%s]: %w", path, key, err)
		}
		c.servos[uint8(id)] = sc
	}
	for _, k := range md.Undecoded() {
		warn("unknown key %q ignored", k.String())
	}
	// A servo can be in one group only; drop invalid or duplicate members.
	keys := make([]string, 0, len(c.groups))
	for k := range c.groups {
		keys = append(keys, k)
	}
	slices.Sort(keys)
	taken := map[uint8]string{}
	for _, k := range keys {
		g := c.groups[k]
		var members []uint8
		for _, id := range g.Members {
			switch {
			case id > 253:
				warn("group %q: %d is not a servo ID, ignored", k, id)
			case taken[id] != "":
				warn("group %q: servo %d is already in group %q, ignored", k, id, taken[id])
			default:
				taken[id] = k
				members = append(members, id)
			}
		}
		g.Members = members
		if len(members) < 2 {
			warn("group %q has fewer than 2 members, ignored", k)
			delete(c.groups, k)
			continue
		}
		c.groups[k] = g
	}
	if _, ok := raw["ListenAddr"]; !ok {
		c.listenAddr = DefaultListenAddr
		if err := c.saveLocked(); err != nil {
			warn("could not add ListenAddr: %v", err)
		}
	}
	return c, warnings, nil
}

// Get returns servo id's settings (the zero value if there are none).
func (c *Config) Get(id uint8) ServoConfig {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.servos[id]
}

// All returns a copy of every servo entry.
func (c *Config) All() map[uint8]ServoConfig {
	c.mu.Lock()
	defer c.mu.Unlock()
	out := make(map[uint8]ServoConfig, len(c.servos))
	for id, sc := range c.servos {
		out[id] = sc
	}
	return out
}

// AllGroups returns a copy of every group.
func (c *Config) AllGroups() map[string]GroupConfig {
	c.mu.Lock()
	defer c.mu.Unlock()
	out := make(map[string]GroupConfig, len(c.groups))
	for k, g := range c.groups {
		g.Members = slices.Clone(g.Members)
		out[k] = g
	}
	return out
}

// Group returns the group with this key.
func (c *Config) Group(key string) (GroupConfig, bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	g, ok := c.groups[key]
	g.Members = slices.Clone(g.Members)
	return g, ok
}

// GroupOf returns the key of the group containing id, or "".
func (c *Config) GroupOf(id uint8) string {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.groupOfLocked(id)
}

func (c *Config) groupOfLocked(id uint8) string {
	for k, g := range c.groups {
		if slices.Contains(g.Members, id) {
			return k
		}
	}
	return ""
}

// Update changes one servo entry and saves the file.
func (c *Config) Update(id uint8, f func(*ServoConfig)) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	sc := c.servos[id]
	f(&sc)
	if sc.empty() {
		delete(c.servos, id)
	} else {
		c.servos[id] = sc
	}
	return c.saveLocked()
}

var slugRe = regexp.MustCompile(`[^a-z0-9]+`)

// SetGroup creates (key == "") or updates a group and saves the file. It
// returns the group's key.
func (c *Config) SetGroup(key string, g GroupConfig) (string, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if len(g.Members) < 2 {
		return "", errors.New("a group needs at least 2 servos")
	}
	if g.OnFight != "" && g.OnFight != "warn" && g.OnFight != "torque-off" {
		return "", fmt.Errorf("OnFight must be \"warn\" or \"torque-off\"")
	}
	seen := map[uint8]bool{}
	for _, id := range g.Members {
		if seen[id] {
			return "", fmt.Errorf("servo %d is listed twice", id)
		}
		seen[id] = true
		if other := c.groupOfLocked(id); other != "" && other != key {
			return "", fmt.Errorf("servo %d is already in group %q", id, c.groups[other].Name)
		}
	}
	if key == "" {
		base := strings.Trim(slugRe.ReplaceAllString(strings.ToLower(g.Name), "-"), "-")
		if base == "" {
			base = "group"
		}
		key = base
		for n := 2; c.groups[key].Members != nil; n++ {
			key = fmt.Sprintf("%s-%d", base, n)
		}
	} else if _, ok := c.groups[key]; !ok {
		return "", fmt.Errorf("unknown group %q", key)
	}
	c.groups[key] = g
	return key, c.saveLocked()
}

// DeleteGroup removes a group and saves the file.
func (c *Config) DeleteGroup(key string) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if _, ok := c.groups[key]; !ok {
		return fmt.Errorf("unknown group %q", key)
	}
	delete(c.groups, key)
	return c.saveLocked()
}

// Move re-keys a servo after an ID change (entry and group membership) and
// saves the file.
func (c *Config) Move(from, to uint8) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if sc, ok := c.servos[from]; ok {
		delete(c.servos, from)
		c.servos[to] = sc
	}
	for k, g := range c.groups {
		if i := slices.Index(g.Members, from); i >= 0 {
			g.Members[i] = to
			c.groups[k] = g
		}
	}
	return c.saveLocked()
}

// saveLocked writes the file atomically: servo tables in numeric ID order,
// then groups by key.
func (c *Config) saveLocked() error {
	ids := make([]int, 0, len(c.servos))
	for id := range c.servos {
		ids = append(ids, int(id))
	}
	slices.Sort(ids)
	keys := make([]string, 0, len(c.groups))
	for k := range c.groups {
		keys = append(keys, k)
	}
	slices.Sort(keys)

	var buf bytes.Buffer
	buf.WriteString(configHeader)
	fmt.Fprintf(&buf, "\n# Web console address: \"host:port\"; an empty host listens on every interface.\nListenAddr = %s\n", strconv.Quote(c.listenAddr))
	for _, id := range ids {
		fmt.Fprintf(&buf, "\n[%d]\n", id)
		if err := toml.NewEncoder(&buf).Encode(c.servos[uint8(id)]); err != nil {
			return err
		}
	}
	for _, k := range keys {
		fmt.Fprintf(&buf, "\n[group.%s]\n", k)
		if err := toml.NewEncoder(&buf).Encode(c.groups[k]); err != nil {
			return err
		}
	}

	if err := os.MkdirAll(filepath.Dir(c.path), 0o755); err != nil {
		return err
	}
	tmp, err := os.CreateTemp(filepath.Dir(c.path), ".config-*.toml")
	if err != nil {
		return err
	}
	defer os.Remove(tmp.Name())
	if _, err := tmp.Write(buf.Bytes()); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	return os.Rename(tmp.Name(), c.path)
}

// ValidName limits names to something that displays well.
func ValidName(name string) (string, error) {
	name = strings.TrimSpace(name)
	if len([]rune(name)) > 40 {
		return "", errors.New("name is longer than 40 characters")
	}
	if strings.ContainsAny(name, "\n\r\t") {
		return "", errors.New("name must be a single line")
	}
	return name, nil
}

var colorRe = regexp.MustCompile(`^#[0-9a-fA-F]{6}$`)

// ValidColor accepts "#rrggbb" or "" (back to the palette color).
func ValidColor(color string) (string, error) {
	color = strings.TrimSpace(color)
	if color != "" && !colorRe.MatchString(color) {
		return "", errors.New(`color must look like "#1f77b4"`)
	}
	return strings.ToLower(color), nil
}
