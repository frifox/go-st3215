package main

import (
	"bytes"
	"errors"
	"fmt"
	"io/fs"
	"os"
	"path/filepath"
	"slices"
	"strconv"
	"strings"
	"sync"

	"github.com/BurntSushi/toml"
)

// servoConfig holds the per-servo settings kept in config.toml. They live in
// the demo, not on the servo (which has no room for user data).
type servoConfig struct {
	Name     string `toml:"Name,omitempty"`
	Mirrored bool   `toml:"Mirrored,omitempty"`
}

func (c servoConfig) empty() bool { return c == servoConfig{} }

// config is config.toml: one table per servo ID.
//
//	[1]
//	Name = "Left"
//
//	[2]
//	Name = "Right"
//	Mirrored = true
type config struct {
	path string

	mu     sync.Mutex
	servos map[uint8]servoConfig
}

const configHeader = `# go-st3215 demo: per-servo settings, keyed by servo ID.
# Edited by the web UI (Name, Mirrored); changes made here are read on startup.
`

// loadConfig reads path; a missing file yields an empty config.
func loadConfig(path string) (*config, []string, error) {
	c := &config{path: path, servos: map[uint8]servoConfig{}}
	var raw map[string]servoConfig
	md, err := toml.DecodeFile(path, &raw)
	if errors.Is(err, fs.ErrNotExist) {
		return c, nil, nil
	}
	if err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
	}
	var warnings []string
	for _, k := range md.Undecoded() {
		warnings = append(warnings, fmt.Sprintf("%s: unknown key %q ignored", path, k.String()))
	}
	for key, sc := range raw {
		id, err := strconv.Atoi(key)
		if err != nil || id < 0 || id > 253 {
			warnings = append(warnings, fmt.Sprintf("%s: [%s] is not a servo ID (0-253), ignored", path, key))
			continue
		}
		c.servos[uint8(id)] = sc
	}
	return c, warnings, nil
}

func (c *config) get(id uint8) servoConfig {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.servos[id]
}

// all returns a copy of every entry.
func (c *config) all() map[uint8]servoConfig {
	c.mu.Lock()
	defer c.mu.Unlock()
	out := make(map[uint8]servoConfig, len(c.servos))
	for id, sc := range c.servos {
		out[id] = sc
	}
	return out
}

// update changes one entry and saves the file.
func (c *config) update(id uint8, f func(*servoConfig)) error {
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

// move re-keys an entry after a servo ID change and saves the file.
func (c *config) move(from, to uint8) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	sc, ok := c.servos[from]
	if !ok {
		return nil
	}
	delete(c.servos, from)
	c.servos[to] = sc
	return c.saveLocked()
}

// saveLocked writes the file atomically, tables in numeric ID order.
func (c *config) saveLocked() error {
	ids := make([]int, 0, len(c.servos))
	for id := range c.servos {
		ids = append(ids, int(id))
	}
	slices.Sort(ids)

	var buf bytes.Buffer
	buf.WriteString(configHeader)
	for _, id := range ids {
		fmt.Fprintf(&buf, "\n[%d]\n", id)
		if err := toml.NewEncoder(&buf).Encode(c.servos[uint8(id)]); err != nil {
			return err
		}
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

// validName limits names to something that displays well.
func validName(name string) (string, error) {
	name = strings.TrimSpace(name)
	if len([]rune(name)) > 40 {
		return "", errors.New("name is longer than 40 characters")
	}
	if strings.ContainsAny(name, "\n\r\t") {
		return "", errors.New("name must be a single line")
	}
	return name, nil
}
