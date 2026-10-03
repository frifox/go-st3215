package main

import (
	_ "embed"
	"fmt"
	"slices"
	"strings"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

// factoryReset sends the RESET instruction to one servo, finds it again (the
// ID may become 1) and logs which saved settings the reset changed.
func (s *server) factoryReset(bus *st3215.Bus, id uint8) error {
	sv := bus.Servo(id)
	before, err := sv.ReadMemory()
	if err != nil {
		return err
	}
	// Torque off first: the reset moves the zero under a servo holding a goal.
	if err := sv.EnableTorque(false); err != nil {
		return err
	}
	if err := bus.FactoryReset(id); err != nil {
		return err
	}
	time.Sleep(500 * time.Millisecond) // let it restart with the new settings
	newID := id
	if _, err := bus.Ping(id); err != nil {
		if _, err := bus.Ping(1); err != nil {
			return fmt.Errorf("servo %d didn't answer after the reset (tried ID %d and 1); rescan the bus", id, id)
		}
		newID = 1
	}
	after, err := bus.Servo(newID).ReadMemory()
	if err != nil {
		return err
	}
	var changed []string
	for _, r := range st3215.Registers {
		if r.Area == st3215.EEPROM && !r.ReadOnly && r.Value(before) != r.Value(after) {
			changed = append(changed, fmt.Sprintf("%s %d→%d", r.Name, r.Value(before), r.Value(after)))
		}
	}
	if len(changed) == 0 {
		changed = []string{"no saved settings changed"}
	}
	s.logf("info", "servo %d factory reset: %s", id, strings.Join(changed, ", "))

	s.mu.Lock()
	delete(s.tried, id)
	conflict := newID != id && slices.Contains(s.ids, newID)
	if newID != id {
		s.ids = slices.DeleteFunc(s.ids, func(x uint8) bool { return x == id })
		if !conflict {
			s.ids = append(s.ids, newID)
			slices.Sort(s.ids)
		}
	}
	s.mu.Unlock()
	if newID != id {
		if conflict {
			s.logf("error", "servo %d now answers as ID %d, which another servo already uses: disconnect one of them and give it a new ID", id, newID)
		} else {
			// Keep its name, mirroring and groups with it.
			mirrored := bus.Mirrored(id)
			bus.SetMirrored(id, false)
			bus.SetMirrored(newID, mirrored)
			if r, ok := bus.Range(id); ok {
				bus.ClearRange(id)
				bus.SetRange(newID, r)
			}
			if err := s.cfg.move(id, newID); err != nil {
				s.logf("error", "config: %v", err)
			}
			s.logf("info", "servo %d is now ID %d", id, newID)
		}
		s.broadcastState()
	}
	s.scheduleRefresh(newID, -1)
	return nil
}
