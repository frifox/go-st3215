package main

import (
	_ "embed"
)

// Tried-but-not-saved tuning values
//
// "Try" writes EEPROM settings with the lock closed, so the servo uses them
// until it is power-cycled. Reading the servo back can't tell tried values
// from saved ones, so the server remembers them; "Save" can then persist them.

type triedValue struct {
	Value int // active until power-off
	Saved int // stored on the servo (what a power-cycle brings back)
}

// setTried records values written with "Try" (save false) or saved (save
// true). saved holds each register's value before the write; it is only used
// for registers that weren't tried yet. Trying the saved value again ends the
// trial.
func (s *server) setTried(id uint8, values []regValue, save bool, saved map[string]int) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.tried == nil {
		s.tried = map[uint8]map[string]triedValue{}
	}
	t := s.tried[id]
	if t == nil {
		t = map[string]triedValue{}
		s.tried[id] = t
	}
	for _, v := range values {
		if save {
			delete(t, v.Register)
			continue
		}
		tv, ok := t[v.Register]
		if !ok {
			tv.Saved = saved[v.Register]
		}
		tv.Value = v.Value
		if tv.Value == tv.Saved {
			delete(t, v.Register)
		} else {
			t[v.Register] = tv
		}
	}
	if len(t) == 0 {
		delete(s.tried, id)
	}
}

// triedValues returns the tried values still active on the servo. Entries
// whose register no longer holds the tried value (the servo was power-cycled
// and reverted) are forgotten.
// The second map has the saved value of each.
func (s *server) triedValues(id uint8, regs []registerInfo) (map[string]int, map[string]int) {
	s.mu.Lock()
	defer s.mu.Unlock()
	t := s.tried[id]
	out, saved := map[string]int{}, map[string]int{}
	for _, r := range regs {
		if v, ok := t[r.Name]; ok {
			if r.Value == v.Value {
				out[r.Name] = v.Value
				saved[r.Name] = v.Saved
			} else {
				delete(t, r.Name)
			}
		}
	}
	if len(t) == 0 {
		delete(s.tried, id)
	}
	return out, saved
}
