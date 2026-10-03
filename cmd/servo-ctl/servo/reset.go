package servo

import (
	"fmt"
	"strings"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

// FactoryReset sends the RESET instruction to one servo, finds it again (the
// ID may become 1) and logs which saved settings the reset changed. It
// returns the ID the servo answers to now.
func (c *Controller) FactoryReset(bus *st3215.Bus, id uint8) (uint8, error) {
	sv := bus.Servo(id)
	before, err := sv.ReadMemory()
	if err != nil {
		return 0, err
	}
	// Torque off first: the reset moves the zero under a servo holding a goal.
	if err := sv.EnableTorque(false); err != nil {
		return 0, err
	}
	if err := bus.FactoryReset(id); err != nil {
		return 0, err
	}
	time.Sleep(500 * time.Millisecond) // let it restart with the new settings
	newID := id
	if _, err := bus.Ping(id); err != nil {
		if _, err := bus.Ping(1); err != nil {
			return 0, fmt.Errorf("servo %d didn't answer after the reset (tried ID %d and 1); rescan the bus", id, id)
		}
		newID = 1
	}
	after, err := bus.Servo(newID).ReadMemory()
	if err != nil {
		return 0, err
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
	c.n.Logf("info", "servo %d factory reset: %s", id, strings.Join(changed, ", "))
	c.forgetTried(id)
	return newID, nil
}

// Renumber moves what servo-ctl keeps about a servo (mirroring and motion
// range on the bus, config.toml entry and group membership, tried values)
// from ID from to ID to, after the servo changed its ID by itself (a factory
// reset).
func (c *Controller) Renumber(bus *st3215.Bus, from, to uint8) {
	mirrored := bus.Mirrored(from)
	bus.SetMirrored(from, false)
	bus.SetMirrored(to, mirrored)
	if r, ok := bus.Range(from); ok {
		bus.ClearRange(from)
		bus.SetRange(to, r)
	}
	if err := c.cfg.Move(from, to); err != nil {
		c.n.Logf("error", "config: %v", err)
	}
	c.MoveTried(from, to)
}

// SetID gives the servo a new ID (saved on the servo) and moves what
// servo-ctl keeps about it along.
func (c *Controller) SetID(bus *st3215.Bus, id, newID uint8) error {
	if err := bus.Servo(id).SetID(newID); err != nil { // the bus moves mirroring and range itself
		return err
	}
	if err := c.cfg.Move(id, newID); err != nil {
		c.n.Logf("error", "config: %v", err)
	}
	c.MoveTried(id, newID)
	return nil
}
