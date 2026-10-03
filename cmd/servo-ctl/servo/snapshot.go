package servo

import (
	st3215 "github.com/frifox/go-st3215"
	"github.com/frifox/go-st3215/cmd/servo-ctl/internal"
)

// ConfigMsg reads servo id's settings and full memory table for the
// browser, with the tuning values tried on it.
func (c *Controller) ConfigMsg(bus *st3215.Bus, id uint8) (internal.ConfigMsg, error) {
	sv := bus.Servo(id)
	cfg, err := sv.ReadConfig()
	if err != nil {
		return internal.ConfigMsg{}, err
	}
	mem, err := sv.ReadMemory()
	if err != nil {
		return internal.ConfigMsg{}, err
	}
	regs := make([]internal.RegisterInfo, len(st3215.Registers))
	for i, r := range st3215.Registers {
		regs[i] = internal.RegisterInfo{Name: r.Name, Addr: r.Addr, Size: r.Size, Area: r.Area.String(),
			ReadOnly: r.ReadOnly, Min: r.Min, Max: r.Max, Unit: r.Unit, Value: r.Value(mem)}
	}
	tried, saved := c.triedValues(id, regs)
	return internal.ConfigMsg{Type: "config", ID: id, Config: cfg, Registers: regs, Tried: tried, Saved: saved}, nil
}
