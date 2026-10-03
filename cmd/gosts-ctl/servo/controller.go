// Package servo runs gosts-ctl's commands on the servo motors: single servos
// and groups, angle moves, the motion range and zero, tuning values tried
// until power-off, factory reset, and fight protection for groups.
package servo

import (
	"fmt"
	"log"
	"math"
	"sync"

	"github.com/frifox/gosts"
	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
)

// Controller runs servo commands. It keeps what isn't stored on the servos:
// tried tuning values and fight protection state.
type Controller struct {
	cfg *internal.Config
	n   internal.Notifier

	mu     sync.Mutex
	tried  map[uint8]map[string]triedValue // values applied with "Try" but not saved, per servo
	fights map[string]*fightState          // fight protection per group key
}

// New returns a controller using the settings in cfg and reporting to n.
func New(cfg *internal.Config, n internal.Notifier) *Controller {
	return &Controller{cfg: cfg, n: n}
}

// Exec runs a command on one servo (req.ID), which the caller has checked is
// on the bus.
func (c *Controller) Exec(bus *gosts.Bus, req internal.Request) error {
	sv := bus.Servo(req.ID)
	switch req.Type {
	case "torque":
		return sv.EnableTorque(req.On)
	case "move":
		return sv.MoveTo(req.Position, req.Speed, req.Acc)
	case "step":
		return sv.StepBy(req.Position, req.Speed, req.Acc)
	case "stop":
		return sv.Stop()
	case "wheel":
		return sv.SetWheelSpeed(req.Speed, req.Acc)
	case "pwm":
		return sv.SetPWM(req.Duty)
	case "mode":
		return sv.SetMode(gosts.Mode(req.Mode))
	case "mirror":
		bus.SetMirrored(req.ID, req.On)
		err := c.cfg.Update(req.ID, func(sc *internal.ServoConfig) { sc.Mirrored = req.On })
		c.n.BroadcastState()
		if err != nil {
			return fmt.Errorf("mirrored is active but not saved: %w", err)
		}
		return nil
	case "servoEdit": // the Edit servo dialog: name, color, mirrored and virtual zero at once
		name, err := internal.ValidName(req.Name)
		if err != nil {
			return err
		}
		color, err := internal.ValidColor(req.Color)
		if err != nil {
			return err
		}
		if req.Zero < 0 || req.Zero >= 360 || req.DialUp < 0 || req.DialUp >= 360 {
			return fmt.Errorf("angles must be between 0 and 360 degrees")
		}
		bus.SetMirrored(req.ID, req.On)
		if err := c.cfg.Update(req.ID, func(sc *internal.ServoConfig) {
			sc.Name, sc.Color, sc.Mirrored, sc.Zero, sc.DialUp, sc.Signed = name, color, req.On, req.Zero, req.DialUp, req.Signed
		}); err != nil {
			return err
		}
		c.n.BroadcastState()
		return nil
	case "rename":
		name, err := internal.ValidName(req.Name)
		if err != nil {
			return err
		}
		if err := c.cfg.Update(req.ID, func(sc *internal.ServoConfig) { sc.Name = name }); err != nil {
			return err
		}
		c.n.BroadcastState()
		return nil
	case "multiturn":
		return c.SetMultiTurn(bus, req.ID, req.On)
	case "limits":
		return c.SetLimits(bus, sv, req)
	case "limitsClear":
		return c.ClearLimits(bus, sv, req.ID)
	case "torqueLimit":
		return sv.SetTorqueLimit(req.Percent)
	case "zeroAt":
		// Absolute: where the servo's 0° goes on the encoder scale. The servo's
		// own zero replaces a virtual one, so that is cleared.
		vz := c.cfg.Get(req.ID).Zero
		d := math.Mod(math.Mod(req.Degrees, 360)+360, 360)
		steps := int(math.Round(d/gosts.DegreesPerStep)) % gosts.StepsPerRev
		if err := sv.SetZero(steps); err != nil {
			return err
		}
		if vz != 0 {
			if err := c.cfg.Update(req.ID, func(sc *internal.ServoConfig) { sc.Zero = 0 }); err != nil {
				return err
			}
			c.n.BroadcastState()
		}
		c.n.Logf("info", "servo %d: 0° is now at the encoder's %.1f° mark (offset saved on the servo)", req.ID, d)
		return nil
	case "write":
		reg, ok := gosts.RegisterByName(req.Register)
		if !ok {
			return fmt.Errorf("unknown register %q", req.Register)
		}
		if err := sv.Write(reg, req.Value); err != nil {
			return err
		}
		c.SetTried(req.ID, []internal.RegValue{{Register: reg.Name, Value: req.Value}}, true, nil) // now saved
		return nil
	case "tune":
		before := map[string]int{} // for Try: the values to go back to
		for _, rv := range req.Values {
			reg, ok := gosts.RegisterByName(rv.Register)
			if !ok {
				return fmt.Errorf("unknown register %q", rv.Register)
			}
			if !req.Save {
				v, err := sv.Read(reg)
				if err != nil {
					return fmt.Errorf("%s: %w", reg.Name, err)
				}
				before[reg.Name] = v
			}
			write := sv.WriteTemporary
			if req.Save {
				write = sv.Write
			}
			if err := write(reg, rv.Value); err != nil {
				return fmt.Errorf("%s: %w", reg.Name, err)
			}
		}
		c.SetTried(req.ID, req.Values, req.Save, before)
		how := "until power-off"
		if req.Save {
			how = "saved"
		}
		log.Printf("servo %d: tuned %d register(s), %s", req.ID, len(req.Values), how)
		return nil
	}
	return fmt.Errorf("unknown command %q", req.Type)
}
