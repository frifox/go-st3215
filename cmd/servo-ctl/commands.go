package main

import (
	"context"
	_ "embed"
	"fmt"
	"log"
	"math"
	"slices"

	st3215 "github.com/frifox/go-st3215"
)

// Commands from the browser.

func (s *server) exec(c *client, req request) error {
	switch req.Type {
	case "ports":
		ports, err := listPorts()
		if err != nil {
			return err
		}
		c.push(portsMsg{Type: "ports", Ports: ports, Sim: fmt.Sprintf("Simulated board with servos %v", s.simIDs)})
		return nil
	case "connect":
		return s.connect(req.Port, req.Baud)
	case "disconnect":
		s.disconnect()
		return nil
	case "scan":
		last := req.Last
		if last == 0 || last > st3215.MaxID {
			last = st3215.MaxID
		}
		if req.First > last {
			return fmt.Errorf("invalid ID range %d..%d", req.First, last)
		}
		return s.scan(context.Background(), req.First, last)
	case "cancelScan", "finishScan":
		cause := error(nil) // cancel: discard results
		if req.Type == "finishScan" {
			cause = errScanFinished
		}
		s.mu.Lock()
		if s.scanCancel != nil {
			s.scanCancel(cause)
		}
		s.mu.Unlock()
		return nil
	case "color":
		color, err := validColor(req.Color)
		if err != nil {
			return err
		}
		if err := s.cfg.update(req.ID, func(c *servoConfig) { c.Color = color }); err != nil {
			return err
		}
		s.broadcastState()
		return nil
	case "groupSave":
		return s.groupSave(req)
	case "autotune":
		return s.startAutotune(req)
	case "autotuneStop":
		s.stopAutotune()
		return nil
	case "autotuneSave", "autotuneRevert":
		return s.finishAutotune(req.Type == "autotuneSave")
	case "groupDelete":
		g, _ := s.cfg.group(req.Group)
		if err := s.cfg.deleteGroup(req.Group); err != nil {
			return err
		}
		s.logf("info", "group %s deleted", g.Name)
		s.broadcastState()
		return nil
	}
	// Don't let other commands move servos while auto-tune is testing them.
	if changesServo[req.Type] {
		ids := []uint8{req.ID}
		if req.Group != "" {
			g, _ := s.cfg.group(req.Group)
			ids = g.Members
		}
		for _, id := range ids {
			if s.tuning(id) {
				return fmt.Errorf("servo %d is being auto-tuned; stop the auto-tune first", id)
			}
		}
	}
	if req.Group != "" {
		return s.withBus(func(bus *st3215.Bus) error { return s.groupCommand(bus, req) })
	}
	return s.withBus(func(bus *st3215.Bus) error { return s.servoCommand(c, bus, req) })
}

func (s *server) servoCommand(c *client, bus *st3215.Bus, req request) error {
	if !slices.Contains(s.servoIDs(), req.ID) {
		return fmt.Errorf("servo %d was not found by the last scan", req.ID)
	}
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
		return sv.SetMode(st3215.Mode(req.Mode))
	case "mirror":
		bus.SetMirrored(req.ID, req.On)
		err := s.cfg.update(req.ID, func(c *servoConfig) { c.Mirrored = req.On })
		s.broadcastState()
		if err != nil {
			return fmt.Errorf("mirrored is active but not saved: %w", err)
		}
		return nil
	case "servoEdit": // the Edit servo dialog: name, color, mirrored and virtual zero at once
		name, err := validName(req.Name)
		if err != nil {
			return err
		}
		color, err := validColor(req.Color)
		if err != nil {
			return err
		}
		if req.Zero < 0 || req.Zero >= 360 || req.DialUp < 0 || req.DialUp >= 360 {
			return fmt.Errorf("angles must be between 0 and 360 degrees")
		}
		bus.SetMirrored(req.ID, req.On)
		if err := s.cfg.update(req.ID, func(c *servoConfig) {
			c.Name, c.Color, c.Mirrored, c.Zero, c.DialUp, c.Signed = name, color, req.On, req.Zero, req.DialUp, req.Signed
		}); err != nil {
			return err
		}
		s.broadcastState()
		return nil
	case "rename":
		name, err := validName(req.Name)
		if err != nil {
			return err
		}
		if err := s.cfg.update(req.ID, func(c *servoConfig) { c.Name = name }); err != nil {
			return err
		}
		s.broadcastState()
		return nil
	case "multiturn":
		return s.setMultiTurn(bus, req.ID, req.On)
	case "limits":
		return s.setLimits(bus, sv, req)
	case "limitsClear":
		return s.clearLimits(bus, sv, req.ID)
	case "torqueLimit":
		return sv.SetTorqueLimit(req.Percent)
	case "factoryReset":
		return s.factoryReset(bus, req.ID)
	case "zeroAt":
		// Absolute: where the servo's 0° goes on the encoder scale. The servo's
		// own zero replaces a virtual one, so that is cleared.
		vz := s.cfg.get(req.ID).Zero
		d := math.Mod(math.Mod(req.Degrees, 360)+360, 360)
		steps := int(math.Round(d/st3215.DegreesPerStep)) % st3215.StepsPerRev
		if err := sv.SetZero(steps); err != nil {
			return err
		}
		if vz != 0 {
			if err := s.cfg.update(req.ID, func(c *servoConfig) { c.Zero = 0 }); err != nil {
				return err
			}
			s.broadcastState()
		}
		s.logf("info", "servo %d: 0° is now at the encoder's %.1f° mark (offset saved on the servo)", req.ID, d)
		return nil
	case "setid":
		if slices.Contains(s.servoIDs(), req.NewID) {
			return fmt.Errorf("ID %d is already used on the bus", req.NewID)
		}
		if err := sv.SetID(req.NewID); err != nil {
			return err
		}
		// The bus moved the mirrored flag to the new ID; keep config.toml in step.
		if err := s.cfg.move(req.ID, req.NewID); err != nil {
			s.logf("error", "config: %v", err)
		}
		s.mu.Lock()
		if t, ok := s.tried[req.ID]; ok {
			delete(s.tried, req.ID)
			s.tried[req.NewID] = t
		}
		s.ids = slices.DeleteFunc(s.ids, func(id uint8) bool { return id == req.ID })
		s.ids = append(s.ids, req.NewID)
		slices.Sort(s.ids)
		s.mu.Unlock()
		s.broadcastState()
		return nil
	case "write":
		reg, ok := st3215.RegisterByName(req.Register)
		if !ok {
			return fmt.Errorf("unknown register %q", req.Register)
		}
		if err := sv.Write(reg, req.Value); err != nil {
			return err
		}
		s.setTried(req.ID, []regValue{{Register: reg.Name, Value: req.Value}}, true, nil) // now saved
		return nil
	case "tune":
		before := map[string]int{} // for Try: the values to go back to
		for _, rv := range req.Values {
			reg, ok := st3215.RegisterByName(rv.Register)
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
		s.setTried(req.ID, req.Values, req.Save, before)
		how := "until power-off"
		if req.Save {
			how = "saved"
		}
		log.Printf("servo %d: tuned %d register(s), %s", req.ID, len(req.Values), how)
		return nil
	case "config":
		msg, err := s.readConfigMsg(bus, req.ID)
		if err != nil {
			return err
		}
		c.push(msg)
		return nil
	}
	return fmt.Errorf("unknown command %q", req.Type)
}
