package main

import (
	_ "embed"
	"fmt"
	"log"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

// Keeping several browser windows in sync
//
// State, telemetry, scan progress and logs are broadcast to every window.
// Servo settings are per-servo snapshots (configMsg): after any command that
// changes a servo, a fresh snapshot is broadcast to all windows, and the other
// windows get a short note in their log.

// changeNote describes a servo-changing command for the other windows' logs;
// "" means the command doesn't change servo settings.
func changeNote(req request) string {
	switch req.Type {
	case "torque":
		return fmt.Sprintf("torque %s", onOff(req.On))
	case "move":
		return "" // frequent while dragging; visible in telemetry anyway
	case "stop":
		return "stop"
	case "wheel":
		return fmt.Sprintf("wheel speed %d", req.Speed)
	case "pwm":
		return fmt.Sprintf("pwm %d", req.Duty)
	case "mode":
		return fmt.Sprintf("mode %s", st3215.Mode(req.Mode))
	case "multiturn":
		return fmt.Sprintf("multi-turn %s", onOff(req.On))
	case "torqueLimit":
		return fmt.Sprintf("torque limit %.0f%%", req.Percent)
	case "write":
		return fmt.Sprintf("%s ← %d", req.Register, req.Value)
	case "tune":
		how := "until power-off"
		if req.Save {
			how = "saved"
		}
		return fmt.Sprintf("tuning changed (%d value(s), %s)", len(req.Values), how)
	case "mirror":
		return fmt.Sprintf("mirrored %s", onOff(req.On))
	case "servoEdit":
		return "settings edited"
	case "step":
		return fmt.Sprintf("step %d", req.Position)
	case "align":
		return "aligned to leader"
	case "copyTuning":
		return "tuning copied from leader"
	}
	return ""
}

func onOff(on bool) string {
	if on {
		return "on"
	}
	return "off"
}

// changesServo lists the commands after which a fresh config is broadcast.
var changesServo = map[string]bool{
	"torque": true, "move": true, "stop": true, "wheel": true, "pwm": true, "mode": true,
	"multiturn": true, "limits": true, "limitsClear": true, "zeroHere": true, "torqueLimit": true, "write": true, "tune": true,
	"mirror": true, "setid": true, "servoEdit": true, "zeroAt": true, "angle": true, "jog": true, "step": true, "align": true, "copyTuning": true,
}

func (s *server) afterChange(c *client, req request) {
	if !changesServo[req.Type] {
		return
	}
	if req.Group != "" {
		g, _ := s.cfg.group(req.Group)
		if note := changeNote(req); note != "" {
			s.broadcastExcept(c, logMsg{Type: "log", Level: "info", Message: fmt.Sprintf("another window: group %s %s", g.Name, note)})
		}
		for _, id := range g.Members {
			s.scheduleRefresh(id, c.id)
		}
		return
	}
	id := req.ID
	if req.Type == "setid" {
		id = req.NewID
	}
	if note := changeNote(req); note != "" {
		s.broadcastExcept(c, logMsg{Type: "log", Level: "info", Message: fmt.Sprintf("another window: servo %d %s", req.ID, note)})
	}
	s.scheduleRefresh(id, c.id)
}

type pendingRefresh struct {
	timer  *time.Timer
	origin int
}

// refreshDelay coalesces bursts of changes (e.g. dragging a slider) into one
// config read and broadcast per servo.
const refreshDelay = 150 * time.Millisecond

func (s *server) scheduleRefresh(id uint8, origin int) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.refresh == nil {
		s.refresh = map[uint8]*pendingRefresh{}
	}
	if p, ok := s.refresh[id]; ok {
		if p.origin != origin {
			p.origin = -1 // changes from several windows: everyone resyncs
		}
		p.timer.Reset(refreshDelay)
		return
	}
	p := &pendingRefresh{origin: origin}
	p.timer = time.AfterFunc(refreshDelay, func() {
		s.mu.Lock()
		delete(s.refresh, id)
		origin := p.origin
		s.mu.Unlock()
		var msg configMsg
		err := s.withBus(func(bus *st3215.Bus) error {
			var err error
			msg, err = s.readConfigMsg(bus, id)
			return err
		})
		if err != nil {
			log.Printf("refresh servo %d: %v", id, err)
			return
		}
		msg.Origin = origin
		s.broadcast(msg)
	})
	s.refresh[id] = p
}

func (s *server) readConfigMsg(bus *st3215.Bus, id uint8) (configMsg, error) {
	sv := bus.Servo(id)
	cfg, err := sv.ReadConfig()
	if err != nil {
		return configMsg{}, err
	}
	mem, err := sv.ReadMemory()
	if err != nil {
		return configMsg{}, err
	}
	regs := make([]registerInfo, len(st3215.Registers))
	for i, r := range st3215.Registers {
		regs[i] = registerInfo{Name: r.Name, Addr: r.Addr, Size: r.Size, Area: r.Area.String(),
			ReadOnly: r.ReadOnly, Min: r.Min, Max: r.Max, Unit: r.Unit, Value: r.Value(mem)}
	}
	tried, saved := s.triedValues(id, regs)
	return configMsg{Type: "config", ID: id, Config: cfg, Registers: regs, Tried: tried, Saved: saved}, nil
}
