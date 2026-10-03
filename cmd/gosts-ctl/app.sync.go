package main

import (
	"fmt"
	"log"
	"time"

	"github.com/frifox/gosts"
	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
	"github.com/frifox/gosts/cmd/gosts-ctl/web"
)

// Keeping several browser windows in sync
//
// State, telemetry, scan progress and logs are broadcast to every window.
// Servo settings are per-servo snapshots (configMsg): after any command that
// changes a servo, a fresh snapshot is broadcast to all windows, and the other
// windows get a short note in their log.

// changeNote describes a servo-changing command for the other windows' logs;
// "" means the command doesn't change servo settings.
func changeNote(req internal.Request) string {
	switch req.Type {
	case "torque":
		return fmt.Sprintf("torque %s", internal.OnOff(req.On))
	case "move":
		return "" // frequent while dragging; visible in telemetry anyway
	case "stop":
		return "stop"
	case "wheel":
		return fmt.Sprintf("wheel speed %d", req.Speed)
	case "pwm":
		return fmt.Sprintf("pwm %d", req.Duty)
	case "mode":
		return fmt.Sprintf("mode %s", gosts.Mode(req.Mode))
	case "multiturn":
		return fmt.Sprintf("multi-turn %s", internal.OnOff(req.On))
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
		return fmt.Sprintf("mirrored %s", internal.OnOff(req.On))
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

// changesServo lists the commands after which a fresh config is broadcast.
var changesServo = map[string]bool{
	"torque": true, "move": true, "stop": true, "wheel": true, "pwm": true, "mode": true,
	"multiturn": true, "limits": true, "limitsClear": true, "zeroHere": true, "torqueLimit": true, "write": true, "tune": true,
	"mirror": true, "setid": true, "servoEdit": true, "zeroAt": true, "angle": true, "jog": true, "step": true, "align": true, "copyTuning": true,
}

func (a *app) afterChange(c *web.Client, req internal.Request) {
	if !changesServo[req.Type] {
		return
	}
	if req.Group != "" {
		g, _ := a.cfg.Group(req.Group)
		if note := changeNote(req); note != "" {
			a.srv.BroadcastExcept(c, internal.LogMsg{Type: "log", Level: "info", Message: fmt.Sprintf("another window: group %s %s", g.Name, note)})
		}
		for _, id := range g.Members {
			a.scheduleRefresh(id, c.ID())
		}
		return
	}
	id := req.ID
	if req.Type == "setid" {
		id = req.NewID
	}
	if note := changeNote(req); note != "" {
		a.srv.BroadcastExcept(c, internal.LogMsg{Type: "log", Level: "info", Message: fmt.Sprintf("another window: servo %d %s", req.ID, note)})
	}
	a.scheduleRefresh(id, c.ID())
}

type pendingRefresh struct {
	timer  *time.Timer
	origin int
}

// refreshDelay coalesces bursts of changes (e.g. dragging a slider) into one
// config read and broadcast per servo.
const refreshDelay = 150 * time.Millisecond

func (a *app) scheduleRefresh(id uint8, origin int) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.refresh == nil {
		a.refresh = map[uint8]*pendingRefresh{}
	}
	if p, ok := a.refresh[id]; ok {
		if p.origin != origin {
			p.origin = -1 // changes from several windows: everyone resyncs
		}
		p.timer.Reset(refreshDelay)
		return
	}
	p := &pendingRefresh{origin: origin}
	p.timer = time.AfterFunc(refreshDelay, func() {
		a.mu.Lock()
		delete(a.refresh, id)
		origin := p.origin
		a.mu.Unlock()
		var msg internal.ConfigMsg
		err := a.board.WithBus(func(bus *gosts.Bus) error {
			var err error
			msg, err = a.ctl.ConfigMsg(bus, id)
			return err
		})
		if err != nil {
			log.Printf("refresh servo %d: %v", id, err)
			return
		}
		msg.Origin = origin
		a.Broadcast(msg)
	})
	a.refresh[id] = p
}
