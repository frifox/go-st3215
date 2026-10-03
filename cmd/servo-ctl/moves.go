package main

import (
	_ "embed"
	"fmt"
	"math"
	"slices"

	st3215 "github.com/frifox/go-st3215"
)

// turnWindow keeps angle moves within one turn either side of the center
// (multi-turn mode), so repeated moves can't wind up a cable.
var turnWindow = [2]int{st3215.CenterPosition - st3215.StepsPerRev, st3215.CenterPosition + st3215.StepsPerRev}

// computedMove handles moves whose goal depends on the servo's state, read
// fresh here: "angle" goes to req.Degrees (as the console shows angles) the
// short way round (multi-turn mode), "jog" moves req.Position steps from the
// present position.
// Both use the servo's absolute position: in multi-turn mode the reported
// position wraps every turn, but goals use the servo's turn count.
func (s *server) computedMove(req request) (int, error) {
	ids := []uint8{req.ID}
	if req.Group != "" {
		g, ok := s.cfg.group(req.Group)
		if !ok {
			return 0, fmt.Errorf("unknown group %q", req.Group)
		}
		ids = g.Members
	}
	found := s.servoIDs()
	for _, id := range ids {
		if !slices.Contains(found, id) {
			return 0, fmt.Errorf("servo %d was not found by the last scan", id)
		}
		if s.tuning(id) {
			return 0, fmt.Errorf("servo %d is being auto-tuned; stop the auto-tune first", id)
		}
	}
	var goal int
	err := s.withBus(func(bus *st3215.Bus) error {
		lead := bus.Servo(ids[0]) // the group's leader decides the goal
		var err error
		if req.Type == "jog" {
			var cur int
			if cur, err = lead.AbsolutePosition(); err != nil {
				return err
			}
			goal = cur + req.Position
		} else {
			// req.Degrees is the angle as this console shows it: the servo's
			// reading minus the virtual 0°. Converted here, so a page with
			// stale settings can't send the arm to the wrong place.
			reading := int(math.Round((req.Degrees + s.cfg.get(ids[0]).Zero) / st3215.DegreesPerStep))
			pos := (reading%st3215.StepsPerRev + st3215.StepsPerRev) % st3215.StepsPerRev
			if goal, err = lead.ShortestGoal(pos, turnWindow[0], turnWindow[1]); err != nil {
				return err
			}
		}
		if req.Group != "" {
			return bus.Group(ids...).MoveTo(goal, req.Speed, req.Acc)
		}
		return lead.MoveTo(goal, req.Speed, req.Acc)
	})
	return goal, err
}
