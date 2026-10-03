package servo

import (
	"math"

	st3215 "github.com/frifox/go-st3215"
	"github.com/frifox/go-st3215/cmd/servo-ctl/internal"
)

// turnWindow keeps angle moves within one turn either side of the center
// (multi-turn mode), so repeated moves can't wind up a cable.
var turnWindow = [2]int{st3215.CenterPosition - st3215.StepsPerRev, st3215.CenterPosition + st3215.StepsPerRev}

// Move runs a move whose goal depends on the servo's state, read fresh here:
// "angle" goes to req.Degrees (as the console shows angles) the short way
// round (multi-turn mode), "jog" moves req.Position steps from the present
// position. ids are the servos to move, leader first (one servo, or a
// group's members); the leader decides the goal. It returns the goal.
//
// Both use the servo's absolute position: in multi-turn mode the reported
// position wraps every turn, but goals use the servo's turn count.
func (c *Controller) Move(bus *st3215.Bus, req internal.Request, ids []uint8) (int, error) {
	lead := bus.Servo(ids[0])
	var goal int
	var err error
	if req.Type == "jog" {
		var cur int
		if cur, err = lead.AbsolutePosition(); err != nil {
			return 0, err
		}
		goal = cur + req.Position
	} else {
		// req.Degrees is the angle as the console shows it: the servo's
		// reading minus the virtual 0°. Converted here, so a page with
		// stale settings can't send the arm to the wrong place.
		reading := int(math.Round((req.Degrees + c.cfg.Get(ids[0]).Zero) / st3215.DegreesPerStep))
		if goal, err = lead.ShortestGoal(internal.WrapSteps(reading), turnWindow[0], turnWindow[1]); err != nil {
			return 0, err
		}
	}
	if len(ids) > 1 {
		return goal, bus.Group(ids...).MoveTo(goal, req.Speed, req.Acc)
	}
	return goal, lead.MoveTo(goal, req.Speed, req.Acc)
}
