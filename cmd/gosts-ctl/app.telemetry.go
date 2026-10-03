package main

import (
	"context"
	"strconv"
	"time"

	"github.com/frifox/gosts"
	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
)

// pollLoop reads the found servos' telemetry every a.poll, checks the groups'
// health and streams both to every window.
func (a *app) pollLoop(ctx context.Context) {
	t := time.NewTicker(a.poll)
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
		}
		st := a.board.Status()
		if len(st.IDs) == 0 || st.Scanning {
			continue
		}
		states := map[string]internal.ServoState{}
		var health map[string]internal.GroupHealth
		err := a.board.WithBus(func(bus *gosts.Bus) error {
			defer func() { health = a.ctl.CheckGroups(bus, states) }()
			if a.noSync {
				for _, id := range st.IDs {
					f, err := bus.Servo(id).Feedback()
					states[strconv.Itoa(int(id))] = internal.ToState(f, err)
				}
				return nil
			}
			res, err := bus.SyncFeedback(st.IDs...)
			for id, r := range res {
				states[strconv.Itoa(int(id))] = internal.ToState(r.Feedback, r.Err)
			}
			return err
		})
		if err != nil {
			continue
		}
		a.Broadcast(internal.FeedbackMsg{Type: "feedback", Time: time.Now().UnixMilli(), Servos: states, Groups: health})
	}
}
