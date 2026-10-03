package main

import (
	"context"
	_ "embed"
	"slices"
	"strconv"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

// Telemetry: polling the servos and streaming feedback to the browsers.

type servoState struct {
	st3215.Feedback
	StatusText string `json:"statusText"`
	Error      string `json:"error,omitempty"`
}

func (s *server) pollLoop(ctx context.Context) {
	t := time.NewTicker(s.poll)
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
		}
		s.mu.Lock()
		ids, scanning := slices.Clone(s.ids), s.scanning
		s.mu.Unlock()
		if len(ids) == 0 || scanning {
			continue
		}
		states := map[string]servoState{}
		var health map[string]groupHealth
		err := s.withBus(func(bus *st3215.Bus) error {
			defer func() { health = s.checkGroups(bus, states) }()
			if s.noSync {
				for _, id := range ids {
					f, err := bus.Servo(id).Feedback()
					states[strconv.Itoa(int(id))] = toState(f, err)
				}
				return nil
			}
			res, err := bus.SyncFeedback(ids...)
			for id, r := range res {
				states[strconv.Itoa(int(id))] = toState(r.Feedback, r.Err)
			}
			return err
		})
		if err != nil {
			continue
		}
		s.broadcast(feedbackMsg{Type: "feedback", Time: time.Now().UnixMilli(), Servos: states, Groups: health})
	}
}

func toState(f st3215.Feedback, err error) servoState {
	st := servoState{Feedback: f, StatusText: f.Status.String()}
	if err != nil {
		st.Error = err.Error()
	}
	return st
}
