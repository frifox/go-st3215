package st3215

import "fmt"

// Target is one servo's goal in a synchronized move.
type Target struct {
	ID       uint8
	Position int   // steps (relative in ModeStep)
	Speed    int   // step/s, 0 = maximum
	Acc      uint8 // 100 step/s², 0 = maximum
}

// SyncMove starts all targets at the same instant with one SYNC WRITE packet.
// Positions are logical (see Bus.SetMirrored).
// No acknowledgement is returned by the servos.
func (b *Bus) SyncMove(targets ...Target) error {
	entries := make([]SyncWriteEntry, len(targets))
	for i, t := range targets {
		data, err := moveData(mirrorPos(b.Mirrored(t.ID), t.Position), t.Speed, t.Acc)
		if err != nil {
			return fmt.Errorf("servo %d: %w", t.ID, err)
		}
		entries[i] = SyncWriteEntry{ID: t.ID, Data: data}
	}
	return b.SyncWrite(RegAcceleration.Addr, entries)
}

// SyncTorque enables or disables torque on several servos at once.
func (b *Bus) SyncTorque(on bool, ids ...uint8) error {
	v := byte(0)
	if on {
		v = 1
	}
	entries := make([]SyncWriteEntry, len(ids))
	for i, id := range ids {
		entries[i] = SyncWriteEntry{ID: id, Data: []byte{v}}
	}
	return b.SyncWrite(RegTorqueEnable.Addr, entries)
}

// FeedbackResult is one servo's answer to SyncFeedback.
type FeedbackResult struct {
	Feedback
	Err error // non-nil if the servo did not answer
}

// SyncFeedback reads the live state of several servos with one SYNC READ.
func (b *Bus) SyncFeedback(ids ...uint8) (map[uint8]FeedbackResult, error) {
	res, err := b.SyncRead(feedbackAddr, feedbackLen, ids)
	if err != nil {
		return nil, err
	}
	out := make(map[uint8]FeedbackResult, len(res))
	for id, r := range res {
		if r.Err != nil {
			out[id] = FeedbackResult{Err: r.Err}
			continue
		}
		out[id] = FeedbackResult{Feedback: decodeFeedback(r.Data).mirrored(b.Mirrored(id))}
	}
	return out, nil
}
