package internal

import "github.com/frifox/gosts"

// Notifier is how the gosts-ctl packages report to the browser windows.
type Notifier interface {
	// Logf logs a message and shows it in every window's log ("info" or "error").
	Logf(level, format string, args ...any)
	// Broadcast sends a message to every window.
	Broadcast(msg any)
	// BroadcastState sends the current StateMsg to every window.
	BroadcastState()
	// Refresh re-reads servo id's settings soon and sends them to every window.
	Refresh(id uint8)
}

// ToInts converts IDs for JSON ([]uint8 would be encoded as base64).
func ToInts(ids []uint8) []int {
	out := make([]int, len(ids))
	for i, id := range ids {
		out[i] = int(id)
	}
	return out
}

// OnOff formats a switch state.
func OnOff(on bool) string {
	if on {
		return "on"
	}
	return "off"
}

// Abs is the absolute value of v.
func Abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

// WrapSteps maps a position into one turn, 0..4095.
func WrapSteps(p int) int { return (p%gosts.StepsPerRev + gosts.StepsPerRev) % gosts.StepsPerRev }
