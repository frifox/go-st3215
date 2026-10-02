package autotune

import (
	"context"
	"errors"
	"math"
	"testing"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

// plant is a crude servo + load: inertia, friction, and a controller shaped
// by P, D, start force and dead zone. It runs on virtual time (5 ms per read).
type plant struct {
	p          Params
	pos, vel   float64
	goal       float64
	now        time.Time
	members    int
	offset     float64 // second member's position offset (group test)
	failAt     int     // report overload after this many reads (0 = never)
	reads      int
	applied    []Params
	inertiaMul float64
}

func newPlant(p Params) *plant {
	return &plant{p: p, pos: 2048, goal: 2048, now: time.Unix(0, 0), members: 1, inertiaMul: 1}
}

func (f *plant) step(dt float64) {
	err := f.goal - f.pos
	var u float64
	if math.Abs(err) > float64(f.p.DeadZone) {
		u = float64(f.p.P)*err*0.6 - float64(f.p.D)*f.vel*0.3
		if s := float64(f.p.MinStart) * 2; math.Abs(u) < s {
			u = math.Copysign(s, u)
		}
	} else {
		u = -float64(f.p.D) * f.vel * 0.3
	}
	u = math.Max(-3000, math.Min(3000, u))
	const friction = 60
	acc := u / f.inertiaMul
	if math.Abs(f.vel) < 1 && math.Abs(u) < friction {
		f.vel, acc = 0, 0
	} else {
		acc -= math.Copysign(friction, f.vel) / f.inertiaMul
	}
	f.vel += acc * dt
	f.vel = math.Max(-3400, math.Min(3400, f.vel))
	f.pos += f.vel * dt
}

func (f *plant) Params() (Params, error) { return f.p, nil }
func (f *plant) Apply(p Params) error    { f.p = p; f.applied = append(f.applied, p); return nil }
func (f *plant) Save(p Params) error     { return f.Apply(p) }
func (f *plant) Position() (int, error)  { return int(math.Round(f.pos)), nil }
func (f *plant) Limits() (int, int, error) {
	return 0, 4095, nil
}
func (f *plant) MoveTo(pos, _ int, _ uint8) error { f.goal = float64(pos); return nil }
func (f *plant) Stop() error                      { f.goal = f.pos; return nil }
func (f *plant) Now() time.Time                   { return f.now }
func (f *plant) Read() ([]Reading, error) {
	for i := 0; i < 5; i++ {
		f.step(0.001)
	}
	f.now = f.now.Add(5 * time.Millisecond)
	f.reads++
	fb := st3215.Feedback{Position: int(math.Round(f.pos)), Moving: math.Abs(f.vel) > 5, Load: 0, Current: math.Abs(f.vel) / 5}
	if f.failAt > 0 && f.reads > f.failAt {
		fb.Status = st3215.StatusOverload
	}
	out := []Reading{{ID: 1, Feedback: fb}}
	if f.members > 1 {
		fb2 := fb
		fb2.Position = int(math.Round(f.pos + f.offset))
		out = append(out, Reading{ID: 2, Feedback: fb2})
	}
	return out, nil
}

func TestTunesAnUnderdampedServo(t *testing.T) {
	start := Params{P: 48, D: 16, MinStart: 16, DeadZone: 1}
	f := newPlant(start)
	var progress []Progress
	res, err := Run(context.Background(), f, Options{Progress: func(p Progress) { progress = append(progress, p) }})
	if err != nil {
		t.Fatal(err)
	}
	b, a := res.Before.Metrics, res.Best.Metrics
	t.Logf("before %v: %+v", res.Before.Params, b)
	t.Logf("best   %v: %+v", res.Best.Params, a)
	if a.Score >= b.Score || a.Overshoot > b.Overshoot || a.Wobble > b.Wobble {
		t.Fatal("tuning did not improve the response")
	}
	if !a.Settled {
		t.Fatal("best values don't settle")
	}
	if f.p != res.Best.Params {
		t.Fatal("best values not left applied")
	}
	if p, _ := f.Position(); abs(p-2048) > 3 {
		t.Fatal("not back at the start position", p)
	}
	if len(progress) != len(res.Trials) || progress[0].Phase != "baseline" {
		t.Fatal("progress", len(progress), len(res.Trials))
	}
}

func TestKeepsGoodValues(t *testing.T) {
	// Already well damped: the result must never be worse than the start.
	f := newPlant(Params{P: 32, D: 64, MinStart: 16, DeadZone: 1})
	res, err := Run(context.Background(), f, Options{})
	if err != nil {
		t.Fatal(err)
	}
	if res.Best.Metrics.Score > res.Before.Metrics.Score {
		t.Fatal("result worse than the starting values")
	}
}

func TestAbortRestoresValues(t *testing.T) {
	start := Params{P: 32, D: 32, MinStart: 16, DeadZone: 1}
	f := newPlant(start)
	f.failAt = 2000
	_, err := Run(context.Background(), f, Options{})
	if !errors.Is(err, ErrAborted) {
		t.Fatal("want ErrAborted, got", err)
	}
	if f.p != start {
		t.Fatal("original values not restored", f.p)
	}
}

func TestCancelRestoresValues(t *testing.T) {
	start := Params{P: 32, D: 32, MinStart: 16, DeadZone: 1}
	f := newPlant(start)
	ctx, cancel := context.WithCancel(context.Background())
	_, err := Run(ctx, f, Options{Progress: func(p Progress) {
		if p.Trial == 3 {
			cancel()
		}
	}})
	if !errors.Is(err, context.Canceled) || f.p != start {
		t.Fatal(err, f.p)
	}
}

func TestGroupSpreadPenalty(t *testing.T) {
	f := newPlant(Params{P: 32, D: 48, MinStart: 16, DeadZone: 1})
	f.members, f.offset = 2, 6 // the second member sits 6 steps off
	res, err := Run(context.Background(), f, Options{})
	if err != nil {
		t.Fatal(err)
	}
	if res.Before.Metrics.Spread != 6 {
		t.Fatal("spread not measured", res.Before.Metrics.Spread)
	}
}

func TestRefusesWithoutRoom(t *testing.T) {
	f := newPlant(Params{P: 32, D: 32})
	f.pos, f.goal = 10, 10 // at the lower angle limit
	if _, err := Run(context.Background(), f, Options{}); err == nil {
		t.Fatal("expected an error near the limit")
	}
}
