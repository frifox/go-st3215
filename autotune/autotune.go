// Package autotune finds position-loop settings for ST3215 servos by
// experiment: it applies candidate values (temporarily), makes short test
// moves with the real load, scores the response and keeps the best.
//
// It tunes P, D, minimum start force and the dead zone (CW = CCW). The
// integral gain I is left unchanged. Scoring favours accurate and calm
// motion: overshoot, wobble after arriving and hunting while holding cost the
// most; speed matters least.
//
//	res, err := autotune.Run(ctx, autotune.ForServo(bus.Servo(1)), autotune.Options{})
//	// res.Best.Params is applied until power-off; persist it with SaveParams.
package autotune

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

// Params are the settings the tuner varies.
type Params struct {
	P        int `json:"p"`        // PositionP
	D        int `json:"d"`        // PositionD
	MinStart int `json:"minStart"` // MinStartForce (0.1%)
	DeadZone int `json:"deadZone"` // CWDeadZone and CCWDeadZone (steps)
}

func (p Params) String() string {
	return fmt.Sprintf("P %d · D %d · start %d · dead zone %d", p.P, p.D, p.MinStart, p.DeadZone)
}

// Metrics summarise one test (the worst value over all moves and members).
type Metrics struct {
	Overshoot  int     `json:"overshoot"`  // steps past the target
	SettleMS   int     `json:"settleMs"`   // time until it stayed within tolerance
	FinalError int     `json:"finalError"` // steps off the target while holding
	Wobble     int     `json:"wobble"`     // direction reversals after reaching the target
	Jitter     int     `json:"jitter"`     // position range while holding (steps)
	PeakMA     float64 `json:"peakMa"`     // largest current (mA)
	// Groups only (e.g. two servos joined by a bar): how much the members
	// disagree while holding. Both are 0 for a single servo.
	Spread   int     `json:"spread"`   // steps between members' positions
	Opposing float64 `json:"opposing"` // % load pushing against each other
	Settled  bool    `json:"settled"`  // every move settled before the timeout
	Score    float64 `json:"score"`    // lower is better
}

// Sample is one telemetry point of a test, relative to the test start.
type Sample struct {
	MS     int         `json:"ms"`
	Target int         `json:"target"`
	Pos    map[int]int `json:"pos"` // member ID -> position
}

// Trial is one tested set of values.
type Trial struct {
	Params  Params   `json:"params"`
	Metrics Metrics  `json:"metrics"`
	Trace   []Sample `json:"trace,omitempty"`
}

// Result of a tuning run. Best is applied (until power-off) when Run returns
// without error.
type Result struct {
	Before Trial   `json:"before"`
	Best   Trial   `json:"best"`
	Trials []Trial `json:"trials"`
}

// Progress is reported after every test.
type Progress struct {
	Phase  string `json:"phase"`
	Trial  int    `json:"trial"`
	Trials int    `json:"trials"` // estimate
	Last   Trial  `json:"last"`
	Best   Trial  `json:"best"`
	Before Trial  `json:"before"`
}

// Options configure a run. Zero values select the defaults.
type Options struct {
	Amplitude int           // test move size in steps either side of the start (default 341 ≈ 30°, max 400 ≈ 35°)
	Speed     int           // step/s (default 1000)
	Acc       uint8         // 100 step/s² (default 30)
	Tolerance int           // steps counted as "on target" (default 2)
	Hold      time.Duration // how long each target is held to measure hunting (default 700ms)
	Timeout   time.Duration // per move (default 4s)
	MaxMA     float64       // abort above this current (default 2500 mA)
	MaxTemp   int           // abort above this temperature (default 65 °C)
	Progress  func(Progress)
}

// MaxAmplitude is the largest test move either side of the start (≈35°).
const MaxAmplitude = 400

// SafeRange is the hard limit either side of the start (≈45°, so 90° of
// travel in total): going beyond it, e.g. a huge overshoot, aborts the run.
const SafeRange = 512

func (o *Options) defaults() {
	if o.Amplitude == 0 {
		o.Amplitude = 341
	}
	o.Amplitude = min(o.Amplitude, MaxAmplitude)
	if o.Speed == 0 {
		o.Speed = 1000
	}
	if o.Acc == 0 {
		o.Acc = 30
	}
	if o.Tolerance == 0 {
		o.Tolerance = 2
	}
	if o.Hold == 0 {
		o.Hold = 700 * time.Millisecond
	}
	if o.Timeout == 0 {
		o.Timeout = 4 * time.Second
	}
	if o.MaxMA == 0 {
		o.MaxMA = 2500
	}
	if o.MaxTemp == 0 {
		o.MaxTemp = 65
	}
}

// ErrAborted wraps safety stops (fault flags, current, temperature, runaway).
var ErrAborted = errors.New("autotune: aborted")

// Candidate values, in the order they are tried.
var (
	candP        = []int{16, 24, 32, 40, 48, 64, 80, 96, 128}
	candD        = []int{8, 16, 24, 32, 48, 64, 80, 96, 128}
	candMinStart = []int{0, 8, 16, 24, 32, 48, 64}
	candDeadZone = []int{0, 1, 2, 3}
)

// Run tunes t. On success the best values stay applied (temporarily) and the
// target is back at its start position. On error or cancellation the
// original values are restored and the target holds where it is.
func Run(ctx context.Context, t Target, opt Options) (Result, error) {
	opt.defaults()
	r := &run{t: t, opt: opt}
	res, err := r.run(ctx)
	if err != nil {
		_ = t.Stop() // hold where it is
		if r.haveBefore {
			_ = t.Apply(r.before)
		}
	}
	return res, err
}

type run struct {
	t           Target
	opt         Options
	start       int
	lo, hi      int // test window
	before      Params
	haveBefore  bool
	trials      []Trial
	best        Trial
	beforeTrial Trial
	total       int
}

func (r *run) run(ctx context.Context) (Result, error) {
	var err error
	if r.before, err = r.t.Params(); err != nil {
		return Result{}, err
	}
	r.haveBefore = true
	if r.start, err = r.t.Position(); err != nil {
		return Result{}, err
	}
	lo, hi, err := r.t.Limits()
	if err != nil {
		return Result{}, err
	}
	amp := r.opt.Amplitude
	if lo != hi { // single-turn: keep the tests inside the angle limits
		amp = min(amp, r.start-lo-r.opt.Tolerance, hi-r.start-r.opt.Tolerance)
	}
	if amp < 40 {
		return Result{}, fmt.Errorf("autotune: not enough room to move around position %d (angle limits %d..%d); move the arm away from the limit first", r.start, lo, hi)
	}
	r.opt.Amplitude = amp
	r.lo, r.hi = r.start-amp, r.start+amp
	r.total = 24 // typical; hill-climbing usually needs 15-30 tests

	// Baseline with the current values.
	if r.beforeTrial, err = r.test(ctx, "baseline", r.before); err != nil {
		return r.result(), err
	}
	r.best = r.beforeTrial

	cur := r.before
	cur.DeadZone = max(cur.DeadZone, 0)
	// Coordinate search in the order that matters: stiffness, damping,
	// start force, dead zone. Each phase keeps the best value found.
	phases := []struct {
		name string
		vals []int
		get  func(Params) int
		set  func(*Params, int)
	}{
		{"stiffness (P)", candP, func(p Params) int { return p.P }, func(p *Params, v int) { p.P = v }},
		{"damping (D)", candD, func(p Params) int { return p.D }, func(p *Params, v int) { p.D = v }},
		{"start force", candMinStart, func(p Params) int { return p.MinStart }, func(p *Params, v int) { p.MinStart = v }},
		{"dead zone", candDeadZone, func(p Params) int { return p.DeadZone }, func(p *Params, v int) { p.DeadZone = v }},
	}
	for _, ph := range phases {
		if cur, err = r.scan(ctx, ph.name, cur, ph.vals, ph.get, ph.set); err != nil {
			return r.result(), err
		}
	}
	// P and D interact: one more pass of each with the other settled.
	for _, ph := range phases[:2] {
		if cur, err = r.scan(ctx, ph.name+" (2nd pass)", cur, ph.vals, ph.get, ph.set); err != nil {
			return r.result(), err
		}
	}
	// Refine P with the final damping.
	for _, dp := range []int{-8, 8} {
		c := cur
		c.P = max(1, cur.P+dp)
		tr, err := r.test(ctx, "refine", c)
		if err != nil {
			return r.result(), err
		}
		if tr.Metrics.Score < r.scoreOf(cur) {
			cur = c
		}
	}
	// Confirm: repeat the winner and keep the average score honest.
	var sum float64
	for i := 0; i < 2; i++ {
		tr, err := r.test(ctx, "confirm", cur)
		if err != nil {
			return r.result(), err
		}
		sum += tr.Metrics.Score
	}
	best := r.bestFor(cur)
	best.Metrics.Score = (best.Metrics.Score + sum) / 3
	// Never return something worse than what we started with.
	if best.Metrics.Score >= r.beforeTrial.Metrics.Score {
		best = r.beforeTrial
	}
	r.best = best
	if err := r.t.Apply(best.Params); err != nil {
		return r.result(), err
	}
	if err := r.t.MoveTo(r.start, r.opt.Speed, r.opt.Acc); err != nil {
		return r.result(), err
	}
	return r.result(), nil
}

// scan hill-climbs one setting: starting from its current value it steps
// through the neighbouring candidates in each direction and stops a direction
// as soon as a result is not better. Extreme values (very low damping, very
// high stiffness) are only reached through gradually improving neighbours,
// which keeps the test moves safe.
func (r *run) scan(ctx context.Context, phase string, cur Params, vals []int, get func(Params) int, set func(*Params, int)) (Params, error) {
	best, bestScore := cur, r.scoreOf(cur)
	if math.IsInf(bestScore, 1) {
		tr, err := r.test(ctx, phase, cur)
		if err != nil {
			return cur, err
		}
		bestScore = tr.Metrics.Score
	}
	// Index of the candidate closest to the current value.
	at, v0 := 0, get(cur)
	for i, v := range vals {
		if abs(v-v0) < abs(vals[at]-v0) {
			at = i
		}
	}
	for _, step := range []int{1, -1} {
		for i := at + step; i >= 0 && i < len(vals); i += step {
			c := best
			set(&c, vals[i])
			if c == best {
				continue
			}
			score := r.scoreOf(c)
			if math.IsInf(score, 1) {
				tr, err := r.test(ctx, phase, c)
				if err != nil {
					return best, err
				}
				score = tr.Metrics.Score
			}
			if score >= bestScore {
				break
			}
			best, bestScore = c, score
		}
	}
	return best, nil
}

func (r *run) scoreOf(p Params) float64 {
	if tr, ok := r.find(p); ok {
		return tr.Metrics.Score
	}
	return math.Inf(1)
}

func (r *run) find(p Params) (Trial, bool) {
	for i := len(r.trials) - 1; i >= 0; i-- {
		if r.trials[i].Params == p {
			return r.trials[i], true
		}
	}
	return Trial{}, false
}

func (r *run) bestFor(p Params) Trial {
	tr, _ := r.find(p)
	return tr
}

func (r *run) result() Result {
	return Result{Before: r.beforeTrial, Best: r.best, Trials: r.trials}
}

// test applies p, runs the move sequence and scores it.
func (r *run) test(ctx context.Context, phase string, p Params) (Trial, error) {
	if err := r.t.Apply(p); err != nil {
		return Trial{}, err
	}
	a := r.opt.Amplitude
	targets := []int{r.start + a, r.start, r.start - a, r.start}
	tr := Trial{Params: p}
	m := &tr.Metrics
	m.Settled = true
	t0 := r.t.Now()
	from := r.start
	for _, target := range targets {
		mv, trace, err := r.move(ctx, from, target, t0)
		if err != nil {
			return tr, err
		}
		tr.Trace = append(tr.Trace, trace...)
		m.Overshoot = max(m.Overshoot, mv.Overshoot)
		m.SettleMS = max(m.SettleMS, mv.SettleMS)
		m.FinalError = max(m.FinalError, mv.FinalError)
		m.Wobble = max(m.Wobble, mv.Wobble)
		m.Jitter = max(m.Jitter, mv.Jitter)
		m.PeakMA = math.Max(m.PeakMA, mv.PeakMA)
		m.Spread = max(m.Spread, mv.Spread)
		m.Opposing = math.Max(m.Opposing, mv.Opposing)
		m.Settled = m.Settled && mv.Settled
		from = target
	}
	m.Score = r.score(*m)
	tr.Trace = downsample(tr.Trace, 400)
	r.trials = append(r.trials, tr)
	if r.best.Metrics.Score == 0 || tr.Metrics.Score < r.best.Metrics.Score {
		r.best = tr
	}
	if r.opt.Progress != nil {
		r.opt.Progress(Progress{Phase: phase, Trial: len(r.trials), Trials: max(r.total, len(r.trials)),
			Last: tr, Best: r.best, Before: r.beforeTrial})
	}
	return tr, nil
}

// score: accurate & calm. Weights are per step / per event.
func (r *run) score(m Metrics) float64 {
	s := 3*float64(m.Overshoot) + 6*float64(m.Wobble) + 4*float64(m.Jitter) +
		4*float64(m.Spread) + m.Opposing/2 +
		10*float64(max(0, m.FinalError-r.opt.Tolerance)) + float64(m.SettleMS)/100 + m.PeakMA/1000
	if !m.Settled {
		s += 500
	}
	return s
}

// move commands one target and measures the response of every member.
func (r *run) move(ctx context.Context, from, target int, t0 time.Time) (Metrics, []Sample, error) {
	var m Metrics
	var trace []Sample
	if err := r.t.MoveTo(target, r.opt.Speed, r.opt.Acc); err != nil {
		return m, nil, err
	}
	dir := 1
	if target < from {
		dir = -1
	}
	tol := r.opt.Tolerance
	start := r.t.Now()
	var settledAt, holdStart time.Time
	type memberState struct {
		arrived bool
		ext     int // furthest position in the current direction
		lastDir int
		holdMin int
		holdMax int
		holdSum int
		holdN   int
	}
	states := map[int]*memberState{}
	readErrs := 0
	for {
		if err := ctx.Err(); err != nil {
			return m, trace, err
		}
		now := r.t.Now()
		rd, err := r.t.Read()
		if err != nil {
			if readErrs++; readErrs >= 5 {
				return m, trace, fmt.Errorf("%w: lost contact: %v", ErrAborted, err)
			}
			continue
		}
		readErrs = 0
		sample := Sample{MS: int(now.Sub(t0) / time.Millisecond), Target: target, Pos: map[int]int{}}
		allIn := true
		for _, f := range rd {
			if f.Err != nil {
				return m, trace, fmt.Errorf("%w: servo %d: %v", ErrAborted, f.ID, f.Err)
			}
			if bad := f.Status & (st3215.StatusOverload | st3215.StatusCurrent | st3215.StatusTemperature | st3215.StatusVoltage); bad != 0 {
				return m, trace, fmt.Errorf("%w: servo %d reports %s", ErrAborted, f.ID, bad)
			}
			if math.Abs(f.Current) > r.opt.MaxMA {
				return m, trace, fmt.Errorf("%w: servo %d current %.0f mA", ErrAborted, f.ID, f.Current)
			}
			if f.Temperature > r.opt.MaxTemp {
				return m, trace, fmt.Errorf("%w: servo %d at %d °C", ErrAborted, f.ID, f.Temperature)
			}
			if abs(f.Position-r.start) > SafeRange {
				return m, trace, fmt.Errorf("%w: servo %d moved more than %d steps from the start (position %d)", ErrAborted, f.ID, SafeRange, f.Position)
			}
			id := int(f.ID)
			sample.Pos[id] = f.Position
			st := states[id]
			if st == nil {
				st = &memberState{}
				states[id] = st
			}
			m.PeakMA = math.Max(m.PeakMA, math.Abs(f.Current))
			err := f.Position - target
			m.Overshoot = max(m.Overshoot, err*dir)
			switch {
			case !st.arrived && err*dir >= -tol:
				st.arrived, st.ext, st.lastDir = true, f.Position, dir
			case st.arrived:
				// Count reversals after arriving; moves back of more than the
				// tolerance only, so sensor flicker isn't counted.
				if (f.Position-st.ext)*st.lastDir > 0 {
					st.ext = f.Position
				} else if abs(f.Position-st.ext) > tol {
					m.Wobble++
					st.lastDir, st.ext = -st.lastDir, f.Position
				}
			}
			if abs(err) > tol || f.Moving {
				allIn = false
			}
			if !holdStart.IsZero() {
				if st.holdN == 0 {
					st.holdMin, st.holdMax = f.Position, f.Position
				}
				st.holdMin, st.holdMax = min(st.holdMin, f.Position), max(st.holdMax, f.Position)
				st.holdSum += f.Position
				st.holdN++
			}
		}
		trace = append(trace, sample)
		if !holdStart.IsZero() && len(rd) > 1 { // members disagreeing while holding
			lo, hi := rd[0].Position, rd[0].Position
			push, pull := 0.0, 0.0
			for _, f := range rd {
				lo, hi = min(lo, f.Position), max(hi, f.Position)
				push, pull = math.Max(push, f.Load), math.Min(pull, f.Load)
			}
			m.Spread = max(m.Spread, hi-lo)
			if push > 0 && pull < 0 {
				m.Opposing = math.Max(m.Opposing, math.Min(push, -pull))
			}
		}

		switch {
		case holdStart.IsZero() && allIn:
			if settledAt.IsZero() {
				settledAt = now
			}
			if now.Sub(settledAt) >= 100*time.Millisecond {
				m.SettleMS = int(settledAt.Sub(start) / time.Millisecond)
				m.Settled = true
				holdStart = now
			}
		case holdStart.IsZero():
			settledAt = time.Time{}
			if now.Sub(start) > r.opt.Timeout {
				// Never settled: measure the hold anyway (it shows the hunting).
				m.SettleMS = int(r.opt.Timeout / time.Millisecond)
				holdStart = now
			}
		case now.Sub(holdStart) >= r.opt.Hold:
			for _, st := range states {
				if st.holdN > 0 {
					m.Jitter = max(m.Jitter, st.holdMax-st.holdMin)
					mean := float64(st.holdSum) / float64(st.holdN)
					m.FinalError = max(m.FinalError, int(math.Round(math.Abs(mean-float64(target)))))
				}
			}
			m.Overshoot = max(0, m.Overshoot)
			return m, trace, nil
		}
	}
}

func downsample(s []Sample, n int) []Sample {
	if len(s) <= n {
		return s
	}
	out := make([]Sample, 0, n)
	step := float64(len(s)) / float64(n)
	for i := 0; i < n; i++ {
		out = append(out, s[int(float64(i)*step)])
	}
	return out
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}
