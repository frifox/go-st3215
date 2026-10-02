package main

import (
	"math"
	"math/rand/v2"
	"sync"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

// simPort implements st3215.Port with simulated servos so the demo can run
// without hardware. It speaks the real wire protocol and models motion
// roughly (constant speed, no inertia).
type simPort struct {
	mu      sync.Mutex
	servos  []*simServo
	out     []byte
	timeout time.Duration
}

type simServo struct {
	mem    [71]byte
	pos    float64 // physical position in steps (multi-turn, unwrapped)
	vel    float64 // step/s
	duty   float64 // -1..1 drive duty, for load/current
	regBuf []byte
	last   time.Time
}

func newSimPort(ids ...uint8) *simPort {
	p := &simPort{timeout: 50 * time.Millisecond}
	for _, id := range ids {
		s := &simServo{pos: 1024 + rand.Float64()*2048, last: time.Now()}
		m := &s.mem
		m[0], m[1], m[3], m[4] = 3, 6, 9, 3
		m[5] = id
		m[8] = 1
		put16(m[:], 11, 4095)
		m[13], m[14], m[15] = 70, 140, 40
		put16(m[:], 16, 1000)
		m[19], m[20] = 44, 47
		m[21], m[22] = 32, 32
		put16(m[:], 24, 16)
		m[26], m[27] = 1, 1
		put16(m[:], 28, 500)
		m[30] = 1
		m[34], m[35], m[36], m[37], m[38], m[39] = 20, 200, 80, 10, 200, 10
		put16(m[:], 48, 1000)
		m[55] = 1
		put16(m[:], 42, uint16(s.pos))
		p.servos = append(p.servos, s)
	}
	return p
}

func put16(m []byte, a int, v uint16) { m[a], m[a+1] = byte(v), byte(v>>8) }
func get16(m []byte, a int) uint16    { return uint16(m[a]) | uint16(m[a+1])<<8 }

func signMag(raw uint16, bit uint) int {
	if raw&(1<<bit) != 0 {
		return -int(raw &^ (1 << bit))
	}
	return int(raw)
}

func toSignMag(v int, bit uint) uint16 {
	if v < 0 {
		return uint16(-v) | 1<<bit
	}
	return uint16(v)
}

func (p *simPort) SetReadTimeout(t time.Duration) error { p.timeout = t; return nil }

func (p *simPort) ResetInputBuffer() error {
	p.mu.Lock()
	p.out = nil
	p.mu.Unlock()
	return nil
}

func (p *simPort) Close() error { return nil }

func (p *simPort) Read(b []byte) (int, error) {
	p.mu.Lock()
	if len(p.out) == 0 {
		p.mu.Unlock()
		time.Sleep(min(p.timeout, 2*time.Millisecond))
		return 0, nil
	}
	n := copy(b, p.out)
	p.out = p.out[n:]
	p.mu.Unlock()
	return n, nil
}

func (p *simPort) Write(b []byte) (int, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.handle(b)
	return len(b), nil
}

func (p *simPort) reply(s *simServo, params []byte) {
	body := append([]byte{s.mem[5], byte(len(params) + 2), s.mem[65]}, params...)
	var sum byte
	for _, v := range body {
		sum += v
	}
	p.out = append(p.out, 0xFF, 0xFF)
	p.out = append(p.out, body...)
	p.out = append(p.out, ^sum)
}

func (p *simPort) handle(pkt []byte) {
	if len(pkt) < 6 || pkt[0] != 0xFF || pkt[1] != 0xFF {
		return
	}
	id, inst, params := pkt[2], pkt[4], pkt[5:len(pkt)-1]
	now := time.Now()
	for _, s := range p.servos {
		s.step(now)
	}
	if inst == st3215.InstSyncRead {
		addr, n := int(params[0]), int(params[1])
		for _, want := range params[2:] {
			for _, s := range p.servos {
				if s.mem[5] == want {
					p.reply(s, append([]byte(nil), s.mem[addr:addr+n]...))
				}
			}
		}
		return
	}
	for _, s := range p.servos {
		if id != st3215.BroadcastID && s.mem[5] != id {
			continue
		}
		ack := id != st3215.BroadcastID
		switch inst {
		case st3215.InstPing:
			p.reply(s, nil)
		case st3215.InstRead:
			addr, n := int(params[0]), int(params[1])
			if addr+n <= len(s.mem) {
				p.reply(s, append([]byte(nil), s.mem[addr:addr+n]...))
			}
		case st3215.InstWrite:
			s.write(params)
			if ack {
				p.reply(s, nil)
			}
		case st3215.InstRegWrite:
			s.regBuf = append([]byte(nil), params...)
			s.mem[64] = 1
			if ack {
				p.reply(s, nil)
			}
		case st3215.InstAction:
			if s.regBuf != nil {
				s.write(s.regBuf)
				s.regBuf, s.mem[64] = nil, 0
			}
		case st3215.InstSyncWrite:
			addr, l := params[0], int(params[1])
			for i := 2; i+1+l <= len(params); i += l + 1 {
				if params[i] == s.mem[5] {
					s.write(append([]byte{addr}, params[i+1:i+1+l]...))
				}
			}
		}
	}
}

func (s *simServo) offset() int { return signMag(get16(s.mem[:], 31), 11) }

// reported converts the physical position to the reported one.
func (s *simServo) reported() int { return int(math.Round(s.pos)) - s.offset() }

func (s *simServo) multiTurn() bool { return get16(s.mem[:], 9) == 0 && get16(s.mem[:], 11) == 0 }

func (s *simServo) write(params []byte) {
	addr, data := int(params[0]), params[1:]
	if addr == 40 && len(data) == 1 && data[0] == 128 { // calibrate middle
		off := int(math.Round(s.pos)) - 2048
		put16(s.mem[:], 31, toSignMag(off, 11))
		put16(s.mem[:], 42, 2048)
		return
	}
	if addr+len(data) > len(s.mem) {
		return
	}
	copy(s.mem[addr:], data)
	// Like the real ST3215, a new goal position switches torque on.
	if addr <= 42 && addr+len(data) >= 44 {
		s.mem[40] = 1
	}
	// Step mode: goal position is relative to the present position.
	if s.mem[33] == 3 && addr <= 42 && addr+len(data) >= 44 {
		rel := signMag(get16(s.mem[:], 42), 15)
		put16(s.mem[:], 42, toSignMag(s.reported()+rel, 15))
	}
}

// step advances the simulation to now and refreshes the feedback registers.
func (s *simServo) step(now time.Time) {
	dt := now.Sub(s.last).Seconds()
	s.last = now
	m := s.mem[:]
	torque := m[40] == 1
	maxSpeed := 3400.0 * float64(get16(m, 48)) / 1000
	s.vel, s.duty = 0, 0

	if torque {
		switch m[33] {
		case 0, 3: // position / step
			goal := float64(signMag(get16(m, 42), 15) + s.offset())
			if !s.multiTurn() {
				lo := float64(int(get16(m, 9)) + s.offset())
				hi := float64(int(get16(m, 11)) + s.offset())
				goal = math.Max(lo, math.Min(hi, goal))
			}
			sp := float64(get16(m, 46))
			if sp == 0 || sp > maxSpeed {
				sp = maxSpeed
			}
			d := goal - s.pos
			move := math.Copysign(math.Min(math.Abs(d), sp*dt), d)
			s.pos += move
			if dt > 0 {
				s.vel = move / dt
			}
			s.duty = math.Copysign(math.Min(1, 0.15+math.Abs(s.vel)/3400*0.5), d)
			if math.Abs(d) < 1 {
				s.duty = 0.02 * math.Copysign(1, d)
			}
		case 1: // wheel
			s.vel = math.Max(-maxSpeed, math.Min(maxSpeed, float64(signMag(get16(m, 46), 15))))
			s.pos += s.vel * dt
			s.duty = s.vel / 3400 * 0.6
		case 2: // pwm
			s.duty = float64(signMag(get16(m, 44), 10)) / 1000
			s.vel = s.duty * 3400
			s.pos += s.vel * dt
		}
	}
	if m[33] != 0 || s.multiTurn() {
		// keep the simulated multi-turn position in the reportable range
		s.pos = math.Max(-30000, math.Min(30000, s.pos))
	}

	pos := s.reported()
	if !s.multiTurn() && m[33] != 3 {
		pos = ((pos % 4096) + 4096) % 4096
	}
	put16(m, 56, toSignMag(pos, 15))
	put16(m, 58, toSignMag(int(s.vel), 15))
	put16(m, 60, toSignMag(int(math.Abs(s.duty)*1000)*sign(s.duty), 10))
	m[62] = byte(74 + rand.IntN(3) - 1)
	cur := math.Abs(s.duty)*300 + 5 + rand.Float64()*3
	put16(m, 69, uint16(cur/6.5))
	temp := 30 + int(math.Abs(s.duty)*15)
	m[63] = byte(temp)
	if math.Abs(s.vel) > 1 {
		m[66] = 1
	} else {
		m[66] = 0
	}
	var status byte
	if v := m[62]; v > m[14] || v < m[15] {
		status |= 1
	}
	m[65] = status
}

func sign(v float64) int {
	if v < 0 {
		return -1
	}
	return 1
}
