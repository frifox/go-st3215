package gosts

import (
	"sync"
	"time"
)

// fakeServo is an in-memory ST3215 memory table.
type fakeServo struct {
	mem     [memoryTableSize]byte
	reg     []byte // pending REG WRITE (addr + data)
	status  Status
	noReply bool // simulate a dead servo
}

func newFakeServo(id uint8) *fakeServo {
	s := &fakeServo{}
	s.mem[RegFirmwareMajor.Addr] = 3
	s.mem[RegFirmwareMinor.Addr] = 6
	s.mem[RegServoMajor.Addr] = 9
	s.mem[RegServoMinor.Addr] = 3
	s.mem[RegID.Addr] = id
	s.mem[RegResponseLevel.Addr] = 1
	putU16(s.mem[RegMaxAngleLimit.Addr:], 4095)
	s.mem[RegMaxTemperature.Addr] = 70
	s.mem[RegMaxVoltage.Addr] = 80
	s.mem[RegMinVoltage.Addr] = 40
	putU16(s.mem[RegMaxTorque.Addr:], 1000)
	putU16(s.mem[RegTorqueLimit.Addr:], 1000)
	s.mem[RegPresentVoltage.Addr] = 74
	s.mem[RegPresentTemperature.Addr] = 31
	s.mem[RegLock.Addr] = 1
	return s
}

// fakePort emulates the adapter plus a set of servos.
type fakePort struct {
	mu      sync.Mutex
	servos  map[uint8]*fakeServo
	out     []byte   // bytes waiting to be read by the host
	written [][]byte // every packet the host sent
	timeout time.Duration
	garbage []byte // prepended to the next reply (noise test)
	corrupt int    // corrupt the checksum of the next n replies
}

func newFakePort(ids ...uint8) *fakePort {
	p := &fakePort{servos: map[uint8]*fakeServo{}}
	for _, id := range ids {
		p.servos[id] = newFakeServo(id)
	}
	return p
}

func (p *fakePort) SetReadTimeout(t time.Duration) error { p.timeout = t; return nil }

func (p *fakePort) ResetInputBuffer() error {
	p.mu.Lock()
	p.out = nil
	p.mu.Unlock()
	return nil
}

func (p *fakePort) Read(b []byte) (int, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if len(p.out) == 0 {
		p.mu.Unlock()
		time.Sleep(time.Millisecond) // emulate a (short) read timeout
		p.mu.Lock()
		return 0, nil
	}
	// Deliver in small chunks to exercise reassembly.
	n := copy(b, p.out[:min(len(p.out), 5)])
	p.out = p.out[n:]
	return n, nil
}

func (p *fakePort) Write(b []byte) (int, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	pkt := append([]byte(nil), b...)
	p.written = append(p.written, pkt)
	p.handle(pkt)
	return len(b), nil
}

func (p *fakePort) reply(s *fakeServo, params []byte) {
	if s.noReply {
		return
	}
	pkt, _ := encodePacket(s.mem[RegID.Addr], byte(s.status), params)
	if p.corrupt > 0 {
		p.corrupt--
		pkt[len(pkt)-1] ^= 0xFF
	}
	p.out = append(p.out, p.garbage...)
	p.garbage = nil
	p.out = append(p.out, pkt...)
}

func (p *fakePort) handle(pkt []byte) {
	if len(pkt) < 6 || pkt[0] != 0xFF || pkt[1] != 0xFF || checksum(pkt[2:len(pkt)-1]) != pkt[len(pkt)-1] {
		return
	}
	id, inst, params := pkt[2], pkt[4], pkt[5:len(pkt)-1]
	targets := []*fakeServo{}
	for _, s := range p.servos {
		if id == BroadcastID || s.mem[RegID.Addr] == id {
			targets = append(targets, s)
		}
	}
	ack := id != BroadcastID
	for _, s := range targets {
		switch inst {
		case InstPing:
			p.reply(s, nil)
		case InstRead:
			addr, n := int(params[0]), int(params[1])
			if addr+n <= len(s.mem) {
				p.reply(s, append([]byte(nil), s.mem[addr:addr+n]...))
			}
		case InstWrite:
			s.write(params)
			if ack && s.mem[RegResponseLevel.Addr] == 1 {
				p.reply(s, nil)
			}
		case InstRegWrite:
			s.reg = append([]byte(nil), params...)
			s.mem[RegRegWriteFlag.Addr] = 1
			if ack {
				p.reply(s, nil)
			}
		case InstAction:
			if s.reg != nil {
				s.write(s.reg)
				s.reg = nil
				s.mem[RegRegWriteFlag.Addr] = 0
			}
		case InstReset:
			id := s.mem[RegID.Addr]
			*s = *newFakeServo(id)
			if ack {
				p.reply(s, nil)
			}
		case InstSyncWrite:
			addr, l := params[0], int(params[1])
			for i := 2; i+1+l <= len(params); i += l + 1 {
				if params[i] == s.mem[RegID.Addr] {
					s.write(append([]byte{addr}, params[i+1:i+1+l]...))
				}
			}
		}
	}
	if inst == InstSyncRead {
		addr, n := int(params[0]), int(params[1])
		for _, want := range params[2:] {
			for _, s := range p.servos {
				if s.mem[RegID.Addr] == want {
					p.reply(s, append([]byte(nil), s.mem[addr:addr+n]...))
				}
			}
		}
	}
}

func (s *fakeServo) write(params []byte) {
	addr := int(params[0])
	data := params[1:]
	if addr == int(RegTorqueEnable.Addr) && len(data) == 1 && data[0] == torqueCalibrate {
		// Make the current position read 2048.
		pos := int(getU16(s.mem[RegPresentPosition.Addr:]))
		putU16(s.mem[RegPositionOffset.Addr:], encodeSignMag(pos-CenterPosition, 11))
		putU16(s.mem[RegPresentPosition.Addr:], CenterPosition)
		return
	}
	copy(s.mem[addr:], data)
}
