package gosts

import (
	"errors"
	"fmt"
)

// Instruction codes of the Feetech/Waveshare smart bus servo protocol.
const (
	InstPing      byte = 0x01
	InstRead      byte = 0x02
	InstWrite     byte = 0x03
	InstRegWrite  byte = 0x04
	InstAction    byte = 0x05
	InstReset     byte = 0x06
	InstSyncRead  byte = 0x82
	InstSyncWrite byte = 0x83
)

// BroadcastID addresses every servo on the bus. Servos do not reply to
// broadcast packets (except PING, which must only be used with one servo).
const BroadcastID uint8 = 0xFE

// MaxID is the highest assignable servo ID.
const MaxID uint8 = 0xFD

// headerByte is repeated twice at the start of every packet.
const headerByte = 0xFF

// maxPacketParams is the maximum number of parameter bytes in one packet:
// the length byte is LEN = params + 2 and must fit in a byte.
const maxPacketParams = 0xFF - 2

var (
	// ErrTimeout is returned when no (complete) reply arrives in time.
	ErrTimeout = errors.New("gosts: timeout waiting for reply")
	// ErrPacketTooLong is returned when a packet would exceed 255 bytes of payload.
	ErrPacketTooLong = errors.New("gosts: packet too long")
	// ErrBadReply is returned when a reply is well-formed but unexpected.
	ErrBadReply = errors.New("gosts: unexpected reply")
)

// checksum computes ~(sum of bytes) & 0xFF over ID, LEN, INST/ERR and params.
func checksum(b []byte) byte {
	var s byte
	for _, v := range b {
		s += v
	}
	return ^s
}

// encodePacket builds an instruction packet:
//
//	FF FF ID LEN INST P1..PN CHK   with LEN = N + 2
func encodePacket(id, inst byte, params []byte) ([]byte, error) {
	if len(params) > maxPacketParams {
		return nil, ErrPacketTooLong
	}
	pkt := make([]byte, 0, len(params)+6)
	pkt = append(pkt, headerByte, headerByte, id, byte(len(params)+2), inst)
	pkt = append(pkt, params...)
	pkt = append(pkt, checksum(pkt[2:]))
	return pkt, nil
}

// reply is a decoded status packet returned by a servo.
type reply struct {
	ID     uint8
	Status Status
	Params []byte
}

// parseReply scans buf for the first valid status packet.
//
// It returns the packet, the number of bytes consumed from buf (everything up
// to and including the packet), and ok=true. When no complete packet is
// present it returns ok=false and the number of leading bytes that can be
// safely discarded (garbage that can never start a valid packet).
func parseReply(buf []byte) (r reply, consumed int, ok bool) {
	i := 0
	for {
		// Find header FF FF followed by a non-FF ID byte.
		for i+1 < len(buf) && !(buf[i] == headerByte && buf[i+1] == headerByte) {
			i++
		}
		if i+3 >= len(buf) {
			// Keep a possible partial header at the tail.
			if i < len(buf) && buf[i] != headerByte {
				i = len(buf)
			}
			return reply{}, i, false
		}
		if buf[i+2] == headerByte {
			// FF FF FF ... — slide by one and retry.
			i++
			continue
		}
		n := int(buf[i+3]) // LEN = params + 2 (ERR + CHK)
		if n < 2 {
			i++
			continue
		}
		end := i + 4 + n // index after checksum
		if end > len(buf) {
			return reply{}, i, false
		}
		body := buf[i+2 : end-1] // ID LEN ERR params
		if checksum(body) != buf[end-1] {
			i++
			continue
		}
		params := make([]byte, n-2)
		copy(params, buf[i+5:end-1])
		return reply{ID: buf[i+2], Status: Status(buf[i+4]), Params: params}, end, true
	}
}

// Status is the ERROR byte carried in every reply packet and also exposed in
// the "servo status" register (address 65). A set bit indicates the condition
// is currently active.
type Status uint8

// Status bits (see memory table, address 65 / 19 / 20).
const (
	StatusVoltage     Status = 1 << 0 // input voltage outside min/max limit
	StatusSensor      Status = 1 << 1 // magnetic angle sensor fault
	StatusTemperature Status = 1 << 2 // over temperature
	StatusCurrent     Status = 1 << 3 // over current
	StatusAngle       Status = 1 << 4 // angle out of limits
	StatusOverload    Status = 1 << 5 // overload protection tripped
)

var statusNames = []struct {
	bit  Status
	name string
}{
	{StatusVoltage, "voltage"},
	{StatusSensor, "sensor"},
	{StatusTemperature, "temperature"},
	{StatusCurrent, "current"},
	{StatusAngle, "angle"},
	{StatusOverload, "overload"},
}

// OK reports whether no error condition is flagged.
func (s Status) OK() bool { return s == 0 }

// Has reports whether all bits of f are set.
func (s Status) Has(f Status) bool { return s&f == f }

func (s Status) String() string {
	if s == 0 {
		return "ok"
	}
	out := ""
	rest := s
	for _, n := range statusNames {
		if s&n.bit != 0 {
			if out != "" {
				out += "|"
			}
			out += n.name
			rest &^= n.bit
		}
	}
	if rest != 0 {
		if out != "" {
			out += "|"
		}
		out += fmt.Sprintf("0x%02x", uint8(rest))
	}
	return out
}

// encodeSignMag encodes v in the servo's sign-magnitude format where bit
// signBit carries the sign (1 = negative).
func encodeSignMag(v int, signBit uint) uint16 {
	if v < 0 {
		return uint16(-v) | 1<<signBit
	}
	return uint16(v)
}

// decodeSignMag is the inverse of encodeSignMag.
func decodeSignMag(raw uint16, signBit uint) int {
	if raw&(1<<signBit) != 0 {
		return -int(raw &^ (1 << signBit))
	}
	return int(raw)
}

// The ST series stores 16-bit values little-endian (low byte first).
func putU16(b []byte, v uint16) { b[0] = byte(v); b[1] = byte(v >> 8) }
func getU16(b []byte) uint16    { return uint16(b[0]) | uint16(b[1])<<8 }
