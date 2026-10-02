package st3215

import (
	"context"
	"fmt"
	"io"
	"sync"
	"time"
)

// Port is the byte transport to the Bus Servo Adapter. *serial.Port from
// go.bug.st/serial satisfies it; tests use an in-memory emulator.
//
// Read must return (0, nil) when the read timeout elapses with no data.
type Port interface {
	io.ReadWriter
	SetReadTimeout(t time.Duration) error
	ResetInputBuffer() error
}

// Bus is one half-duplex servo bus (one adapter / serial port). It is safe for
// concurrent use; transactions are serialized.
type Bus struct {
	mu       sync.Mutex
	port     Port
	closer   io.Closer
	timeout  time.Duration
	retries  int
	noAck    bool
	onStatus func(id uint8, s Status)
	rx       []byte
	tmp      []byte

	cfgMu  sync.RWMutex
	mirror map[uint8]bool // IDs of mirrored servos, see mirror.go
}

// Option configures a Bus.
type Option func(*Bus)

// WithTimeout sets how long to wait for each reply packet (default 50ms).
func WithTimeout(d time.Duration) Option { return func(b *Bus) { b.timeout = d } }

// WithRetries sets how many times a failed transaction (timeout or corrupt
// reply) is retried (default 2). Writes are idempotent, so retrying is safe.
func WithRetries(n int) Option { return func(b *Bus) { b.retries = n } }

// WithoutWriteAck tells the bus that the servos have ResponseLevel=0 and will
// not acknowledge WRITE / REG WRITE / ACTION / RESET packets.
func WithoutWriteAck() Option { return func(b *Bus) { b.noAck = true } }

// WithStatusHandler registers a callback invoked whenever a reply carries a
// non-zero status byte (overload, over-temperature, ...). The callback runs
// while the bus lock is held: it must not call back into the Bus.
func WithStatusHandler(f func(id uint8, s Status)) Option {
	return func(b *Bus) { b.onStatus = f }
}

// NewBus wraps an already opened Port. If the port implements io.Closer it is
// closed by Bus.Close.
func NewBus(p Port, opts ...Option) (*Bus, error) {
	b := &Bus{port: p, timeout: 50 * time.Millisecond, retries: 2, tmp: make([]byte, 512)}
	if c, ok := p.(io.Closer); ok {
		b.closer = c
	}
	for _, o := range opts {
		o(b)
	}
	if err := p.SetReadTimeout(b.timeout); err != nil {
		return nil, fmt.Errorf("st3215: set read timeout: %w", err)
	}
	return b, nil
}

// Close closes the underlying port.
func (b *Bus) Close() error {
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.closer != nil {
		return b.closer.Close()
	}
	return nil
}

// Servo returns a handle for the servo with the given ID. It does not talk to
// the bus.
func (b *Bus) Servo(id uint8) *Servo { return &Servo{bus: b, id: id} }

// ---------------------------------------------------------------------------
// Low level transactions

// send discards stale input and writes one packet.
func (b *Bus) send(id, inst byte, params []byte) error {
	pkt, err := encodePacket(id, inst, params)
	if err != nil {
		return err
	}
	_ = b.port.ResetInputBuffer()
	b.rx = b.rx[:0]
	for len(pkt) > 0 {
		n, err := b.port.Write(pkt)
		if err != nil {
			return fmt.Errorf("st3215: write: %w", err)
		}
		pkt = pkt[n:]
	}
	return nil
}

// recv waits for the next valid reply from id with exactly nParams parameters.
// Replies from other IDs or of other sizes are skipped.
func (b *Bus) recv(id uint8, nParams int) (reply, error) {
	deadline := time.Now().Add(b.timeout)
	for {
		for {
			r, used, ok := parseReply(b.rx)
			b.rx = b.rx[:copy(b.rx, b.rx[used:])]
			if !ok {
				break
			}
			if (r.ID == id || id == BroadcastID) && len(r.Params) == nParams {
				if r.Status != 0 && b.onStatus != nil {
					b.onStatus(r.ID, r.Status)
				}
				return r, nil
			}
		}
		if time.Now().After(deadline) {
			return reply{}, ErrTimeout
		}
		n, err := b.port.Read(b.tmp)
		if err != nil {
			return reply{}, fmt.Errorf("st3215: read: %w", err)
		}
		b.rx = append(b.rx, b.tmp[:n]...)
	}
}

// transact sends a packet and, if a reply is expected, waits for it. Retries
// on timeouts and corrupt replies.
func (b *Bus) transact(id, inst byte, params []byte, wantReply bool, nParams int) (reply, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	var lastErr error
	for attempt := 0; attempt <= b.retries; attempt++ {
		if err := b.send(id, inst, params); err != nil {
			return reply{}, err
		}
		if !wantReply {
			return reply{}, nil
		}
		r, err := b.recv(id, nParams)
		if err == nil {
			return r, nil
		}
		lastErr = err
	}
	return reply{}, fmt.Errorf("st3215: servo %d: %w", id, lastErr)
}

func (b *Bus) expectAck(id uint8) bool { return id != BroadcastID && !b.noAck }

// Ping checks that a servo answers and returns its status byte.
func (b *Bus) Ping(id uint8) (Status, error) {
	r, err := b.transact(id, InstPing, nil, true, 0)
	return r.Status, err
}

// Identify broadcasts a PING and returns the ID of the servo that answers.
// Only use it with exactly one servo connected to the bus (e.g. to find the
// ID of a new servo before assigning a unique one).
func (b *Bus) Identify() (uint8, error) {
	r, err := b.transact(BroadcastID, InstPing, nil, true, 0)
	return r.ID, err
}

// Read reads n bytes of the memory table starting at addr.
func (b *Bus) Read(id, addr uint8, n int) ([]byte, Status, error) {
	if id == BroadcastID {
		return nil, 0, fmt.Errorf("st3215: cannot READ from broadcast ID")
	}
	r, err := b.transact(id, InstRead, []byte{addr, byte(n)}, true, n)
	return r.Params, r.Status, err
}

// Write writes data to the memory table starting at addr and waits for the
// acknowledgement (unless id is BroadcastID or acks are disabled).
func (b *Bus) Write(id, addr uint8, data []byte) (Status, error) {
	r, err := b.transact(id, InstWrite, append([]byte{addr}, data...), b.expectAck(id), 0)
	return r.Status, err
}

// RegWrite stores a write in the servo's buffer; it takes effect on Action.
func (b *Bus) RegWrite(id, addr uint8, data []byte) (Status, error) {
	r, err := b.transact(id, InstRegWrite, append([]byte{addr}, data...), b.expectAck(id), 0)
	return r.Status, err
}

// Action executes pending RegWrite buffers on all servos simultaneously.
func (b *Bus) Action() error {
	_, err := b.transact(BroadcastID, InstAction, nil, false, 0)
	return err
}

// FactoryReset restores the memory table of a servo to factory defaults.
// The ST3215 manual doesn't list which settings are reset; expect the ID
// (to 1), baud rate, offset, limits, mode and tuning. The acknowledgement
// may come from the new ID, so a missing one is not an error: look for the
// servo afterwards (Ping the old ID, then ID 1). Switch torque off first, as
// the zero may change under a servo that is holding a goal.
func (b *Bus) FactoryReset(id uint8) error {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.send(id, InstReset, nil)
}

// SyncWriteEntry is one servo's payload in a SyncWrite.
type SyncWriteEntry struct {
	ID   uint8
	Data []byte
}

// SyncWrite writes the same register range on many servos with a single
// broadcast packet. All entries must carry the same number of bytes.
// No replies are returned.
func (b *Bus) SyncWrite(addr uint8, entries []SyncWriteEntry) error {
	if len(entries) == 0 {
		return nil
	}
	l := len(entries[0].Data)
	params := make([]byte, 0, 2+len(entries)*(l+1))
	params = append(params, addr, byte(l))
	for _, e := range entries {
		if len(e.Data) != l {
			return fmt.Errorf("st3215: SyncWrite entries must have equal length")
		}
		params = append(params, e.ID)
		params = append(params, e.Data...)
	}
	_, err := b.transact(BroadcastID, InstSyncWrite, params, false, 0)
	return err
}

// SyncReadResult is one servo's answer to a SyncRead.
type SyncReadResult struct {
	Data   []byte
	Status Status
	Err    error // non-nil if this servo did not answer
}

// SyncRead reads n bytes from addr on several servos with a single request.
// Servos answer in the order given. The returned error is non-nil only for
// bus-level failures; per-servo failures are reported in SyncReadResult.Err.
func (b *Bus) SyncRead(addr uint8, n int, ids []uint8) (map[uint8]SyncReadResult, error) {
	params := append([]byte{addr, byte(n)}, ids...)
	b.mu.Lock()
	defer b.mu.Unlock()
	if err := b.send(BroadcastID, InstSyncRead, params); err != nil {
		return nil, err
	}
	out := make(map[uint8]SyncReadResult, len(ids))
	for _, id := range ids {
		r, err := b.recv(id, n)
		if err != nil {
			out[id] = SyncReadResult{Err: fmt.Errorf("st3215: servo %d: %w", id, err)}
			continue
		}
		out[id] = SyncReadResult{Data: r.Params, Status: r.Status}
	}
	return out, nil
}

// ScanResult describes a servo found by Scan.
type ScanResult struct {
	ID     uint8
	Status Status
}

// Scan pings IDs 0..253 and returns the servos that answered. timeout is the
// reply timeout per ID (e.g. 10ms); retries are disabled while probing.
func (b *Bus) Scan(ctx context.Context, timeout time.Duration) ([]ScanResult, error) {
	return b.ScanRange(ctx, 0, MaxID, timeout, nil)
}

// ScanRange pings IDs first..last and returns the servos that answered.
// progress, if not nil, is called after each probe with the ID just probed
// and whether it answered. Other bus users are not affected by the short
// probe timeout.
func (b *Bus) ScanRange(ctx context.Context, first, last uint8, timeout time.Duration, progress func(id uint8, found bool)) ([]ScanResult, error) {
	var found []ScanResult
	for id := int(first); id <= int(last) && id <= int(MaxID); id++ {
		if err := ctx.Err(); err != nil {
			return found, err
		}
		st, err := b.probe(uint8(id), timeout)
		if err == nil {
			found = append(found, ScanResult{ID: uint8(id), Status: st})
		}
		if progress != nil {
			progress(uint8(id), err == nil)
		}
	}
	return found, nil
}

// probe pings one ID with a custom timeout and no retries.
func (b *Bus) probe(id uint8, timeout time.Duration) (Status, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	saved := b.timeout
	b.timeout = timeout
	_ = b.port.SetReadTimeout(timeout)
	defer func() {
		b.timeout = saved
		_ = b.port.SetReadTimeout(saved)
	}()
	if err := b.send(id, InstPing, nil); err != nil {
		return 0, err
	}
	r, err := b.recv(id, 0)
	return r.Status, err
}
