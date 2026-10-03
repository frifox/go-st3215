// Package board is the driver board side of servo-ctl: listing serial ports,
// connecting to a board (or the simulator), scanning it for servos, and
// giving the rest of servo-ctl access to the bus.
package board

import (
	"context"
	"errors"
	"fmt"
	"runtime"
	"slices"
	"strings"
	"sync"
	"time"

	st3215 "github.com/frifox/go-st3215"
	"github.com/frifox/go-st3215/cmd/servo-ctl/internal"
	servosim "github.com/frifox/go-st3215/cmd/servo-ctl/servo-sim"
	"go.bug.st/serial/enumerator"
)

// SimPort is the pseudo port that selects the simulated driver board.
const SimPort = "sim"

// Board is the connection to one driver board and the servos found on it.
type Board struct {
	cfg    *internal.Config // mirroring and motion ranges are applied to the bus
	n      internal.Notifier
	simIDs []uint8

	// busMu guards bus: users hold it for reading, Connect/Disconnect swap it.
	busMu sync.RWMutex
	bus   *st3215.Bus

	mu         sync.Mutex
	port       string
	baud       int
	ids        []uint8 // servos found by the last scan
	scanned    bool    // a scan has completed on this connection
	scanning   bool
	scanCancel context.CancelCauseFunc
}

// New returns a disconnected board. simIDs are the servos of the simulator.
func New(cfg *internal.Config, n internal.Notifier, simIDs []uint8) *Board {
	return &Board{cfg: cfg, n: n, simIDs: simIDs}
}

// Status is the board's connection and scan state.
type Status struct {
	Port     string // "" = not connected
	Baud     int
	IDs      []uint8
	Scanning bool
	Scanned  bool
}

// Status returns a snapshot of the connection and scan state.
func (b *Board) Status() Status {
	b.mu.Lock()
	defer b.mu.Unlock()
	return Status{Port: b.port, Baud: b.baud, IDs: slices.Clone(b.ids), Scanning: b.scanning, Scanned: b.scanned}
}

// IDs returns the servos found by the last scan.
func (b *Board) IDs() []uint8 {
	b.mu.Lock()
	defer b.mu.Unlock()
	return slices.Clone(b.ids)
}

// SimDescription describes the simulated board for the port picker.
func (b *Board) SimDescription() string {
	return fmt.Sprintf("Simulated board with servos %v", b.simIDs)
}

// WithBus runs f with the connected bus, or fails if there is none.
func (b *Board) WithBus(f func(*st3215.Bus) error) error {
	b.busMu.RLock()
	defer b.busMu.RUnlock()
	if b.bus == nil {
		return errors.New("no driver board connected")
	}
	return f(b.bus)
}

// usbBridges maps USB vendor IDs of common USB-UART bridges to a name.
// The Bus Servo Adapter (A) uses a WCH CH34x chip.
var usbBridges = map[string]string{
	"1A86": "WCH CH34x",
	"0403": "FTDI",
	"10C4": "Silicon Labs CP210x",
	"067B": "Prolific PL2303",
}

// ListPorts lists the serial ports, likely Bus Servo Adapters first.
func ListPorts() ([]internal.PortInfo, error) {
	details, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return nil, err
	}
	out := []internal.PortInfo{}
	for _, d := range details {
		// On macOS every device appears as /dev/tty.* and /dev/cu.*;
		// the cu device is the one to use for outgoing connections.
		if runtime.GOOS == "darwin" && strings.HasPrefix(d.Name, "/dev/tty.") {
			continue
		}
		p := internal.PortInfo{Name: d.Name, USB: d.IsUSB, VID: strings.ToUpper(d.VID), PID: strings.ToUpper(d.PID), Serial: d.SerialNumber}
		bridge, known := usbBridges[p.VID]
		p.Likely = d.IsUSB && known
		var desc []string
		if d.Product != "" {
			desc = append(desc, d.Product)
		}
		if known {
			desc = append(desc, bridge)
		}
		if d.IsUSB {
			desc = append(desc, fmt.Sprintf("USB %s:%s", p.VID, p.PID))
		}
		p.Description = strings.Join(desc, " · ")
		out = append(out, p)
	}
	rank := func(p internal.PortInfo) int {
		switch {
		case p.Likely:
			return 0
		case p.USB:
			return 1
		}
		return 2
	}
	slices.SortStableFunc(out, func(a, b internal.PortInfo) int { return rank(a) - rank(b) })
	return out, nil
}

// Connect opens port (SimPort for the simulator), closing any previous
// connection. Mirroring and motion ranges from the config are applied to the
// new bus.
func (b *Board) Connect(port string, baud int) error {
	if port == "" {
		return errors.New("no port selected")
	}
	b.Disconnect()
	if baud <= 0 {
		baud = st3215.DefaultBaudRate
	}
	var bus *st3215.Bus
	var err error
	if port == SimPort {
		bus, err = st3215.NewBus(servosim.NewPort(b.simIDs...))
	} else {
		bus, err = st3215.Open(port, baud)
	}
	if err != nil {
		return err
	}
	for id, sc := range b.cfg.All() {
		bus.SetMirrored(id, sc.Mirrored)
		if len(sc.Range) == 2 {
			bus.SetRange(id, st3215.Range{Lo: sc.Range[0], Hi: sc.Range[1]})
		}
	}
	b.busMu.Lock()
	b.bus = bus
	b.busMu.Unlock()
	b.mu.Lock()
	b.port, b.baud, b.ids, b.scanned = port, baud, nil, false
	b.mu.Unlock()
	b.n.Logf("info", "connected to %s at %d baud", port, baud)
	b.n.BroadcastState()
	return nil
}

// Disconnect stops a scan and closes the connection.
func (b *Board) Disconnect() {
	b.mu.Lock()
	if b.scanCancel != nil {
		b.scanCancel(nil)
	}
	wasConnected := b.port != ""
	b.mu.Unlock()

	b.busMu.Lock() // waits for an in-flight scan or command to finish
	if b.bus != nil {
		b.bus.Close()
		b.bus = nil
	}
	b.busMu.Unlock()

	b.mu.Lock()
	b.port, b.baud, b.ids, b.scanned = "", 0, nil, false
	b.mu.Unlock()
	if wasConnected {
		b.n.Logf("info", "disconnected")
		b.n.BroadcastState()
	}
}

// errScanFinished is the cancel cause used by "Done": stop scanning but keep
// the servos found so far.
var errScanFinished = errors.New("scan finished early")

// Scan looks for servos with IDs first..last, broadcasting progress. The
// servos found replace the previous list.
func (b *Board) Scan(ctx context.Context, first, last uint8) error {
	b.mu.Lock()
	if b.scanning {
		b.mu.Unlock()
		return errors.New("a scan is already running")
	}
	ctx, cancel := context.WithCancelCause(ctx)
	b.scanning, b.scanCancel = true, cancel
	b.mu.Unlock()
	defer cancel(nil)
	b.n.BroadcastState()

	total := int(last) - int(first) + 1
	var found []uint8
	err := b.WithBus(func(bus *st3215.Bus) error {
		_, err := bus.ScanRange(ctx, first, last, 15*time.Millisecond, func(id uint8, ok bool) {
			if ok {
				found = append(found, id)
			}
			b.n.Broadcast(internal.ScanProgressMsg{Type: "scanProgress", Done: int(id) - int(first) + 1,
				Total: total, Current: int(id), Found: internal.ToInts(found)})
		})
		return err
	})
	early := err != nil && errors.Is(context.Cause(ctx), errScanFinished)
	if early {
		err = nil // "Done": keep the servos found so far
	}

	b.mu.Lock()
	b.scanning, b.scanCancel = false, nil
	if err == nil {
		b.ids, b.scanned = found, true
	}
	b.mu.Unlock()
	switch {
	case early:
		b.n.Logf("info", "scan stopped early, using %d servo(s) found so far: %v", len(found), found)
	case errors.Is(err, context.Canceled):
		b.n.Logf("info", "scan cancelled")
	case err != nil:
		b.n.Logf("error", "scan stopped: %v", err)
	default:
		b.n.Logf("info", "scan found %d servo(s): %v", len(found), found)
	}
	b.n.BroadcastState()
	return err
}

// StopScan ends a running scan: keep = use the servos found so far ("Done"),
// otherwise the scan is cancelled and its results discarded.
func (b *Board) StopScan(keep bool) {
	cause := error(nil)
	if keep {
		cause = errScanFinished
	}
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.scanCancel != nil {
		b.scanCancel(cause)
	}
}

// ChangeID updates the found servos after servo from now answers as to. It
// reports false (and only drops from) if another servo already uses to.
func (b *Board) ChangeID(from, to uint8) bool {
	b.mu.Lock()
	defer b.mu.Unlock()
	conflict := from != to && slices.Contains(b.ids, to)
	b.ids = slices.DeleteFunc(b.ids, func(x uint8) bool { return x == from })
	if !conflict {
		b.ids = append(b.ids, to)
		slices.Sort(b.ids)
	}
	return !conflict
}
