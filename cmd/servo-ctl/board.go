package main

import (
	"context"
	_ "embed"
	"errors"
	"fmt"
	"runtime"
	"slices"
	"strings"
	"time"

	st3215 "github.com/frifox/go-st3215"
	"go.bug.st/serial/enumerator"
)

// Serial ports, connecting to a driver board and scanning it for servos.

type portInfo struct {
	Name        string `json:"name"`
	Description string `json:"description"`
	USB         bool   `json:"usb"`
	VID         string `json:"vid,omitempty"`
	PID         string `json:"pid,omitempty"`
	Serial      string `json:"serial,omitempty"`
	Likely      bool   `json:"likely"` // looks like a Bus Servo Adapter (USB-UART bridge)
}

// usbBridges maps USB vendor IDs of common USB-UART bridges to a name.
// The Bus Servo Adapter (A) uses a WCH CH34x chip.
var usbBridges = map[string]string{
	"1A86": "WCH CH34x",
	"0403": "FTDI",
	"10C4": "Silicon Labs CP210x",
	"067B": "Prolific PL2303",
}

func listPorts() ([]portInfo, error) {
	details, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return nil, err
	}
	out := []portInfo{}
	for _, d := range details {
		// On macOS every device appears as /dev/tty.* and /dev/cu.*;
		// the cu device is the one to use for outgoing connections.
		if runtime.GOOS == "darwin" && strings.HasPrefix(d.Name, "/dev/tty.") {
			continue
		}
		p := portInfo{Name: d.Name, USB: d.IsUSB, VID: strings.ToUpper(d.VID), PID: strings.ToUpper(d.PID), Serial: d.SerialNumber}
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
	rank := func(p portInfo) int {
		switch {
		case p.Likely:
			return 0
		case p.USB:
			return 1
		}
		return 2
	}
	slices.SortStableFunc(out, func(a, b portInfo) int { return rank(a) - rank(b) })
	return out, nil
}

func (s *server) connect(port string, baud int) error {
	if port == "" {
		return errors.New("no port selected")
	}
	s.disconnect()
	if baud <= 0 {
		baud = st3215.DefaultBaudRate
	}
	var bus *st3215.Bus
	var err error
	if port == simPortName {
		bus, err = st3215.NewBus(newSimPort(s.simIDs...))
	} else {
		bus, err = st3215.Open(port, baud)
	}
	if err != nil {
		return err
	}
	for id, sc := range s.cfg.all() {
		bus.SetMirrored(id, sc.Mirrored)
		if len(sc.Range) == 2 {
			bus.SetRange(id, st3215.Range{Lo: sc.Range[0], Hi: sc.Range[1]})
		}
	}
	s.busMu.Lock()
	s.bus = bus
	s.busMu.Unlock()
	s.mu.Lock()
	s.port, s.baud, s.ids, s.scanned = port, baud, nil, false
	s.mu.Unlock()
	s.logf("info", "connected to %s at %d baud", port, baud)
	s.broadcastState()
	return nil
}

func (s *server) disconnect() {
	s.stopAutotune()
	s.mu.Lock()
	if s.scanCancel != nil {
		s.scanCancel(nil)
	}
	wasConnected := s.port != ""
	s.mu.Unlock()

	s.busMu.Lock() // waits for an in-flight scan or command to finish
	if s.bus != nil {
		s.bus.Close()
		s.bus = nil
	}
	s.busMu.Unlock()

	s.mu.Lock()
	s.port, s.baud, s.ids, s.scanned = "", 0, nil, false
	s.mu.Unlock()
	if wasConnected {
		s.logf("info", "disconnected")
		s.broadcastState()
	}
}

// errScanFinished is the cancel cause used by "Done": stop scanning but keep
// the servos found so far.
var errScanFinished = errors.New("scan finished early")

func (s *server) scan(ctx context.Context, first, last uint8) error {
	s.mu.Lock()
	if s.scanning {
		s.mu.Unlock()
		return errors.New("a scan is already running")
	}
	ctx, cancel := context.WithCancelCause(ctx)
	s.scanning, s.scanCancel = true, cancel
	s.mu.Unlock()
	defer cancel(nil)
	s.broadcastState()

	total := int(last) - int(first) + 1
	var found []uint8
	err := s.withBus(func(bus *st3215.Bus) error {
		_, err := bus.ScanRange(ctx, first, last, 15*time.Millisecond, func(id uint8, ok bool) {
			if ok {
				found = append(found, id)
			}
			s.broadcast(scanProgressMsg{Type: "scanProgress", Done: int(id) - int(first) + 1,
				Total: total, Current: int(id), Found: toInts(found)})
		})
		return err
	})
	early := err != nil && errors.Is(context.Cause(ctx), errScanFinished)
	if early {
		err = nil // "Done": keep the servos found so far
	}

	s.mu.Lock()
	s.scanning, s.scanCancel = false, nil
	if err == nil {
		s.ids, s.scanned = found, true
	}
	s.mu.Unlock()
	switch {
	case early:
		s.logf("info", "scan stopped early, using %d servo(s) found so far: %v", len(found), found)
	case errors.Is(err, context.Canceled):
		s.logf("info", "scan cancelled")
	case err != nil:
		s.logf("error", "scan stopped: %v", err)
	default:
		s.logf("info", "scan found %d servo(s): %v", len(found), found)
	}
	s.broadcastState()
	return err
}
