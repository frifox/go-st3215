package gosts

import (
	"fmt"

	"go.bug.st/serial"
)

// DefaultBaudRate is the factory baud rate of the ST3215 (1 Mbps).
const DefaultBaudRate = 1000000

// Open opens the Bus Servo Adapter on a serial device, for example
// "/dev/ttyUSB0" or "/dev/ttyACM0" on Linux, "/dev/cu.usbserial-XXXX" or
// "/dev/cu.wchusbserialXXXX" on macOS. baud is the bus speed in bits per
// second (DefaultBaudRate for factory-new servos); 8N1 framing is used.
//
// Make sure the adapter's mode jumper is set to the USB position (A).
func Open(device string, baud int, opts ...Option) (*Bus, error) {
	if baud <= 0 {
		baud = DefaultBaudRate
	}
	p, err := serial.Open(device, &serial.Mode{
		BaudRate: baud,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	})
	if err != nil {
		return nil, fmt.Errorf("gosts: open %s: %w", device, err)
	}
	b, err := NewBus(p, opts...)
	if err != nil {
		p.Close()
		return nil, err
	}
	return b, nil
}

// ListPorts returns the serial devices present on the system.
func ListPorts() ([]string, error) { return serial.GetPortsList() }
