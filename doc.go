// Package gosts (Go STServo) controls and monitors Waveshare ST3215 (Feetech
// STS3215) serial bus servos through a Waveshare Bus Servo Adapter (A) or any
// other half-duplex TTL adapter. It runs on Linux and macOS.
//
// The package has three layers:
//
//   - Bus: the raw protocol (PING, READ, WRITE, REG WRITE, ACTION, RESET,
//     SYNC READ, SYNC WRITE) over a serial port, with timeouts, retries and
//     resynchronisation on line noise.
//   - Servo: a typed API for one servo — motion (MoveTo, wheel, PWM, step
//     modes), monitoring (Feedback, Position, Load, Voltage, Current,
//     Temperature, Status), and persistent configuration (ID, baud rate,
//     limits, offsets, PID, protection) with automatic EEPROM unlock/lock.
//   - Registers: a descriptor for every entry of the memory table, usable
//     with Servo.Read / Servo.Write for anything not covered by a helper.
//
// Minimal example:
//
//	bus, err := gosts.Open("/dev/ttyACM0", gosts.DefaultBaudRate)
//	if err != nil { ... }
//	defer bus.Close()
//
//	s := bus.Servo(1)
//	s.EnableTorque(true)
//	fb, err := s.MoveToAndWait(ctx, 2048, 1500, 50, gosts.WaitOptions{})
//	fmt.Println(fb.Position, fb.Voltage, fb.Temperature)
//
// Several servos can be moved simultaneously with Bus.SyncMove (one packet)
// or Servo.RegMoveTo followed by Bus.Action, and polled together with
// Bus.SyncFeedback.
//
// Units: positions are encoder steps (4096 per turn, 2048 = center),
// speeds are step/s, acceleration is in 100 step/s². Feedback converts
// voltage, current and load to V, mA and %.
package gosts
