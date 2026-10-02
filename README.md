# go-st3215

Go package for controlling and monitoring **Waveshare ST3215** serial bus servos
(Feetech STS3215 protocol) through the **Waveshare Bus Servo Adapter (A)**.
Works on Linux and macOS. The only dependency is `go.bug.st/serial`.

```go
import st3215 "github.com/frifox/go-st3215"

bus, err := st3215.Open("/dev/ttyACM0", st3215.DefaultBaudRate) // 1 Mbps, factory default
if err != nil {
    log.Fatal(err)
}
defer bus.Close()

pan, tilt := bus.Servo(1), bus.Servo(2)
bus.SyncTorque(true, 1, 2)

// Move both servos at the same time, then wait until each one arrives.
bus.SyncMove(
    st3215.Target{ID: 1, Position: st3215.DegreesToSteps(90), Speed: 1500, Acc: 50},
    st3215.Target{ID: 2, Position: st3215.DegreesToSteps(200), Speed: 1500, Acc: 50},
)
opt := st3215.WaitOptions{FailOn: st3215.StatusOverload}
pan.WaitForPosition(ctx, st3215.DegreesToSteps(90), opt)
tilt.WaitForPosition(ctx, st3215.DegreesToSteps(200), opt)

// Read the status of both servos in one round trip.
fb, _ := bus.SyncFeedback(1, 2)
fmt.Printf("%.1f° %.1fV %d°C %.0fmA %s\n",
    fb[1].Degrees(), fb[1].Voltage, fb[1].Temperature, fb[1].Current, fb[1].Status)
```

## What it covers

| Area | API |
|---|---|
| Discovery | `Bus.Scan`, `Bus.Ping`, `Bus.Identify` (one servo on the bus) |
| Position mode | `MoveTo`, `MoveToDegrees`, `MoveToAndWait`, `Stop`, `SetAcceleration` |
| Multi-turn | `SetMultiTurn(true)` → goal range ±30719 steps (±7.5 turns) |
| Other modes | `SetMode(ModeWheel/ModePWM/ModeStep)`, `SetWheelSpeed`, `SetPWM` |
| Multiple servos | `Bus.SyncMove`, `Bus.SyncTorque`, `Bus.SyncFeedback`, `RegMoveTo` + `Bus.Action` |
| Monitoring | `Feedback` (position, speed, load, voltage, temperature, current, moving, status in one read), plus single getters |
| Torque | `EnableTorque`, `SetTorqueLimit` (runtime), `SetMaxTorque` (persisted) |
| Calibration | `CalibrateMiddle` (current position becomes 2048), `SetPositionOffset` |
| Configuration | `SetID`, `SetBaudRate`, `SetAngleLimits`, `SetVoltageLimits`, `SetMaxTemperature`, `SetProtection`, `SetPID`, `SetDeadZone`, `ReadConfig` |
| Raw access | `Servo.Read(reg)` / `Servo.Write(reg, v)` for every register in `Registers`; `Bus.Read/Write/SyncRead/SyncWrite` |
| Faults | `Status` bit set (voltage, sensor, temperature, current, angle, overload); `WithStatusHandler` callback |

EEPROM writes are wrapped in unlock → write → lock automatically so they survive power cycles.
The `Bus` is safe for concurrent use.

## Demo: web console

[`example/`](example) is a WebSocket server plus a web page to monitor and drive the servos:
a live position dial, telemetry, 30 s history charts, controls for every mode, setup actions
(center calibration, ID change, multi-turn) and an editable view of the full memory table.

```bash
cd example
go run . -port /dev/ttyACM0            # Linux
go run . -port /dev/cu.usbmodem1101    # macOS
go run . -sim 1,2,3                    # simulated servos, no hardware needed
```

Then open http://localhost:8080. Flags: `-baud`, `-addr`, `-poll` (telemetry interval,
default 50 ms) and `-nosync` (poll servos one by one if SYNC READ misbehaves).

The demo is a separate Go module (`example/go.mod`), so the library itself only depends on
`go.bug.st/serial`.

## Platform notes

- **Adapter**: set the jumper on the Bus Servo Adapter (A) to the USB/PC position.
  Power the servos from the adapter's DC input (6–12 V per the Waveshare manual). The
  memory table's factory `MaxVoltage` is 8.0 V. If your supply is higher, check the `Status` in `Feedback`
  for a `voltage` status and raise it with `SetVoltageLimits`.
- **Linux**: the adapter shows up as `/dev/ttyACM0` or `/dev/ttyUSB0`. Add your user to
  the `dialout` group (`uucp` on Arch). For lower latency with FTDI/CH34x drivers,
  `setserial /dev/ttyUSB0 low_latency` helps when polling many servos quickly.
- **macOS**: use the `/dev/cu.*` device (not `/dev/tty.*`), e.g. `/dev/cu.usbmodem…` or
  `/dev/cu.wchusbserial…`. Recent macOS includes the CH34x driver.
- Each new servo ships with ID 1. Connect them **one at a time** and give each a unique
  ID (`Servo.SetID`, or the Setup panel of the demo) before chaining them.

## Things to verify on hardware

- `PWMSignBit`: the vendor memory table says the PWM direction bit is bit 11, while the
  vendor SC-series library uses bit 10. The package uses bit 10 — check the direction
  in `ModePWM` if you use it.
- `SetBaudRate`: after changing it, reopen the bus at the new speed.

## Tests

```bash
go test -race ./...
```

The tests run against an in-memory servo emulator and check packet encoding against the
example frames in the manufacturer's protocol manual.
