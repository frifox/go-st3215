package st3215

import (
	"context"
	"errors"
	"fmt"
	"time"
)

// Physical constants of the ST3215.
const (
	StepsPerRev      = 4096                // encoder resolution, steps per 360°
	DegreesPerStep   = 360.0 / StepsPerRev // 0.087890625°
	CenterPosition   = 2048                // middle of the single-turn range
	MaxSpeed         = 3400                // no-load speed at 7.4V, step/s (~50 rpm)
	MultiTurnLimit   = 30719               // max |GoalPosition| in multi-turn mode (±7.5 turns)
	CurrentPerUnitMA = 6.5                 // PresentCurrent / ProtectionCurrent unit, mA
	AccelUnit        = 100                 // Acceleration unit, step/s²
	SpeedUnitRPM     = 60.0 / StepsPerRev  // rpm per step/s
	torqueCalibrate  = 128                 // TorqueEnable value: set current position as 2048
)

// StepsToDegrees converts encoder steps to degrees (2048 → 180°).
func StepsToDegrees(steps int) float64 { return float64(steps) * DegreesPerStep }

// DegreesToSteps converts degrees to the nearest encoder step.
func DegreesToSteps(deg float64) int {
	s := deg / DegreesPerStep
	if s < 0 {
		return int(s - 0.5)
	}
	return int(s + 0.5)
}

// Servo is a handle to one servo on a Bus. All methods are safe for
// concurrent use (they are serialized by the Bus).
type Servo struct {
	bus *Bus
	id  uint8
}

// ID returns the servo ID this handle addresses.
func (s *Servo) ID() uint8 { return s.id }

// Bus returns the bus the servo is attached to.
func (s *Servo) Bus() *Bus { return s.bus }

// Ping checks that the servo responds.
func (s *Servo) Ping() (Status, error) { return s.bus.Ping(s.id) }

// ---------------------------------------------------------------------------
// Generic register access

// Read reads one register and decodes it (sign handled).
func (s *Servo) Read(r Register) (int, error) {
	b, _, err := s.bus.Read(s.id, r.Addr, int(r.Size))
	if err != nil {
		return 0, err
	}
	return r.decode(b), nil
}

// Write encodes and writes one register. Writes to EEPROM registers are
// wrapped in an unlock/lock sequence so they persist across power cycles.
func (s *Servo) Write(r Register, v int) error {
	if r.ReadOnly {
		return fmt.Errorf("st3215: register %s is read-only", r.Name)
	}
	data, err := r.encode(v)
	if err != nil {
		return err
	}
	if r.Area == EEPROM {
		return s.withUnlockedEEPROM(func() error {
			_, err := s.bus.Write(s.id, r.Addr, data)
			return err
		})
	}
	_, err = s.bus.Write(s.id, r.Addr, data)
	return err
}

// WriteTemporary writes a register without persisting it: for EEPROM
// registers the write lock is closed first, so the servo uses the new value
// until it is power-cycled and then reverts to the stored one. Handy for
// trying tuning values under load before saving them with Write. SRAM
// registers behave as with Write.
func (s *Servo) WriteTemporary(r Register, v int) error {
	if r.ReadOnly {
		return fmt.Errorf("st3215: register %s is read-only", r.Name)
	}
	data, err := r.encode(v)
	if err != nil {
		return err
	}
	if r.Area == EEPROM {
		if err := s.LockEEPROM(); err != nil {
			return fmt.Errorf("lock EEPROM: %w", err)
		}
	}
	_, err = s.bus.Write(s.id, r.Addr, data)
	return err
}

// ReadMemory returns the raw memory table (addresses 0..70).
func (s *Servo) ReadMemory() ([]byte, error) {
	b, _, err := s.bus.Read(s.id, 0, memoryTableSize)
	return b, err
}

// UnlockEEPROM opens the EEPROM write lock: subsequent EEPROM writes persist.
func (s *Servo) UnlockEEPROM() error {
	_, err := s.bus.Write(s.id, RegLock.Addr, []byte{0})
	return err
}

// LockEEPROM closes the EEPROM write lock.
func (s *Servo) LockEEPROM() error {
	_, err := s.bus.Write(s.id, RegLock.Addr, []byte{1})
	return err
}

func (s *Servo) withUnlockedEEPROM(f func() error) error {
	if err := s.UnlockEEPROM(); err != nil {
		return fmt.Errorf("unlock EEPROM: %w", err)
	}
	ferr := f()
	lerr := s.LockEEPROM()
	if ferr != nil {
		return ferr
	}
	if lerr != nil {
		return fmt.Errorf("lock EEPROM: %w", lerr)
	}
	return nil
}

// ---------------------------------------------------------------------------
// Monitoring

// Feedback is a snapshot of the servo's live state (addresses 56..70).
type Feedback struct {
	Position     int     `json:"position"`        // steps within one turn (0..4095), also in multi-turn mode; see AbsolutePosition
	Speed        int     `json:"speed"`           // step/s, signed by direction
	Load         float64 `json:"load"`            // % of max drive duty, signed by direction (-100..100)
	Voltage      float64 `json:"voltage"`         // volts
	Temperature  int     `json:"temperature"`     // °C
	RegWritePend bool    `json:"regWritePending"` // a RegWrite is waiting for Action
	Status       Status  `json:"status"`          // active fault conditions
	Moving       bool    `json:"moving"`
	Current      float64 `json:"current"` // mA, signed
}

// Degrees returns Position in degrees.
func (f Feedback) Degrees() float64 { return StepsToDegrees(f.Position) }

// RPM returns Speed in revolutions per minute.
func (f Feedback) RPM() float64 { return float64(f.Speed) * SpeedUnitRPM }

const (
	feedbackAddr = 56
	feedbackLen  = 70 - 56 + 1
)

func decodeFeedback(b []byte) Feedback {
	m := func(r Register) []byte { return b[r.Addr-feedbackAddr:] }
	return Feedback{
		Position:     RegPresentPosition.decode(m(RegPresentPosition)),
		Speed:        RegPresentSpeed.decode(m(RegPresentSpeed)),
		Load:         float64(RegPresentLoad.decode(m(RegPresentLoad))) / 10,
		Voltage:      float64(RegPresentVoltage.decode(m(RegPresentVoltage))) / 10,
		Temperature:  RegPresentTemperature.decode(m(RegPresentTemperature)),
		RegWritePend: m(RegRegWriteFlag)[0] != 0,
		Status:       Status(m(RegStatus)[0]),
		Moving:       m(RegMoving)[0] != 0,
		Current:      float64(RegPresentCurrent.decode(m(RegPresentCurrent))) * CurrentPerUnitMA,
	}
}

// Feedback reads all live values with a single READ.
func (s *Servo) Feedback() (Feedback, error) {
	b, _, err := s.bus.Read(s.id, feedbackAddr, feedbackLen)
	if err != nil {
		return Feedback{}, err
	}
	return decodeFeedback(b).mirrored(s.Mirrored()), nil
}

// Position returns the present position in steps.
func (s *Servo) Position() (int, error) {
	v, err := s.Read(RegPresentPosition)
	return mirrorPos(s.Mirrored(), v), err
}

// Speed returns the present speed in step/s.
func (s *Servo) Speed() (int, error) {
	v, err := s.Read(RegPresentSpeed)
	return mirrorSign(s.Mirrored(), v), err
}

// Load returns the present load in % (-100..100).
func (s *Servo) Load() (float64, error) {
	v, err := s.Read(RegPresentLoad)
	return mirrorSign(s.Mirrored(), float64(v)/10), err
}

// Voltage returns the supply voltage in volts.
func (s *Servo) Voltage() (float64, error) {
	v, err := s.Read(RegPresentVoltage)
	return float64(v) / 10, err
}

// Temperature returns the internal temperature in °C.
func (s *Servo) Temperature() (int, error) { return s.Read(RegPresentTemperature) }

// Current returns the motor current in mA.
func (s *Servo) Current() (float64, error) {
	v, err := s.Read(RegPresentCurrent)
	return mirrorSign(s.Mirrored(), float64(v)*CurrentPerUnitMA), err
}

// Moving reports whether the servo is currently moving.
func (s *Servo) Moving() (bool, error) {
	v, err := s.Read(RegMoving)
	return v != 0, err
}

// Status returns the active fault conditions (register 65).
func (s *Servo) Status() (Status, error) {
	v, err := s.Read(RegStatus)
	return Status(v), err
}

// ---------------------------------------------------------------------------
// Configuration

// Info identifies the servo hardware and firmware.
type Info struct {
	FirmwareMajor int `json:"firmwareMajor"`
	FirmwareMinor int `json:"firmwareMinor"`
	ServoMajor    int `json:"servoMajor"`
	ServoMinor    int `json:"servoMinor"`
}

func (i Info) String() string {
	return fmt.Sprintf("servo %d.%d, firmware %d.%d", i.ServoMajor, i.ServoMinor, i.FirmwareMajor, i.FirmwareMinor)
}

// Config holds every EEPROM setting plus the volatile SRAM control values.
// Values are raw register units (see the Register definitions).
type Config struct {
	Info
	ID                int      `json:"id"`
	BaudRate          BaudRate `json:"baudRate"`
	ReturnDelay       int      `json:"returnDelay"`    // 2µs
	ResponseLevel     int      `json:"responseLevel"`  // 0: only READ/PING reply, 1: all instructions reply
	MinAngleLimit     int      `json:"minAngleLimit"`  // step
	MaxAngleLimit     int      `json:"maxAngleLimit"`  // step (both 0 = multi-turn)
	MaxTemperature    int      `json:"maxTemperature"` // °C
	MaxVoltage        int      `json:"maxVoltage"`     // 0.1V
	MinVoltage        int      `json:"minVoltage"`     // 0.1V
	MaxTorque         int      `json:"maxTorque"`      // 0.1%
	Phase             int      `json:"phase"`
	UnloadCondition   Status   `json:"unloadCondition"`   // conditions that cut torque
	LEDAlarmCondition Status   `json:"ledAlarmCondition"` // conditions that blink the LED
	PositionP         int      `json:"positionP"`
	PositionD         int      `json:"positionD"`
	PositionI         int      `json:"positionI"`
	MinStartForce     int      `json:"minStartForce"`     // 0.1%
	CWDeadZone        int      `json:"cwDeadZone"`        // step
	CCWDeadZone       int      `json:"ccwDeadZone"`       // step
	ProtectionCurrent int      `json:"protectionCurrent"` // 6.5mA
	AngularResolution int      `json:"angularResolution"`
	PositionOffset    int      `json:"positionOffset"` // step
	Mode              Mode     `json:"mode"`
	ProtectiveTorque  int      `json:"protectiveTorque"` // %
	ProtectionTime    int      `json:"protectionTime"`   // 10ms
	OverloadTorque    int      `json:"overloadTorque"`   // %
	SpeedP            int      `json:"speedP"`
	OverCurrentTime   int      `json:"overCurrentTime"` // 10ms
	SpeedI            int      `json:"speedI"`

	TorqueEnabled bool `json:"torqueEnabled"`
	Acceleration  int  `json:"acceleration"` // 100 step/s²
	GoalPosition  int  `json:"goalPosition"` // step
	GoalTime      int  `json:"goalTime"`
	GoalSpeed     int  `json:"goalSpeed"`   // step/s
	TorqueLimit   int  `json:"torqueLimit"` // 0.1%
	EEPROMLocked  bool `json:"eepromLocked"`
}

// MultiTurn reports whether the angle limits put the servo in multi-turn mode.
func (c Config) MultiTurn() bool { return c.MinAngleLimit == 0 && c.MaxAngleLimit == 0 }

// ReadConfig reads the whole memory table and decodes the configuration.
// For a mirrored servo the goal and angle limits are in logical coordinates.
func (s *Servo) ReadConfig() (Config, error) {
	m, err := s.ReadMemory()
	if err != nil {
		return Config{}, err
	}
	g := func(r Register) int { return r.decode(m[r.Addr:]) }
	c := Config{
		Info: Info{
			FirmwareMajor: g(RegFirmwareMajor), FirmwareMinor: g(RegFirmwareMinor),
			ServoMajor: g(RegServoMajor), ServoMinor: g(RegServoMinor),
		},
		ID:                g(RegID),
		BaudRate:          BaudRate(g(RegBaudRate)),
		ReturnDelay:       g(RegReturnDelay),
		ResponseLevel:     g(RegResponseLevel),
		MinAngleLimit:     g(RegMinAngleLimit),
		MaxAngleLimit:     g(RegMaxAngleLimit),
		MaxTemperature:    g(RegMaxTemperature),
		MaxVoltage:        g(RegMaxVoltage),
		MinVoltage:        g(RegMinVoltage),
		MaxTorque:         g(RegMaxTorque),
		Phase:             g(RegPhase),
		UnloadCondition:   Status(g(RegUnloadCondition)),
		LEDAlarmCondition: Status(g(RegLEDAlarmCondition)),
		PositionP:         g(RegPositionP),
		PositionD:         g(RegPositionD),
		PositionI:         g(RegPositionI),
		MinStartForce:     g(RegMinStartForce),
		CWDeadZone:        g(RegCWDeadZone),
		CCWDeadZone:       g(RegCCWDeadZone),
		ProtectionCurrent: g(RegProtectionCurrent),
		AngularResolution: g(RegAngularResolution),
		PositionOffset:    g(RegPositionOffset),
		Mode:              Mode(g(RegMode)),
		ProtectiveTorque:  g(RegProtectiveTorque),
		ProtectionTime:    g(RegProtectionTime),
		OverloadTorque:    g(RegOverloadTorque),
		SpeedP:            g(RegSpeedP),
		OverCurrentTime:   g(RegOverCurrentTime),
		SpeedI:            g(RegSpeedI),
		TorqueEnabled:     g(RegTorqueEnable) == 1,
		Acceleration:      g(RegAcceleration),
		GoalPosition:      g(RegGoalPosition),
		GoalTime:          g(RegGoalTime),
		GoalSpeed:         g(RegGoalSpeed),
		TorqueLimit:       g(RegTorqueLimit),
		EEPROMLocked:      g(RegLock) != 0,
	}
	if on := s.Mirrored(); on {
		c.MinAngleLimit, c.MaxAngleLimit = mirrorLimits(on, c.MinAngleLimit, c.MaxAngleLimit)
		switch c.Mode {
		case ModePosition:
			c.GoalPosition = mirrorPos(on, c.GoalPosition)
		case ModeStep:
			c.GoalPosition = -c.GoalPosition
		case ModeWheel:
			c.GoalSpeed = -c.GoalSpeed
		case ModePWM:
			c.GoalTime = -c.GoalTime
		}
	}
	return c, nil
}

// Info reads hardware and firmware versions.
func (s *Servo) Info() (Info, error) {
	b, _, err := s.bus.Read(s.id, 0, 5)
	if err != nil {
		return Info{}, err
	}
	return Info{FirmwareMajor: int(b[0]), FirmwareMinor: int(b[1]), ServoMajor: int(b[3]), ServoMinor: int(b[4])}, nil
}

// SetID changes the servo ID (persisted). The handle is updated to the new ID.
func (s *Servo) SetID(newID uint8) error {
	if newID > MaxID {
		return fmt.Errorf("st3215: invalid ID %d", newID)
	}
	if err := s.UnlockEEPROM(); err != nil {
		return err
	}
	// The acknowledgement may already come from the new ID, so a timeout here
	// is expected; success is verified by pinging the new ID instead.
	if _, err := s.bus.Write(s.id, RegID.Addr, []byte{newID}); err != nil && !errors.Is(err, ErrTimeout) {
		return err
	}
	if _, err := s.bus.Ping(newID); err != nil {
		return fmt.Errorf("st3215: servo did not answer under new ID %d: %w", newID, err)
	}
	if s.Mirrored() {
		s.bus.SetMirrored(s.id, false)
		s.bus.SetMirrored(newID, true)
	}
	s.id = newID
	return s.LockEEPROM()
}

// SetBaudRate changes the servo's serial speed (persisted). After this call
// the servo no longer understands the current bus speed: reopen the bus at
// the new speed. The EEPROM stays unlocked until LockEEPROM is called at the
// new speed (this does not affect persistence of the baud rate itself).
func (s *Servo) SetBaudRate(br BaudRate) error {
	if br.BitsPerSecond() == 0 {
		return fmt.Errorf("st3215: invalid baud rate index %d", br)
	}
	if err := s.UnlockEEPROM(); err != nil {
		return err
	}
	_, err := s.bus.Write(s.id, RegBaudRate.Addr, []byte{byte(br)})
	if errors.Is(err, ErrTimeout) {
		err = nil // the ack may come at the new speed
	}
	return err
}

// SetMode changes the operating mode (persisted).
func (s *Servo) SetMode(m Mode) error { return s.Write(RegMode, int(m)) }

// Mode reads the operating mode.
func (s *Servo) Mode() (Mode, error) {
	v, err := s.Read(RegMode)
	return Mode(v), err
}

// SetAngleLimits sets the allowed position range in steps (persisted). Both 0
// selects multi-turn mode. For a mirrored servo the range is logical.
func (s *Servo) SetAngleLimits(min, max int) error {
	if min != 0 || max != 0 {
		if min >= max {
			return fmt.Errorf("st3215: min angle limit %d must be below max %d", min, max)
		}
	}
	min, max = mirrorLimits(s.Mirrored(), min, max)
	return s.setAngleLimitsRaw(min, max)
}

func (s *Servo) setAngleLimitsRaw(min, max int) error {
	if min != 0 || max != 0 {
		if min >= max {
			return fmt.Errorf("st3215: min angle limit %d must be below max %d", min, max)
		}
	}
	if _, err := RegMinAngleLimit.encode(min); err != nil {
		return err
	}
	if _, err := RegMaxAngleLimit.encode(max); err != nil {
		return err
	}
	data := make([]byte, 4)
	putU16(data[0:], uint16(min))
	putU16(data[2:], uint16(max))
	return s.withUnlockedEEPROM(func() error {
		_, err := s.bus.Write(s.id, RegMinAngleLimit.Addr, data)
		return err
	})
}

// AngleLimits reads the allowed position range in steps.
func (s *Servo) AngleLimits() (min, max int, err error) {
	b, _, err := s.bus.Read(s.id, RegMinAngleLimit.Addr, 4)
	if err != nil {
		return 0, 0, err
	}
	min, max = mirrorLimits(s.Mirrored(), int(getU16(b)), int(getU16(b[2:])))
	return min, max, nil
}

// SetMultiTurn enables multi-turn absolute positioning (angle limits 0/0,
// goal range ±30719 steps) or restores the single-turn range 0..4095.
// The turn count is not saved across power cycles.
func (s *Servo) SetMultiTurn(on bool) error {
	if on {
		return s.setAngleLimitsRaw(0, 0)
	}
	return s.setAngleLimitsRaw(0, StepsPerRev-1)
}

// SetPositionOffset sets the position correction in steps (-2047..2047, persisted).
func (s *Servo) SetPositionOffset(steps int) error { return s.Write(RegPositionOffset, steps) }

// CalibrateMiddle makes the current physical position read as 2048 (the
// center) by adjusting PositionOffset. Persisted.
func (s *Servo) CalibrateMiddle() error {
	return s.withUnlockedEEPROM(func() error {
		_, err := s.bus.Write(s.id, RegTorqueEnable.Addr, []byte{torqueCalibrate})
		return err
	})
}

// SetZeroAt makes the position that currently reads `at` (steps, logical)
// read 0 from now on: every reading shifts by -at. It changes PositionOffset
// (persisted), i.e. the servo's own zero, wherever the arm is. For example
// SetZeroAt(3072) turns the 270° mark into 0° and straight up (0°) into 90°.
// If the servo holds torque its goal is shifted too, so it doesn't move; with
// torque off the goal isn't touched (writing one would switch torque on). In
// single-turn mode the 0/4095 seam the servo can't cross moves with it.
//
// Verified on hardware: reported position = raw position - PositionOffset.
// The offset range is ±2047 steps, so a shift that lands exactly half a turn
// away is one step short.
func (s *Servo) SetZeroAt(at int) error {
	// Logical readings shift by -at; physical ones by -at, or +at mirrored.
	shift := at
	if s.Mirrored() {
		shift = -at
	}
	return s.shiftOffset(shift)
}

// SetZero puts the servo's 0° at `at` (steps) measured on the encoder scale,
// i.e. what the position reads with PositionOffset 0. Unlike SetZeroAt it is
// absolute: SetZero(3072) always means "the encoder's 270° mark is 0°", however
// often it is applied, and Zero reports it back. SetZero(0) is ResetZero.
func (s *Servo) SetZero(at int) error {
	off, err := s.Read(RegPositionOffset)
	if err != nil {
		return err
	}
	want := at
	if s.Mirrored() {
		want = -at
	}
	return s.shiftOffset(CircularDiff(want, off))
}

// Zero returns where the servo's 0° is on the encoder scale (steps, 0..4095),
// the value SetZero takes: 0 means no offset.
func (s *Servo) Zero() (int, error) {
	off, err := s.Read(RegPositionOffset)
	if err != nil {
		return 0, err
	}
	if s.Mirrored() {
		off = -off
	}
	return (off%StepsPerRev + StepsPerRev) % StepsPerRev, nil
}

// ResetZero sets PositionOffset to 0, so readings are the raw encoder angle.
// Note this is not the factory state: a factory reset restores Waveshare's
// factory calibration offset (e.g. 85 steps), which is servo-specific. Like SetZeroAt, a servo holding torque doesn't
// move.
func (s *Servo) ResetZero() error {
	off, err := s.Read(RegPositionOffset)
	if err != nil {
		return err
	}
	return s.shiftOffset(-off)
}

// SetPositionAs makes the current position read pos (steps, logical), by the
// same mechanism as SetZeroAt. CalibrateMiddle is SetPositionAs(2048).
func (s *Servo) SetPositionAs(pos int) error {
	cur, err := s.Position()
	if err != nil {
		return err
	}
	return s.SetZeroAt(cur - pos)
}

// shiftOffset adds delta (physical steps) to PositionOffset, so physical
// readings drop by delta. If the servo is holding torque, its goal moves by
// the same amount so it stays put. With torque off the goal is left alone:
// writing a goal switches torque on (seen on hardware), which would drive
// the arm to a stale goal.
func (s *Servo) shiftOffset(delta int) error {
	off, err := s.Read(RegPositionOffset)
	if err != nil {
		return err
	}
	holding, err := s.TorqueEnabled()
	if err != nil {
		return err
	}
	goal, err := s.Read(RegGoalPosition)
	if err != nil {
		return err
	}
	lo, hi, err := s.AngleLimits()
	if err != nil {
		return err
	}
	newOff := CircularDiff(off+delta, 0) // keep within one turn
	if newOff == -StepsPerRev/2 {
		newOff = StepsPerRev/2 - 1 // -2048 isn't representable; one step off
	}
	applied := newOff - off
	newGoal := goal - applied
	if lo != 0 || hi != 0 { // single-turn goals live within one turn
		newGoal = (newGoal%StepsPerRev + StepsPerRev) % StepsPerRev
	}
	if err := s.Write(RegPositionOffset, newOff); err != nil {
		return err
	}
	if !holding {
		return nil
	}
	_, err = s.bus.Write(s.id, RegGoalPosition.Addr, mustEncode(RegGoalPosition, newGoal))
	return err
}

// SetMaxTorque sets the persisted torque ceiling in % (0..100), which is also
// copied into TorqueLimit at power-up.
func (s *Servo) SetMaxTorque(percent float64) error {
	return s.Write(RegMaxTorque, int(percent*10+0.5))
}

// SetVoltageLimits sets the allowed supply range in volts (persisted).
func (s *Servo) SetVoltageLimits(min, max float64) error {
	if err := s.Write(RegMinVoltage, int(min*10+0.5)); err != nil {
		return err
	}
	return s.Write(RegMaxVoltage, int(max*10+0.5))
}

// SetMaxTemperature sets the temperature protection threshold in °C (persisted).
func (s *Servo) SetMaxTemperature(c int) error { return s.Write(RegMaxTemperature, c) }

// SetProtection selects which conditions cut torque (unload) and which blink
// the LED (persisted).
func (s *Servo) SetProtection(unload, ledAlarm Status) error {
	if err := s.Write(RegUnloadCondition, int(unload)); err != nil {
		return err
	}
	return s.Write(RegLEDAlarmCondition, int(ledAlarm))
}

// SetPID sets the position loop gains (persisted).
func (s *Servo) SetPID(p, i, d int) error {
	if err := s.Write(RegPositionP, p); err != nil {
		return err
	}
	if err := s.Write(RegPositionI, i); err != nil {
		return err
	}
	return s.Write(RegPositionD, d)
}

// SetDeadZone sets the clockwise and counter-clockwise dead bands in steps (persisted).
func (s *Servo) SetDeadZone(cw, ccw int) error {
	if err := s.Write(RegCWDeadZone, cw); err != nil {
		return err
	}
	return s.Write(RegCCWDeadZone, ccw)
}

// ---------------------------------------------------------------------------
// Control

// EnableTorque switches the motor output on (holding position) or off (free).
func (s *Servo) EnableTorque(on bool) error {
	v := 0
	if on {
		v = 1
	}
	return s.Write(RegTorqueEnable, v)
}

// TorqueEnabled reports whether the motor output is on.
func (s *Servo) TorqueEnabled() (bool, error) {
	v, err := s.Read(RegTorqueEnable)
	return v == 1, err
}

// SetTorqueLimit sets the runtime torque limit in % (0..100, not persisted).
func (s *Servo) SetTorqueLimit(percent float64) error {
	return s.Write(RegTorqueLimit, int(percent*10+0.5))
}

// SetAcceleration sets the acceleration in units of 100 step/s² (0 = maximum).
func (s *Servo) SetAcceleration(acc uint8) error { return s.Write(RegAcceleration, int(acc)) }

func moveData(pos, speed int, acc uint8) ([]byte, error) {
	p, err := RegGoalPosition.encode(pos)
	if err != nil {
		return nil, err
	}
	if speed < 0 || speed > RegGoalSpeed.Max {
		return nil, fmt.Errorf("st3215: speed %d out of range [0, %d]", speed, RegGoalSpeed.Max)
	}
	// acc, pos L/H, time L/H (0), speed L/H — the layout of WritePosEx.
	b := make([]byte, 7)
	b[0] = acc
	copy(b[1:3], p)
	putU16(b[5:], uint16(speed))
	return b, nil
}

// MoveTo commands an absolute position (steps) with a speed in step/s
// (0 = maximum) and acceleration in 100 step/s² (0 = maximum). It returns as
// soon as the command is accepted. In ModeStep use StepBy instead.
func (s *Servo) MoveTo(pos, speed int, acc uint8) error {
	data, err := moveData(mirrorPos(s.Mirrored(), pos), speed, acc)
	if err != nil {
		return err
	}
	_, err = s.bus.Write(s.id, RegAcceleration.Addr, data)
	return err
}

// MoveToDegrees is MoveTo with the target in degrees (0..360, or beyond in multi-turn).
func (s *Servo) MoveToDegrees(deg float64, speed int, acc uint8) error {
	return s.MoveTo(DegreesToSteps(deg), speed, acc)
}

// NearestEquivalent returns the position that points the same way as target
// (equal modulo one turn) and is closest to current, i.e. the end point of the
// shortest way round. Only meaningful in multi-turn mode; in single-turn mode
// the servo cannot cross the 0/4095 seam.
func NearestEquivalent(current, target int) int {
	want := ((target % StepsPerRev) + StepsPerRev) % StepsPerRev
	base := current - ((current%StepsPerRev)+StepsPerRev)%StepsPerRev
	best := base + want
	for _, c := range []int{best - StepsPerRev, best + StepsPerRev} {
		if abs(c-current) < abs(best-current) {
			best = c
		}
	}
	return best
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

// CircularDiff returns a - b as the shortest signed distance around one turn
// (-2048..2047). Use it to compare positions near the 0/4095 seam.
func CircularDiff(a, b int) int {
	d := ((a-b)%StepsPerRev + StepsPerRev) % StepsPerRev
	if d >= StepsPerRev/2 {
		d -= StepsPerRev
	}
	return d
}

// AbsolutePosition returns the present position including the turn count,
// in the same coordinates as goals.
//
// In multi-turn mode the ST3215 reports its position within one turn
// (0..4095) but interprets goals in its own, unbounded turn count, so the
// reported position can be whole turns away from where a goal is measured
// from. The goal register keeps that turn count, so the present position is
// taken as the equivalent position nearest the current goal. This is exact
// whenever the servo is within half a turn of its goal (always true after a
// move has finished, or during any move shorter than half a turn). In
// single-turn mode it is just Position.
func (s *Servo) AbsolutePosition() (int, error) {
	lo, hi, err := s.AngleLimits()
	if err != nil {
		return 0, err
	}
	cur, err := s.Position()
	if err != nil || lo != 0 || hi != 0 {
		return cur, err
	}
	goal, err := s.Read(RegGoalPosition)
	if err != nil {
		return 0, err
	}
	goal = mirrorPos(s.Mirrored(), goal)
	return goal + CircularDiff(cur, goal), nil
}

// ShortestGoal returns the goal that reaches the angle of pos (taken modulo
// one turn) the short way round from the present position, kept within
// [lo, hi] by going the other way if needed. Only multi-turn mode can cross
// the 0/4095 seam; in single-turn mode the goal is pos within 0..4095.
func (s *Servo) ShortestGoal(pos, lo, hi int) (int, error) {
	l, h, err := s.AngleLimits()
	if err != nil {
		return 0, err
	}
	if l != 0 || h != 0 {
		return ((pos % StepsPerRev) + StepsPerRev) % StepsPerRev, nil
	}
	cur, err := s.AbsolutePosition()
	if err != nil {
		return 0, err
	}
	return within(NearestEquivalent(cur, pos), lo, hi), nil
}

// MoveToShortest moves to the angle of pos the short way round (multi-turn
// mode; see ShortestGoal) and returns the goal it used.
func (s *Servo) MoveToShortest(pos, speed int, acc uint8) (int, error) {
	goal, err := s.ShortestGoal(pos, -MultiTurnLimit, MultiTurnLimit)
	if err != nil {
		return 0, err
	}
	return goal, s.MoveTo(goal, speed, acc)
}

// within shifts p by whole turns into [lo, hi] (if it fits).
func within(p, lo, hi int) int {
	for p > hi && p-StepsPerRev >= lo {
		p -= StepsPerRev
	}
	for p < lo && p+StepsPerRev <= hi {
		p += StepsPerRev
	}
	return p
}

// RegMoveTo stages a MoveTo that executes on the next Bus.Action, so several
// servos can start at exactly the same time.
func (s *Servo) RegMoveTo(pos, speed int, acc uint8) error {
	data, err := moveData(mirrorPos(s.Mirrored(), pos), speed, acc)
	if err != nil {
		return err
	}
	_, err = s.bus.RegWrite(s.id, RegAcceleration.Addr, data)
	return err
}

// StepBy moves a relative number of steps in ModeStep (sign = direction).
func (s *Servo) StepBy(steps, speed int, acc uint8) error {
	data, err := moveData(mirrorSign(s.Mirrored(), steps), speed, acc)
	if err != nil {
		return err
	}
	_, err = s.bus.Write(s.id, RegAcceleration.Addr, data)
	return err
}

// Stop holds the servo at its present position (ModePosition). In ModeWheel
// use SetWheelSpeed(0, acc); in ModePWM use SetPWM(0).
func (s *Servo) Stop() error {
	pos, err := s.AbsolutePosition() // with the turn count, so it never spins a turn
	if err != nil {
		return err
	}
	_, err = s.bus.Write(s.id, RegGoalPosition.Addr, mustEncode(RegGoalPosition, mirrorPos(s.Mirrored(), pos)))
	return err
}

// SetWheelSpeed sets the rotation speed in ModeWheel (step/s, sign = direction,
// 0 = stop) with an acceleration in 100 step/s².
func (s *Servo) SetWheelSpeed(speed int, acc uint8) error {
	sp, err := RegGoalSpeed.encode(mirrorSign(s.Mirrored(), speed))
	if err != nil {
		return err
	}
	if err := s.Write(RegAcceleration, int(acc)); err != nil {
		return err
	}
	_, err = s.bus.Write(s.id, RegGoalSpeed.Addr, sp)
	return err
}

// SetPWM sets the open-loop duty in ModePWM, -1000..1000 (0.1%, sign = direction).
func (s *Servo) SetPWM(duty int) error { return s.Write(RegGoalTime, mirrorSign(s.Mirrored(), duty)) }

func mustEncode(r Register, v int) []byte {
	b, err := r.encode(v)
	if err != nil {
		panic(err)
	}
	return b
}

// ---------------------------------------------------------------------------
// Waiting

// ErrFault is returned by wait helpers when the servo reports a fault.
var ErrFault = errors.New("st3215: servo fault")

// WaitOptions tunes WaitForPosition / WaitStopped.
type WaitOptions struct {
	Poll      time.Duration // polling interval (default 20ms)
	Tolerance int           // allowed position error in steps (default 2)
	// FailOn aborts waiting with ErrFault when any of these status bits is set.
	FailOn Status
}

func (o *WaitOptions) defaults() {
	if o.Poll == 0 {
		o.Poll = 20 * time.Millisecond
	}
	if o.Tolerance == 0 {
		o.Tolerance = 2
	}
}

// WaitForPosition polls until the servo has stopped within Tolerance of
// target, ctx is done, or a FailOn fault is reported. It returns the last
// feedback read.
func (s *Servo) WaitForPosition(ctx context.Context, target int, opt WaitOptions) (Feedback, error) {
	opt.defaults()
	return s.waitUntil(ctx, opt, func(f Feedback) bool {
		d := CircularDiff(f.Position, target) // reported positions wrap every turn
		if d < 0 {
			d = -d
		}
		return !f.Moving && d <= opt.Tolerance
	})
}

// WaitStopped polls until the Moving flag is clear on two consecutive reads.
func (s *Servo) WaitStopped(ctx context.Context, opt WaitOptions) (Feedback, error) {
	opt.defaults()
	still := 0
	return s.waitUntil(ctx, opt, func(f Feedback) bool {
		if f.Moving {
			still = 0
			return false
		}
		still++
		return still >= 2
	})
}

func (s *Servo) waitUntil(ctx context.Context, opt WaitOptions, done func(Feedback) bool) (Feedback, error) {
	t := time.NewTicker(opt.Poll)
	defer t.Stop()
	for {
		f, err := s.Feedback()
		if err != nil {
			return f, err
		}
		if f.Status&opt.FailOn != 0 {
			return f, fmt.Errorf("%w: servo %d: %s", ErrFault, s.id, f.Status)
		}
		if done(f) {
			return f, nil
		}
		select {
		case <-ctx.Done():
			return f, ctx.Err()
		case <-t.C:
		}
	}
}

// MoveToAndWait commands a move and blocks until the target is reached.
func (s *Servo) MoveToAndWait(ctx context.Context, pos, speed int, acc uint8, opt WaitOptions) (Feedback, error) {
	if err := s.MoveTo(pos, speed, acc); err != nil {
		return Feedback{}, err
	}
	return s.WaitForPosition(ctx, pos, opt)
}
