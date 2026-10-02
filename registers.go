package st3215

import (
	"fmt"
	"strings"
)

// Area tells where a register lives. EEPROM values survive power cycles (when
// the EEPROM lock is open); SRAM values reset on power-up.
type Area uint8

const (
	EEPROM Area = iota
	SRAM
)

func (a Area) String() string {
	if a == EEPROM {
		return "EEPROM"
	}
	return "SRAM"
}

// Register describes one entry of the ST3215 memory table.
type Register struct {
	Name     string
	Addr     uint8
	Size     uint8 // 1 or 2 bytes (2-byte values are little-endian)
	SignBit  int8  // bit holding the sign in sign-magnitude values, or -1 if unsigned
	Area     Area
	ReadOnly bool
	Min, Max int    // accepted range for writes
	Unit     string // human readable unit of the raw value
}

// Signed reports whether the register uses a sign-magnitude encoding.
func (r Register) Signed() bool { return r.SignBit >= 0 }

func (r Register) String() string { return r.Name }

// Value decodes the register from a full memory table dump as returned by
// Servo.ReadMemory (index = address).
func (r Register) Value(mem []byte) int { return r.decode(mem[r.Addr:]) }

func (r Register) decode(b []byte) int {
	var raw uint16
	if r.Size == 2 {
		raw = getU16(b)
	} else {
		raw = uint16(b[0])
	}
	if r.Signed() {
		return decodeSignMag(raw, uint(r.SignBit))
	}
	return int(raw)
}

func (r Register) encode(v int) ([]byte, error) {
	if v < r.Min || v > r.Max {
		return nil, fmt.Errorf("st3215: %s value %d out of range [%d, %d]", r.Name, v, r.Min, r.Max)
	}
	var raw uint16
	if r.Signed() {
		raw = encodeSignMag(v, uint(r.SignBit))
	} else {
		raw = uint16(v)
	}
	if r.Size == 2 {
		b := make([]byte, 2)
		putU16(b, raw)
		return b, nil
	}
	return []byte{byte(raw)}, nil
}

// Memory table of the ST3215 (STS3215, firmware 3.x). Addresses, ranges and
// units come from sts3215_memory_table.xlsx shipped by the manufacturer.
var (
	// ---- EEPROM, read-only ----
	RegFirmwareMajor = Register{Name: "FirmwareMajor", Addr: 0, Size: 1, SignBit: -1, Area: EEPROM, ReadOnly: true, Max: 255}
	RegFirmwareMinor = Register{Name: "FirmwareMinor", Addr: 1, Size: 1, SignBit: -1, Area: EEPROM, ReadOnly: true, Max: 255}
	RegServoMajor    = Register{Name: "ServoMajor", Addr: 3, Size: 1, SignBit: -1, Area: EEPROM, ReadOnly: true, Max: 255}
	RegServoMinor    = Register{Name: "ServoMinor", Addr: 4, Size: 1, SignBit: -1, Area: EEPROM, ReadOnly: true, Max: 255}

	// ---- EEPROM, read/write ----
	RegID                = Register{Name: "ID", Addr: 5, Size: 1, SignBit: -1, Area: EEPROM, Max: 253}
	RegBaudRate          = Register{Name: "BaudRate", Addr: 6, Size: 1, SignBit: -1, Area: EEPROM, Max: 7, Unit: "BaudRate index"}
	RegReturnDelay       = Register{Name: "ReturnDelay", Addr: 7, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "2us"}
	RegResponseLevel     = Register{Name: "ResponseLevel", Addr: 8, Size: 1, SignBit: -1, Area: EEPROM, Max: 1}
	RegMinAngleLimit     = Register{Name: "MinAngleLimit", Addr: 9, Size: 2, SignBit: -1, Area: EEPROM, Max: 4094, Unit: "step"}
	RegMaxAngleLimit     = Register{Name: "MaxAngleLimit", Addr: 11, Size: 2, SignBit: -1, Area: EEPROM, Max: 4095, Unit: "step"}
	RegMaxTemperature    = Register{Name: "MaxTemperature", Addr: 13, Size: 1, SignBit: -1, Area: EEPROM, Max: 100, Unit: "°C"}
	RegMaxVoltage        = Register{Name: "MaxVoltage", Addr: 14, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "0.1V"}
	RegMinVoltage        = Register{Name: "MinVoltage", Addr: 15, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "0.1V"}
	RegMaxTorque         = Register{Name: "MaxTorque", Addr: 16, Size: 2, SignBit: -1, Area: EEPROM, Max: 1000, Unit: "0.1%"}
	RegPhase             = Register{Name: "Phase", Addr: 18, Size: 1, SignBit: -1, Area: EEPROM, Max: 254}
	RegUnloadCondition   = Register{Name: "UnloadCondition", Addr: 19, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "Status mask"}
	RegLEDAlarmCondition = Register{Name: "LEDAlarmCondition", Addr: 20, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "Status mask"}
	RegPositionP         = Register{Name: "PositionP", Addr: 21, Size: 1, SignBit: -1, Area: EEPROM, Max: 254}
	RegPositionD         = Register{Name: "PositionD", Addr: 22, Size: 1, SignBit: -1, Area: EEPROM, Max: 254}
	RegPositionI         = Register{Name: "PositionI", Addr: 23, Size: 1, SignBit: -1, Area: EEPROM, Max: 254}
	RegMinStartForce     = Register{Name: "MinStartForce", Addr: 24, Size: 2, SignBit: -1, Area: EEPROM, Max: 1000, Unit: "0.1%"}
	RegCWDeadZone        = Register{Name: "CWDeadZone", Addr: 26, Size: 1, SignBit: -1, Area: EEPROM, Max: 32, Unit: "step"}
	RegCCWDeadZone       = Register{Name: "CCWDeadZone", Addr: 27, Size: 1, SignBit: -1, Area: EEPROM, Max: 32, Unit: "step"}
	RegProtectionCurrent = Register{Name: "ProtectionCurrent", Addr: 28, Size: 2, SignBit: -1, Area: EEPROM, Max: 511, Unit: "6.5mA"}
	RegAngularResolution = Register{Name: "AngularResolution", Addr: 30, Size: 1, SignBit: -1, Area: EEPROM, Min: 1, Max: 100}
	RegPositionOffset    = Register{Name: "PositionOffset", Addr: 31, Size: 2, SignBit: 11, Area: EEPROM, Min: -2047, Max: 2047, Unit: "step"}
	RegMode              = Register{Name: "Mode", Addr: 33, Size: 1, SignBit: -1, Area: EEPROM, Max: 3, Unit: "Mode"}
	RegProtectiveTorque  = Register{Name: "ProtectiveTorque", Addr: 34, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "1%"}
	RegProtectionTime    = Register{Name: "ProtectionTime", Addr: 35, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "10ms"}
	RegOverloadTorque    = Register{Name: "OverloadTorque", Addr: 36, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "1%"}
	RegSpeedP            = Register{Name: "SpeedP", Addr: 37, Size: 1, SignBit: -1, Area: EEPROM, Max: 254}
	RegOverCurrentTime   = Register{Name: "OverCurrentTime", Addr: 38, Size: 1, SignBit: -1, Area: EEPROM, Max: 254, Unit: "10ms"}
	RegSpeedI            = Register{Name: "SpeedI", Addr: 39, Size: 1, SignBit: -1, Area: EEPROM, Max: 254}

	// ---- SRAM, read/write ----
	RegTorqueEnable = Register{Name: "TorqueEnable", Addr: 40, Size: 1, SignBit: -1, Area: SRAM, Max: 128}
	RegAcceleration = Register{Name: "Acceleration", Addr: 41, Size: 1, SignBit: -1, Area: SRAM, Max: 254, Unit: "100step/s²"}
	RegGoalPosition = Register{Name: "GoalPosition", Addr: 42, Size: 2, SignBit: 15, Area: SRAM, Min: -32766, Max: 32766, Unit: "step"}
	// RegGoalTime doubles as the PWM output in ModePWM (sign bit: see PWMSignBit).
	RegGoalTime    = Register{Name: "GoalTime", Addr: 44, Size: 2, SignBit: PWMSignBit, Area: SRAM, Min: -1000, Max: 1000, Unit: "0.1%"}
	RegGoalSpeed   = Register{Name: "GoalSpeed", Addr: 46, Size: 2, SignBit: 15, Area: SRAM, Min: -32767, Max: 32767, Unit: "step/s"}
	RegTorqueLimit = Register{Name: "TorqueLimit", Addr: 48, Size: 2, SignBit: -1, Area: SRAM, Max: 1000, Unit: "0.1%"}
	RegLock        = Register{Name: "Lock", Addr: 55, Size: 1, SignBit: -1, Area: SRAM, Max: 1}

	// ---- SRAM, read-only ----
	RegPresentPosition    = Register{Name: "PresentPosition", Addr: 56, Size: 2, SignBit: 15, Area: SRAM, ReadOnly: true, Unit: "step"}
	RegPresentSpeed       = Register{Name: "PresentSpeed", Addr: 58, Size: 2, SignBit: 15, Area: SRAM, ReadOnly: true, Unit: "step/s"}
	RegPresentLoad        = Register{Name: "PresentLoad", Addr: 60, Size: 2, SignBit: 10, Area: SRAM, ReadOnly: true, Unit: "0.1%"}
	RegPresentVoltage     = Register{Name: "PresentVoltage", Addr: 62, Size: 1, SignBit: -1, Area: SRAM, ReadOnly: true, Unit: "0.1V"}
	RegPresentTemperature = Register{Name: "PresentTemperature", Addr: 63, Size: 1, SignBit: -1, Area: SRAM, ReadOnly: true, Unit: "°C"}
	RegRegWriteFlag       = Register{Name: "RegWriteFlag", Addr: 64, Size: 1, SignBit: -1, Area: SRAM, ReadOnly: true}
	RegStatus             = Register{Name: "Status", Addr: 65, Size: 1, SignBit: -1, Area: SRAM, ReadOnly: true, Unit: "Status mask"}
	RegMoving             = Register{Name: "Moving", Addr: 66, Size: 1, SignBit: -1, Area: SRAM, ReadOnly: true}
	RegPresentCurrent     = Register{Name: "PresentCurrent", Addr: 69, Size: 2, SignBit: 15, Area: SRAM, ReadOnly: true, Unit: "6.5mA"}
)

// PWMSignBit is the direction bit of the GoalTime register when it is used as
// the PWM duty in ModePWM. The vendor memory table text says "BIT11"; the
// vendor SC-series library and the PresentLoad register use bit 10. Bit 10 is
// used here since the duty never exceeds 1000 (< 1024). Verify on hardware.
const PWMSignBit = 10

// Registers lists every register of the memory table in address order.
var Registers = []Register{
	RegFirmwareMajor, RegFirmwareMinor, RegServoMajor, RegServoMinor,
	RegID, RegBaudRate, RegReturnDelay, RegResponseLevel,
	RegMinAngleLimit, RegMaxAngleLimit, RegMaxTemperature, RegMaxVoltage, RegMinVoltage,
	RegMaxTorque, RegPhase, RegUnloadCondition, RegLEDAlarmCondition,
	RegPositionP, RegPositionD, RegPositionI, RegMinStartForce,
	RegCWDeadZone, RegCCWDeadZone, RegProtectionCurrent, RegAngularResolution,
	RegPositionOffset, RegMode, RegProtectiveTorque, RegProtectionTime, RegOverloadTorque,
	RegSpeedP, RegOverCurrentTime, RegSpeedI,
	RegTorqueEnable, RegAcceleration, RegGoalPosition, RegGoalTime, RegGoalSpeed,
	RegTorqueLimit, RegLock,
	RegPresentPosition, RegPresentSpeed, RegPresentLoad, RegPresentVoltage,
	RegPresentTemperature, RegRegWriteFlag, RegStatus, RegMoving, RegPresentCurrent,
}

// RegisterByName finds a register by name (case-insensitive).
func RegisterByName(name string) (Register, bool) {
	for _, r := range Registers {
		if strings.EqualFold(r.Name, name) {
			return r, true
		}
	}
	return Register{}, false
}

// memoryTableSize covers addresses 0..70 (through PresentCurrent).
const memoryTableSize = 71

// Mode is the operating mode (register 33).
type Mode uint8

const (
	// ModePosition: absolute position servo (default). With both angle limits
	// set to 0 the servo works in multi-turn mode (goal ±30719 steps, ±7.5 turns).
	ModePosition Mode = 0
	// ModeWheel: closed-loop constant speed (continuous rotation) driven by GoalSpeed.
	ModeWheel Mode = 1
	// ModePWM: open-loop duty cycle driven by GoalTime (-1000..1000).
	ModePWM Mode = 2
	// ModeStep: relative stepper mode, GoalPosition is a relative step count.
	ModeStep Mode = 3
)

func (m Mode) String() string {
	switch m {
	case ModePosition:
		return "position"
	case ModeWheel:
		return "wheel"
	case ModePWM:
		return "pwm"
	case ModeStep:
		return "step"
	}
	return fmt.Sprintf("mode(%d)", uint8(m))
}

// BaudRate is the baud rate index stored in register 6.
type BaudRate uint8

const (
	Baud1M     BaudRate = 0 // factory default
	Baud500K   BaudRate = 1
	Baud250K   BaudRate = 2
	Baud128K   BaudRate = 3
	Baud115200 BaudRate = 4
	Baud76800  BaudRate = 5
	Baud57600  BaudRate = 6
	Baud38400  BaudRate = 7
)

var baudValues = [...]int{1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400}

// BitsPerSecond returns the serial speed for the index, or 0 if unknown.
func (b BaudRate) BitsPerSecond() int {
	if int(b) < len(baudValues) {
		return baudValues[b]
	}
	return 0
}

func (b BaudRate) String() string { return fmt.Sprintf("%d", b.BitsPerSecond()) }

// BaudRateFor returns the index for a serial speed in bits per second.
func BaudRateFor(bps int) (BaudRate, bool) {
	for i, v := range baudValues {
		if v == bps {
			return BaudRate(i), true
		}
	}
	return 0, false
}
