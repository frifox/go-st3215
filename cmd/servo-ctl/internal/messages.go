package internal

import (
	"github.com/frifox/gosts"
	"github.com/frifox/gosts/autotune"
)

// Messages exchanged with the browser over the WebSocket.

// StateMsg is the connection, scan and settings state shared by all windows.
type StateMsg struct {
	Type      string             `json:"type"`
	Connected bool               `json:"connected"`
	Port      string             `json:"port"`
	Baud      int                `json:"baud"`
	Scanning  bool               `json:"scanning"`
	Scanned   bool               `json:"scanned"`
	IDs       []int              `json:"ids"` // not []uint8: encoding/json would emit base64
	Mirrored  []int              `json:"mirrored"`
	Signed    []int              `json:"signed"`  // servos whose angles are shown as -180..180
	Names     map[string]string  `json:"names"`   // servo ID -> name from config.toml
	Colors    map[string]string  `json:"colors"`  // servo ID -> color override from config.toml
	Zeros     map[string]float64 `json:"zeros"`   // servo ID -> virtual 0° in degrees
	DialUps   map[string]float64 `json:"dialUps"` // servo ID -> factory-scale angle that is physically up
	Ranges    map[string][]int   `json:"ranges"`  // servo ID -> motion range [lo, hi], encoder-scale steps
	Groups    []GroupInfo        `json:"groups"`
}

type PortsMsg struct {
	Type  string     `json:"type"`
	Ports []PortInfo `json:"ports"`
	Sim   string     `json:"sim"` // description of the simulated board
}

type ScanProgressMsg struct {
	Type    string `json:"type"`
	Done    int    `json:"done"`
	Total   int    `json:"total"`
	Current int    `json:"current"`
	Found   []int  `json:"found"`
}

type FeedbackMsg struct {
	Type   string                 `json:"type"`
	Time   int64                  `json:"time"`
	Servos map[string]ServoState  `json:"servos"`
	Groups map[string]GroupHealth `json:"groups"`
}

type LogMsg struct {
	Type    string `json:"type"`
	Level   string `json:"level"`
	Message string `json:"message"`
}

type ResultMsg struct {
	Type  string `json:"type"`
	Seq   int    `json:"seq"`
	OK    bool   `json:"ok"`
	Error string `json:"error,omitempty"`
	Goal  *int   `json:"goal,omitempty"` // for "angle" and "jog": the goal the server chose
}

type RegisterInfo struct {
	Name     string `json:"name"`
	Addr     uint8  `json:"addr"`
	Size     uint8  `json:"size"`
	Area     string `json:"area"`
	ReadOnly bool   `json:"readOnly"`
	Min      int    `json:"min"`
	Max      int    `json:"max"`
	Unit     string `json:"unit"`
	Value    int    `json:"value"`
}

type HelloMsg struct {
	Type     string `json:"type"`
	ClientID int    `json:"clientId"`
}

type ConfigMsg struct {
	Type      string         `json:"type"`
	ID        uint8          `json:"id"`
	Origin    int            `json:"origin"` // client whose change triggered this; 0 = plain Request, -1 = several
	Config    gosts.Config   `json:"config"`
	Registers []RegisterInfo `json:"registers"`
	Tried     map[string]int `json:"tried"` // tuning values tried but not saved
	Saved     map[string]int `json:"saved"` // for each tried value, the value saved on the servo
}

// Request is a command from the browser. Only the fields relevant to Type
// are set.
type Request struct {
	Seq      int        `json:"seq"`
	Type     string     `json:"type"`
	ID       uint8      `json:"id"`
	Port     string     `json:"port"`
	Baud     int        `json:"baud"`
	First    uint8      `json:"first"`
	Last     uint8      `json:"last"`
	Position int        `json:"position"`
	Speed    int        `json:"speed"`
	Acc      uint8      `json:"acc"`
	On       bool       `json:"on"`
	Mode     int        `json:"mode"`
	Duty     int        `json:"duty"`
	NewID    uint8      `json:"newId"`
	Register string     `json:"register"`
	Value    int        `json:"value"`
	Percent  float64    `json:"percent"`
	Name     string     `json:"name"`    // for "rename"
	Values   []RegValue `json:"values"`  // for "tune"
	Save     bool       `json:"save"`    // for "tune"/"copyTuning": persist instead of until power-off
	Color    string     `json:"color"`   // for "color" and "servoEdit"
	Zero     float64    `json:"zero"`    // for "servoEdit": virtual 0° in degrees
	DialUp   float64    `json:"dialUp"`  // for "servoEdit": factory-scale angle that is physically up
	Signed   bool       `json:"signed"`  // for "servoEdit": show angles as -180..180
	Degrees  float64    `json:"degrees"` // for "zeroAt": where 0° goes, in degrees on the encoder scale (offset 0)
	MinDeg   float64    `json:"minDeg"`  // for "limits": range start, in degrees as the console shows them
	MaxDeg   float64    `json:"maxDeg"`  // for "limits": range end (clockwise from MinDeg)
	// Groups: Group targets a command at a group; the rest is for "groupSave".
	Group        string  `json:"group"`
	Members      []int   `json:"members"`
	MaxSpread    int     `json:"maxSpread"`
	MaxFightLoad float64 `json:"maxFightLoad"`
	OnFight      string  `json:"onFight"`
	// Auto-tune
	Amplitude float64 `json:"amplitude"` // degrees either side of the start
	Tolerance int     `json:"tolerance"` // steps
}

type RegValue struct {
	Register string `json:"register"`
	Value    int    `json:"value"`
}

// PortInfo describes a serial port for the driver board picker.
type PortInfo struct {
	Name        string `json:"name"`
	Description string `json:"description"`
	USB         bool   `json:"usb"`
	VID         string `json:"vid,omitempty"`
	PID         string `json:"pid,omitempty"`
	Serial      string `json:"serial,omitempty"`
	Likely      bool   `json:"likely"` // looks like a Bus Servo Adapter (USB-UART bridge)
}

// ServoState is one servo's telemetry in a FeedbackMsg.
type ServoState struct {
	gosts.Feedback
	StatusText string `json:"statusText"`
	Error      string `json:"error,omitempty"`
}

// ToState turns a feedback read into a ServoState.
func ToState(f gosts.Feedback, err error) ServoState {
	st := ServoState{Feedback: f, StatusText: f.Status.String()}
	if err != nil {
		st.Error = err.Error()
	}
	return st
}

// GroupInfo describes a group in a StateMsg.
type GroupInfo struct {
	Key          string  `json:"key"`
	Name         string  `json:"name"`
	Members      []int   `json:"members"` // leader first
	MaxSpread    int     `json:"maxSpread"`
	MaxFightLoad float64 `json:"maxFightLoad"`
	OnFight      string  `json:"onFight"`
}

// GroupHealth is sent with every telemetry frame.
type GroupHealth struct {
	Spread   int    `json:"spread"`   // steps between the members' logical positions
	Fighting bool   `json:"fighting"` // members push in opposite directions
	Tripped  bool   `json:"tripped"`  // torque was cut by fight protection
	Problem  string `json:"problem,omitempty"`
}

// AutotuneMsg reports an auto-tune run (progress, result, save/revert).
type AutotuneMsg struct {
	Type     string             `json:"type"` // "autotune"
	Key      string             `json:"key"`
	Label    string             `json:"label"`
	Members  []int              `json:"members"`
	Running  bool               `json:"running"`
	Progress *autotune.Progress `json:"progress,omitempty"`
	Result   *autotune.Result   `json:"result,omitempty"`
	Error    string             `json:"error,omitempty"`
	Saved    bool               `json:"saved,omitempty"`
	Reverted bool               `json:"reverted,omitempty"`
}
