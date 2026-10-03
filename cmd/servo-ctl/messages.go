package main

import (
	_ "embed"

	st3215 "github.com/frifox/go-st3215"
)

// Messages exchanged with the browser over the WebSocket.

type stateMsg struct {
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
	Groups    []groupInfo        `json:"groups"`
}

type portsMsg struct {
	Type  string     `json:"type"`
	Ports []portInfo `json:"ports"`
	Sim   string     `json:"sim"` // description of the simulated board
}

type scanProgressMsg struct {
	Type    string `json:"type"`
	Done    int    `json:"done"`
	Total   int    `json:"total"`
	Current int    `json:"current"`
	Found   []int  `json:"found"`
}

type feedbackMsg struct {
	Type   string                 `json:"type"`
	Time   int64                  `json:"time"`
	Servos map[string]servoState  `json:"servos"`
	Groups map[string]groupHealth `json:"groups"`
}

type logMsg struct {
	Type    string `json:"type"`
	Level   string `json:"level"`
	Message string `json:"message"`
}

type resultMsg struct {
	Type  string `json:"type"`
	Seq   int    `json:"seq"`
	OK    bool   `json:"ok"`
	Error string `json:"error,omitempty"`
	Goal  *int   `json:"goal,omitempty"` // for "angle" and "jog": the goal the server chose
}

type registerInfo struct {
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

type helloMsg struct {
	Type     string `json:"type"`
	ClientID int    `json:"clientId"`
}

type configMsg struct {
	Type      string         `json:"type"`
	ID        uint8          `json:"id"`
	Origin    int            `json:"origin"` // client whose change triggered this; 0 = plain request, -1 = several
	Config    st3215.Config  `json:"config"`
	Registers []registerInfo `json:"registers"`
	Tried     map[string]int `json:"tried"` // tuning values tried but not saved
	Saved     map[string]int `json:"saved"` // for each tried value, the value saved on the servo
}

// request is a command from the browser. Only the fields relevant to Type
// are set.
type request struct {
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
	Values   []regValue `json:"values"`  // for "tune"
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

type regValue struct {
	Register string `json:"register"`
	Value    int    `json:"value"`
}
