// Command gosts-ctl is a web console for ST3215 servos built on the gosts
// package (Go ST Servo): a WebSocket server that streams servo telemetry and
// accepts control commands, plus a web page to set up, monitor and drive the
// servos.
//
// Packages: web (HTTP and WebSocket, the page in web/dist), board (driver
// board: ports, connection, scanning), servo (commands on the servo motors),
// servo-sim (a simulated board), internal (settings file, messages, shared
// helpers); this package ties them together (app*.go) and parses the flags.
//
// The page walks through three steps: pick the driver board (serial port),
// scan it for servos, then monitor/control the servos that were found.
//
//	go install github.com/frifox/gosts/cmd/gosts-ctl@latest
//
//	gosts-ctl                               # choose the port in the browser
//	gosts-ctl -port /dev/ttyACM0            # connect on startup (Linux)
//	gosts-ctl -port /dev/cu.usbmodem1101    # connect on startup (macOS)
//	gosts-ctl -sim 1,2,3                    # connect to simulated servos on startup
//
// Then open http://localhost:8080.
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/frifox/gosts"
	"github.com/frifox/gosts/cmd/gosts-ctl/board"
	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
)

// defaultConfigPath is gosts-ctl/config.toml in the user's config directory
// (~/.config on Linux, ~/Library/Application Support on macOS).
func defaultConfigPath() string {
	dir, err := os.UserConfigDir()
	if err != nil {
		return "config.toml"
	}
	return filepath.Join(dir, "gosts-ctl", "config.toml")
}

func main() {
	port := flag.String("port", os.Getenv("ST3215_PORT"), "serial device to connect to on startup (optional)")
	baud := flag.Int("baud", gosts.DefaultBaudRate, "bus baud rate used with -port")
	addr := flag.String("addr", "localhost:8080", "HTTP listen address")
	sim := flag.String("sim", "", "connect to simulated servos with these IDs on startup, e.g. 1,2,3")
	poll := flag.Duration("poll", 50*time.Millisecond, "telemetry polling interval")
	noSync := flag.Bool("nosync", false, "poll servos one by one instead of SYNC READ")
	cfgPath := flag.String("config", defaultConfigPath(), "settings file (servo names, mirroring, zero, ranges, groups), created on first change")
	flag.Parse()

	cfg, warnings, err := internal.LoadConfig(*cfgPath)
	if err != nil {
		log.Fatal(err)
	}
	for _, w := range warnings {
		log.Print(w)
	}
	log.Printf("config: %s (%d servo entries)", *cfgPath, len(cfg.All()))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	simIDs := []uint8{1, 2, 3}
	if *sim != "" {
		if simIDs, err = parseIDs(*sim); err != nil {
			log.Fatal(err)
		}
		*port = board.SimPort
	}
	if err := newApp(cfg, simIDs, *poll, *noSync).run(ctx, *addr, *port, *baud); err != nil {
		log.Fatal(err)
	}
}

func parseIDs(s string) ([]uint8, error) {
	var ids []uint8
	for _, f := range strings.Split(s, ",") {
		v, err := strconv.Atoi(strings.TrimSpace(f))
		if err != nil || v < 0 || v > int(gosts.MaxID) {
			return nil, fmt.Errorf("invalid servo ID %q", f)
		}
		ids = append(ids, uint8(v))
	}
	return ids, nil
}
