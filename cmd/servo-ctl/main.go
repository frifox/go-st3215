// Command servo-ctl is a web console for ST3215 servos built on the st3215
// package: a WebSocket server that streams servo telemetry and accepts
// control commands, plus a web page (web/index.html, embedded) to set up,
// monitor and drive the servos.
//
// The page walks through three steps: pick the driver board (serial port),
// scan it for servos, then monitor/control the servos that were found.
//
//	go install github.com/frifox/go-st3215/cmd/servo-ctl@latest
//
//	servo-ctl                               # choose the port in the browser
//	servo-ctl -port /dev/ttyACM0            # connect on startup (Linux)
//	servo-ctl -port /dev/cu.usbmodem1101    # connect on startup (macOS)
//	servo-ctl -sim 1,2,3                    # connect to simulated servos on startup
//
// Then open http://localhost:8080.
package main

import (
	"context"
	_ "embed"
	"errors"
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	st3215 "github.com/frifox/go-st3215"
)

//go:embed web/index.html
var indexHTML []byte

// simPortName is the pseudo port that selects the simulated driver board.
const simPortName = "sim"

// defaultConfigPath is servo-ctl/config.toml in the user's config directory
// (~/.config on Linux, ~/Library/Application Support on macOS).
func defaultConfigPath() string {
	dir, err := os.UserConfigDir()
	if err != nil {
		return "config.toml"
	}
	return filepath.Join(dir, "servo-ctl", "config.toml")
}

func main() {
	port := flag.String("port", os.Getenv("ST3215_PORT"), "serial device to connect to on startup (optional)")
	baud := flag.Int("baud", st3215.DefaultBaudRate, "bus baud rate used with -port")
	addr := flag.String("addr", "localhost:8080", "HTTP listen address")
	sim := flag.String("sim", "", "connect to simulated servos with these IDs on startup, e.g. 1,2,3")
	poll := flag.Duration("poll", 50*time.Millisecond, "telemetry polling interval")
	noSync := flag.Bool("nosync", false, "poll servos one by one instead of SYNC READ")
	cfgPath := flag.String("config", defaultConfigPath(), "settings file (servo names, mirroring, zero, ranges, groups), created on first change")
	flag.Parse()

	cfg, warnings, err := loadConfig(*cfgPath)
	if err != nil {
		log.Fatal(err)
	}
	for _, w := range warnings {
		log.Print(w)
	}
	log.Printf("config: %s (%d servo entries)", *cfgPath, len(cfg.all()))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	srv := &server{clients: map[*client]struct{}{}, cfg: cfg, poll: *poll, noSync: *noSync, simIDs: []uint8{1, 2, 3}}
	if *sim != "" {
		ids, err := parseIDs(*sim)
		if err != nil {
			log.Fatal(err)
		}
		srv.simIDs = ids
		*port = simPortName
	}
	if *port != "" {
		if err := srv.connect(*port, *baud); err != nil {
			log.Fatal(err)
		}
		go srv.scan(ctx, 0, st3215.MaxID)
	}
	defer srv.disconnect()
	go srv.pollLoop(ctx)

	mux := http.NewServeMux()
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Write(indexHTML)
	})
	mux.HandleFunc("/ws", srv.handleWS)
	hs := &http.Server{Addr: *addr, Handler: mux}
	go func() {
		<-ctx.Done()
		hs.Close()
	}()
	log.Printf("open http://%s", *addr)
	if err := hs.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
		log.Fatal(err)
	}
}

func parseIDs(s string) ([]uint8, error) {
	var ids []uint8
	for _, f := range strings.Split(s, ",") {
		v, err := strconv.Atoi(strings.TrimSpace(f))
		if err != nil || v < 0 || v > int(st3215.MaxID) {
			return nil, fmt.Errorf("invalid servo ID %q", f)
		}
		ids = append(ids, uint8(v))
	}
	return ids, nil
}
