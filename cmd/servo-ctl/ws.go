package main

import (
	_ "embed"
	"encoding/json"
	"log"
	"net/http"
	"time"

	"github.com/gorilla/websocket"
)

// The WebSocket endpoint: one connection per browser window.

var upgrader = websocket.Upgrader{
	// servo-ctl is meant for localhost; accept any origin.
	CheckOrigin: func(*http.Request) bool { return true },
}

func (s *server) handleWS(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		return
	}
	c := &client{id: int(lastClientID.Add(1)), conn: conn, send: make(chan any, 256)}
	s.mu.Lock()
	s.clients[c] = struct{}{}
	s.mu.Unlock()
	c.push(helloMsg{Type: "hello", ClientID: c.id})
	c.push(s.stateMsg())
	if m := s.autotuneState(); m != nil {
		c.push(*m)
	}

	done := make(chan struct{})
	go func() { // writer
		defer conn.Close()
		for {
			select {
			case msg := <-c.send:
				conn.SetWriteDeadline(time.Now().Add(5 * time.Second))
				if err := conn.WriteJSON(msg); err != nil {
					return
				}
			case <-done:
				return
			}
		}
	}()

	defer func() {
		s.mu.Lock()
		delete(s.clients, c)
		s.mu.Unlock()
		close(done)
	}()

	// Commands of one client run in order, so rapid moves can't overtake each
	// other. Scans run in the background so they can be cancelled.
	for {
		_, data, err := conn.ReadMessage()
		if err != nil {
			return
		}
		var req request
		if err := json.Unmarshal(data, &req); err != nil {
			c.push(resultMsg{Type: "result", Seq: req.Seq, Error: "bad request: " + err.Error()})
			continue
		}
		if req.Type == "scan" || req.Type == "autotune" { // long-running
			go s.handle(c, req)
			continue
		}
		s.handle(c, req)
	}
}

func (s *server) handle(c *client, req request) {
	var goal *int
	var err error
	switch req.Type {
	case "angle", "jog":
		var g int
		if g, err = s.computedMove(req); err == nil {
			goal = &g
		}
	default:
		err = s.exec(c, req)
	}
	res := resultMsg{Type: "result", Seq: req.Seq, OK: err == nil, Goal: goal}
	if err != nil {
		res.Error = err.Error()
		log.Printf("%s: %v", req.Type, err)
	}
	c.push(res)
	if err == nil {
		s.afterChange(c, req)
	}
}
