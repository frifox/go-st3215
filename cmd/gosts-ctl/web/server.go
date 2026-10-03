// Package web is gosts-ctl's HTTP side: it serves the web page and keeps
// one WebSocket per browser window, passing requests to a Handler and
// broadcasting messages to every window. The page is in dist.
package web

import (
	"context"
	_ "embed"
	"encoding/json"
	"errors"
	"log"
	"net/http"
	"sync"
	"sync/atomic"
	"time"

	"github.com/frifox/gosts/cmd/gosts-ctl/internal"
	"github.com/gorilla/websocket"
)

//go:embed dist/index.html
var indexHTML []byte

// Handler runs the requests of the browser windows.
type Handler interface {
	// Connected is called when a window connects, to send it the current state.
	Connected(c *Client)
	// Handle runs one request. The requests of one window are handled in
	// order; Handle may run long work in the background.
	Handle(c *Client, req internal.Request)
}

// Server serves the page and the WebSocket endpoint.
type Server struct {
	h       Handler
	mu      sync.Mutex
	clients map[*Client]struct{}
}

// New returns a server that passes requests to h.
func New(h Handler) *Server { return &Server{h: h, clients: map[*Client]struct{}{}} }

// Routes returns the HTTP handler: the page at / and the WebSocket at /ws.
func (s *Server) Routes() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Write(indexHTML)
	})
	mux.HandleFunc("/ws", s.handleWS)
	return mux
}

// ListenAndServe serves on addr until ctx is done.
func (s *Server) ListenAndServe(ctx context.Context, addr string) error {
	hs := &http.Server{Addr: addr, Handler: s.Routes()}
	go func() {
		<-ctx.Done()
		hs.Close()
	}()
	log.Printf("open http://%s", addr)
	if err := hs.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
		return err
	}
	return nil
}

// Client is one browser window.
type Client struct {
	id   int // sent to the browser so it can tell its own changes from others'
	conn *websocket.Conn
	send chan any
}

var lastClientID atomic.Int64

// ID identifies the window.
func (c *Client) ID() int { return c.id }

// Push queues a message without blocking; it is dropped if the client is too slow.
func (c *Client) Push(msg any) {
	select {
	case c.send <- msg:
	default:
	}
}

// Broadcast sends msg to every window.
func (s *Server) Broadcast(msg any) { s.BroadcastExcept(nil, msg) }

// BroadcastExcept sends msg to every window but skip.
func (s *Server) BroadcastExcept(skip *Client, msg any) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for c := range s.clients {
		if c != skip {
			c.Push(msg)
		}
	}
}

var upgrader = websocket.Upgrader{
	// gosts-ctl is meant for localhost; accept any origin.
	CheckOrigin: func(*http.Request) bool { return true },
}

func (s *Server) handleWS(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		return
	}
	c := &Client{id: int(lastClientID.Add(1)), conn: conn, send: make(chan any, 256)}
	s.mu.Lock()
	s.clients[c] = struct{}{}
	s.mu.Unlock()
	c.Push(internal.HelloMsg{Type: "hello", ClientID: c.id})
	s.h.Connected(c)

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

	for {
		_, data, err := conn.ReadMessage()
		if err != nil {
			return
		}
		var req internal.Request
		if err := json.Unmarshal(data, &req); err != nil {
			c.Push(internal.ResultMsg{Type: "result", Seq: req.Seq, Error: "bad request: " + err.Error()})
			continue
		}
		s.h.Handle(c, req)
	}
}
