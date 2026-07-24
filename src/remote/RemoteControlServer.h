#pragma once

// RemoteControlServer
//
// A tiny out-of-process control bridge for Splatshop. It runs a background TCP
// thread that speaks a newline-delimited JSON-RPC protocol with a Python
// FastAPI frontend (see remote_api/). Every accepted request is dispatched onto
// the main render thread via the existing EventQueue / schedule() mechanism
// (include/unsuck.hpp), because the GLFW/CUDA/GL contexts are bound to the
// thread that owns the render loop. Each request carries an `id`; the server
// fulfills it with a std::promise<json> that the main thread sets, and the
// socket thread blocks on the matching future before writing the response line.
//
// Protocol (one JSON object per line, UTF-8, '\n' terminated):
//   request : {"id": <int>, "cmd": "<name>", "args": { ... }}
//   response: {"id": <int>, "ok": true,  "data": { ... }}
//            | {"id": <int>, "ok": false, "error": "<message>"}
//
// The command set mirrors the HTTP surface documented in docs/remote_api.md:
//   health, camera.* , mouse.*, keyboard.*, scene.nodes, motion.* .
//
// This header is intentionally self-contained: include it from main.cpp and
// call RemoteControlServer::start(port). No teardown is needed (the listener
// thread is detached and dies with the process).

#include <atomic>
#include <cstdint>
#include <string>

namespace remote {

class RemoteControlServer {
public:
	// Start the listener thread bound to 127.0.0.1:`port`. Returns true if the
	// listening socket was created. Safe to call once, from the main thread,
	// after SplatEditor::setup() and after the EventQueue is initialized.
	static bool start(uint16_t port = 7654);

	// True once the listener thread is accepting connections.
	static bool isRunning() { return s_running.load(std::memory_order_relaxed); }

private:
	static std::atomic<bool> s_running;

	// Platform-agnostic entry: accept loop, per-connection handler.
	static void listenerMain(uint16_t port);
	// Read one request, dispatch it, write one response. Returns false to close
	// the connection.
	static bool handleConnection(int clientSock);
};

} // namespace remote
