"""TCP JSON-RPC client for the Splatshop C++ RemoteControlServer.

Protocol (one JSON object per line, UTF-8, '\\n' terminated):
    request : {"id": <int>, "cmd": "<name>", "args": { ... }}
    response: {"id": <int>, "ok": true,  "data": { ... }}
             | {"id": <int>, "ok": false, "error": "<message>"}

This module is connection-agnostic: it opens a fresh short-lived socket per
request, which keeps the bridge stateless and avoids races between concurrent
HTTP requests sharing one TCP stream. The C++ server spawns a detached thread
per accepted connection, so this is cheap.
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Any, Dict, Optional

from . import config

# Module-level auto-incrementing request id, guarded by a lock so concurrent
# FastAPI workers don't collide.
_id_lock = threading.Lock()
_id_counter = 0


def _next_id() -> int:
    global _id_counter
    with _id_lock:
        _id_counter += 1
        return _id_counter


class BridgeError(Exception):
    """Raised when the C++ bridge returns an ``ok: false`` response."""

    def __init__(self, message: str, cmd: str = ""):
        super().__init__(message)
        self.cmd = cmd


class BridgeUnavailable(ConnectionError):
    """Raised when the TCP connection to the bridge cannot be established."""


def request(cmd: str, args: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None) -> Dict[str, Any]:
    """Send one command to the bridge and return its ``data`` payload.

    Raises:
        BridgeUnavailable: the C++ server is not reachable.
        BridgeError: the server rejected the command (ok == false).
    """
    payload = {"id": _next_id(), "cmd": cmd, "args": args or {}}
    line = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")

    host, port = config.BRIDGE_HOST, config.BRIDGE_PORT
    tout = config.BRIDGE_TIMEOUT if timeout is None else timeout

    try:
        with socket.create_connection((host, port), timeout=tout) as sock:
            sock.settimeout(tout)
            sock.sendall(line)
            # Read until newline.
            buf = bytearray()
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
                if b"\n" in chunk:
                    break
    except (OSError, socket.timeout) as e:
        raise BridgeUnavailable(
            f"cannot reach Splatshop bridge at {host}:{port}: {e}"
        ) from e

    if not buf:
        raise BridgeUnavailable("bridge closed connection without a response")

    raw = bytes(buf).split(b"\n", 1)[0].decode("utf-8", errors="replace")
    try:
        resp = json.loads(raw)
    except json.JSONDecodeError as e:
        raise BridgeUnavailable(f"invalid JSON from bridge: {raw!r}") from e

    if not resp.get("ok", False):
        raise BridgeError(resp.get("error", "unknown error"), cmd=cmd)
    return resp.get("data", {}) or {}
