"""Configuration for the Splatshop remote control API.

These can be overridden via environment variables (prefixed ``SPLAT_``).
"""

from __future__ import annotations

import os

# Host/port of the C++ bridge (RemoteControlServer, src/remote/). The bridge
# binds to 127.0.0.1 only, so the Python server must run on the same machine
# as the Splatshop executable unless you forward the port.
BRIDGE_HOST: str = os.environ.get("SPLAT_BRIDGE_HOST", "127.0.0.1")
BRIDGE_PORT: int = int(os.environ.get("SPLAT_BRIDGE_PORT", "7654"))

# Per-request socket timeout when talking to the C++ bridge (seconds). The
# bridge blocks up to one render frame (~16ms) plus the action itself; 10s
# matches the main-thread watchdog in RemoteControlServer.cpp.
BRIDGE_TIMEOUT: float = float(os.environ.get("SPLAT_BRIDGE_TIMEOUT", "11.0"))

# HTTP server (FastAPI / uvicorn) listen settings. This is what remote WebRTC
# receiving clients connect to. Bind to 0.0.0.0 to accept remote clients, but
# be aware the control channel is unauthenticated by default.
HTTP_HOST: str = os.environ.get("SPLAT_HTTP_HOST", "0.0.0.0")
HTTP_PORT: int = int(os.environ.get("SPLAT_HTTP_PORT", "8080"))

# Optional shared-secret token. If non-empty, requests must send it in the
# ``X-Splat-Token`` header. Keep it empty for local-only / trusted-network use.
API_TOKEN: str = os.environ.get("SPLAT_API_TOKEN", "")
