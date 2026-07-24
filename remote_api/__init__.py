"""Splatshop remote control API (Python frontend).

This package turns the Splatshop C++ editor into a network-controllable
renderer: a FastAPI HTTP server (remote_api.server) translates remote
requests into JSON-RPC commands sent over a local TCP socket to the C++
RemoteControlServer (src/remote/RemoteControlServer.cpp), which runs them on
the main render thread.

Public submodules:
  - server      : the FastAPI app + all HTTP endpoints
  - splat_client: low-level bridge client (TCP JSON-RPC)
  - models      : pydantic request schemas
  - keymap      : GLFW key-name resolution
  - config      : host/port/token settings (env-overridable)
"""

from __future__ import annotations

__version__ = "0.1.0"
