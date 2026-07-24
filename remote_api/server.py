"""Splatshop remote control HTTP API (FastAPI).

Exposes mouse / keyboard / camera / rigid-motion control of a running
Splatshop instance to remote WebRTC receiving clients. This server talks to
the C++ SplatEditor process over a local TCP JSON-RPC bridge
(remote_api/splat_client.py -> src/remote/RemoteControlServer.cpp).

Run with:
    uvicorn remote_api.server:app --host 0.0.0.0 --port 8080

See docs/remote_api.md for the full endpoint reference and the WebRTC receiver
integration guide.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from fastapi import Depends, FastAPI, Header, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from . import config, splat_client
from . import models as M

# --------------------------------------------------------------------------- #
# App setup
# --------------------------------------------------------------------------- #
app = FastAPI(
    title="Splatshop Remote Control API",
    description=(
        "Control a running Splatshop Gaussian-splatting viewer's mouse, "
        "keyboard, camera and rigid-object motion over HTTP. Intended for "
        "remote WebRTC receiving clients to drive the viewpoint."
    ),
    version="0.1.0",
)

# Allow browser-based WebRTC receivers to call us cross-origin.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# --------------------------------------------------------------------------- #
# Auth dependency (optional shared-secret token)
# --------------------------------------------------------------------------- #
def _check_token(x_splat_token: Optional[str] = Header(default=None)) -> None:
    if not config.API_TOKEN:
        return
    if x_splat_token != config.API_TOKEN:
        raise HTTPException(status.HTTP_401_UNAUTHORIZED, "invalid or missing token")


# Bridge errors -> HTTP 502 (bad gateway); unavailable -> 503.
def _bridge_error_to_http(e: Exception) -> HTTPException:
    if isinstance(e, splat_client.BridgeUnavailable):
        return HTTPException(status.HTTP_503_SERVICE_UNAVAILABLE, str(e))
    return HTTPException(status.HTTP_502_BAD_GATEWAY, str(e))


def _call(cmd: str, args: Optional[Dict[str, Any]] = None,
          token_ok: None = None) -> Dict[str, Any]:
    try:
        return splat_client.request(cmd, args)
    except (splat_client.BridgeError, splat_client.BridgeUnavailable) as e:
        raise _bridge_error_to_http(e) from e


# --------------------------------------------------------------------------- #
# System
# --------------------------------------------------------------------------- #
@app.get("/", dependencies=[Depends(_check_token)])
def api_info():
    return {
        "name": "Splatshop Remote Control API",
        "version": app.version,
        "bridge": {"host": config.BRIDGE_HOST, "port": config.BRIDGE_PORT},
        "docs": "/docs",
    }


@app.get("/health", dependencies=[Depends(_check_token)])
def health():
    """Ping the C++ bridge; returns fps / frame / window size when up."""
    return _call("health", token_ok=None)


# --------------------------------------------------------------------------- #
# Camera
# --------------------------------------------------------------------------- #
@app.post("/camera/orbit", dependencies=[Depends(_check_token)])
def camera_orbit(req: M.OrbitRequest):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("camera.orbit", args)


@app.post("/camera/pan", dependencies=[Depends(_check_token)])
def camera_pan(req: M.PanRequest):
    return _call("camera.pan", req.model_dump())


@app.post("/camera/zoom", dependencies=[Depends(_check_token)])
def camera_zoom(req: M.ZoomRequest):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("camera.zoom", args)


@app.post("/camera/pose", dependencies=[Depends(_check_token)])
def camera_pose_set(req: M.CameraPose):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("camera.pose.set", args)


@app.get("/camera/pose", dependencies=[Depends(_check_token)])
def camera_pose_get():
    return _call("camera.pose.get")


@app.post("/camera/focus", dependencies=[Depends(_check_token)])
def camera_focus(req: M.FocusRequest):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("camera.focus", args)


# --------------------------------------------------------------------------- #
# Mouse
# --------------------------------------------------------------------------- #
@app.post("/mouse/move", dependencies=[Depends(_check_token)])
def mouse_move(req: M.MouseMoveRequest):
    return _call("mouse.move", req.model_dump())


@app.post("/mouse/button", dependencies=[Depends(_check_token)])
def mouse_button(req: M.MouseButtonRequest):
    return _call("mouse.button", req.model_dump())


@app.post("/mouse/scroll", dependencies=[Depends(_check_token)])
def mouse_scroll(req: M.MouseScrollRequest):
    return _call("mouse.scroll", req.model_dump())


@app.post("/mouse/event", dependencies=[Depends(_check_token)])
def mouse_event(req: M.MouseEventRequest):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("mouse.event", args)


# --------------------------------------------------------------------------- #
# Keyboard
# --------------------------------------------------------------------------- #
@app.post("/keyboard/key", dependencies=[Depends(_check_token)])
def keyboard_key(req: M.KeyRequest):
    return _call("keyboard.key", req.model_dump())


@app.post("/keyboard/press", dependencies=[Depends(_check_token)])
def keyboard_press(req: M.KeyPressRequest):
    """Press then release a key, sleeping between the two events."""
    splat_client.request("keyboard.key",
                         {"key": req.key, "action": "press", "mods": req.mods or 0})
    time.sleep(max(0.0, (req.duration_ms or 0) / 1000.0))
    try:
        return splat_client.request("keyboard.key",
                                    {"key": req.key, "action": "release", "mods": req.mods or 0})
    except (splat_client.BridgeError, splat_client.BridgeUnavailable) as e:
        raise _bridge_error_to_http(e) from e


@app.post("/keyboard/sequence", dependencies=[Depends(_check_token)])
def keyboard_sequence(req: M.KeySequenceRequest):
    results: List[Dict[str, Any]] = []
    for ch in req.text:
        try:
            splat_client.request("keyboard.key", {"key": ch, "action": "press", "mods": 0})
            splat_client.request("keyboard.key", {"key": ch, "action": "release", "mods": 0})
            results.append({"char": ch, "ok": True})
        except (splat_client.BridgeError, splat_client.BridgeUnavailable) as e:
            results.append({"char": ch, "ok": False, "error": str(e)})
    return {"results": results}


# --------------------------------------------------------------------------- #
# Scene / Motion (rigid body)
# --------------------------------------------------------------------------- #
@app.get("/scene/nodes", dependencies=[Depends(_check_token)])
def scene_nodes():
    return _call("scene.nodes")


@app.get("/motion/node/{node_id}/transform", dependencies=[Depends(_check_token)])
def motion_get(node_id: int):
    return _call("motion.get", {"id": node_id})


@app.post("/motion/node/{node_id}/transform", dependencies=[Depends(_check_token)])
def motion_set_transform(node_id: int, req: M.Transform):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    args["id"] = node_id
    return _call("motion.set_transform", args)


@app.post("/motion/node/{node_id}/translate", dependencies=[Depends(_check_token)])
def motion_translate(node_id: int, req: M.TranslateRequest):
    return _call("motion.translate", {"id": node_id, "delta": req.delta})


@app.post("/motion/node/{node_id}/rotate", dependencies=[Depends(_check_token)])
def motion_rotate(node_id: int, req: M.RotateRequest):
    return _call("motion.rotate", {"id": node_id, "delta": req.delta})


@app.post("/motion/node/{node_id}/scale", dependencies=[Depends(_check_token)])
def motion_scale(node_id: int, req: M.ScaleRequest):
    return _call("motion.scale", {"id": node_id, "factor": req.factor})


@app.post("/motion/node/{node_id}/animate", dependencies=[Depends(_check_token)])
def motion_animate(node_id: int, req: M.AnimateRequest):
    target = {k: v for k, v in req.target.model_dump().items() if v is not None}
    args = {"id": node_id, "target": target,
            "duration_s": req.duration_s, "ease": req.ease}
    return _call("motion.animate", args)


# --------------------------------------------------------------------------- #
# Entrypoint
# --------------------------------------------------------------------------- #
def main():
    import uvicorn
    uvicorn.run(
        "remote_api.server:app",
        host=config.HTTP_HOST,
        port=config.HTTP_PORT,
        reload=False,
    )


if __name__ == "__main__":
    main()
