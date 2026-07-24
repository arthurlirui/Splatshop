"""Example: a Python-side WebRTC receiving client driving Splatshop.

This is a *control-only* example. It assumes a separate pipeline is already
streaming the Splatshop window video to a browser/player. Here we demonstrate
how a receiving client (Python, e.g. driving a local preview + reading a
joystick/mouse, or a headless automation script) calls the HTTP API to drive
the remote viewpoint.

It covers the recommended patterns from docs/remote_api.md:
  - pull camera pose once on connect to establish a baseline,
  - throttled mouse drag for orbit (60Hz, edge-triggered press/release),
  - edge-triggered keyboard for shortcuts,
  - exponential backoff on bridge outage,
  - an animated rigid-body move via /motion/.../animate.

Usage:
    pip install -r ../requirements.txt   # httpx not required; stdlib urllib used
    python webrtc_receiver.py --host 127.0.0.1 --port 8080

Run alongside a Splatshop instance (with the C++ bridge on 7654) and the
Python FastAPI server started via:
    uvicorn remote_api.server:app --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Optional


class SplatshopClient:
    """Thin HTTP client for the remote control API (stdlib only)."""

    def __init__(self, host: str, port: int, token: Optional[str] = None,
                 timeout: float = 11.0):
        self.base = f"http://{host}:{port}"
        self.headers = {"Content-Type": "application/json"}
        if token:
            self.headers["X-Splat-Token"] = token
        self.timeout = timeout

    def _call(self, method: str, path: str, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        url = self.base + path
        data = json.dumps(body).encode("utf-8") if body is not None else None
        req = urllib.request.Request(url, data=data, headers=self.headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"{method} {path} -> {e.code}: {detail}") from e
        except urllib.error.URLError as e:
            raise ConnectionError(f"cannot reach API at {url}: {e}") from e

    # -- system --
    def health(self) -> Dict[str, Any]:
        return self._call("GET", "/health")

    def camera_pose(self) -> Dict[str, Any]:
        return self._call("GET", "/camera/pose")

    def set_camera_pose(self, **kwargs) -> Dict[str, Any]:
        args = {k: v for k, v in kwargs.items() if v is not None}
        return self._call("POST", "/camera/pose", args)

    def orbit(self, d_yaw: float = 0.0, d_pitch: float = 0.0) -> Dict[str, Any]:
        return self._call("POST", "/camera/orbit", {"d_yaw": d_yaw, "d_pitch": d_pitch})

    def mouse_event(self, x: Optional[float] = None, y: Optional[float] = None,
                    button: Optional[str] = None, action: Optional[str] = None,
                    scroll_dx: Optional[float] = None,
                    scroll_dy: Optional[float] = None) -> Dict[str, Any]:
        body: Dict[str, Any] = {}
        if x is not None: body["x"] = x
        if y is not None: body["y"] = y
        if button:        body["button"] = button
        if action:        body["action"] = action
        if scroll_dx is not None: body["scroll_dx"] = scroll_dx
        if scroll_dy is not None: body["scroll_dy"] = scroll_dy
        return self._call("POST", "/mouse/event", body)

    def key(self, key: str, action: str, mods: int = 0) -> Dict[str, Any]:
        return self._call("POST", "/keyboard/key", {"key": key, "action": action, "mods": mods})

    def press(self, key: str, mods: int = 0) -> Dict[str, Any]:
        return self._call("POST", "/keyboard/press", {"key": key, "duration_ms": 50, "mods": mods})

    def scene_nodes(self) -> Dict[str, Any]:
        return self._call("GET", "/scene/nodes")

    def animate(self, node_id: int, translation=None, rotation=None,
                scale=None, duration_s: float = 1.5, ease: str = "in_out") -> Dict[str, Any]:
        target: Dict[str, Any] = {}
        if translation is not None: target["translation"] = translation
        if rotation is not None:    target["rotation"] = rotation
        if scale is not None:       target["scale"] = scale
        return self._call("POST", f"/motion/node/{node_id}/animate",
                          {"target": target, "duration_s": duration_s, "ease": ease})


def wait_for_bridge(client: SplatshopClient, max_wait: float = 30.0) -> None:
    """Exponential backoff until /health succeeds."""
    delay = 0.5
    deadline = time.time() + max_wait
    while time.time() < deadline:
        try:
            h = client.health()
            print(f"[health] bridge={h['data']['bridge']} fps={h['data'].get('fps')}")
            return
        except (ConnectionError, RuntimeError) as e:
            print(f"[health] not ready, retrying in {delay:.1f}s: {e}")
            time.sleep(delay)
            delay = min(delay * 1.5, 3.0)
    raise RuntimeError("bridge did not come up in time")


def demo_orbit_drag(client: SplatshopClient, center=(960, 540)) -> None:
    """Simulate a left-drag orbit: press at center, sweep right, release.

    This shows the edge-triggered + throttled pattern. In a real browser
    receiver you'd hook DOM pointerdown/pointermove/pointerup and throttle
    pointermove to ~60Hz, sending only the last position per tick.
    """
    x, y = center
    print("[demo] orbit drag")
    client.mouse_event(x=x, y=y, button="left", action="press")
    step = 12  # pixels per tick
    for i in range(20):
        x += step
        client.mouse_event(x=x, y=y)  # move with button held
        time.sleep(1 / 60)            # 60Hz throttle
    client.mouse_event(x=x, y=y, button="left", action="release")


def demo_keyboard_shortcut(client: SplatshopClient, key: str = "t") -> None:
    """Edge-triggered key press (e.g. toggle gizmo mode 't')."""
    print(f"[demo] press '{key}'")
    client.press(key)


def demo_animate_object(client: SplatshopClient) -> None:
    """Find the first splats node and animate it to a new position."""
    nodes = client.scene_nodes().get("nodes", [])
    target = next((n for n in nodes if n["type"] == "SNSplats"), None)
    if not target:
        print("[demo] no SNSplats node found, skipping animate")
        return
    print(f"[demo] animate node {target['id']} ({target['name']})")
    client.animate(target["id"], translation=[2.0, 0.0, 0.0], duration_s=2.0, ease="in_out")


def main() -> None:
    ap = argparse.ArgumentParser(description="Splatshop remote control demo client")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--token", default=None)
    args = ap.parse_args()

    client = SplatshopClient(args.host, args.port, args.token)
    wait_for_bridge(client)

    pose = client.camera_pose()["data"]
    print(f"[pose] yaw={pose['yaw']:.3f} pitch={pose['pitch']:.3f} "
          f"radius={pose['radius']:.3f} target={pose['target']}")

    # Establish a mouse baseline to avoid a first-frame jump (OrbitControls
    # integrates mousePos deltas).
    client.mouse_event(x=960, y=540)

    demo_orbit_drag(client)
    demo_keyboard_shortcut(client, "t")
    demo_animate_object(client)

    # Set a known camera pose at the end.
    client.set_camera_pose(yaw=-1.325, pitch=-0.330, radius=4.691,
                           target=[-0.028, -0.100, 2.301])
    print("[done] camera reset; example finished")


if __name__ == "__main__":
    main()
