"""Pydantic request models for the Splatshop HTTP API.

Wire conventions (also enforced server-side, see docs/remote_api.md):
  - Quaternions are arrays ``[x, y, z, w]`` (matches the motion Timeline JSON).
  - Angles are radians.
  - Vectors are 3-element arrays.
  - Mouse coordinates are window pixel space, origin top-left, Y down.
"""

from __future__ import annotations

from typing import List, Literal, Optional, Union

from pydantic import BaseModel, Field, model_validator

# GLFW modifier bitmask: shift=1, ctrl=2, alt=4, super=8 (max 15).
MODS_MAX = 15

MouseButton = Literal["left", "right", "middle"]
KeyAction = Literal["press", "release", "repeat"]
EaseName = Literal["linear", "in", "out", "in_out"]


class Vec3(BaseModel):
    """A 3-component vector (used for translation, scale, target, deltas)."""
    values: List[float] = Field(..., min_length=3, max_length=3)


class Quat(BaseModel):
    """A quaternion stored as [x, y, z, w]."""
    values: List[float] = Field(..., min_length=4, max_length=4)


# --------------------------------------------------------------------------- #
# Camera
# --------------------------------------------------------------------------- #
class OrbitRequest(BaseModel):
    """Set or nudge the orbit angles (radians). Either absolute or delta fields."""
    yaw: Optional[float] = None
    pitch: Optional[float] = None
    d_yaw: Optional[float] = None
    d_pitch: Optional[float] = None


class PanRequest(BaseModel):
    """Pan the camera target in its local frame (units scale with radius)."""
    dx: float = 0.0
    dy: float = 0.0


class ZoomRequest(BaseModel):
    """Zoom in/out. ``radius`` is absolute; ``factor`` is multiplicative."""
    radius: Optional[float] = None
    factor: Optional[float] = None


class CameraPose(BaseModel):
    """Full orbit-camera pose."""
    yaw: Optional[float] = None
    pitch: Optional[float] = None
    radius: Optional[float] = None
    target: Optional[List[float]] = Field(None, min_length=3, max_length=3)


class FocusRequest(BaseModel):
    """Focus the camera on a node bounding box or explicit AABB."""
    node_id: Optional[int] = None
    min: Optional[List[float]] = Field(None, min_length=3, max_length=3)
    max: Optional[List[float]] = Field(None, min_length=3, max_length=3)
    factor: Optional[float] = 1.0


# --------------------------------------------------------------------------- #
# Mouse
# --------------------------------------------------------------------------- #
class MouseMoveRequest(BaseModel):
    x: float
    y: float


class MouseButtonRequest(BaseModel):
    button: MouseButton
    action: KeyAction
    mods: Optional[int] = Field(0, ge=0, le=MODS_MAX)


class MouseScrollRequest(BaseModel):
    dx: float = 0.0
    dy: float = 0.0


class MouseEventRequest(BaseModel):
    """Composite event: optional move, button, and scroll in one round-trip."""
    x: Optional[float] = None
    y: Optional[float] = None
    button: Optional[MouseButton] = None
    action: Optional[KeyAction] = None
    mods: Optional[int] = Field(0, ge=0, le=MODS_MAX)
    scroll_dx: Optional[float] = None
    scroll_dy: Optional[float] = None

    @model_validator(mode="after")
    def _check_button_action_pair(self):
        # The C++ bridge only injects a button event when BOTH button and action
        # are present; silently skipping one without the other would mislead the
        # client into thinking the click took effect. Require them together.
        if (self.button is None) != (self.action is None):
            raise ValueError(
                "button and action must be provided together (both or neither)")
        return self


# --------------------------------------------------------------------------- #
# Keyboard
# --------------------------------------------------------------------------- #
class KeyRequest(BaseModel):
    """Inject a single key event. ``key`` is a GLFW key name (e.g. "W", "SPACE")
    or a numeric GLFW key code. ``mods`` is a bitmask (shift=1, ctrl=2, alt=4, super=8)."""
    key: Union[int, str]
    action: KeyAction
    mods: Optional[int] = Field(0, ge=0, le=MODS_MAX)


class KeyPressRequest(BaseModel):
    """Convenience: press a key, hold for ``duration_ms``, then release."""
    key: Union[int, str]
    duration_ms: Optional[int] = 50
    mods: Optional[int] = Field(0, ge=0, le=MODS_MAX)


class KeySequenceRequest(BaseModel):
    """Type a string of printable characters (press+release each)."""
    text: str


# --------------------------------------------------------------------------- #
# Motion (rigid body)
# --------------------------------------------------------------------------- #
class Transform(BaseModel):
    """A rigid transform: translation, rotation (quaternion [x,y,z,w]), scale."""
    translation: Optional[List[float]] = Field(None, min_length=3, max_length=3)
    rotation: Optional[List[float]] = Field(None, min_length=4, max_length=4)
    scale: Optional[List[float]] = Field(None, min_length=3, max_length=3)


class TranslateRequest(BaseModel):
    delta: List[float] = Field(..., min_length=3, max_length=3)


class RotateRequest(BaseModel):
    delta: List[float] = Field(..., min_length=4, max_length=4)


class ScaleRequest(BaseModel):
    factor: List[float] = Field(..., min_length=3, max_length=3)


class AnimateRequest(BaseModel):
    target: Transform
    duration_s: Optional[float] = 1.0
    ease: Optional[EaseName] = "in_out"


# --------------------------------------------------------------------------- #
# Scene Splats creation / modification
# --------------------------------------------------------------------------- #
SplatsPrimitive = Literal["sphere", "box", "points"]


class SphereParams(BaseModel):
    """Create a sphere of Gaussian splats."""
    center: Optional[List[float]] = Field(default=[0, 0, 0], min_length=3, max_length=3)
    radius: Optional[float] = 1.0
    count: Optional[int] = 576
    color: Optional[List[float]] = Field(default=[1, 0, 0, 1], min_length=3, max_length=4)


class BoxParams(BaseModel):
    """Create splats filling an axis-aligned bounding box."""
    min: List[float] = Field(..., min_length=3, max_length=3)
    max: List[float] = Field(..., min_length=3, max_length=3)
    count: Optional[int] = 1000
    color: Optional[List[float]] = Field(default=[1, 0, 0, 1], min_length=3, max_length=4)


class PointsParams(BaseModel):
    """Create independent splats at explicit positions."""
    positions: List[List[float]]
    scale: Optional[float] = 0.02
    color: Optional[List[float]] = Field(default=[1, 0, 0, 1], min_length=3, max_length=4)


class SplatsCreateRequest(BaseModel):
    type: SplatsPrimitive
    params: Dict[str, Any]


class LoadFileRequest(BaseModel):
    path: str


class SetColorRequest(BaseModel):
    color: List[float] = Field(..., min_length=3, max_length=4)


# --------------------------------------------------------------------------- #
# VR remote stereo
# --------------------------------------------------------------------------- #
# Feeds an externally-tracked HMD pose into the C++ renderer's
# VIEWMODE_REMOTE_STEREO path. Matrices are flat 16-element arrays in
# COLUMN-MAJOR order (the order produced by glm::value_ptr / make_mat4 and by
# OpenVR/WebXR column-major matrices). See common.h (PoseSpace) for the
# coordinate-space contract.
PoseSpace = Literal["openvr", "webxr", "raw_view"]


class VRPoseRequest(BaseModel):
    """One HMD pose packet.

    For ``pose_space="openvr"`` supply head/eye tracking transforms + the
    per-eye projection/VP. For ``"webxr"``/``"raw_view"`` supply finished
    per-eye view matrices (world->eye, GL convention) + projection/VP.
    width/height set the per-eye render target size.
    """
    pose_space: PoseSpace = "webxr"
    # OpenVR tracking-space transforms
    head_pose: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    eye_left: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    eye_right: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    # Finished view matrices (webxr / raw_view)
    view_left: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    view_right: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    # Per-eye projection + viewport-inclusive VP (all spaces)
    proj_left: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    proj_right: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    vp_left: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    vp_right: Optional[List[float]] = Field(None, min_length=16, max_length=16)
    width: Optional[int] = None
    height: Optional[int] = None

