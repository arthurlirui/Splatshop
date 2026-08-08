#pragma once

constexpr int64_t MAX_SPLATS = 100'000'000;
constexpr int64_t MAX_LINES = 10'000'000;

constexpr int VIEWMODE_DESKTOP = 0;
constexpr int VIEWMODE_DESKTOP_VR = 1;
constexpr int VIEWMODE_IMMERSIVE_VR = 2;
constexpr int VIEWMODE_REMOTE_STEREO = 3;

static inline int viewmode = VIEWMODE_DESKTOP;

// Coordinate space of an externally-supplied stereo pose (e.g. from a remote
// HMD). The server uses this to decide whether the SteamVR->GL `flip` matrix
// must be applied and whether the supplied head/eye transforms are
// world->head (needs inverse) or already world->eye (view matrices).
//   OPENVR  : OpenVR dmat4 tracking poses (+Y up, +Z forward). Server applies
//             `flip` and inverts, exactly like the local immersive path.
//   WEBXR   : WebXR view/projection matrices (+Y up, -Z forward, right-handed,
//             column-major on the wire). Server consumes them directly (no
//             flip); a change of basis into the app's GL space is applied.
//   RAW_VIEW: caller already supplies finished view matrices in the app's GL
//             space (+Z up, +Y forward); server uses them verbatim.
enum PoseSpace { POSE_SPACE_OPENVR = 0, POSE_SPACE_WEBXR = 1, POSE_SPACE_RAW_VIEW = 2 };


