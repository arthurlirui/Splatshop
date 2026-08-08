#pragma once

// OrbbecTypes.h
//
// Orbbec RGBD camera module — public data types.
//
// These types describe the user-facing parameters and the host-side frame
// payload of the Orbbec capture module. When the OrbbecSDK is available
// (SPLATSHOP_HAS_ORBBEC defined) the SDK's own enumerations are reused
// directly so the values can never drift from the headers; when the SDK is
// absent the module compiles to empty stubs and a small mirrored enum
// provides ABI-compatible fallback values.
//
// The OBPropertyID values referenced in CameraParams comments are stable
// integer codes from libobsensor/h/Property.h — they are forwarded to the
// SDK as plain ints and do not require the header here.

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "unsuck.hpp"   // Buffer

// When the SDK is present, pull in its enum definitions so we can reuse
// OBAlignMode / OBFormat / OBStreamType / OBSensorType directly instead of
// maintaining mirrored copies. The header is C-compatible and lightweight
// (enum + struct declarations only).
#ifdef SPLATSHOP_HAS_ORBBEC
#include <libobsensor/h/ObTypes.h>
#endif

namespace orbbec {

// ---------------------------------------------------------------------------
// Alignment mode.
//
// With the SDK present this aliases OBAlignMode directly; without it, a
// local enum mirrors the same values so downstream code (GUI combo indices,
// CameraParams::alignMode) compiles either way.
// ---------------------------------------------------------------------------
#ifdef SPLATSHOP_HAS_ORBBEC
using AlignMode = OBAlignMode;
#else
enum AlignMode : int {
    ALIGN_DISABLE     = 0,
    ALIGN_D2C_HW_MODE = 1,
    ALIGN_D2C_SW_MODE = 2,
    ALIGN_C2D_SW_MODE = 3,
};
#endif

// ---------------------------------------------------------------------------
// Stream configuration (resolution / frame rate / pixel format).
//
// A field of 0 / -1 means "ANY" — the SDK picks a default profile. The
// `format` field stores an OBFormat value (e.g. OB_FORMAT_RGB = 22,
// OB_FORMAT_MJPG = 5, OB_FORMAT_Y16 = 8); -1 means OB_FORMAT_ANY.
// ---------------------------------------------------------------------------
struct StreamConfig {
    bool enable  = true;
    int  width   = 0;      // 0 == OB_WIDTH_ANY
    int  height  = 0;      // 0 == OB_HEIGHT_ANY
    int  fps     = 0;      // 0 == OB_FPS_ANY
    int  format  = -1;     // OBFormat enum value; -1 == OB_FORMAT_ANY

    bool operator==(const StreamConfig& o) const {
        return enable == o.enable && width == o.width && height == o.height &&
               fps == o.fps && format == o.format;
    }
    bool operator!=(const StreamConfig& o) const { return !(*this == o); }
};

// ---------------------------------------------------------------------------
// Camera control parameters. Each scalar field accepts the special value
// -1 meaning "leave at device default / do not write". Boolean fields are
// always applied. The capture layer checks isPropertySupported and clamps
// each value to the device-reported range (honouring the step) before
// writing, so passing an out-of-range value is safe.
//
// The numeric values mirror the OBPropertyID enumerations from
// libobsensor/h/Property.h (see the comments inline).
// ---------------------------------------------------------------------------
struct CameraParams {
    // --- Color camera ---
    bool colorAutoExposure      = true;   // OB_PROP_COLOR_AUTO_EXPOSURE_BOOL      (2000)
    int  colorExposure          = -1;     // OB_PROP_COLOR_EXPOSURE_INT            (2001)
    int  colorGain              = -1;     // OB_PROP_COLOR_GAIN_INT                (2002)
    bool colorAutoWhiteBalance  = true;   // OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL (2003)
    int  colorWhiteBalance      = -1;     // OB_PROP_COLOR_WHITE_BALANCE_INT       (2004)
    int  colorBrightness        = -1;     // OB_PROP_COLOR_BRIGHTNESS_INT          (2005)
    int  colorSaturation        = -1;     // OB_PROP_COLOR_SATURATION_INT          (2008)
    int  colorContrast          = -1;     // OB_PROP_COLOR_CONTRAST_INT            (2009)
    int  colorGamma             = -1;     // OB_PROP_COLOR_GAMMA_INT               (2010)
    bool colorMirror            = false;  // OB_PROP_COLOR_MIRROR_BOOL             (81)

    // --- Depth camera ---
    bool depthAutoExposure      = true;   // OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL      (2016)
    int  depthExposure          = -1;     // OB_PROP_DEPTH_EXPOSURE_INT            (2017)
    int  depthGain              = -1;     // OB_PROP_DEPTH_GAIN_INT                (2018)
    int  depthPrecisionLevel    = -1;     // OB_PROP_DEPTH_PRECISION_LEVEL_INT     (75)
    bool depthMirror            = false;  // OB_PROP_DEPTH_MIRROR_BOOL             (14)
    int  minDepth               = -1;     // OB_PROP_MIN_DEPTH_INT                 (22)  [mm]
    int  maxDepth               = -1;     // OB_PROP_MAX_DEPTH_INT                 (23)  [mm]

    // --- IR / Laser / LDP ---
    int  irExposure             = -1;     // OB_PROP_IR_EXPOSURE_INT               (2026)
    int  irGain                 = -1;     // OB_PROP_IR_GAIN_INT                   (2027)
    bool laserOn                = true;   // OB_PROP_LASER_BOOL                    (3)
    bool ldpOn                  = false;  // OB_PROP_LDP_BOOL                      (2)

    // --- Alignment / synchronisation ---
    AlignMode alignMode           = AlignMode(0);  // ALIGN_DISABLE / ALIGN_D2C_HW_MODE / ...
    bool frameSync              = true;   // pipeline->enableFrameSync()
    bool aggregateAllRequired   = true;   // OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE

    // --- Depth denoising (SDK post-processing filters, applied in captureLoop) ---
    // These are NOT device properties (OBPropertyID) — they are read by start()
    // to construct the corresponding ob::Filter objects. -1 / -1.f means "use
    // SDK default"; booleans gate each filter on/off.

    // Hardware noise removal: LutNoiseRemovalFilter (SPLIT / LUT-based).
    // Operates on a 4x4 LUT of max noise pixel group sizes; effective on
    // devices that support hardware-side noise filtering.
    bool  hwNoiseRemovalEnabled  = false;
    int   hwNoiseMaxLut          = -1;     // uniform LUT value for all 16 entries (default 10)
    int   hwNoiseMinDiff         = -1;     // neighbor pixel diff threshold (default 5)

    // Software noise removal: NoiseRemovalFilter (NON_SPLIT / OVERALL).
    // Removes scattered depth pixels across the whole frame.
    bool  denoiseFilterEnabled  = false;  // NoiseRemovalFilter: remove scattered noise pixels
    int   denoiseMaxSize        = -1;     // max scattering pixel group size (default 80)
    int   denoiseDispDiff       = -1;     // min disparity diff threshold (default 256)

    bool  spatialFilterEnabled  = false;  // SpatialAdvancedFilter: edge-preserving spatial smoothing
    float spatialAlpha          = -1.f;   // weight of current pixel (0-1, default 0.1)
    int   spatialRadius         = -1;     // kernel radius (default 1)
    int   spatialMagnitude      = -1;     // number of iterations (default 2)
    int   spatialDispDiff       = -1;     // depth gradient threshold (default 160)

    bool  temporalFilterEnabled = false;  // TemporalFilter: temporal jitter reduction
    float temporalWeight        = -1.f;   // blending weight (0-1, default 0.4)
    float temporalDiffScale     = -1.f;   // diff scale (default 0.1)
};

// ---------------------------------------------------------------------------
// A single synchronised RGBD frame held in host memory.
//
// `colorData` / `depthData` are owned Buffers; the pixel layout is given by
// `colorFormat` / `depthFormat` (OBFormat values). For depth, each pixel is
// a uint16_t and `depthScale` converts it to millimetres:
//     distance_mm = pixel * depthScale
//
// When the point-cloud path is enabled, the same struct is reused with the
// point cloud laid out as `colorFormat = OB_FORMAT_RGB_POINT`: each point is
// an OBColorPoint {float x, y, z, r, g, b} (sizeof(OBColorPoint) bytes), and
// `colorWidth` holds the point count with `colorHeight = 1`.
// ---------------------------------------------------------------------------
struct RGBDFrame {
    shared_ptr<Buffer> colorData = nullptr;
    int   colorWidth  = 0;
    int   colorHeight = 0;
    int   colorFormat = -1;     // OBFormat of the color buffer

    shared_ptr<Buffer> depthData = nullptr;
    int   depthWidth  = 0;
    int   depthHeight = 0;
    int   depthFormat = -1;     // OBFormat of the depth buffer
    float depthScale  = 0.001f; // pixel * scale = mm

    // Optional IR stream (used for chessboard-based lens calibration; the
    // IR sensor is co-located with the depth sensor, so its distortion is
    // shared with the depth stream). Filled only when IR streaming is
    // enabled (see OrbbecCapture::setIrStreamConfig). IR frames are usually
    // 8-bit or 16-bit grayscale; the OBFormat is in `irFormat`.
    shared_ptr<Buffer> irData = nullptr;
    int   irWidth  = 0;
    int   irHeight = 0;
    int   irFormat = -1;        // OBFormat of the IR buffer

    uint64_t timestampUs = 0;   // device timestamp
    uint64_t frameIndex  = 0;
};

// ---------------------------------------------------------------------------
// Device enumeration result (no SDK handle retained).
// ---------------------------------------------------------------------------
struct DeviceInfo {
    string name;
    string serialNumber;
    string uid;
    string connectionType;     // "USB" / "Ethernet" / ...
    int    pid = 0;
    int    vid = 0;
};

// ---------------------------------------------------------------------------
// Property range query result (mirrors OBIntPropertyRange).
// Used by the GUI to bound sliders and quantise user input to `step`.
// ---------------------------------------------------------------------------
struct IntRange {
    int min = 0;
    int max = 0;
    int step = 1;
    int def = 0;
    int cur = 0;
};

// ---------------------------------------------------------------------------
// Camera intrinsics (mirrors OBCameraIntrinsic, float subset).
// ---------------------------------------------------------------------------
struct Intrinsics {
    float fx = 0.f, fy = 0.f, cx = 0.f, cy = 0.f;
    int   w  = 0,   h  = 0;
};

// ---------------------------------------------------------------------------
// Lens distortion coefficients — 5-parameter Brown-Conrady model.
//
//   k1,k2,k3 — radial distortion.
//   p1,p2    — tangential distortion.
//
// Mirrors the first 5 coefficients of OpenCV's distCoeffs and the
// k1..k3,p1,p2 fields of the SDK's OBCameraDistortion (Brown-Conrady
// family). Defined here (not in Calibration.h) so the capture layer can
// read device distortion without pulling in the JSON/OpenCV headers.
// ---------------------------------------------------------------------------
struct DistortionCoeffs {
    float k1 = 0.f, k2 = 0.f, p1 = 0.f, p2 = 0.f, k3 = 0.f;
};

} // namespace orbbec
