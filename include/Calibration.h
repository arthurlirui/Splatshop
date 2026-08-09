#pragma once

// Calibration.h
//
// Lens-calibration data structures for the Orbbec RGBD camera module.
//
// These types are pure data (no SDK, no OpenCV dependency) so they can be
// included from anywhere. `StreamCalibration` holds the optimised intrinsics
// + the 5-parameter Brown-Conrady distortion model (k1, k2, p1, p2, k3)
// which is what OpenCV's cv::calibrateCamera produces by default and what
// the OrbbecSDK's OBCameraDistortion uses for the Brown-Conrady family.
//
// JSON (de)serialisation is provided inline via nlohmann::json so callers
// can read/write calibration files without pulling in the store module.

#include <string>
#include <vector>

#include "json/json.hpp"
#include "camera/OrbbecTypes.h"   // orbbec::Intrinsics / DistortionCoeffs (resolves via ./src include path)

namespace orbbec {

// DistortionCoeffs is defined in OrbbecTypes.h (5-parameter Brown-Conrady)
// and re-exported here via the include above.

// ---------------------------------------------------------------------------
// Per-stream calibration result.
// ---------------------------------------------------------------------------
struct StreamCalibration {
    Intrinsics        intrinsics;              // optimised fx,fy,cx,cy + w,h
    DistortionCoeffs  distortion;
    double            rmsReprojectionError = -1.0;
    int               usedImageCount       = 0;
    std::string       timestamp;              // ISO-8601 of when it was computed
    bool              valid                 = false;
};

// ---------------------------------------------------------------------------
// Board pose (extrinsic) snapshot.
//
// The result of a single solvePnP solve: the chessboard's pose relative to
// the camera. `rvec` is a Rodrigues rotation vector, `tvec` is the
// translation in millimetres (squareSizeMm sets the unit), and `distanceMm`
// is |tvec| - the ground-truth distance used to fit the depth correction.
// ---------------------------------------------------------------------------
struct ExtrinsicPose {
    float rvec[3]      = {0.f, 0.f, 0.f};
    float tvec[3]      = {0.f, 0.f, 0.f};
    float distanceMm   = 0.f;
    bool  valid        = false;
};

// ---------------------------------------------------------------------------
// Depth metric correction.
//
// Linear fit over (ground-truth distance, depth reading) pairs collected at
// different distances:  depth_true_mm = a * depth_measured_mm + b.
// Corrects the camera's systematic metric scale/bias error. `valid` is set
// after a successful fitDepthCorrection(); until then a=1, b=0 (identity).
// ---------------------------------------------------------------------------
struct DepthCorrection {
    float  a           = 1.f;
    float  b           = 0.f;
    double rmsMm       = -1.0;     // RMS residual (mm), -1 until fitted
    int    sampleCount = 0;
    bool   valid       = false;
};

// ---------------------------------------------------------------------------
// Whole-device calibration. The depth stream shares the IR stream's
// distortion because the depth sensor and the IR projector/sensor are
// physically co-located (the depth map is computed from the IR pattern).
// ---------------------------------------------------------------------------
struct DeviceCalibration {
    std::string   deviceSerial;
    std::string   deviceName;
    StreamCalibration color;
    StreamCalibration ir;
    StreamCalibration depth;
    std::string   notes;
    std::string   filePath;                    // set after load/save
    ExtrinsicPose     extrinsic;               // last solvePnP board pose
    DepthCorrection   depthCorrection;         // linear depth metric fit
};

} // namespace orbbec

// ---------------------------------------------------------------------------
// nlohmann::json adapters.
// ---------------------------------------------------------------------------
namespace orbbec {

inline void to_json(nlohmann::json& j, const Intrinsics& v) {
    j = nlohmann::json{{"fx", v.fx}, {"fy", v.fy}, {"cx", v.cx}, {"cy", v.cy},
                       {"w",  v.w},  {"h",  v.h}};
}
inline void from_json(const nlohmann::json& j, Intrinsics& v) {
    v.fx = j.value("fx", 0.f);
    v.fy = j.value("fy", 0.f);
    v.cx = j.value("cx", 0.f);
    v.cy = j.value("cy", 0.f);
    v.w  = j.value("w",  0);
    v.h  = j.value("h",  0);
}

inline void to_json(nlohmann::json& j, const DistortionCoeffs& v) {
    j = nlohmann::json{{"k1", v.k1}, {"k2", v.k2}, {"p1", v.p1},
                       {"p2", v.p2}, {"k3", v.k3}};
}
inline void from_json(const nlohmann::json& j, DistortionCoeffs& v) {
    v.k1 = j.value("k1", 0.f);
    v.k2 = j.value("k2", 0.f);
    v.p1 = j.value("p1", 0.f);
    v.p2 = j.value("p2", 0.f);
    v.k3 = j.value("k3", 0.f);
}

inline void to_json(nlohmann::json& j, const StreamCalibration& v) {
    j = nlohmann::json{
        {"intrinsics", v.intrinsics},
        {"distortion", v.distortion},
        {"rmsReprojectionError", v.rmsReprojectionError},
        {"usedImageCount", v.usedImageCount},
        {"timestamp", v.timestamp},
        {"valid", v.valid}};
}
inline void from_json(const nlohmann::json& j, StreamCalibration& v) {
    if (j.contains("intrinsics")) j.at("intrinsics").get_to(v.intrinsics);
    if (j.contains("distortion")) j.at("distortion").get_to(v.distortion);
    v.rmsReprojectionError = j.value("rmsReprojectionError", -1.0);
    v.usedImageCount       = j.value("usedImageCount", 0);
    v.timestamp            = j.value("timestamp", std::string{});
    v.valid                = j.value("valid", false);
}

inline void to_json(nlohmann::json& j, const ExtrinsicPose& v) {
    j = nlohmann::json{{"rvec",      std::vector<float>{v.rvec[0], v.rvec[1], v.rvec[2]}},
                       {"tvec",      std::vector<float>{v.tvec[0], v.tvec[1], v.tvec[2]}},
                       {"distanceMm", v.distanceMm},
                       {"valid",     v.valid}};
}
inline void from_json(const nlohmann::json& j, ExtrinsicPose& v) {
    if (j.contains("rvec") && j.at("rvec").is_array()) {
        const auto& a = j.at("rvec");
        for (int i = 0; i < 3 && i < (int)a.size(); ++i) v.rvec[i] = a[i].get<float>();
    }
    if (j.contains("tvec") && j.at("tvec").is_array()) {
        const auto& a = j.at("tvec");
        for (int i = 0; i < 3 && i < (int)a.size(); ++i) v.tvec[i] = a[i].get<float>();
    }
    v.distanceMm = j.value("distanceMm", 0.f);
    v.valid      = j.value("valid", false);
}

inline void to_json(nlohmann::json& j, const DepthCorrection& v) {
    j = nlohmann::json{{"a", v.a}, {"b", v.b}, {"rmsMm", v.rmsMm},
                       {"sampleCount", v.sampleCount}, {"valid", v.valid}};
}
inline void from_json(const nlohmann::json& j, DepthCorrection& v) {
    v.a           = j.value("a", 1.f);
    v.b           = j.value("b", 0.f);
    v.rmsMm       = j.value("rmsMm", -1.0);
    v.sampleCount = j.value("sampleCount", 0);
    v.valid       = j.value("valid", false);
}

inline void to_json(nlohmann::json& j, const DeviceCalibration& v) {
    j = nlohmann::json{
        {"deviceSerial", v.deviceSerial},
        {"deviceName",   v.deviceName},
        {"color",        v.color},
        {"ir",           v.ir},
        {"depth",        v.depth},
        {"notes",        v.notes},
        {"extrinsic",       v.extrinsic},
        {"depthCorrection", v.depthCorrection}};
}
inline void from_json(const nlohmann::json& j, DeviceCalibration& v) {
    v.deviceSerial = j.value("deviceSerial", std::string{});
    v.deviceName   = j.value("deviceName",   std::string{});
    if (j.contains("color")) j.at("color").get_to(v.color);
    if (j.contains("ir"))    j.at("ir").get_to(v.ir);
    if (j.contains("depth")) j.at("depth").get_to(v.depth);
    v.notes = j.value("notes", std::string{});
    if (j.contains("extrinsic"))       j.at("extrinsic").get_to(v.extrinsic);
    if (j.contains("depthCorrection")) j.at("depthCorrection").get_to(v.depthCorrection);
}

// ---------------------------------------------------------------------------
// CameraParams / StreamConfig / CameraParamsFile (de)serialisation.
//
// CameraParams holds the live device-control + depth-filter + point-cloud
// settings; StreamConfig holds the per-stream resolution/fps/format. They are
// persisted together in a CameraParamsFile so a device can be restored to a
// known working configuration. AlignMode is stored as int so the file is
// portable across builds with/without the OrbbecSDK.
// ---------------------------------------------------------------------------
inline void to_json(nlohmann::json& j, const StreamConfig& v) {
    j = nlohmann::json{{"enable", v.enable}, {"width", v.width}, {"height", v.height},
                       {"fps",    v.fps},    {"format", v.format}};
}
inline void from_json(const nlohmann::json& j, StreamConfig& v) {
    v.enable = j.value("enable", true);
    v.width  = j.value("width",  0);
    v.height = j.value("height", 0);
    v.fps    = j.value("fps",    0);
    v.format = j.value("format", -1);
}

inline void to_json(nlohmann::json& j, const CameraParams& v) {
    j = nlohmann::json{
        // Color camera
        {"colorAutoExposure",     v.colorAutoExposure},
        {"colorExposure",         v.colorExposure},
        {"colorGain",             v.colorGain},
        {"colorAutoWhiteBalance", v.colorAutoWhiteBalance},
        {"colorWhiteBalance",     v.colorWhiteBalance},
        {"colorBrightness",       v.colorBrightness},
        {"colorSaturation",       v.colorSaturation},
        {"colorContrast",         v.colorContrast},
        {"colorGamma",            v.colorGamma},
        {"colorMirror",           v.colorMirror},
        // Depth camera
        {"depthAutoExposure",     v.depthAutoExposure},
        {"depthExposure",         v.depthExposure},
        {"depthGain",             v.depthGain},
        {"depthPrecisionLevel",   v.depthPrecisionLevel},
        {"depthMirror",           v.depthMirror},
        {"minDepth",              v.minDepth},
        {"maxDepth",              v.maxDepth},
        // IR / Laser / LDP
        {"irExposure",            v.irExposure},
        {"irGain",                v.irGain},
        {"laserOn",               v.laserOn},
        {"ldpOn",                 v.ldpOn},
        // Alignment / sync
        {"alignMode",             static_cast<int>(v.alignMode)},
        {"frameSync",             v.frameSync},
        {"aggregateAllRequired",  v.aggregateAllRequired},
        // Depth denoising
        {"hwNoiseRemovalEnabled", v.hwNoiseRemovalEnabled},
        {"hwNoiseMaxLut",         v.hwNoiseMaxLut},
        {"hwNoiseMinDiff",        v.hwNoiseMinDiff},
        {"denoiseFilterEnabled",  v.denoiseFilterEnabled},
        {"denoiseMaxSize",        v.denoiseMaxSize},
        {"denoiseDispDiff",       v.denoiseDispDiff},
        {"spatialFilterEnabled",  v.spatialFilterEnabled},
        {"spatialAlpha",          v.spatialAlpha},
        {"spatialRadius",         v.spatialRadius},
        {"spatialMagnitude",      v.spatialMagnitude},
        {"spatialDispDiff",       v.spatialDispDiff},
        {"temporalFilterEnabled", v.temporalFilterEnabled},
        {"temporalWeight",        v.temporalWeight},
        {"temporalDiffScale",     v.temporalDiffScale},
        // Point cloud
        {"pointCloudUseDenoisedDepth", v.pointCloudUseDenoisedDepth},
        {"pointCloudAlignMode",   static_cast<int>(v.pointCloudAlignMode)},
        // Depth distance filter
        {"depthDistFilterEnabled", v.depthDistFilterEnabled},
        {"depthDistMinMm",        v.depthDistMinMm},
        {"depthDistMaxMm",        v.depthDistMaxMm}};
}
inline void from_json(const nlohmann::json& j, CameraParams& v) {
    v.colorAutoExposure     = j.value("colorAutoExposure",     true);
    v.colorExposure         = j.value("colorExposure",         -1);
    v.colorGain             = j.value("colorGain",             -1);
    v.colorAutoWhiteBalance = j.value("colorAutoWhiteBalance", true);
    v.colorWhiteBalance     = j.value("colorWhiteBalance",     -1);
    v.colorBrightness       = j.value("colorBrightness",       -1);
    v.colorSaturation       = j.value("colorSaturation",       -1);
    v.colorContrast         = j.value("colorContrast",         -1);
    v.colorGamma            = j.value("colorGamma",            -1);
    v.colorMirror           = j.value("colorMirror",           false);
    v.depthAutoExposure     = j.value("depthAutoExposure",     true);
    v.depthExposure         = j.value("depthExposure",         -1);
    v.depthGain             = j.value("depthGain",             -1);
    v.depthPrecisionLevel   = j.value("depthPrecisionLevel",   -1);
    v.depthMirror           = j.value("depthMirror",           false);
    v.minDepth              = j.value("minDepth",              -1);
    v.maxDepth              = j.value("maxDepth",              -1);
    v.irExposure            = j.value("irExposure",            -1);
    v.irGain                = j.value("irGain",                -1);
    v.laserOn               = j.value("laserOn",               true);
    v.ldpOn                 = j.value("ldpOn",                 false);
    v.alignMode             = static_cast<AlignMode>(j.value("alignMode", 0));
    v.frameSync             = j.value("frameSync",             true);
    v.aggregateAllRequired  = j.value("aggregateAllRequired",  true);
    v.hwNoiseRemovalEnabled = j.value("hwNoiseRemovalEnabled", false);
    v.hwNoiseMaxLut         = j.value("hwNoiseMaxLut",         -1);
    v.hwNoiseMinDiff        = j.value("hwNoiseMinDiff",        -1);
    v.denoiseFilterEnabled  = j.value("denoiseFilterEnabled",  false);
    v.denoiseMaxSize        = j.value("denoiseMaxSize",        -1);
    v.denoiseDispDiff       = j.value("denoiseDispDiff",       -1);
    v.spatialFilterEnabled  = j.value("spatialFilterEnabled",  false);
    v.spatialAlpha          = j.value("spatialAlpha",          -1.f);
    v.spatialRadius         = j.value("spatialRadius",         -1);
    v.spatialMagnitude      = j.value("spatialMagnitude",      -1);
    v.spatialDispDiff       = j.value("spatialDispDiff",       -1);
    v.temporalFilterEnabled = j.value("temporalFilterEnabled", false);
    v.temporalWeight        = j.value("temporalWeight",        -1.f);
    v.temporalDiffScale     = j.value("temporalDiffScale",     -1.f);
    v.pointCloudUseDenoisedDepth = j.value("pointCloudUseDenoisedDepth", true);
    v.pointCloudAlignMode   = static_cast<AlignMode>(j.value("pointCloudAlignMode", 3));
    v.depthDistFilterEnabled = j.value("depthDistFilterEnabled", false);
    v.depthDistMinMm        = j.value("depthDistMinMm",        300.0f);
    v.depthDistMaxMm        = j.value("depthDistMaxMm",        5000.0f);
}

// On-disk container for a device's camera parameters + stream config.
struct CameraParamsFile {
    std::string   deviceSerial;
    std::string   deviceName;
    CameraParams  params;
    StreamConfig  colorCfg;
    StreamConfig  depthCfg;
    std::string   timestamp;   // ISO-8601 of when it was saved
    std::string   filePath;    // set after load/save
};

inline void to_json(nlohmann::json& j, const CameraParamsFile& v) {
    j = nlohmann::json{
        {"deviceSerial", v.deviceSerial},
        {"deviceName",   v.deviceName},
        {"params",       v.params},
        {"colorCfg",     v.colorCfg},
        {"depthCfg",     v.depthCfg},
        {"timestamp",    v.timestamp}};
}
inline void from_json(const nlohmann::json& j, CameraParamsFile& v) {
    v.deviceSerial = j.value("deviceSerial", std::string{});
    v.deviceName   = j.value("deviceName",   std::string{});
    if (j.contains("params"))   j.at("params").get_to(v.params);
    if (j.contains("colorCfg")) j.at("colorCfg").get_to(v.colorCfg);
    if (j.contains("depthCfg")) j.at("depthCfg").get_to(v.depthCfg);
    v.timestamp    = j.value("timestamp",    std::string{});
}

} // namespace orbbec
