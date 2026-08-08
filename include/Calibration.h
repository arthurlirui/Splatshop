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

inline void to_json(nlohmann::json& j, const DeviceCalibration& v) {
    j = nlohmann::json{
        {"deviceSerial", v.deviceSerial},
        {"deviceName",   v.deviceName},
        {"color",        v.color},
        {"ir",           v.ir},
        {"depth",        v.depth},
        {"notes",        v.notes}};
}
inline void from_json(const nlohmann::json& j, DeviceCalibration& v) {
    v.deviceSerial = j.value("deviceSerial", std::string{});
    v.deviceName   = j.value("deviceName",   std::string{});
    if (j.contains("color")) j.at("color").get_to(v.color);
    if (j.contains("ir"))    j.at("ir").get_to(v.ir);
    if (j.contains("depth")) j.at("depth").get_to(v.depth);
    v.notes = j.value("notes", std::string{});
}

} // namespace orbbec
