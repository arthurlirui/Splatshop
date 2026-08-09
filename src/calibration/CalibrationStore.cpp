// CalibrationStore.cpp
//
// See CalibrationStore.h for the public interface.

#include <fstream>
#include <sstream>
#include <filesystem>

#include "unsuck.hpp"
#include "CalibrationStore.h"

namespace orbbec {

namespace {
// Wrap the nlohmann json object in a helper that writes/reads pretty JSON.
bool writeJson(const nlohmann::json& j, const std::string& path) {
    try {
        std::filesystem::path p(path);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
        std::ofstream ofs(p, std::ios::binary);
        if (!ofs) {
            println("CalibrationStore: failed to open '{}' for writing", path);
            return false;
        }
        ofs << j.dump(2);
        return true;
    } catch (const std::exception& e) {
        println("CalibrationStore: write failed: {}", e.what());
        return false;
    }
}

bool readJson(nlohmann::json& j, const std::string& path) {
    try {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) return false;
        std::stringstream ss;
        ss << ifs.rdbuf();
        j = nlohmann::json::parse(ss.str(), nullptr, true);
        return true;
    } catch (const std::exception& e) {
        println("CalibrationStore: parse failed for '{}': {}", path, e.what());
        return false;
    }
}
} // namespace

std::string CalibrationStore::baseDir() {
    // Co-located with the executable / working directory so files survive
    // alongside OrbbecSDKConfig.xml etc. Using the current working directory
    // keeps it portable and predictable from the GUI.
    return (std::filesystem::current_path() / "calibration").string();
}

std::string CalibrationStore::defaultPathFor(const std::string& serial,
                                             int refW, int refH) {
    std::string sn = serial.empty() ? "unknown" : serial;
    // Sanitise the serial so it is filesystem-safe.
    for (char& c : sn) {
        if (c == '/' || c == '\\' || c == ':' || c == ' ' || c == '?')
            c = '_';
    }
    std::string name = format("{}_{}x{}.json", sn, refW, refH);
    return (std::filesystem::path(baseDir()) / name).string();
}

bool CalibrationStore::save(const DeviceCalibration& cal, const std::string& path) {
    nlohmann::json j = cal;
    if (!writeJson(j, path)) return false;
    println("CalibrationStore: saved '{}' for device '{}'", path, cal.deviceSerial);
    return true;
}

bool CalibrationStore::saveDefault(const DeviceCalibration& cal, int refW, int refH,
                                   std::string& outPath) {
    outPath = defaultPathFor(cal.deviceSerial, refW, refH);
    return save(cal, outPath);
}

bool CalibrationStore::load(DeviceCalibration& cal, const std::string& path) {
    nlohmann::json j;
    if (!readJson(j, path)) return false;
    try {
        cal = j.get<DeviceCalibration>();
        cal.filePath = path;
    } catch (const std::exception& e) {
        println("CalibrationStore: schema mismatch for '{}': {}", path, e.what());
        return false;
    }
    println("CalibrationStore: loaded '{}'", path);
    return true;
}

bool CalibrationStore::loadForDevice(DeviceCalibration& cal, const std::string& serial,
                                     int refW, int refH) {
    std::string path = defaultPathFor(serial, refW, refH);
    if (!std::filesystem::exists(path)) return false;
    return load(cal, path);
}

// ---------------------------------------------------------------------------
// Camera parameters (device control + depth filters + stream config).
// Reuses the private writeJson/readJson helpers above; same JSON format and
// calibration/ directory as the lens-calibration files.
// ---------------------------------------------------------------------------

std::string CalibrationStore::cameraParamsPathFor(const std::string& serial,
                                                  int refW, int refH) {
    std::string sn = serial.empty() ? "unknown" : serial;
    for (char& c : sn) {
        if (c == '/' || c == '\\' || c == ':' || c == ' ' || c == '?')
            c = '_';
    }
    std::string name = format("{}_params_{}x{}.json", sn, refW, refH);
    return (std::filesystem::path(baseDir()) / name).string();
}

bool CalibrationStore::saveCameraParams(const CameraParamsFile& cpf,
                                        const std::string& path) {
    nlohmann::json j = cpf;
    if (!writeJson(j, path)) return false;
    println("CalibrationStore: saved camera params '{}' for device '{}'",
            path, cpf.deviceSerial);
    return true;
}

bool CalibrationStore::loadCameraParams(CameraParamsFile& cpf,
                                        const std::string& path) {
    nlohmann::json j;
    if (!readJson(j, path)) return false;
    try {
        cpf = j.get<CameraParamsFile>();
        cpf.filePath = path;
    } catch (const std::exception& e) {
        println("CalibrationStore: camera-params schema mismatch for '{}': {}",
                path, e.what());
        return false;
    }
    println("CalibrationStore: loaded camera params '{}'", path);
    return true;
}

bool CalibrationStore::loadCameraParamsForDevice(CameraParamsFile& cpf,
                                                 const std::string& serial,
                                                 int refW, int refH) {
    std::string path = cameraParamsPathFor(serial, refW, refH);
    if (!std::filesystem::exists(path)) return false;
    return loadCameraParams(cpf, path);
}

} // namespace orbbec
