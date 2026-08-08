#pragma once

// CalibrationStore.h
//
// Load / save orbbec::DeviceCalibration to JSON files on disk and locate a
// calibration file for a given device serial number + resolution.
//
// Files are written under <exeDir>/calibration/<serial>_<w>x<h>.json by
// default; the user may also pass an explicit path from the GUI.

#include <string>

#include "Calibration.h"

namespace orbbec {

class CalibrationStore {
public:
    // Save to the default per-device path derived from serial + resolution.
    // `refW/refH` are the reference resolution used to build the filename
    // (typically the color stream resolution). Returns the written path in
    // `outPath` on success.
    static bool saveDefault(const DeviceCalibration& cal, int refW, int refH,
                            std::string& outPath);

    // Save to an explicit absolute or relative path.
    static bool save(const DeviceCalibration& cal, const std::string& path);

    // Load from an explicit path.
    static bool load(DeviceCalibration& cal, const std::string& path);

    // Locate and load the default file for a device, if one exists.
    // Returns false (without error) if no matching file is present.
    static bool loadForDevice(DeviceCalibration& cal, const std::string& serial,
                              int refW, int refH);

    // Compose the default file path for a device (does not check existence).
    static std::string defaultPathFor(const std::string& serial,
                                      int refW, int refH);

    // The base directory under which calibration files are stored.
    // Defaults to "<cwd>/calibration" but is resolved once on first use.
    static std::string baseDir();
};

} // namespace orbbec
