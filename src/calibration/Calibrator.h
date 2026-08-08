#pragma once

// Calibrator.h
//
// OpenCV-backed lens calibration engine. Detects chessboard corners in
// captured frames, runs cv::calibrateCamera (5-parameter Brown-Conrady
// model: k1, k2, p1, p2, k3), evaluates per-frame reprojection error, and
// builds cached undistort LUTs (cv::initUndistortRectifyMap) for real-time
// preview undistortion.
//
// All OpenCV headers are kept inside this translation unit's .cpp; the
// public interface exposes only std-ty types + orbbec::StreamCalibration so
// callers (the GUI) do not need OpenCV on the include path.

#include <memory>
#include <string>
#include <vector>

#include "Calibration.h"

namespace orbbec {

enum class CalibStream {
    COLOR = 0,
    IR    = 1,
    DEPTH = 2,
};

#ifdef SPLATSHOP_HAS_OPENCV

class Calibrator {
public:
    Calibrator();
    ~Calibrator();

    // --- Chessboard specification ---------------------------------------
    // `cols`/`rows` are the number of *inner* corners (not squares).
    void setChessboard(int cols, int rows, float squareSizeMm);
    int  chessCols() const;
    int  chessRows() const;
    float squareSizeMm() const;

    // --- Frame ingestion -------------------------------------------------
    // Detect chessboard corners in `img` (single-channel or BGR). Returns
    // true if the full pattern was found and stores the corners for the
    // live-preview overlay (accessible via lastCorners()). The frame is NOT
    // retained as a calibration sample.
    bool detectOnly(const uint8_t* data, int w, int h, int channels);

    // Like detectOnly but also stores the image + corners as a calibration
    // sample. Returns true if the pattern was found and the sample was
    // added. Optional `poseDiverse` heuristics can be used to skip near-
    // duplicate poses - pass true to enable.
    bool detectAndAddFrame(const uint8_t* data, int w, int h, int channels,
                           bool poseDiverse = true);

    // Remove the most recently added sample.
    void removeLastSample();

    // Drop all captured samples (and any cached calibration).
    void clear();

    int  sampleCount() const;

    // Per-frame reprojection error for each stored sample (populated after
    // runCalibration()). Length == sampleCount(); -1 if not yet calibrated.
    const std::vector<double>& perViewErrors() const;

    // --- Calibration -----------------------------------------------------
    // Run cv::calibrateCamera over the stored samples. On success fills
    // `out` and returns the RMS reprojection error (pixels). Returns < 0 on
    // failure (too few samples, no convergence). `fixAspectRatio` locks
    // fx/fy. `alpha` is accepted for API symmetry with
    // ensureUndistortMaps() but is not applied here - the new-camera-matrix
    // crop it controls is computed when the undistort maps are built.
    double runCalibration(StreamCalibration& out,
                          bool fixAspectRatio = false,
                          float alpha = 0.f);

    // --- Undistortion ----------------------------------------------------
    // Build (or rebuild if resolution/calibration/alpha changed) the remap
    // LUT for the given calibration. Thread-safe to call repeatedly; cheap
    // if cached. `alpha` controls the new-camera-matrix crop (0 = all valid
    // pixels, 1 = keep all source pixels incl. black borders); changing it
    // or the calibration contents invalidates the cache.
    void ensureUndistortMaps(const StreamCalibration& cal, int w, int h, float alpha = 0.f);

    // Apply the cached remap LUT to `src`. For uint16 depth data
    // `isDepth=true` selects nearest-neighbour interpolation so no synthetic
    // depth values are introduced. The output has the same type/size as the
    // input. Maps must have been built for the matching resolution.
    void undistort(const uint8_t* src, uint8_t* dst, int w, int h, int channels,
                   bool isDepth = false);

    // Variant for uint16 depth data (raw depth pixels). Uses nearest-neighbour
    // interpolation so no synthetic depth values are introduced. The IR
    // stream's distortion calibration is shared with the depth stream.
    void undistortDepth(const uint16_t* src, uint16_t* dst, int w, int h);

    bool hasUndistortMaps(int w, int h) const;

    // --- Live-preview overlay -------------------------------------------
    // Corners found by the most recent detectOnly()/detectAndAddFrame().
    // Empty if the last detection failed. Coordinates are in image pixels.
    struct Vec2 { float x, y; };
    const std::vector<Vec2>& lastCorners() const;
    bool lastDetectionOk() const;

    // Access the stored sample images as raw bytes for thumbnail display.
    // Each entry is w*h*channels bytes (BGR, channels==3).
    struct Sample {
        std::vector<uint8_t> pixels;   // BGR8
        int w = 0, h = 0;
    };
    const std::vector<Sample>& samples() const;

    // Helper: render the latest detection overlay into a BGR8 buffer
    // suitable for ImGui display (draws the corner pattern + indices).
    // `outW/outH` receive the buffer dimensions. Returns false if no frame
    // has been fed yet.
    bool buildOverlay(const uint8_t* src, int w, int h, int channels,
                      std::vector<uint8_t>& outBgr, int& outW, int& outH);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

#else
// -----------------------------------------------------------------------
// Stub when OpenCV is unavailable. Provides the same API as no-op /
// always-fail inline methods so any translation unit that includes this
// header (even without guarding on SPLATSHOP_HAS_OPENCV) links cleanly.
// Mirrors the stub pattern used by PointCloudBA.h.
// -----------------------------------------------------------------------
class Calibrator {
public:
    struct Vec2 { float x, y; };
    struct Sample { std::vector<uint8_t> pixels; int w = 0, h = 0; };

    inline Calibrator() = default;
    inline ~Calibrator() = default;
    inline void setChessboard(int, int, float) {}
    inline int  chessCols() const { return 0; }
    inline int  chessRows() const { return 0; }
    inline float squareSizeMm() const { return 0.f; }
    inline bool detectOnly(const uint8_t*, int, int, int) { return false; }
    inline bool detectAndAddFrame(const uint8_t*, int, int, int, bool = true) { return false; }
    inline void removeLastSample() {}
    inline void clear() {}
    inline int  sampleCount() const { return 0; }
    inline const std::vector<double>& perViewErrors() const { static const std::vector<double> e; return e; }
    inline double runCalibration(StreamCalibration& out, bool = false, float = 0.f) {
        out.valid = false; return -1.0;
    }
    inline void ensureUndistortMaps(const StreamCalibration&, int, int, float = 0.f) {}
    inline void undistort(const uint8_t*, uint8_t*, int, int, int, bool = false) {}
    inline void undistortDepth(const uint16_t*, uint16_t*, int, int) {}
    inline bool hasUndistortMaps(int, int) const { return false; }
    inline const std::vector<Vec2>& lastCorners() const { static const std::vector<Vec2> e; return e; }
    inline bool lastDetectionOk() const { return false; }
    inline const std::vector<Sample>& samples() const { static const std::vector<Sample> e; return e; }
    inline bool buildOverlay(const uint8_t*, int, int, int,
                             std::vector<uint8_t>&, int&, int&) { return false; }
};
#endif // SPLATSHOP_HAS_OPENCV

} // namespace orbbec
