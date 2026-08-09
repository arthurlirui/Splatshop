#pragma once

// ExtrinsicCalibrator.h
//
// Extrinsic calibration (board pose via solvePnP) + depth metric correction.
//
// Detects a chessboard in the live IR frame, solves the board's pose relative
// to the camera (Rodrigues rvec + translation tvec in mm), and uses the
// pose-derived ground-truth distance to fit a linear depth correction
//   depth_true_mm = a * depth_measured_mm + b
// over a set of (true, measured) samples collected at different distances.
//
// The board-pose solver reuses the same chessboard detection pipeline as
// Calibrator (findChessboardCornersSB -> findChessboardCorners fallback ->
// cornerSubPix), then runs cv::solvePnP. The depth correction is a simple
// least-squares line fit; it corrects the camera's systematic metric scale /
// bias error, not lens distortion (that is the Calibrator's job).
//
// All OpenCV headers are kept inside the .cpp; the public interface exposes
// only std/orbbec types so callers (the GUI) do not need OpenCV on the
// include path - mirroring Calibrator.h.

#include <memory>
#include <vector>

#include "Calibration.h"   // Intrinsics, DistortionCoeffs, ExtrinsicPose, DepthCorrection

namespace orbbec {

#ifdef SPLATSHOP_HAS_OPENCV

class ExtrinsicCalibrator {
public:
    ExtrinsicCalibrator();
    ~ExtrinsicCalibrator();

    // --- Chessboard specification ---------------------------------------
    // `cols`/`rows` are the number of *inner* corners (not squares), same
    // convention as Calibrator. `squareSizeMm` sets the translation unit.
    void setChessboard(int cols, int rows, float squareSizeMm);
    int  chessCols() const;
    int  chessRows() const;
    float squareSizeMm() const;

    // --- Board pose (solvePnP) ------------------------------------------
    // Detect the chessboard in `img` (single-channel or BGR) and solve its
    // pose relative to the camera using `intrinsics`/`distortion`. On
    // success fills `outPose` (rvec = Rodrigues rotation, tvec = translation
    // in mm, distanceMm = |tvec|) and returns true. Also caches the pose for
    // lastPose()/buildOverlay(). Returns false if the pattern is not found.
    bool solvePose(const uint8_t* img, int w, int h, int channels,
                   const Intrinsics& intrinsics, const DistortionCoeffs& dist,
                   ExtrinsicPose& outPose);

    // --- Depth correction samples ---------------------------------------
    // Collect (groundTruthMm, measuredMm) pairs. The ground truth typically
    // comes from a solved pose's distanceMm; the measured value is the
    // depth reading (pixel * depthScale) averaged over the board region.
    void addSample(float trueMm, float measuredMm);
    void removeLastSample();
    void clearSamples();
    int  sampleCount() const;

    // Access the stored samples for GUI display. Each entry is {trueMm,
    // measuredMm}.
    const std::vector<std::pair<float, float>>& samples() const;

    // --- Fit depth_true_mm = a * depth_measured_mm + b ------------------
    // Least-squares fit over the stored samples (true as y, measured as x).
    // Returns false if fewer than 2 samples or a degenerate fit. On success
    // fills outA/outB/outRms (RMS residual in mm) and caches the result.
    bool fitDepthCorrection(float& outA, float& outB, double& outRms);

    // Current cached coefficients (after fitDepthCorrection; a=1, b=0 until
    // a successful fit).
    float a() const;
    float b() const;

    // Apply the cached correction to a single measured depth (mm).
    float correctMm(float measuredMm) const;

    // --- Live-preview overlay -------------------------------------------
    // The most recent solvePose() result. `.valid == false` if the last
    // detection failed or no frame has been fed yet.
    ExtrinsicPose lastPose() const;
    bool lastDetectionOk() const;

    // Render the latest detection overlay into a BGR8 buffer suitable for
    // ImGui display: draws the corner pattern, and - when a pose was solved -
    // projects the board's coordinate axes (X red, Y green, Z blue) so the
    // pose is visually verifiable. `outW/outH` receive the buffer dims.
    bool buildOverlay(const uint8_t* src, int w, int h, int channels,
                      std::vector<uint8_t>& outBgr, int& outW, int& outH,
                      const Intrinsics& intrinsics, const DistortionCoeffs& dist);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

#else
// -----------------------------------------------------------------------
// Stub when OpenCV is unavailable. Same API as no-op / always-fail inline
// methods, mirroring the Calibrator stub pattern.
// -----------------------------------------------------------------------
class ExtrinsicCalibrator {
public:
    inline ExtrinsicCalibrator() = default;
    inline ~ExtrinsicCalibrator() = default;
    inline void setChessboard(int, int, float) {}
    inline int  chessCols() const { return 0; }
    inline int  chessRows() const { return 0; }
    inline float squareSizeMm() const { return 0.f; }
    inline bool solvePose(const uint8_t*, int, int, int,
                          const Intrinsics&, const DistortionCoeffs&,
                          ExtrinsicPose& out) { out.valid = false; return false; }
    inline void addSample(float, float) {}
    inline void removeLastSample() {}
    inline void clearSamples() {}
    inline int  sampleCount() const { return 0; }
    inline const std::vector<std::pair<float, float>>& samples() const {
        static const std::vector<std::pair<float, float>> e; return e;
    }
    inline bool fitDepthCorrection(float& a, float& b, double& rms) {
        a = 1.f; b = 0.f; rms = -1.0; return false;
    }
    inline float a() const { return 1.f; }
    inline float b() const { return 0.f; }
    inline float correctMm(float mm) const { return mm; }
    inline ExtrinsicPose lastPose() const { return ExtrinsicPose{}; }
    inline bool lastDetectionOk() const { return false; }
    inline bool buildOverlay(const uint8_t*, int, int, int,
                             std::vector<uint8_t>&, int&, int&,
                             const Intrinsics&, const DistortionCoeffs&) { return false; }
};
#endif // SPLATSHOP_HAS_OPENCV

} // namespace orbbec
