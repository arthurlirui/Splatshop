// ExtrinsicCalibrator.cpp
//
// OpenCV-backed implementation of orbbec::ExtrinsicCalibrator. See the header.

#include <cmath>
#include <cstring>
#include <string>

#include "unsuck.hpp"
#include "ExtrinsicCalibrator.h"

#ifdef SPLATSHOP_HAS_OPENCV

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace orbbec {

namespace {

// Wrap a raw byte buffer (with `channels` per pixel) into a non-owning Mat.
// Same helper as Calibrator.cpp - kept local to avoid a shared dependency.
cv::Mat wrapMat(const uint8_t* data, int w, int h, int channels) {
    int type = (channels == 1) ? CV_8UC1 : CV_8UC3;
    return cv::Mat(h, w, type, const_cast<uint8_t*>(data));
}

// Detect the chessboard (SB first, classic fallback) + sub-pixel refine.
// Identical pipeline to Calibrator::detectOnly so a board that calibrates
// the intrinsics also solves a pose here. Fills `corners` in image space.
bool detectBoard(const cv::Mat& gray, int cols, int rows,
                 std::vector<cv::Point2f>& corners) {
    cv::Size pattern(cols, rows);
    bool found = false;
    try {
        found = cv::findChessboardCornersSB(
            gray, pattern, corners,
            cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
    } catch (...) { found = false; }
    if (!found) {
        found = cv::findChessboardCorners(gray, pattern, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }
    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS +
                                           cv::TermCriteria::MAX_ITER, 50, 1e-3));
    }
    return found;
}

} // namespace

struct ExtrinsicCalibrator::Impl {
    // Chessboard spec.
    int   boardCols = 9;
    int   boardRows = 6;
    float squareMm  = 25.f;

    // Cached object points (one template, Z=0 plane in board frame).
    std::vector<cv::Point3f> objectTemplate;

    // Depth-correction samples: {trueMm, measuredMm}.
    std::vector<std::pair<float, float>> depthSamples;
    float fitA = 1.f;
    float fitB = 0.f;
    bool  fitValid = false;

    // Last detection.
    ExtrinsicPose lastPose{};
    bool lastOk = false;
    // Last detected corners (image space) for buildOverlay().
    std::vector<cv::Point2f> lastCorners;

    void rebuildObjectTemplate() {
        objectTemplate.clear();
        objectTemplate.reserve((size_t)boardCols * boardRows);
        for (int r = 0; r < boardRows; ++r)
            for (int c = 0; c < boardCols; ++c)
                objectTemplate.emplace_back(c * squareMm, r * squareMm, 0.f);
    }
};

ExtrinsicCalibrator::ExtrinsicCalibrator() : impl(std::make_unique<Impl>()) {
    impl->rebuildObjectTemplate();
}
ExtrinsicCalibrator::~ExtrinsicCalibrator() = default;

void ExtrinsicCalibrator::setChessboard(int cols, int rows, float squareSizeMm) {
    if (cols < 2) cols = 2;
    if (rows < 2) rows = 2;
    if (squareSizeMm <= 0.f) squareSizeMm = 1.f;
    impl->boardCols = cols;
    impl->boardRows = rows;
    impl->squareMm  = squareSizeMm;
    impl->rebuildObjectTemplate();
    // Spec changed: prior samples are no longer comparable - drop them.
    impl->depthSamples.clear();
    impl->fitValid = false;
    impl->fitA = 1.f; impl->fitB = 0.f;
}

int  ExtrinsicCalibrator::chessCols() const { return impl->boardCols; }
int  ExtrinsicCalibrator::chessRows() const { return impl->boardRows; }
float ExtrinsicCalibrator::squareSizeMm() const { return impl->squareMm; }

bool ExtrinsicCalibrator::solvePose(const uint8_t* img, int w, int h, int channels,
                                    const Intrinsics& intrinsics,
                                    const DistortionCoeffs& dist,
                                    ExtrinsicPose& outPose) {
    outPose.valid = false;
    impl->lastOk = false;
    impl->lastCorners.clear();
    impl->lastPose.valid = false;
    if (!img || w <= 0 || h <= 0) return false;

    cv::Mat raw = wrapMat(img, w, h, channels);
    cv::Mat gray;
    if (channels == 1) gray = raw;
    else               cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    if (!detectBoard(gray, impl->boardCols, impl->boardRows, corners)) return false;

    // Camera matrix + distortion from the orbbec types.
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        intrinsics.fx, 0.0, intrinsics.cx,
        0.0, intrinsics.fy, intrinsics.cy,
        0.0, 0.0, 1.0);
    cv::Mat dcMat = (cv::Mat_<double>(1, 5) <<
        dist.k1, dist.k2, dist.p1, dist.p2, dist.k3);

    cv::Mat rvec, tvec;
    bool ok = false;
    try {
        // IPPE is the recommended planar-tag PnP method; it needs >= 4
        // coplanar points, which a chessboard always provides. Fall back to
        // the iterative method if IPPE is unavailable for the configuration.
        ok = cv::solvePnP(impl->objectTemplate, corners, K, dcMat,
                          rvec, tvec, false, cv::SOLVEPNP_IPPE);
        if (!ok) {
            ok = cv::solvePnP(impl->objectTemplate, corners, K, dcMat,
                              rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        }
    } catch (const cv::Exception& e) {
        println("ExtrinsicCalibrator: solvePnP failed: {}", e.what());
        ok = false;
    }

    impl->lastCorners = std::move(corners);
    if (!ok) return false;

    double tx = tvec.at<double>(0, 0);
    double ty = tvec.at<double>(1, 0);
    double tz = tvec.at<double>(2, 0);
    double distMm = std::sqrt(tx*tx + ty*ty + tz*tz);

    outPose.rvec[0] = (float)rvec.at<double>(0, 0);
    outPose.rvec[1] = (float)rvec.at<double>(1, 0);
    outPose.rvec[2] = (float)rvec.at<double>(2, 0);
    outPose.tvec[0] = (float)tx;
    outPose.tvec[1] = (float)ty;
    outPose.tvec[2] = (float)tz;
    outPose.distanceMm = (float)distMm;
    outPose.valid = true;

    impl->lastPose = outPose;
    impl->lastOk = true;
    return true;
}

void ExtrinsicCalibrator::addSample(float trueMm, float measuredMm) {
    if (trueMm <= 0.f || measuredMm <= 0.f) return;
    impl->depthSamples.emplace_back(trueMm, measuredMm);
}

void ExtrinsicCalibrator::removeLastSample() {
    if (!impl->depthSamples.empty()) impl->depthSamples.pop_back();
}

void ExtrinsicCalibrator::clearSamples() {
    impl->depthSamples.clear();
    impl->fitValid = false;
    impl->fitA = 1.f; impl->fitB = 0.f;
}

int ExtrinsicCalibrator::sampleCount() const {
    return (int)impl->depthSamples.size();
}

const std::vector<std::pair<float, float>>& ExtrinsicCalibrator::samples() const {
    return impl->depthSamples;
}

bool ExtrinsicCalibrator::fitDepthCorrection(float& outA, float& outB, double& outRms) {
    outA = 1.f; outB = 0.f; outRms = -1.0;
    const auto& s = impl->depthSamples;
    int n = (int)s.size();
    if (n < 2) {
        println("ExtrinsicCalibrator: need >= 2 samples to fit, have {}", n);
        impl->fitValid = false;
        return false;
    }

    // Least-squares: true = a * measured + b.  true -> y, measured -> x.
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const auto& p : s) {
        double x = p.second;  // measured
        double y = p.first;   // true
        sx  += x;
        sy  += y;
        sxx += x * x;
        sxy += x * y;
    }
    double denom = (double)n * sxx - sx * sx;
    if (std::abs(denom) < 1e-12) {
        println("ExtrinsicCalibrator: degenerate fit (zero variance)");
        impl->fitValid = false;
        return false;
    }
    double a = ((double)n * sxy - sx * sy) / denom;
    double b = (sy - a * sx) / (double)n;

    // RMS residual (mm).
    double err2 = 0.0;
    for (const auto& p : s) {
        double x = p.second, y = p.first;
        double r = y - (a * x + b);
        err2 += r * r;
    }
    double rms = std::sqrt(err2 / (double)n);

    outA = (float)a;
    outB = (float)b;
    outRms = rms;
    impl->fitA = (float)a;
    impl->fitB = (float)b;
    impl->fitValid = true;
    println("ExtrinsicCalibrator: fit depth' = {:.6f} * depth + {:.3f} mm  (RMS {:.3f} mm, N={})",
            a, b, rms, n);
    return true;
}

float ExtrinsicCalibrator::a() const { return impl->fitA; }
float ExtrinsicCalibrator::b() const { return impl->fitB; }

float ExtrinsicCalibrator::correctMm(float measuredMm) const {
    return impl->fitA * measuredMm + impl->fitB;
}

ExtrinsicPose ExtrinsicCalibrator::lastPose() const { return impl->lastPose; }
bool ExtrinsicCalibrator::lastDetectionOk() const { return impl->lastOk; }

bool ExtrinsicCalibrator::buildOverlay(const uint8_t* src, int w, int h, int channels,
                                       std::vector<uint8_t>& outBgr, int& outW, int& outH,
                                       const Intrinsics& intrinsics,
                                       const DistortionCoeffs& dist) {
    if (!src || w <= 0 || h <= 0) return false;
    cv::Mat raw = wrapMat(src, w, h, channels);
    cv::Mat bgr;
    if (channels == 1) cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
    else               bgr = raw.clone();

    if (impl->lastOk && !impl->lastCorners.empty()) {
        cv::drawChessboardCorners(bgr,
            cv::Size(impl->boardCols, impl->boardRows), impl->lastCorners, true);

        // Project the board's coordinate axes so the pose is visually
        // verifiable: origin + one square along each axis.
        if (impl->lastPose.valid) {
            cv::Mat K = (cv::Mat_<double>(3, 3) <<
                intrinsics.fx, 0.0, intrinsics.cx,
                0.0, intrinsics.fy, intrinsics.cy,
                0.0, 0.0, 1.0);
            cv::Mat dc = (cv::Mat_<double>(1, 5) <<
                dist.k1, dist.k2, dist.p1, dist.p2, dist.k3);
            cv::Mat rvec = (cv::Mat_<double>(3, 1) <<
                impl->lastPose.rvec[0], impl->lastPose.rvec[1], impl->lastPose.rvec[2]);
            cv::Mat tvec = (cv::Mat_<double>(3, 1) <<
                impl->lastPose.tvec[0], impl->lastPose.tvec[1], impl->lastPose.tvec[2]);
            // Origin + one square along each board axis. Z points towards
            // the camera (board frame), so the -Z axis is drawn into the board.
            std::vector<cv::Point3f> axisPts = {
                {0.f, 0.f, 0.f},
                {impl->squareMm, 0.f, 0.f},
                {0.f, impl->squareMm, 0.f},
                {0.f, 0.f, -impl->squareMm}
            };
            std::vector<cv::Point2f> proj;
            try {
                cv::projectPoints(axisPts, rvec, tvec, K, dc, proj);
            } catch (...) { proj.clear(); }
            if (proj.size() == 4) {
                auto line = [&](int i, int j, const cv::Scalar& c) {
                    cv::line(bgr, proj[i], proj[j], c, 2, cv::LINE_AA);
                };
                line(0, 1, cv::Scalar(0, 0, 255));   // X = red (BGR)
                line(0, 2, cv::Scalar(0, 255, 0));   // Y = green
                line(0, 3, cv::Scalar(255, 0, 0));   // Z = blue
            }
        }
    }

    outW = w; outH = h;
    outBgr.assign(bgr.data, bgr.data + (size_t)w * h * 3);
    return true;
}

} // namespace orbbec

#endif // SPLATSHOP_HAS_OPENCV
