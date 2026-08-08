// Calibrator.cpp
//
// OpenCV-backed implementation of orbbec::Calibrator. See Calibrator.h.

#include <chrono>
#include <ctime>
#include <string>
#include <cmath>
#include <cstring>

#include "unsuck.hpp"
#include "Calibrator.h"

#ifdef SPLATSHOP_HAS_OPENCV

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace orbbec {

namespace {

// Build an ISO-8601 timestamp string for the current UTC time.
std::string nowIso8601() {
    using std::chrono::system_clock;
    auto t  = system_clock::to_time_t(system_clock::now());
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

// Convert a raw byte buffer (with `channels` per pixel) into an OpenCV Mat.
// The Mat does NOT own the data — callers must keep the source alive while
// the Mat is in use.
cv::Mat wrapMat(const uint8_t* data, int w, int h, int channels) {
    int type = (channels == 1) ? CV_8UC1 : CV_8UC3;
    return cv::Mat(h, w, type, const_cast<uint8_t*>(data));
}

} // namespace

struct Calibrator::Impl {
    // Chessboard spec.
    int   boardCols = 9;       // inner corners
    int   boardRows = 6;
    float squareMm  = 25.f;

    // Captured samples (BGR8) + their detected image points.
    std::vector<Sample>    sampleImgs;
    std::vector<std::vector<cv::Point2f>> samplePoints;

    // Cached object points (one template, replicated per sample).
    std::vector<cv::Point3f> objectTemplate;

    // Per-view reprojection errors after runCalibration().
    std::vector<double> perViewErrs;

    // Last detection (for live overlay).
    std::vector<Vec2>  lastCornersBuf;
    bool               lastOk = false;
    cv::Size           lastSize{0, 0};

    // Undistort LUT cache. Keyed on (resolution, alpha, source calibration
    // contents) so recalibration or an alpha change rebuilds the maps instead
    // of silently reusing a stale LUT.
    cv::Mat mapX, mapY;
    int     mapW = 0, mapH = 0;
    float   mapAlpha = 0.f;
    // Fingerprint of the calibration the maps were built from (zero until a
    // successful build, so the first build always runs).
    float   mapFx = 0.f, mapFy = 0.f, mapCx = 0.f, mapCy = 0.f;
    float   mapK1 = 0.f, mapK2 = 0.f, mapP1 = 0.f, mapP2 = 0.f, mapK3 = 0.f;

    void rebuildObjectTemplate() {
        objectTemplate.clear();
        objectTemplate.reserve((size_t)boardCols * boardRows);
        for (int r = 0; r < boardRows; ++r)
            for (int c = 0; c < boardCols; ++c)
                objectTemplate.emplace_back(
                    c * squareMm, r * squareMm, 0.f);
    }
};

Calibrator::Calibrator() : impl(std::make_unique<Impl>()) {
    impl->rebuildObjectTemplate();
}
Calibrator::~Calibrator() = default;

void Calibrator::setChessboard(int cols, int rows, float squareSizeMm) {
    if (cols < 2) cols = 2;
    if (rows < 2) rows = 2;
    if (squareSizeMm <= 0.f) squareSizeMm = 1.f;
    impl->boardCols = cols;
    impl->boardRows = rows;
    impl->squareMm  = squareSizeMm;
    impl->rebuildObjectTemplate();
    // Invalidate prior samples - the pattern changed. The undistort maps
    // depend only on the calibration result, not the board spec, so they are
    // intentionally left intact (a live-undistorted preview keeps working).
    impl->sampleImgs.clear();
    impl->samplePoints.clear();
    impl->perViewErrs.clear();
}

int  Calibrator::chessCols() const { return impl->boardCols; }
int  Calibrator::chessRows() const { return impl->boardRows; }
float Calibrator::squareSizeMm() const { return impl->squareMm; }

bool Calibrator::detectOnly(const uint8_t* data, int w, int h, int channels) {
    if (!data || w <= 0 || h <= 0) return false;
    cv::Mat raw = wrapMat(data, w, h, channels);
    cv::Mat gray;
    if (channels == 1) gray = raw;
    else               cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);

    cv::Size pattern(impl->boardCols, impl->boardRows);
    std::vector<cv::Point2f> corners;
    bool found = false;
    try {
        // The SB detector is more robust (sector-based) but requires a
        // clean pattern; fall back to the classic detector if it fails.
        found = cv::findChessboardCornersSB(
            gray, pattern, corners,
            cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
    } catch (...) { found = false; }

    if (!found) {
        found = cv::findChessboardCorners(gray, pattern, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }

    if (found) {
        // Sub-pixel refinement on the grayscale image.
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS +
                                           cv::TermCriteria::MAX_ITER, 50, 1e-3));
    }

    impl->lastCornersBuf.clear();
    impl->lastOk = found;
    impl->lastSize = cv::Size(w, h);
    if (found) {
        impl->lastCornersBuf.reserve(corners.size());
        for (const auto& c : corners)
            impl->lastCornersBuf.push_back({c.x, c.y});
    }
    return found;
}

bool Calibrator::detectAndAddFrame(const uint8_t* data, int w, int h, int channels,
                                   bool poseDiverse) {
    if (!detectOnly(data, w, h, channels)) return false;

    // Pose-diversity heuristic: reject samples whose first-corner location
    // is within `tol` pixels of an existing sample's first corner. Cheap
    // proxy for "different enough viewpoint"; the user can always force-add
    // by toggling poseDiverse off.
    if (poseDiverse && !impl->samplePoints.empty()) {
        const float tol = 0.12f * (float)std::min(w, h);
        const auto& cur0 = impl->lastCornersBuf.front();
        bool dup = false;
        for (const auto& s : impl->samplePoints) {
            if (s.empty()) continue;
            float dx = s[0].x - cur0.x;
            float dy = s[0].y - cur0.y;
            if (std::sqrt(dx*dx + dy*dy) < tol) { dup = true; break; }
        }
        if (dup) return false;
    }

    // Store a BGR8 copy of the image + the corners.
    cv::Mat raw = wrapMat(data, w, h, channels);
    Sample s;
    s.w = w; s.h = h;
    s.pixels.resize((size_t)w * h * 3);
    if (channels == 3) {
        std::memcpy(s.pixels.data(), raw.data, s.pixels.size());
    } else {
        cv::Mat bgr;
        cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
        std::memcpy(s.pixels.data(), bgr.data, s.pixels.size());
    }
    impl->sampleImgs.push_back(std::move(s));

    std::vector<cv::Point2f> pts;
    pts.reserve(impl->lastCornersBuf.size());
    for (const auto& c : impl->lastCornersBuf)
        pts.emplace_back(c.x, c.y);
    impl->samplePoints.push_back(std::move(pts));
    return true;
}

void Calibrator::removeLastSample() {
    if (!impl->sampleImgs.empty())  impl->sampleImgs.pop_back();
    if (!impl->samplePoints.empty()) impl->samplePoints.pop_back();
    impl->perViewErrs.clear();
}

void Calibrator::clear() {
    impl->sampleImgs.clear();
    impl->samplePoints.clear();
    impl->perViewErrs.clear();
    impl->lastCornersBuf.clear();
    impl->lastOk = false;
    impl->mapX.release(); impl->mapY.release();
    impl->mapW = impl->mapH = 0;
    impl->mapAlpha = 0.f;
    impl->mapFx = impl->mapFy = impl->mapCx = impl->mapCy = 0.f;
    impl->mapK1 = impl->mapK2 = impl->mapP1 = impl->mapP2 = impl->mapK3 = 0.f;
}

int Calibrator::sampleCount() const {
    return (int)impl->samplePoints.size();
}

const std::vector<double>& Calibrator::perViewErrors() const {
    return impl->perViewErrs;
}

const std::vector<Calibrator::Vec2>& Calibrator::lastCorners() const {
    return impl->lastCornersBuf;
}
bool Calibrator::lastDetectionOk() const { return impl->lastOk; }

const std::vector<Calibrator::Sample>& Calibrator::samples() const {
    return impl->sampleImgs;
}

double Calibrator::runCalibration(StreamCalibration& out, bool fixAspectRatio, float alpha) {
    out.valid = false;
    if ((int)impl->samplePoints.size() < 3) {
        println("Calibrator: need at least 3 samples, have {}", impl->samplePoints.size());
        return -1.0;
    }
    if (impl->samplePoints.empty()) return -1.0;
    int w = impl->sampleImgs.front().w;
    int h = impl->sampleImgs.front().h;
    cv::Size imgSize(w, h);

    // Assemble object/image point arrays.
    std::vector<std::vector<cv::Point3f>> objectPoints(
        impl->samplePoints.size(), impl->objectTemplate);
    const auto& imagePoints = impl->samplePoints;

    cv::Mat cameraMatrix = cv::initCameraMatrix2D(objectPoints, imagePoints, imgSize, 0);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    int flags = 0;
    if (fixAspectRatio) flags |= cv::CALIB_FIX_ASPECT_RATIO;
    // Keep the model to k1,k2,p1,p2,k3 (do NOT set CALIB_RATIONAL_MODEL).

    std::vector<cv::Mat> rvecs, tvecs;
    double rms = -1.0;
    try {
        rms = cv::calibrateCamera(objectPoints, imagePoints, imgSize,
                                  cameraMatrix, distCoeffs, rvecs, tvecs, flags);
    } catch (const cv::Exception& e) {
        println("Calibrator: calibrateCamera failed: {}", e.what());
        return -1.0;
    }

    // Per-view reprojection error.
    impl->perViewErrs.assign(imagePoints.size(), -1.0);
    std::vector<cv::Point2f> proj;
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        try {
            cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i],
                              cameraMatrix, distCoeffs, proj);
            double err = 0.0;
            for (size_t j = 0; j < imagePoints[i].size(); ++j) {
                double dx = proj[j].x - imagePoints[i][j].x;
                double dy = proj[j].y - imagePoints[i][j].y;
                err += dx*dx + dy*dy;
            }
            impl->perViewErrs[i] = std::sqrt(err / imagePoints[i].size());
        } catch (...) { impl->perViewErrs[i] = -1.0; }
    }

    // Fill the StreamCalibration result.
    out.intrinsics.fx = (float)cameraMatrix.at<double>(0, 0);
    out.intrinsics.fy = (float)cameraMatrix.at<double>(1, 1);
    out.intrinsics.cx = (float)cameraMatrix.at<double>(0, 2);
    out.intrinsics.cy = (float)cameraMatrix.at<double>(1, 2);
    out.intrinsics.w  = w;
    out.intrinsics.h  = h;
    out.distortion.k1 = (float)distCoeffs.at<double>(0, 0);
    out.distortion.k2 = (float)distCoeffs.at<double>(0, 1);
    out.distortion.p1 = (float)distCoeffs.at<double>(0, 2);
    out.distortion.p2 = (float)distCoeffs.at<double>(0, 3);
    out.distortion.k3 = (float)distCoeffs.at<double>(0, 4);
    out.rmsReprojectionError = rms;
    out.usedImageCount = (int)imagePoints.size();
    out.timestamp      = nowIso8601();
    out.valid          = true;
    return rms;
}

void Calibrator::ensureUndistortMaps(const StreamCalibration& cal, int w, int h, float alpha) {
    // Cache hit only when resolution, alpha, AND the source calibration
    // contents all match - otherwise a recalibration (new fx/fy/cx/cy or
    // distortion) at the same resolution would silently reuse stale maps.
    if (impl->mapW == w && impl->mapH == h &&
        impl->mapAlpha == alpha && !impl->mapX.empty() &&
        impl->mapFx == cal.intrinsics.fx && impl->mapFy == cal.intrinsics.fy &&
        impl->mapCx == cal.intrinsics.cx && impl->mapCy == cal.intrinsics.cy &&
        impl->mapK1 == cal.distortion.k1 && impl->mapK2 == cal.distortion.k2 &&
        impl->mapP1 == cal.distortion.p1 && impl->mapP2 == cal.distortion.p2 &&
        impl->mapK3 == cal.distortion.k3) {
        return;
    }
    if (!cal.valid || w <= 0 || h <= 0) return;

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        cal.intrinsics.fx, 0.0, cal.intrinsics.cx,
        0.0, cal.intrinsics.fy, cal.intrinsics.cy,
        0.0, 0.0, 1.0);
    cv::Mat dist = (cv::Mat_<double>(1, 5) <<
        cal.distortion.k1, cal.distortion.k2,
        cal.distortion.p1, cal.distortion.p2, cal.distortion.k3);

    cv::Mat newK = cv::getOptimalNewCameraMatrix(
        K, dist, cv::Size(w, h), (double)alpha, cv::Size(w, h), nullptr);

    cv::initUndistortRectifyMap(K, dist, cv::noArray(), newK, cv::Size(w, h),
                                CV_16SC2, impl->mapX, impl->mapY);
    impl->mapW = w;
    impl->mapH = h;
    impl->mapAlpha = alpha;
    impl->mapFx = cal.intrinsics.fx; impl->mapFy = cal.intrinsics.fy;
    impl->mapCx = cal.intrinsics.cx; impl->mapCy = cal.intrinsics.cy;
    impl->mapK1 = cal.distortion.k1; impl->mapK2 = cal.distortion.k2;
    impl->mapP1 = cal.distortion.p1; impl->mapP2 = cal.distortion.p2;
    impl->mapK3 = cal.distortion.k3;
}

bool Calibrator::hasUndistortMaps(int w, int h) const {
    return impl->mapW == w && impl->mapH == h && !impl->mapX.empty();
}

void Calibrator::undistort(const uint8_t* src, uint8_t* dst, int w, int h, int channels,
                           bool isDepth) {
    if (!hasUndistortMaps(w, h) || !src || !dst) return;
    int type = (channels == 1) ? CV_8UC1 : CV_8UC3;
    cv::Mat in (h, w, type, const_cast<uint8_t*>(src));
    cv::Mat out(h, w, type, dst);
    int interp = isDepth ? cv::INTER_NEAREST : cv::INTER_LINEAR;
    cv::remap(in, out, impl->mapX, impl->mapY, interp, cv::BORDER_CONSTANT,
              cv::Scalar(0));
}

void Calibrator::undistortDepth(const uint16_t* src, uint16_t* dst, int w, int h) {
    if (!hasUndistortMaps(w, h) || !src || !dst) return;
    cv::Mat in (h, w, CV_16UC1, const_cast<uint16_t*>(src));
    cv::Mat out(h, w, CV_16UC1, dst);
    // Nearest neighbour: never blend depth values.
    cv::remap(in, out, impl->mapX, impl->mapY, cv::INTER_NEAREST,
              cv::BORDER_CONSTANT, cv::Scalar(0));
}

bool Calibrator::buildOverlay(const uint8_t* src, int w, int h, int channels,
                              std::vector<uint8_t>& outBgr, int& outW, int& outH) {
    if (!src || w <= 0 || h <= 0) return false;
    cv::Mat raw = wrapMat(src, w, h, channels);
    cv::Mat bgr;
    if (channels == 1) cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
    else               bgr = raw.clone();

    if (impl->lastOk && !impl->lastCornersBuf.empty()) {
        std::vector<cv::Point2f> corners;
        corners.reserve(impl->lastCornersBuf.size());
        for (const auto& c : impl->lastCornersBuf)
            corners.emplace_back(c.x, c.y);
        cv::drawChessboardCorners(bgr, cv::Size(impl->boardCols, impl->boardRows),
                                  corners, true);
    }

    outW = w; outH = h;
    outBgr.assign(bgr.data, bgr.data + (size_t)w * h * 3);
    return true;
}

} // namespace orbbec

#endif // SPLATSHOP_HAS_OPENCV
