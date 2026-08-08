// OrbbecCapture.cpp
//
// Implementation of the Orbbec RGBD capture layer. This is the only
// translation unit in the project that includes the libobsensor (OrbbecSDK)
// headers; everything SDK-specific lives in OrbbecCapture::Impl. The rest
// of the codebase only sees the SDK-free OrbbecCapture.h interface.
//
// When SPLATSHOP_HAS_ORBBEC is undefined (SDK absent) the entire body below
// compiles to nothing. OrbbecCapture.h is itself only included by TUs that
// guard on SPLATSHOP_HAS_ORBBEC (SplatEditor.h, gui/orbbec.h), so the class
// is never referenced and no link-time undefined symbols result.

#include "OrbbecCapture.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstring>

#include "unsuck.hpp"
#include "Calibration.h"

#ifdef SPLATSHOP_HAS_ORBBEC

#include <libobsensor/ObSensor.hpp>

namespace orbbec {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

// Convert an OBFormat to its int code (used by RGBDFrame::colorFormat /
// depthFormat, which store raw enum values). OB_FORMAT_ANY (-1) is the
// sentinel for "unknown".
int formatToInt(OBFormat f) {
    return static_cast<int>(f);
}

OBFormat intToFormat(int v) {
    if (v < 0) return OB_FORMAT_ANY;
    return static_cast<OBFormat>(v);
}

// Quantise `value` into [min, max] on the given `step` grid.
int clampToRange(int value, int min, int max, int step) {
    if (step <= 0) step = 1;
    if (value < min) value = min;
    if (value > max) value = max;
    value = min + ((value - min) / step) * step;
    return value;
}

} // namespace

// ---------------------------------------------------------------------------
// PIMPL
// ---------------------------------------------------------------------------
struct OrbbecCapture::Impl {
    std::shared_ptr<ob::Context>      ctx;
    std::shared_ptr<ob::Device>       device;
    std::shared_ptr<ob::Pipeline>     pipeline;

    // Filters (created on demand).
    // `frameAlign` aligns the raw color/depth frames (D2C SW / C2D SW);
    // `align` + `pointCloud` are for the point-cloud path (always D2C).
    // `noiseRemoval` / `spatialFilter` / `temporalFilter` are depth
    // denoising post-processing filters applied after alignment.
    std::shared_ptr<ob::Align>            frameAlign;
    std::shared_ptr<ob::Align>            align;
    std::shared_ptr<ob::PointCloudFilter> pointCloud;
    std::shared_ptr<ob::NoiseRemovalFilter>    noiseRemoval;
    std::shared_ptr<ob::LutNoiseRemovalFilter> hwNoiseRemoval;
    std::shared_ptr<ob::SpatialAdvancedFilter> spatialFilter;
    std::shared_ptr<ob::TemporalFilter>        temporalFilter;

    // Configuration (set before start()).
    StreamConfig  cfgColor;
    StreamConfig  cfgDepth;
    StreamConfig  cfgIR;       // IR stream (for lens calibration)
    CameraParams  params;
    bool          pointCloudEnabled = false;

    // Runtime state.
    std::atomic<bool>      streaming{false};
    std::atomic<bool>      stopRequested{false};
    std::thread            thread;

    mutable std::mutex     frameMutex;
    std::shared_ptr<RGBDFrame> latestFrame;
    std::shared_ptr<RGBDFrame> latestRawFrame;      // pre-filter depth snapshot
    std::shared_ptr<RGBDFrame> latestPointCloud;

    // FPS measurement.
    std::atomic<float>     measuredFps{0.f};
    int                    frameCounter = 0;
    std::chrono::steady_clock::time_point fpsT0;

    // Cached intrinsics, refreshed from the first received frame's stream
    // profile (the active profile, not the device's supported list).
    Intrinsics colorIntrinsics;
    Intrinsics depthIntrinsics;
    Intrinsics irIntrinsics;
    DistortionCoeffs colorDistortion;
    DistortionCoeffs depthDistortion;
    DistortionCoeffs irDistortion;
    std::atomic<bool>      intrinsicsReady{false};

    // External (user-calibrated) lens calibration, applied in software only.
    // When present, the getEffective*() accessors prefer these values.
    bool hasExternalCal = false;
    Intrinsics        extColorIntrinsics, extDepthIntrinsics, extIrIntrinsics;
    DistortionCoeffs  extColorDistortion, extDepthDistortion, extIrDistortion;

    // The serial/uid that was opened, for re-open on profile switches.
    std::string openedDeviceId;
};

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

OrbbecCapture::OrbbecCapture() : impl(std::make_unique<Impl>()) {}

OrbbecCapture::~OrbbecCapture() {
    close();
}

// ---------------------------------------------------------------------------
// Device enumeration
// ---------------------------------------------------------------------------
std::vector<DeviceInfo> OrbbecCapture::enumerateDevices() {
    std::vector<DeviceInfo> result;
    try {
        auto ctx = std::make_shared<ob::Context>();
        auto list = ctx->queryDeviceList();
        uint32_t n = list->getCount();
        for (uint32_t i = 0; i < n; ++i) {
            DeviceInfo info;
            try { info.name           = list->getName(i); }            catch (...) {}
            try { info.serialNumber   = list->getSerialNumber(i); }    catch (...) {}
            try { info.uid            = list->getUid(i); }             catch (...) {}
            try { info.connectionType = list->getConnectionType(i); }  catch (...) {}
            try { info.pid            = list->getPid(i); }             catch (...) {}
            try { info.vid            = list->getVid(i); }             catch (...) {}
            result.push_back(std::move(info));
        }
    } catch (const ob::Error& e) {
        println("Orbbec: enumerateDevices failed: {} ({})", e.what(), e.getFunction());
    }
    return result;
}

// ---------------------------------------------------------------------------
// open / close
// ---------------------------------------------------------------------------
bool OrbbecCapture::open(const std::string& deviceUidOrSerial) {
    close();
    try {
        impl->ctx = std::make_shared<ob::Context>();
        auto list = impl->ctx->queryDeviceList();
        uint32_t n = list->getCount();
        if (n == 0) {
            println("Orbbec: no device connected.");
            return false;
        }

        if (deviceUidOrSerial.empty()) {
            impl->device = list->getDevice(0);
        } else {
            // Try serial first, then uid.
            try {
                impl->device = list->getDeviceBySN(deviceUidOrSerial.c_str());
            } catch (...) {
                impl->device = list->getDeviceByUid(deviceUidOrSerial.c_str());
            }
        }
        if (!impl->device) {
            println("Orbbec: device '{}' not found.", deviceUidOrSerial);
            return false;
        }
        impl->pipeline = std::make_shared<ob::Pipeline>(impl->device);
        impl->openedDeviceId = deviceUidOrSerial;
        return true;
    } catch (const ob::Error& e) {
        println("Orbbec: open failed: {} ({})", e.what(), e.getFunction());
        impl->ctx.reset();
        impl->device.reset();
        impl->pipeline.reset();
        return false;
    } catch (const std::exception& e) {
        println("Orbbec: open failed: {}", e.what());
        impl->ctx.reset();
        impl->device.reset();
        impl->pipeline.reset();
        return false;
    }
}

void OrbbecCapture::close() {
    stop();
    impl->frameAlign.reset();
    impl->align.reset();
    impl->pointCloud.reset();
    impl->noiseRemoval.reset();
    impl->hwNoiseRemoval.reset();
    impl->spatialFilter.reset();
    impl->temporalFilter.reset();
    impl->pipeline.reset();
    impl->device.reset();
    impl->ctx.reset();
    impl->openedDeviceId.clear();
    impl->hasExternalCal = false;
}

bool OrbbecCapture::isOpen() const {
    return impl->pipeline != nullptr;
}

// ---------------------------------------------------------------------------
// start / stop
// ---------------------------------------------------------------------------
bool OrbbecCapture::start() {
    if (!isOpen()) {
        println("Orbbec: start() called but no device is open.");
        return false;
    }
    if (impl->streaming.load()) return true;

    try {
        auto config = std::make_shared<ob::Config>();

        if (impl->cfgColor.enable) {
            // When the user has not specified a format (OB_FORMAT_ANY),
            // explicitly request OB_FORMAT_RGB. Most Orbbec cameras transmit
            // color as MJPG over USB; the SDK's Pipeline decodes it to RGB
            // when that format is requested, so downstream code always gets
            // uncompressed RGB888 without needing a JPEG decoder.
            int colorFmt = impl->cfgColor.format;
            if (colorFmt < 0) colorFmt = (int)OB_FORMAT_RGB;
            config->enableVideoStream(OB_STREAM_COLOR,
                impl->cfgColor.width, impl->cfgColor.height,
                impl->cfgColor.fps, intToFormat(colorFmt));
        }
        if (impl->cfgDepth.enable) {
            config->enableVideoStream(OB_STREAM_DEPTH,
                impl->cfgDepth.width, impl->cfgDepth.height,
                impl->cfgDepth.fps, intToFormat(impl->cfgDepth.format));
        }
        if (impl->cfgIR.enable) {
            // IR stream for lens calibration. Default format is Y8/Y16;
            // when the user has not specified one, let the SDK pick.
            int irFmt = impl->cfgIR.format;
            if (irFmt < 0) irFmt = (int)OB_FORMAT_Y8;
            config->enableVideoStream(OB_STREAM_IR,
                impl->cfgIR.width, impl->cfgIR.height,
                impl->cfgIR.fps, intToFormat(irFmt));
        }
        if (!impl->cfgColor.enable && !impl->cfgDepth.enable) {
            // Nothing requested - fall back to defaults so the user sees
            // something rather than a silent no-op. Request RGB for color so
            // the SDK decodes MJPG internally (see comment above).
            config->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY,
                OB_FPS_ANY, OB_FORMAT_RGB);
            config->enableVideoStream(OB_STREAM_DEPTH);
        }

        if (impl->params.aggregateAllRequired) {
            config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
        }

        // Alignment: the SDK's config->setAlignMode() only supports
        // ALIGN_D2C_HW_MODE (hardware D2C). For software alignment modes
        // (D2C SW, C2D SW) we must create an ob::Align filter and apply it
        // to each frameset in captureLoop(). See SDK doc on
        // getD2CDepthProfileList: "For other align modes, please using the
        // AlignFilter interface."
        auto alignMode = static_cast<OBAlignMode>(impl->params.alignMode);
        impl->frameAlign.reset();
        if (alignMode == ALIGN_D2C_HW_MODE) {
            // Hardware D2C — the pipeline aligns internally.
            config->setAlignMode(alignMode);
        } else if (alignMode == ALIGN_D2C_SW_MODE) {
            // Software D2C: align depth to color so the depth frame takes
            // the color frame's resolution.
            impl->frameAlign = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            impl->frameAlign->setMatchTargetResolution(true);
        } else if (alignMode == ALIGN_C2D_SW_MODE) {
            // Software C2D: align color to depth so the color frame takes
            // the depth frame's resolution.
            impl->frameAlign = std::make_shared<ob::Align>(OB_STREAM_DEPTH);
            impl->frameAlign->setMatchTargetResolution(true);
        }
        // ALIGN_DISABLE: no alignment at all.

        // Depth denoising filters — always create all four so that
        // applyDepthFilterParams() can toggle them on/off and update
        // parameters live without restarting the stream. Use enable(bool)
        // to gate each filter.
        impl->hwNoiseRemoval = std::make_shared<ob::LutNoiseRemovalFilter>("");
        {
            OBLutNoiseRemovalFilterParams lp{};
            int lutVal  = impl->params.hwNoiseMaxLut  >= 0 ? impl->params.hwNoiseMaxLut  : 10;
            int diffVal = impl->params.hwNoiseMinDiff >= 0 ? impl->params.hwNoiseMinDiff : 5;
            for (int i = 0; i < 16; ++i) lp.max_lut[i] = (uint16_t)lutVal;
            lp.min_diff = (uint16_t)diffVal;
            lp.width  = 0; lp.height = 0;
            try { impl->hwNoiseRemoval->setFilterParams(lp); } catch (...) {}
            impl->hwNoiseRemoval->enable(impl->params.hwNoiseRemovalEnabled);
        }
        impl->noiseRemoval = std::make_shared<ob::NoiseRemovalFilter>("");
        {
            OBNoiseRemovalFilterParams np{};
            np.max_size  = (uint16_t)(impl->params.denoiseMaxSize  >= 0 ? impl->params.denoiseMaxSize  : 80);
            np.disp_diff = (uint16_t)(impl->params.denoiseDispDiff >= 0 ? impl->params.denoiseDispDiff : 256);
            try { impl->noiseRemoval->setFilterParams(np); } catch (...) {}
            impl->noiseRemoval->enable(impl->params.denoiseFilterEnabled);
        }
        impl->spatialFilter = std::make_shared<ob::SpatialAdvancedFilter>("");
        {
            OBSpatialAdvancedFilterParams sp{};
            sp.alpha     = impl->params.spatialAlpha     >= 0.f ? impl->params.spatialAlpha     : 0.1f;
            sp.radius    = (uint16_t)(impl->params.spatialRadius    >= 0   ? impl->params.spatialRadius    : 1);
            sp.magnitude = (uint8_t)(impl->params.spatialMagnitude >= 0   ? impl->params.spatialMagnitude : 2);
            sp.disp_diff = (uint16_t)(impl->params.spatialDispDiff >= 0   ? impl->params.spatialDispDiff : 160);
            try { impl->spatialFilter->setFilterParams(sp); } catch (...) {}
            impl->spatialFilter->enable(impl->params.spatialFilterEnabled);
        }
        impl->temporalFilter = std::make_shared<ob::TemporalFilter>("");
        {
            if (impl->params.temporalWeight >= 0.f)
                try { impl->temporalFilter->setWeight(impl->params.temporalWeight); } catch (...) {}
            if (impl->params.temporalDiffScale >= 0.f)
                try { impl->temporalFilter->setDiffScale(impl->params.temporalDiffScale); } catch (...) {}
            impl->temporalFilter->enable(impl->params.temporalFilterEnabled);
        }

        if (impl->params.frameSync) {
            impl->pipeline->enableFrameSync();
        } else {
            impl->pipeline->disableFrameSync();
        }
        impl->pipeline->start(config);

        // Intrinsics are refreshed from the first frame's stream profile in
        // captureLoop() - that reflects the actually-active profile, which
        // getStreamProfileList() does not.
        impl->intrinsicsReady.store(false);

        // (Re)create the point-cloud filter if requested. The point-cloud
        // path always aligns depth to color (OB_STREAM_COLOR) regardless of
        // the user's alignMode, because the RGB point cloud needs color and
        // depth in the same coordinate system.
        if (impl->pointCloudEnabled) {
            impl->align = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            impl->pointCloud = std::make_shared<ob::PointCloudFilter>();
            impl->pointCloud->setCreatePointFormat(OB_FORMAT_RGB_POINT);
        } else {
            impl->align.reset();
            impl->pointCloud.reset();
        }

        // Apply current camera parameters to the now-live device.
        applyCameraParams(impl->params);

        // Launch the polling thread.
        impl->stopRequested.store(false);
        impl->streaming.store(true);
        impl->fpsT0 = std::chrono::steady_clock::now();
        impl->frameCounter = 0;
        impl->thread = std::thread([this]{ captureLoop(); });
        return true;
    } catch (const ob::Error& e) {
        println("Orbbec: start failed: {} ({})", e.what(), e.getFunction());
        // If pipeline->start() succeeded but a later step threw, the
        // pipeline is still running with no polling thread. Stop it so
        // stop()/close() can cleanly tear it down (stop() early-returns
        // when streaming==false, which we set below).
        if (impl->pipeline) {
            try { impl->pipeline->stop(); } catch (...) {}
        }
        impl->streaming.store(false);
        return false;
    } catch (const std::exception& e) {
        // std::bad_alloc from make_shared<ob::Config/Pipeline/...> etc.
        println("Orbbec: start failed: {}", e.what());
        if (impl->pipeline) {
            try { impl->pipeline->stop(); } catch (...) {}
        }
        impl->streaming.store(false);
        return false;
    }
}

void OrbbecCapture::stop() {
    if (!impl->streaming.load()) return;
    impl->stopRequested.store(true);
    impl->streaming.store(false);
    if (impl->thread.joinable()) impl->thread.join();
    try {
        if (impl->pipeline) impl->pipeline->stop();
    } catch (const ob::Error& e) {
        println("Orbbec: stop failed: {} ({})", e.what(), e.getFunction());
    }
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    impl->latestFrame.reset();
    impl->latestRawFrame.reset();
    impl->latestPointCloud.reset();
    impl->measuredFps.store(0.f);
    // Intrinsics/distortion are re-read from the next stream's first frame.
    impl->intrinsicsReady.store(false);
    impl->irIntrinsics = {};
    impl->colorIntrinsics = {};
    impl->depthIntrinsics = {};
    impl->irDistortion = {};
    impl->colorDistortion = {};
    impl->depthDistortion = {};
}

bool OrbbecCapture::isStreaming() const {
    return impl->streaming.load();
}

// ---------------------------------------------------------------------------
// Capture loop (runs on the polling thread)
// ---------------------------------------------------------------------------
void OrbbecCapture::captureLoop() {
    while (!impl->stopRequested.load()) {
        std::shared_ptr<ob::FrameSet> frameSet;
        try {
            frameSet = impl->pipeline->waitForFrameset(100);
        } catch (const ob::Error& e) {
            // Timeouts are normal (OB_ERROR_WAIT_TIMEOUT); only log real errors.
            if (e.getStatus() != OB_ERROR_WAIT_TIMEOUT) {
                println("Orbbec: waitForFrameset error: {} ({})", e.what(), e.getFunction());
            }
            continue;
        }
        if (!frameSet) continue;

        // Apply software alignment (D2C SW / C2D SW) to the raw frameset
        // before extracting color/depth. The Align filter returns a new
        // frameset with the aligned frames; for D2C the depth frame is
        // warped to the color frame's resolution, and vice-versa for C2D.
        if (impl->frameAlign) {
            try {
                auto aligned = impl->frameAlign->process(frameSet);
                if (aligned) {
                    frameSet = aligned->as<ob::FrameSet>();
                }
            } catch (const ob::Error& e) {
                // Alignment failure is non-fatal; fall back to raw frames.
            }
        }

        // The remainder of the loop (frame extraction, depth-filter
        // processing, snapshot building, intrinsics refresh, point-cloud
        // generation) all touches SDK objects that may throw ob::Error.
        // An uncaught exception in this std::thread would call
        // std::terminate, so wrap the whole section and skip the frame on
        // any SDK error.
        try {

        auto colorFrame = frameSet->getColorFrame();
        auto depthFrame = frameSet->getDepthFrame();
        auto irFrame    = frameSet->getIrFrame();

        // Snapshot the raw (pre-filter) depth for before/after comparison.
        // Only the depth buffer is needed; color is left null.
        auto rawFrame = std::make_shared<RGBDFrame>();
        if (depthFrame) {
            auto df = depthFrame->as<ob::DepthFrame>();
            if (df) {
                uint32_t sz = df->getDataSize();
                if (sz > 0 && df->getData() != nullptr) {
                    auto buf = std::make_shared<Buffer>((int64_t)sz);
                    std::memcpy(buf->data, df->getData(), sz);
                    rawFrame->depthData   = buf;
                }
                rawFrame->depthWidth  = (int)df->getWidth();
                rawFrame->depthHeight = (int)df->getHeight();
                rawFrame->depthFormat = formatToInt(df->getFormat());
                rawFrame->depthScale  = df->getValueScale();
                rawFrame->frameIndex  = df->getIndex();
            }
        }

        // Apply depth denoising filters to the DEPTH FRAME ONLY (not the
        // whole frameset). Depth post-processing filters operate on a single
        // depth frame and return a new depth frame; applying them to a
        // frameset can corrupt or drop the color frame. See the SDK's
        // post_processing example which processes depthFrameRaw, not frameSet.
        if (depthFrame) {
            if (impl->hwNoiseRemoval) {
                try {
                    auto r = impl->hwNoiseRemoval->process(depthFrame);
                    if (r) {
                        auto df = r->as<ob::DepthFrame>();
                        if (df) depthFrame = df;
                    }
                } catch (...) {}
            }
            if (impl->noiseRemoval) {
                try {
                    auto r = impl->noiseRemoval->process(depthFrame);
                    if (r) {
                        auto df = r->as<ob::DepthFrame>();
                        if (df) depthFrame = df;
                    }
                } catch (...) {}
            }
            if (impl->spatialFilter) {
                try {
                    auto r = impl->spatialFilter->process(depthFrame);
                    if (r) {
                        auto df = r->as<ob::DepthFrame>();
                        if (df) depthFrame = df;
                    }
                } catch (...) {}
            }
            if (impl->temporalFilter) {
                try {
                    auto r = impl->temporalFilter->process(depthFrame);
                    if (r) {
                        auto df = r->as<ob::DepthFrame>();
                        if (df) depthFrame = df;
                    }
                } catch (...) {}
            }
        }

        // Build the RGBD snapshot.
        auto frame = std::make_shared<RGBDFrame>();

        if (colorFrame) {
            uint32_t w = colorFrame->getWidth();
            uint32_t h = colorFrame->getHeight();
            uint32_t sz = colorFrame->getDataSize();
            if (sz > 0 && colorFrame->getData() != nullptr) {
                auto buf = std::make_shared<Buffer>((int64_t)sz);
                std::memcpy(buf->data, colorFrame->getData(), sz);
                frame->colorData   = buf;
            }
            frame->colorWidth  = (int)w;
            frame->colorHeight = (int)h;
            frame->colorFormat = formatToInt(colorFrame->getFormat());
            frame->timestampUs = colorFrame->getTimeStampUs();
            frame->frameIndex  = colorFrame->getIndex();
        }

        if (depthFrame) {
            auto df = depthFrame->as<ob::DepthFrame>();
            if (df) {
                uint32_t w = df->getWidth();
                uint32_t h = df->getHeight();
                uint32_t sz = df->getDataSize();
                if (sz > 0 && df->getData() != nullptr) {
                    auto buf = std::make_shared<Buffer>((int64_t)sz);
                    std::memcpy(buf->data, df->getData(), sz);
                    frame->depthData   = buf;
                }
                frame->depthWidth  = (int)w;
                frame->depthHeight = (int)h;
                frame->depthFormat = formatToInt(df->getFormat());
                frame->depthScale  = df->getValueScale();
                if (frame->timestampUs == 0) frame->timestampUs = df->getTimeStampUs();
                if (frame->frameIndex  == 0) frame->frameIndex  = df->getIndex();
            }
        }

        if (irFrame) {
            uint32_t w = irFrame->getWidth();
            uint32_t h = irFrame->getHeight();
            uint32_t sz = irFrame->getDataSize();
            if (sz > 0 && irFrame->getData() != nullptr) {
                auto buf = std::make_shared<Buffer>((int64_t)sz);
                std::memcpy(buf->data, irFrame->getData(), sz);
                frame->irData   = buf;
            }
            frame->irWidth  = (int)w;
            frame->irHeight = (int)h;
            frame->irFormat = formatToInt(irFrame->getFormat());
            if (frame->timestampUs == 0) frame->timestampUs = irFrame->getTimeStampUs();
            if (frame->frameIndex  == 0) frame->frameIndex  = irFrame->getIndex();
        }

        // Refresh cached intrinsics from the active frame profiles (once
        // per start). frame->getStreamProfile() returns the profile actually
        // streaming, unlike pipeline->getStreamProfileList() which returns
        // the full supported list.
        if (!impl->intrinsicsReady.load()) {
            // Helper: read intrinsic + distortion (Brown-Conrady subset)
            // from a video frame's active stream profile. Non-Brown-Conrady
            // distortion models are reported as zeros.
            auto readIntrinsic = [](const std::shared_ptr<ob::Frame>& f,
                                    Intrinsics& in, DistortionCoeffs& dc) {
                if (!f) return;
                try {
                    auto vsp = f->getStreamProfile()->as<ob::VideoStreamProfile>();
                    auto kin = vsp->getIntrinsic();
                    in = { kin.fx, kin.fy, kin.cx, kin.cy,
                           (int)kin.width, (int)kin.height };
                    auto kd = vsp->getDistortion();
                    // Only trust the k/p fields for Brown-Conrady variants;
                    // other models (e.g. Kannala-Brandt) use the slots
                    // differently.
                    if (kd.model == OB_DISTORTION_BROWN_CONRADY ||
                        kd.model == OB_DISTORTION_MODIFIED_BROWN_CONRADY ||
                        kd.model == OB_DISTORTION_INVERSE_BROWN_CONRADY ||
                        kd.model == OB_DISTORTION_BROWN_CONRADY_K6) {
                        dc = { kd.k1, kd.k2, kd.p1, kd.p2, kd.k3 };
                    } else {
                        dc = { 0.f, 0.f, 0.f, 0.f, 0.f };
                    }
                } catch (...) {}
            };
            // Read intrinsics into temporaries first, then publish under
            // frameMutex so the getters (which lock the same mutex) never see
            // a half-updated POD intrinsics struct.
            Intrinsics cIn{}, dIn{}, iIn{};
            DistortionCoeffs cDc{}, dDc{}, iDc{};
            readIntrinsic(colorFrame, cIn, cDc);
            readIntrinsic(depthFrame, dIn, dDc);
            readIntrinsic(irFrame,    iIn, iDc);
            {
                std::lock_guard<std::mutex> lk(impl->frameMutex);
                impl->colorIntrinsics = cIn;
                impl->depthIntrinsics = dIn;
                impl->irIntrinsics    = iIn;
                impl->colorDistortion = cDc;
                impl->depthDistortion = dDc;
                impl->irDistortion    = iDc;
            }
            impl->intrinsicsReady.store(true);
        }

        // Optional point cloud.
        std::shared_ptr<RGBDFrame> pcFrame;
        if (impl->pointCloudEnabled && impl->align && impl->pointCloud &&
            colorFrame && depthFrame) {
            try {
                auto aligned = impl->align->process(frameSet);
                auto pcFrameRaw = impl->pointCloud->process(aligned);
                if (pcFrameRaw) {
                    uint32_t sz = pcFrameRaw->getDataSize();
                    if (sz > 0 && pcFrameRaw->getData() != nullptr) {
                        auto buf = std::make_shared<Buffer>((int64_t)sz);
                        std::memcpy(buf->data, pcFrameRaw->getData(), sz);
                        pcFrame = std::make_shared<RGBDFrame>();
                        pcFrame->colorData   = buf;
                        pcFrame->colorFormat = formatToInt(pcFrameRaw->getFormat());
                        // OB_FORMAT_RGB_POINT uses OBColorPoint; derive the point
                        // count from the struct size so layout changes in a future
                        // SDK are picked up automatically.
                        pcFrame->colorWidth  = (int)(sz / sizeof(OBColorPoint));
                        pcFrame->colorHeight = 1;
                        pcFrame->timestampUs = pcFrameRaw->getTimeStampUs();
                        pcFrame->frameIndex  = pcFrameRaw->getIndex();
                    }
                }
            } catch (const ob::Error& e) {
                // Point-cloud failures are non-fatal; just skip this frame.
            }
        }

        // Publish under the lock.
        {
            std::lock_guard<std::mutex> lk(impl->frameMutex);
            impl->latestFrame = frame;
            impl->latestRawFrame = rawFrame;
            if (pcFrame) impl->latestPointCloud = pcFrame;
        }

        // FPS bookkeeping.
        impl->frameCounter++;
        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - impl->fpsT0).count();
        if (dt >= 500) {
            impl->measuredFps.store(float(impl->frameCounter) * 1000.f / float(dt));
            impl->frameCounter = 0;
            impl->fpsT0 = now;
        }

        } catch (const ob::Error& e) {
            // Any SDK error in frame extraction / filter / snapshot building
            // is non-fatal: log and drop this frame rather than letting the
            // exception escape the polling thread (which would terminate).
            println("Orbbec: captureLoop frame error: {} ({})", e.what(), e.getFunction());
            continue;
        } catch (const std::exception& e) {
            println("Orbbec: captureLoop error: {}", e.what());
            continue;
        }
    }
}

// ---------------------------------------------------------------------------
// Stream config
// ---------------------------------------------------------------------------
void OrbbecCapture::setStreamConfig(const StreamConfig& color, const StreamConfig& depth) {
    impl->cfgColor = color;
    impl->cfgDepth = depth;
}

void OrbbecCapture::getStreamConfig(StreamConfig& color, StreamConfig& depth) const {
    color = impl->cfgColor;
    depth = impl->cfgDepth;
}

void OrbbecCapture::setIrStreamConfig(const StreamConfig& ir) {
    impl->cfgIR = ir;
}
void OrbbecCapture::getIrStreamConfig(StreamConfig& ir) const {
    ir = impl->cfgIR;
}

std::vector<StreamConfig> OrbbecCapture::getSupportedProfiles(int sensorType) const {
    std::vector<StreamConfig> result;
    if (!isOpen()) return result;
    try {
        auto sp = impl->pipeline->getStreamProfileList(static_cast<OBSensorType>(sensorType));
        if (!sp) return result;
        uint32_t n = sp->getCount();
        for (uint32_t i = 0; i < n; ++i) {
            try {
                auto vsp = sp->getProfile(i)->as<ob::VideoStreamProfile>();
                StreamConfig c;
                c.enable = true;
                c.width  = (int)vsp->getWidth();
                c.height = (int)vsp->getHeight();
                c.fps    = (int)vsp->getFps();
                c.format = formatToInt(vsp->getFormat());
                result.push_back(c);
            } catch (...) {}
        }
    } catch (const ob::Error& e) {
        println("Orbbec: getSupportedProfiles failed: {} ({})", e.what(), e.getFunction());
    }
    return result;
}

// ---------------------------------------------------------------------------
// Camera parameters
// ---------------------------------------------------------------------------

// Helper functions to keep the apply/get bodies readable. Each takes an
// OBPropertyID; we check support (write or read permission), clamp to range,
// and write/read. applyIntProp/applyBoolProp return true when the value was
// actually written, so applyCameraParams can report a real count.

namespace {

bool applyIntProp(const std::shared_ptr<ob::Device>& dev, OBPropertyID id, int value) {
    if (value < 0) return false;
    if (!dev->isPropertySupported(id, OB_PERMISSION_WRITE)) return false;
    try {
        auto r = dev->getIntPropertyRange(id);
        int v = clampToRange(value, r.min, r.max, r.step);
        dev->setIntProperty(id, v);
        return true;
    } catch (const ob::Error& e) {
        println("Orbbec: setIntProperty({}) failed: {}", (int)id, e.what());
        return false;
    }
}

bool applyBoolProp(const std::shared_ptr<ob::Device>& dev, OBPropertyID id, bool value) {
    if (!dev->isPropertySupported(id, OB_PERMISSION_WRITE)) return false;
    try {
        dev->setBoolProperty(id, value);
        return true;
    } catch (const ob::Error& e) {
        println("Orbbec: setBoolProperty({}) failed: {}", (int)id, e.what());
        return false;
    }
}

int getIntProp(const std::shared_ptr<ob::Device>& dev, OBPropertyID id) {
    if (!dev->isPropertySupported(id, OB_PERMISSION_READ)) return -1;
    try { return dev->getIntProperty(id); } catch (...) { return -1; }
}

bool getBoolProp(const std::shared_ptr<ob::Device>& dev, OBPropertyID id) {
    if (!dev->isPropertySupported(id, OB_PERMISSION_READ)) return false;
    try { return dev->getBoolProperty(id); } catch (...) { return false; }
}

} // namespace

int OrbbecCapture::applyCameraParams(const CameraParams& p) {
    impl->params = p;
    if (!impl->device) return 0;
    int written = 0;
    auto& d = impl->device;

    // Color
    written += applyBoolProp(d, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL,      p.colorAutoExposure);
    written += applyIntProp (d, OB_PROP_COLOR_EXPOSURE_INT,            p.colorExposure);
    written += applyIntProp (d, OB_PROP_COLOR_GAIN_INT,                p.colorGain);
    written += applyBoolProp(d, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, p.colorAutoWhiteBalance);
    written += applyIntProp (d, OB_PROP_COLOR_WHITE_BALANCE_INT,       p.colorWhiteBalance);
    written += applyIntProp (d, OB_PROP_COLOR_BRIGHTNESS_INT,          p.colorBrightness);
    written += applyIntProp (d, OB_PROP_COLOR_SATURATION_INT,          p.colorSaturation);
    written += applyIntProp (d, OB_PROP_COLOR_CONTRAST_INT,            p.colorContrast);
    written += applyIntProp (d, OB_PROP_COLOR_GAMMA_INT,               p.colorGamma);
    written += applyBoolProp(d, OB_PROP_COLOR_MIRROR_BOOL,             p.colorMirror);
    // Depth
    written += applyBoolProp(d, OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL,      p.depthAutoExposure);
    written += applyIntProp (d, OB_PROP_DEPTH_EXPOSURE_INT,            p.depthExposure);
    written += applyIntProp (d, OB_PROP_DEPTH_GAIN_INT,                p.depthGain);
    written += applyIntProp (d, OB_PROP_DEPTH_PRECISION_LEVEL_INT,     p.depthPrecisionLevel);
    written += applyBoolProp(d, OB_PROP_DEPTH_MIRROR_BOOL,             p.depthMirror);
    written += applyIntProp (d, OB_PROP_MIN_DEPTH_INT,                 p.minDepth);
    written += applyIntProp (d, OB_PROP_MAX_DEPTH_INT,                 p.maxDepth);
    // IR / Laser / LDP
    written += applyIntProp (d, OB_PROP_IR_EXPOSURE_INT,               p.irExposure);
    written += applyIntProp (d, OB_PROP_IR_GAIN_INT,                   p.irGain);
    written += applyBoolProp(d, OB_PROP_LASER_BOOL,                    p.laserOn);
    written += applyBoolProp(d, OB_PROP_LDP_BOOL,                      p.ldpOn);

    return written;
}

CameraParams OrbbecCapture::getCameraParams() const {
    CameraParams p;
    if (!impl->device) return p;
    auto& d = impl->device;
    p.colorAutoExposure     = getBoolProp(d, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
    p.colorExposure         = getIntProp (d, OB_PROP_COLOR_EXPOSURE_INT);
    p.colorGain             = getIntProp (d, OB_PROP_COLOR_GAIN_INT);
    p.colorAutoWhiteBalance = getBoolProp(d, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
    p.colorWhiteBalance     = getIntProp (d, OB_PROP_COLOR_WHITE_BALANCE_INT);
    p.colorBrightness       = getIntProp (d, OB_PROP_COLOR_BRIGHTNESS_INT);
    p.colorSaturation       = getIntProp (d, OB_PROP_COLOR_SATURATION_INT);
    p.colorContrast         = getIntProp (d, OB_PROP_COLOR_CONTRAST_INT);
    p.colorGamma            = getIntProp (d, OB_PROP_COLOR_GAMMA_INT);
    p.colorMirror           = getBoolProp(d, OB_PROP_COLOR_MIRROR_BOOL);
    p.depthAutoExposure     = getBoolProp(d, OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
    p.depthExposure         = getIntProp (d, OB_PROP_DEPTH_EXPOSURE_INT);
    p.depthGain             = getIntProp (d, OB_PROP_DEPTH_GAIN_INT);
    p.depthPrecisionLevel   = getIntProp (d, OB_PROP_DEPTH_PRECISION_LEVEL_INT);
    p.depthMirror           = getBoolProp(d, OB_PROP_DEPTH_MIRROR_BOOL);
    p.minDepth              = getIntProp (d, OB_PROP_MIN_DEPTH_INT);
    p.maxDepth              = getIntProp (d, OB_PROP_MAX_DEPTH_INT);
    p.irExposure            = getIntProp (d, OB_PROP_IR_EXPOSURE_INT);
    p.irGain                = getIntProp (d, OB_PROP_IR_GAIN_INT);
    p.laserOn               = getBoolProp(d, OB_PROP_LASER_BOOL);
    p.ldpOn                 = getBoolProp(d, OB_PROP_LDP_BOOL);
    p.alignMode             = impl->params.alignMode;
    p.frameSync             = impl->params.frameSync;
    p.aggregateAllRequired  = impl->params.aggregateAllRequired;
    return p;
}

bool OrbbecCapture::isPropertySupported(int propertyId) const {
    if (!impl->device) return false;
    try {
        auto id = static_cast<OBPropertyID>(propertyId);
        // A property is "supported" if either read or write is available -
        // matching how applyIntProp (WRITE) and getIntRange (READ) use it.
        return impl->device->isPropertySupported(id, OB_PERMISSION_WRITE) ||
               impl->device->isPropertySupported(id, OB_PERMISSION_READ);
    } catch (...) {
        return false;
    }
}

IntRange OrbbecCapture::getIntRange(int propertyId) const {
    IntRange r{};
    if (!impl->device) return r;
    try {
        if (impl->device->isPropertySupported(static_cast<OBPropertyID>(propertyId),
                                              OB_PERMISSION_READ)) {
            auto pr = impl->device->getIntPropertyRange(static_cast<OBPropertyID>(propertyId));
            r.min  = pr.min;
            r.max  = pr.max;
            r.step = pr.step;
            r.def  = pr.def;
            r.cur  = pr.cur;
        }
    } catch (...) {}
    return r;
}

// ---------------------------------------------------------------------------
// Frame retrieval
// ---------------------------------------------------------------------------
std::shared_ptr<RGBDFrame> OrbbecCapture::getLatestFrame() {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->latestFrame;
}

std::shared_ptr<RGBDFrame> OrbbecCapture::getLatestRawFrame() {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->latestRawFrame;
}

void OrbbecCapture::applyDepthFilterParams(const CameraParams& p) {
    impl->params = p;
    // Toggle enable/disable on existing filters (no recreation needed).
    if (impl->hwNoiseRemoval) impl->hwNoiseRemoval->enable(p.hwNoiseRemovalEnabled);
    if (impl->noiseRemoval)   impl->noiseRemoval->enable(p.denoiseFilterEnabled);
    if (impl->spatialFilter)  impl->spatialFilter->enable(p.spatialFilterEnabled);
    if (impl->temporalFilter) impl->temporalFilter->enable(p.temporalFilterEnabled);
    // Push updated parameters to enabled filters.
    if (impl->hwNoiseRemoval && p.hwNoiseRemovalEnabled) {
        OBLutNoiseRemovalFilterParams lp{};
        int lutVal  = p.hwNoiseMaxLut  >= 0 ? p.hwNoiseMaxLut  : 10;
        int diffVal = p.hwNoiseMinDiff >= 0 ? p.hwNoiseMinDiff : 5;
        for (int i = 0; i < 16; ++i) lp.max_lut[i] = (uint16_t)lutVal;
        lp.min_diff = (uint16_t)diffVal;
        lp.width  = 0; lp.height = 0;
        try { impl->hwNoiseRemoval->setFilterParams(lp); } catch (...) {}
    }
    if (impl->noiseRemoval && p.denoiseFilterEnabled) {
        OBNoiseRemovalFilterParams np{};
        np.max_size  = (uint16_t)(p.denoiseMaxSize  >= 0 ? p.denoiseMaxSize  : 80);
        np.disp_diff = (uint16_t)(p.denoiseDispDiff >= 0 ? p.denoiseDispDiff : 256);
        try { impl->noiseRemoval->setFilterParams(np); } catch (...) {}
    }
    if (impl->spatialFilter && p.spatialFilterEnabled) {
        OBSpatialAdvancedFilterParams sp{};
        sp.alpha     = p.spatialAlpha     >= 0.f ? p.spatialAlpha     : 0.1f;
        sp.radius    = (uint16_t)(p.spatialRadius    >= 0   ? p.spatialRadius    : 1);
        sp.magnitude = (uint8_t)(p.spatialMagnitude >= 0   ? p.spatialMagnitude : 2);
        sp.disp_diff = (uint16_t)(p.spatialDispDiff >= 0   ? p.spatialDispDiff : 160);
        try { impl->spatialFilter->setFilterParams(sp); } catch (...) {}
    }
    if (impl->temporalFilter && p.temporalFilterEnabled) {
        if (p.temporalWeight >= 0.f)
            try { impl->temporalFilter->setWeight(p.temporalWeight); } catch (...) {}
        if (p.temporalDiffScale >= 0.f)
            try { impl->temporalFilter->setDiffScale(p.temporalDiffScale); } catch (...) {}
    }
}

void OrbbecCapture::setPointCloudEnabled(bool enabled) {
    impl->pointCloudEnabled = enabled;
    // The filters themselves are (re)created on the next start(); if we're
    // already streaming, restart so the change takes effect.
    if (impl->streaming.load()) {
        stop();
        start();
    }
}

bool OrbbecCapture::isPointCloudEnabled() const {
    return impl->pointCloudEnabled;
}

std::shared_ptr<RGBDFrame> OrbbecCapture::getLatestPointCloud() {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->latestPointCloud;
}

// ---------------------------------------------------------------------------
// Intrinsics / FPS
// ---------------------------------------------------------------------------
// The cached intrinsics/distortion are written by the polling thread (and
// zeroed by stop()) under frameMutex; the getters take the same lock so they
// never observe a half-updated POD struct. The effective* accessors read the
// external-calibration flag + values, which are set from the main thread and
// also guarded by frameMutex for consistency.
Intrinsics OrbbecCapture::getColorIntrinsics() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->colorIntrinsics;
}
Intrinsics OrbbecCapture::getDepthIntrinsics() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->depthIntrinsics;
}
Intrinsics OrbbecCapture::getIrIntrinsics() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->irIntrinsics;
}

DistortionCoeffs OrbbecCapture::getColorDistortion() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->colorDistortion;
}
DistortionCoeffs OrbbecCapture::getDepthDistortion() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->depthDistortion;
}
DistortionCoeffs OrbbecCapture::getIrDistortion() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->irDistortion;
}

void OrbbecCapture::setExternalCalibration(const DeviceCalibration& cal) {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    impl->hasExternalCal = true;
    impl->extColorIntrinsics  = cal.color.intrinsics;
    impl->extColorDistortion  = cal.color.distortion;
    impl->extIrIntrinsics     = cal.ir.intrinsics;
    impl->extIrDistortion     = cal.ir.distortion;
    impl->extDepthIntrinsics  = cal.depth.intrinsics;
    impl->extDepthDistortion  = cal.depth.distortion;
}
void OrbbecCapture::clearExternalCalibration() {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    impl->hasExternalCal = false;
}
bool OrbbecCapture::hasExternalCalibration() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->hasExternalCal;
}

Intrinsics OrbbecCapture::getEffectiveColorIntrinsics() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->hasExternalCal ? impl->extColorIntrinsics : impl->colorIntrinsics;
}
Intrinsics OrbbecCapture::getEffectiveDepthIntrinsics() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->hasExternalCal ? impl->extDepthIntrinsics : impl->depthIntrinsics;
}
Intrinsics OrbbecCapture::getEffectiveIrIntrinsics() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->hasExternalCal ? impl->extIrIntrinsics : impl->irIntrinsics;
}
DistortionCoeffs OrbbecCapture::getEffectiveColorDistortion() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->hasExternalCal ? impl->extColorDistortion : impl->colorDistortion;
}
DistortionCoeffs OrbbecCapture::getEffectiveDepthDistortion() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->hasExternalCal ? impl->extDepthDistortion : impl->depthDistortion;
}
DistortionCoeffs OrbbecCapture::getEffectiveIrDistortion() const {
    std::lock_guard<std::mutex> lk(impl->frameMutex);
    return impl->hasExternalCal ? impl->extIrDistortion : impl->irDistortion;
}

float OrbbecCapture::getMeasuredFps() const {
    if (!impl->streaming.load()) return 0.f;
    return impl->measuredFps.load();
}

} // namespace orbbec

#endif // SPLATSHOP_HAS_ORBBEC
