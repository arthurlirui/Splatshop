#pragma once

// OrbbecCapture.h
//
// Orbbec RGBD camera capture layer.
//
// OrbbecCapture owns an ob::Pipeline (+ Align / PointCloudFilter) and runs
// a background thread that polls synchronized color+depth framesets via
// waitForFrameset(). Frames are copied into host-side RGBDFrame buffers and
// published as the "latest frame" under a mutex. The capture thread never
// touches CUDA / GL / the scene graph — consumers pull frames from the main
// thread (or via schedule()) and do GPU upload themselves.
//
// The libobsensor (OrbbecSDK) C++ headers are intentionally NOT included
// here: `ob::` types are forward-declared and the real handles live behind
// a PIMPL (`Impl`) defined only in OrbbecCapture.cpp. This keeps the SDK a
// build-time-only dependency of the .cpp translation unit.
//
// All SDK errors are caught and reported via println; methods return
// false / nullptr on failure rather than throwing.

#include <memory>
#include <string>
#include <vector>

#include "OrbbecTypes.h"

// Forward declarations of the libobsensor C++ wrapper types we hold.
// The full definitions are only needed inside OrbbecCapture.cpp.
namespace ob {
class Pipeline;
class Align;
class PointCloudFilter;
class NoiseRemovalFilter;
class LutNoiseRemovalFilter;
class SpatialAdvancedFilter;
class TemporalFilter;
class Device;
class FrameSet;
} // namespace ob

namespace orbbec {

// Forward declaration to avoid pulling Calibration.h into this header.
struct DeviceCalibration;

class OrbbecCapture {
public:
    // ------------------------------------------------------------------
    // Device enumeration. May be called without an instance. Returns one
    // DeviceInfo per connected Orbbec device.
    // ------------------------------------------------------------------
    static std::vector<DeviceInfo> enumerateDevices();

    OrbbecCapture();
    ~OrbbecCapture();

    OrbbecCapture(const OrbbecCapture&) = delete;
    OrbbecCapture& operator=(const OrbbecCapture&) = delete;

    // ------------------------------------------------------------------
    // Lifecycle.
    //
    // open()   — select a device by serial number or UID; empty string
    //             selects the default (first) device. Returns false if no
    //             device is available or the requested one is not found.
    // start()  — apply the current StreamConfig / CameraParams, start the
    //             pipeline and the polling thread. Returns false on failure.
    // stop()   — stop the pipeline and join the polling thread.
    // close()  — stop() + release the device / pipeline.
    // ------------------------------------------------------------------
    bool open(const std::string& deviceUidOrSerial = "");
    void close();
    bool isOpen() const;

    bool start();
    void stop();
    bool isStreaming() const;

    // ------------------------------------------------------------------
    // Stream configuration. Applies at the next start(); if currently
    // streaming, call stop() then start() to switch profiles.
    // ------------------------------------------------------------------
    void setStreamConfig(const StreamConfig& color, const StreamConfig& depth);
    void getStreamConfig(StreamConfig& color, StreamConfig& depth) const;

    // IR stream configuration (used for lens calibration — the IR sensor is
    // co-located with the depth sensor). `enable=false` disables IR. Applies
    // at the next start(); if currently streaming, stop() then start().
    void setIrStreamConfig(const StreamConfig& ir);
    void getIrStreamConfig(StreamConfig& ir) const;

    // Enumerate the profiles the opened device supports for a sensor.
    // `sensorType` is an OBSensorType value (OB_SENSOR_COLOR=2,
    // OB_SENSOR_DEPTH=3). Each returned StreamConfig is filled with the
    // profile's width/height/fps/format.
    std::vector<StreamConfig> getSupportedProfiles(int sensorType) const;

    // ------------------------------------------------------------------
    // Camera control parameters.
    //
    // applyCameraParams() writes every non-(-1) field, checking
    // isPropertySupported and clamping to the device range (quantised to
    // step) first. Safe to call while streaming. Returns the number of
    // properties that were actually written.
    // getCameraParams()   reads back the current values from the device.
    // ------------------------------------------------------------------
    int  applyCameraParams(const CameraParams& p);
    CameraParams getCameraParams() const;

    // Property introspection (for GUI sliders). `propertyId` is an
    // OBPropertyID value.
    bool     isPropertySupported(int propertyId) const;
    IntRange getIntRange(int propertyId) const;

    // ------------------------------------------------------------------
    // Frame retrieval (polling). Returns the most recent synchronized
    // frame, or nullptr if none has arrived yet. The returned frame is a
    // snapshot; it is safe to hold across frames.
    //
    // getLatestFrame()     - the denoised frame (after depth filters).
    // getLatestRawFrame()  - the raw frame (before depth filters), for
    //                        before/after comparison. Only the depth buffer
    //                        is populated; color is null.
    // ------------------------------------------------------------------
    std::shared_ptr<RGBDFrame> getLatestFrame();
    std::shared_ptr<RGBDFrame> getLatestRawFrame();

    // ------------------------------------------------------------------
    // Update depth-denoising filter parameters live (without restarting
    // the stream). Toggles each filter's enable state and pushes new
    // parameter values. The filters must have been created in start().
    // ------------------------------------------------------------------
    void applyDepthFilterParams(const CameraParams& p);

    // ------------------------------------------------------------------
    // Optional RGB point-cloud generation.
    //
    // When enabled, the capture thread additionally runs an
    // ob::Align(OB_STREAM_COLOR) + ob::PointCloudFilter(OB_FORMAT_RGB_POINT)
    // on each frameset and publishes the result via getLatestPointCloud().
    // The point buffer is an array of OBColorPoint {float x,y,z,r,g,b} per
    // point (OB_FORMAT_RGB_POINT), with the point count in `colorWidth` and
    // `colorHeight = 1`.
    // ------------------------------------------------------------------
    void setPointCloudEnabled(bool enabled);
    bool isPointCloudEnabled() const;
    std::shared_ptr<RGBDFrame> getLatestPointCloud();

    // ------------------------------------------------------------------
    // Intrinsics of the opened device (read from the active stream
    // profiles). Zeroed if not open.
    // ------------------------------------------------------------------
    Intrinsics getColorIntrinsics() const;
    Intrinsics getDepthIntrinsics() const;
    Intrinsics getIrIntrinsics() const;

    // Distortion coefficients read from the device's active stream profiles
    // (OBCameraDistortion → Brown-Conrady subset). Returned as the 5-param
    // model; non-Brown-Conrady models are reported as zeros (the GUI flags
    // this). Zeroed if not open or not yet streamed.
    DistortionCoeffs getColorDistortion() const;
    DistortionCoeffs getDepthDistortion() const;
    DistortionCoeffs getIrDistortion() const;

    // ------------------------------------------------------------------
    // External (user-calibrated) lens calibration.
    //
    // setExternalCalibration() caches a calibration produced by the
    // calibration GUI (or loaded from JSON). When present, the
    // getEffective*() accessors return the calibrated intrinsics instead of
    // the device-reported ones; the preview / point-cloud paths use them for
    // software-side undistortion. Nothing is written to device firmware —
    // the calibration is applied purely in software.
    // ------------------------------------------------------------------
    void setExternalCalibration(const DeviceCalibration& cal);
    void clearExternalCalibration();
    bool hasExternalCalibration() const;

    // Effective intrinsics: external calibration if loaded, else device.
    Intrinsics getEffectiveColorIntrinsics() const;
    Intrinsics getEffectiveDepthIntrinsics() const;
    Intrinsics getEffectiveIrIntrinsics() const;
    DistortionCoeffs getEffectiveColorDistortion() const;
    DistortionCoeffs getEffectiveDepthDistortion() const;
    DistortionCoeffs getEffectiveIrDistortion() const;

    // Diagnostic: approximate frames-per-second measured on the polling
    // thread. Returns 0 if not streaming.
    float getMeasuredFps() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;

    // Polling-thread entry point: loops waitForFrameset() and publishes the
    // latest RGBDFrame / point cloud under impl->frameMutex.
    void captureLoop();
};

} // namespace orbbec
