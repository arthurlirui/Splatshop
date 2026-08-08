#pragma once

// SNPointCloudBA.h
//
// Scene node that runs GPU Bundle-Adjustment-style refinement on a point
// cloud (see docs/ba_research.md and optim/PointCloudBA.h).
//
// Derives from SNPoints so it reuses the existing HQS point render path and
// the PointDataManager device buffers, exactly like SNOrbbec. The node holds
// an optim::PointCloudBA instance; when `optimizeEnabled` is set, the per-
// frame update pass (SplatEditor_update.h) calls runBASteps() which advances
// the AdamW optimization against a captured target RGB frame and writes the
// refined position/color back into the manager's device buffers, so the
// forward-only renderer displays the refinement live.
//
// Prototype scope: static camera (pose = identity, points already in camera
// space, as Orbbec's PointCloudFilter produces). The target frame is set
// externally via setTargetFrame() — e.g. from the Orbbec RGBDFrame color
// buffer or a rendered snapshot. Multi-frame pose estimation is future work.

#include "SNPoints.h"
#include "../optim/PointCloudBA.h"

#include <vector>
#include <atomic>
#include <cstdint>
#include <mutex>

struct SNPointCloudBA : public SNPoints {

    // Optimization driver state.
    optim::PointCloudBA ba;
    bool optimizeEnabled = false;        // toggled from GUI / remote

    // Camera + intrinsics used by the differentiable renderer. Set together
    // with the target frame; defaults to identity view (camera-space cloud).
    optim::BACamera camera;

    // Target RGB frame (HxWx3, row-major, channels R,G,B, uint8). Copied to
    // the GPU at init() time; held here so the optimization can be (re)started
    // from the GUI without re-supplying the frame each call.
    std::vector<uint8_t> targetFrame;
    int targetW = 0;
    int targetH = 0;
    std::mutex targetMutex;              // guards targetFrame/W/H (set from
                                         // capture thread, read on main thread)

    // Pending (re)init request: set when a new target frame is supplied while
    // optimization is running, honoured on the next main-thread update tick.
    std::atomic<bool> initRequested{false};

    SNPointCloudBA(string name) : SNPoints(name) {
        this->points = make_shared<Points>();
        this->points->name = name;
        this->points->bytesPerPoint = 16;
        this->points->headerSize = 0;
        this->manager.data.transform = points->world;
    }

    uint64_t getGpuMemoryUsage() override {
        return manager.getGpuMemoryUsage();
    }

    Box3 getBoundingBox() override {
        return {manager.data.min, manager.data.max};
    }

    string toString() override { return "SNPointCloudBA"; }

    // Supply a target RGB frame (HxWx3 uint8, row-major, channels R,G,B) and
    // the pinhole intrinsics to render with. Safe to call from any thread
    // (copies into the host-side buffer under targetMutex and flags a
    // (re)init on the next main-thread update). `view4x4` is the world->view
    // matrix (row-major); pass nullptr for identity (camera-space cloud).
    void setTargetFrame(const uint8_t* rgb, int w, int h,
                        float fx, float fy, float cx, float cy,
                        const float* view4x4 = nullptr) {
        if (rgb == nullptr || w <= 0 || h <= 0) return;
        std::lock_guard<std::mutex> lock(targetMutex);
        targetFrame.assign(rgb, rgb + size_t(w) * h * 3);
        targetW = w;
        targetH = h;
        camera.fx = fx; camera.fy = fy;
        camera.cx = cx; camera.cy = cy;
        camera.width = w; camera.height = h;
        if (view4x4) {
            for (int i = 0; i < 16; ++i) camera.view[i] = view4x4[i];
        } else {
            const float ident[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
            for (int i = 0; i < 16; ++i) camera.view[i] = ident[i];
        }
        initRequested.store(true);
    }

    // Advance the optimization by `steps` AdamW iterations on the main
    // thread. Handles pending (re)init requests first. Called from
    // SplatEditor_update.h each frame when optimizeEnabled is true.
    int runBASteps(CUstream stream, int steps) {
        // Honour a pending (re)init.
        if (initRequested.load()) {
            initRequested.store(false);
            std::vector<uint8_t> frameCopy;
            optim::BACamera camCopy;
            {
                std::lock_guard<std::mutex> lock(targetMutex);
                frameCopy = targetFrame;
                camCopy = camera;
            }
            if (!frameCopy.empty()) {
                ba.reset();
                if (!ba.init(manager.data, camCopy, frameCopy.data())) {
                    println("SNPointCloudBA '{}': BA init failed.", name);
                    return 0;
                }
            }
        }

        if (!ba.initialized()) return 0;
        return ba.step(manager.data, stream, steps);
    }

    // Stop optimization (the refined cloud stays as-is).
    void stopOptimize() {
        optimizeEnabled = false;
    }

    optim::BAStatus getStatus() const { return ba.status; }
};
