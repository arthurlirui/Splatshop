#pragma once

// PointCloudBA.h
//
// GPU Bundle-Adjustment-style point-cloud refinement for Splatshop.
//
// Research conclusion (see docs/ba_research.md): the only paradigm that
// natively optimizes BOTH per-point 3D position AND color on GPU is
// differentiable rendering (paradigm B, à la 3DGS / GS-SLAM / SplaTAM).
// Classical sparse BA (Ceres/g2o/PBA) optimizes camera poses + sparse
// point XYZ but treats color as a measurement, not a free parameter, and
// cannot refine a dense point cloud's color.
//
// This module implements the differentiable-rendering approach: it wraps
// an existing PointDataManager point cloud (position + color) as
// libtorch `requires_grad` leaf tensors via the zero-copy
// `torch::from_blob(..., torch::kCUDA)` pattern proven in
// SN4DGSSplats::runDeformation, runs a tensorized differentiable EWA
// Gaussian-splatting forward pass against a target RGB image, computes a
// photometric L1 + SSIM loss, backpropagates, and steps AdamW. Updates
// land in separate gradient tensors; the optimized position/color are
// then cuMemcpyDtoD'd back into the PointDataManager device buffers so
// the existing forward-only renderer displays the refined cloud.
//
// The whole module is guarded by SPLATSHOP_HAS_LIBTORCH (mirroring
// SN4DGSSplats) so the project still builds without libtorch. The
// "static-camera / single-frame" prototype variant is implemented first
// (camera pose = identity, target = one captured color frame); multi-
// frame pose estimation is left to a future iteration.
//
// Why tensorized (not a custom CUDA backward kernel)?
//   Splatshop's existing gaussians_rendering.cu is forward-only and
//   compiled via NVRTC at runtime (CUDA is NOT a CMake language here).
//   Writing a custom torch::autograd::Function + NVRTC backward kernel is
//   the production path, but for the prototype we use a libtorch-only
//   tensorized EWA splat so autograd supplies the backward pass for free.
//   This keeps the prototype self-contained and lets the forward model be
//   swapped for a tile-based differentiable rasterizer later.

#include "Points.h"
#include "PointsManagement.h"
#include "HostDeviceInterface.h"

#include <memory>
#include <atomic>
#include <vector>
#include <functional>

#ifdef SPLATSHOP_HAS_LIBTORCH
#include <torch/torch.h>
#endif

namespace optim {

// Camera model used by the differentiable renderer.
// Pinhole intrinsics + world-to-view matrix. For the static-camera
// prototype the view matrix is typically identity (points are already in
// camera space, as Orbbec's PointCloudFilter produces).
struct BACamera {
    float fx = 0.f, fy = 0.f, cx = 0.f, cy = 0.f;
    int   width = 0,  height = 0;
    // 4x4 world->view (row-major float). Identity for the static prototype.
    float view[16] = {
        1.f, 0.f, 0.f, 0.f,
        0.f, 1.f, 0.f, 0.f,
        0.f, 0.f, 1.f, 0.f,
        0.f, 0.f, 0.f, 1.f,
    };
};

// Tunable optimization hyperparameters.
struct BAConfig {
    int    maxSteps        = 1000;     // total optimization budget
    float  lrPosition      = 1e-4f;    // AdamW learning rate for xyz
    float  lrColor         = 1e-2f;    // AdamW learning rate for rgb
    float  lambdaL1        = 0.8f;     // L1 loss weight
    float  lambdaSSIM      = 0.2f;     // (1 - SSIM) loss weight
    float  initScale       = 0.005f;   // isotropic Gaussian radius (world units) for splatting
    float  initOpacity     = 0.9f;     // per-Gaussian opacity (logit space: logit(0.9))
    bool   optimizeColor   = true;     // refine color (else freeze at init)
    bool   optimizePosition = true;    // refine xyz (else freeze at init)
    int    stepsPerFrame   = 8;        // BA steps advanced per main-thread update tick
    int    ssimWindow      = 11;       // SSIM averaging window (must be odd, <= min(H,W))
};

// Status snapshot for GUI / remote polling.
struct BAStatus {
    int    step         = 0;
    float  loss         = 0.f;
    float  lossL1       = 0.f;
    float  lossSSIM     = 0.f;
    bool   running      = false;
    bool   initialized  = false;
    int    pointCount   = 0;
    int    targetW      = 0;
    int    targetH      = 0;
};

// -----------------------------------------------------------------------------
// PointCloudBA
// -----------------------------------------------------------------------------
// Owns the optimization state for one point cloud. Created lazily when the
// user (GUI / remote) starts optimization on a node. Holds libtorch
// parameter tensors that are SEPARATE from PointDataManager's device
// buffers — the cloud is copied into optimizable tensors at init(), and
// copied back into the render buffers each step so the forward-only
// renderer reflects the refinement live.
//
// Threading: all step()/init() calls must happen on the main thread (the
// CUDA/GL context is bound there), exactly like SN4DGSSplats::deform().
struct PointCloudBA {

    BAConfig config;
    BAStatus status;

    PointCloudBA();
    ~PointCloudBA();

    // Initialize the optimization from a fully-uploaded point cloud.
    // `pd` provides the source position (vec3*) / color (RGBA8 uint32).
    // `cam` is the rendering camera (intrinsics + view).
    // `targetRGB` is the HxWx3 uint8 target image in row-major order
    //   (channels = R,G,B), copied to the GPU as the photometric target.
    // Returns false if the cloud is empty or the target is malformed.
    bool init(const PointData& pd, const BACamera& cam,
              const uint8_t* targetRGB);

    // Advance the optimization by `steps` AdamW iterations. Each iteration:
    //   forward EWA splat -> L1 + SSIM loss vs target -> backward ->
    //   AdamW step -> copy refined pos/color back into `pd`.
    // No-op (returns 0) if not initialized.
    int step(PointData& pd, CUstream stream, int steps = 1);

    // Stop and drop optimization state (frees GPU tensors). The point
    // cloud retains whatever refinement has already been written back.
    void reset();

    // Has init() succeeded and reset() not been called?
    bool initialized() const { return status.initialized; }

#ifdef SPLATSHOP_HAS_LIBTORCH
  private:
    void allocParams(int64_t N);
    void freeParams();

    // libtorch autograd parameter tensors (CUDA, requires_grad).
    torch::Tensor tPosition;   // [N,3] float32  (world/camera space)
    torch::Tensor tColor;      // [N,3] float32  (linear rgb in [0,1])
    torch::Tensor tScale;      // [N,1] float32  (isotropic, log space)
    torch::Tensor tOpacity;    // [N,1] float32  (logit)
    torch::Tensor tTarget;     // [H,W,3] float32  (target rgb in [0,1])

    // Optimizers (one per parameter group so lr can differ).
    std::unique_ptr<torch::optim::AdamW> optPosition;
    std::unique_ptr<torch::optim::AdamW> optColor;

    // Camera as tensors (constants, no grad).
    torch::Tensor tView;       // [4,4] float32
    float cachedFx = 0, cachedFy = 0, cachedCx = 0, cachedCy = 0;
    int   cachedW = 0, cachedH = 0;
    int64_t cachedN = 0;
#endif
};

} // namespace optim

#ifndef SPLATSHOP_HAS_LIBTORCH
// -----------------------------------------------------------------------
// Stub implementation when libtorch is unavailable. Keeps the header
// includable and the API non-throwing; init() always returns false.
// -----------------------------------------------------------------------
inline optim::PointCloudBA::PointCloudBA() {}
inline optim::PointCloudBA::~PointCloudBA() { reset(); }
inline bool optim::PointCloudBA::init(const PointData&, const BACamera&,
                                       const uint8_t*) { return false; }
inline int  optim::PointCloudBA::step(PointData&, CUstream, int) { return 0; }
inline void optim::PointCloudBA::reset() {}
#endif
