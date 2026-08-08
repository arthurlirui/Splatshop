// SN4DGSSplats implementation.
//
// This file encapsulates the entire LibTorch dependency. No other
// translation unit needs to include LibTorch headers, keeping compile
// times manageable and the dependency optional (the whole file can be
// excluded with #ifdef SPLATSHOP_HAS_LIBTORCH).

#include "SN4DGSSplats.h"

#ifdef SPLATSHOP_HAS_LIBTORCH

#include <torch/script.h>  // torch::jit::script::Module
#include <torch/torch.h>

#include <c10/cuda/CUDAStream.h>

#include <cuda.h>
#include <cuda_runtime.h>

#include "CURuntime.h"

// =========================================================================
// Deform4DGSBuffer
// =========================================================================

void Deform4DGSBuffer::allocDevice(int64_t numSplats) {
    freeDevice();

    vm_deformedPosition = CURuntime::allocVirtual("4dgs_deformedPos");
    vm_deformedScale    = CURuntime::allocVirtual("4dgs_deformedScale");
    vm_deformedRotation = CURuntime::allocVirtual("4dgs_deformedRot");

    vm_deformedPosition->commit(numSplats * sizeof(float3), true);
    vm_deformedScale   ->commit(numSplats * sizeof(float3), true);
    vm_deformedRotation->commit(numSplats * sizeof(float4), true);

    cptr_deformedPosition = vm_deformedPosition->cptr;
    cptr_deformedScale    = vm_deformedScale->cptr;
    cptr_deformedRotation = vm_deformedRotation->cptr;
}

void Deform4DGSBuffer::freeDevice() {
    CURuntime::free(vm_deformedPosition);
    CURuntime::free(vm_deformedScale);
    CURuntime::free(vm_deformedRotation);
}

// =========================================================================
// SN4DGSSplats
// =========================================================================

SN4DGSSplats::SN4DGSSplats(std::string name,
                           shared_ptr<Splats> splats,
                           const std::string& modelPath,
                           const Deform4DGSConfig& config)
    : SNSplats(name, splats)
    , deformConfig(config)
{
    if (splats) {
        // Allocate from the header-declared total (numSplats), NOT the
        // concurrently-incremented numSplatsLoaded counter: the loader runs on a
        // detached thread, so numSplatsLoaded is 0/partial at construction time
        // and runDeformation's D2D copies (sized from data.count, which grows up
        // to numSplats) would otherwise overflow the allocated deform buffers.
        int64_t N = splats->numSplats;
        if (N <= 0) {
            // Fall back to the loaded count if the header field wasn't set.
            N = splats->numSplatsLoaded.load();
        }
        deformBuffer.allocDevice(N);
    }

    // Load the TorchScript model
    try {
        deformModule = std::make_unique<torch::jit::script::Module>(
            torch::jit::load(modelPath, torch::kCUDA)
        );
        deformModule->eval();

        println("SN4DGSSplats '{}': loaded TorchScript model from '{}'", name, modelPath);
    } catch (const c10::Error& e) {
        println("SN4DGSSplats '{}': ERROR loading TorchScript model: {}", name, e.what());
        // Model remains null; deform() will fall back to canonical rendering.
    }
}

SN4DGSSplats::~SN4DGSSplats() {
    println("Destroying SN4DGSSplats node '{}'", name);
    swapToCanonical();  // Ensure canonical pointers are restored before cleanup
    deformBuffer.freeDevice();
}

void SN4DGSSplats::swapToDeformed() {
    if (!deformationEnabled) return;

    auto& data = dmng.data;

    // Save canonical pointers (only once)
    if (canonicalPosition == nullptr) {
        canonicalPosition  = data.position;
        canonicalScale     = data.scale;
        canonicalQuaternion = data.quaternion;
    }

    // Swap dmng.data to deformed buffers
    data.position   = reinterpret_cast<vec3*>(deformBuffer.cptr_deformedPosition);
    data.scale      = reinterpret_cast<vec3*>(deformBuffer.cptr_deformedScale);
    data.quaternion = reinterpret_cast<vec4*>(deformBuffer.cptr_deformedRotation);
}

void SN4DGSSplats::swapToCanonical() {
    if (canonicalPosition == nullptr) return;

    auto& data = dmng.data;
    data.position    = canonicalPosition;
    data.scale       = canonicalScale;
    data.quaternion  = canonicalQuaternion;

    canonicalPosition  = nullptr;
    canonicalScale     = nullptr;
    canonicalQuaternion = nullptr;
}

void SN4DGSSplats::deform(float normalizedTime, CUstream stream) {
    if (!deformationEnabled) return;

    bool timeChanged = (std::abs(normalizedTime - currentTime) > 1e-6f);
    bool needsUpdate = timeChanged || needsRecompute.load(std::memory_order_acquire);

    if (!needsUpdate) return;

    currentTime = normalizedTime;
    needsRecompute.store(false, std::memory_order_release);

    runDeformation(normalizedTime, stream);
}

void SN4DGSSplats::runDeformation(float time, CUstream stream) {
    if (!deformModule) {
        // No model loaded; copy canonical data to deformed buffers as-is
        println("SN4DGSSplats::runDeformation: no model loaded, using identity.");
        return;
    }

    auto& data = dmng.data;
    int64_t N = static_cast<int64_t>(data.count);
    if (N == 0) {
        return;
    }

    // --- Build input tensors (zero-copy from existing CUDA buffers) ---

    auto options = torch::TensorOptions()
        .dtype(torch::kFloat32)
        .device(torch::kCUDA);
    // We need torch::kCUDA to be the same device as Splatshop's CUDA context.
    // By default, LibTorch uses device 0, which should match.

    // Wrap Splatshop's CUDA buffers as PyTorch tensors.
    // dmng.data.position is a vec3* (CUdeviceptr), so we wrap it directly.
    auto means3D = torch::from_blob(
        data.position,
        {N, 3},
        {3 * sizeof(float), sizeof(float)},  // strides: row-major
        options
    );

    auto scales = torch::from_blob(
        data.scale,
        {N, 3},
        {3 * sizeof(float), sizeof(float)},
        options
    );

    auto rotations = torch::from_blob(
        data.quaternion,
        {N, 4},
        {4 * sizeof(float), sizeof(float)},
        options
    );

    // Opacity input: the traced model's forward signature expects a 5th
    // argument. Splatshop stores opacity inside Color (interleaved with rgb) and
    // GaussianData has no separate opacity pointer to swap, so we pass a neutral
    // input and intentionally do NOT consume the model's opacity output — see
    // the comment below the unpack.
    auto opacity = torch::full({N, 1}, 0.0f, options);

    // --- Run the model ---
    // Wrap forward() + unpack in try/catch: a runtime c10::Error (shape mismatch,
    // OOM, bad TorchScript) would otherwise propagate out of draw() and terminate
    // the process. On failure we log and leave deformBuffer un-written, so the
    // node renders its canonical (rest-pose) Gaussians for this frame.
    torch::Tensor deformedPos, deformedScale, deformedRot;
    try {
        std::vector<torch::IValue> inputs;
        inputs.push_back(means3D);
        inputs.push_back(scales);
        inputs.push_back(rotations);
        inputs.push_back(opacity);
        inputs.push_back(time);  // scalar float

        auto outputs = deformModule->forward(inputs);

        // --- Unpack outputs ---
        // We consume only position/scale/rotation. The model may also return an
        // opacity tensor (4th element / cols 10:11), but Splatshop's render path
        // reads opacity from GaussianData.color, which swapToDeformed() does not
        // touch — so a deformed opacity would never be consumed. Ignoring it
        // avoids allocating/copying a buffer that is never read.
        if (outputs.isTuple()) {
            auto tuple = outputs.toTuple();
            deformedPos   = tuple->elements().at(0).toTensor();
            deformedScale = tuple->elements().at(1).toTensor();
            deformedRot   = tuple->elements().at(2).toTensor();
        } else if (outputs.isTensor()) {
            // Single tensor: assume concatenated [pos, scale, rot, (opacity)]
            auto t = outputs.toTensor();
            deformedPos   = t.slice(1, 0, 3);
            deformedScale = t.slice(1, 3, 6);
            deformedRot   = t.slice(1, 6, 10);
        } else {
            println("SN4DGSSplats::runDeformation: unexpected output type");
            return;
        }
    } catch (const c10::Error& e) {
        println("SN4DGSSplats '{}': deformation forward failed: {}", name, e.what());
        return;
    } catch (const std::exception& e) {
        println("SN4DGSSplats '{}': deformation forward failed: {}", name, e.what());
        return;
    }

    // --- Copy outputs to deformation buffers ---

    // Ensure tensors are contiguous before copying
    deformedPos   = deformedPos.contiguous();
    deformedScale = deformedScale.contiguous();
    deformedRot   = deformedRot.contiguous();

    // Copy LibTorch tensor data to our CUDA buffers.
    // We use cuMemcpyDtoDAsync for explicit CUDA stream control — LibTorch
    // tensors reside on the same CUDA device as Splatshop's deformBuffer.
    size_t posBytes   = N * 3 * sizeof(float);
    size_t scaleBytes = N * 3 * sizeof(float);
    size_t rotBytes   = N * 4 * sizeof(float);

    CURuntime::check(cuMemcpyDtoDAsync(
        deformBuffer.cptr_deformedPosition,
        reinterpret_cast<CUdeviceptr>(deformedPos.data_ptr<float>()),
        posBytes, stream
    ));
    CURuntime::check(cuMemcpyDtoDAsync(
        deformBuffer.cptr_deformedScale,
        reinterpret_cast<CUdeviceptr>(deformedScale.data_ptr<float>()),
        scaleBytes, stream
    ));
    CURuntime::check(cuMemcpyDtoDAsync(
        deformBuffer.cptr_deformedRotation,
        reinterpret_cast<CUdeviceptr>(deformedRot.data_ptr<float>()),
        rotBytes, stream
    ));
}

#else  // !SPLATSHOP_HAS_LIBTORCH

// -----------------------------------------------------------------------
// Stub implementation when LibTorch is not available.
// The node behaves as a pass-through (canonical = deformed).
// -----------------------------------------------------------------------

void Deform4DGSBuffer::allocDevice(int64_t)  {}
void Deform4DGSBuffer::freeDevice()          {}

SN4DGSSplats::SN4DGSSplats(std::string name,
                           shared_ptr<Splats> splats,
                           const std::string&,
                           const Deform4DGSConfig& config)
    : SNSplats(name, splats)
    , deformConfig(config)
    , deformModule(nullptr, [](void*){})
{
    println("SN4DGSSplats '{}': LibTorch not available. 4DGS deformation disabled.", name);
}

SN4DGSSplats::~SN4DGSSplats() {}

void SN4DGSSplats::swapToDeformed()  {}
void SN4DGSSplats::swapToCanonical() {}

void SN4DGSSplats::deform(float, CUstream) {
    // Pass-through: canonical data is used directly
}

void SN4DGSSplats::runDeformation(float, CUstream) {
    // No-op
}

#endif // SPLATSHOP_HAS_LIBTORCH
