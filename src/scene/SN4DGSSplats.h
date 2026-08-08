#pragma once

// SN4DGSSplats: a Gaussian-splatting scene node driven by a 4D Gaussian
// Splatting (4DGS) deformation model (HexPlane + MLP). At each timestep,
// the deformation network displaces the canonical (rest-pose) 3D Gaussians
// into a dynamic state; the displaced Gaussians are then rendered by the
// standard 3DGS rasterizer.
//
// The deformation model is loaded as a TorchScript module (.pt file) via
// LibTorch (PyTorch C++ frontend). The canonical Gaussians are stored in a
// standard .ply file and loaded by the existing GSPlyLoader.
//
// Architecture:
//   SN4DGSSplats derives from SNSplats. The canonical Gaussians live in
//   dmng.data (inherited). The deformation forward pass runs via LibTorch
//   on a CUDA stream, and the deformed position/scale/rotation/opacity are
//   written back into dmng.data buffers so the existing render kernels
//   (kernel_stageSplats → sort → rasterize) pick them up unchanged.
//
// Editing:
//   The user edits canonical Gaussians (dmng.data). After an edit, the
//   node marks itself dirty; on the next frame, the deformation is
//   recomputed for the current time step.

#include <memory>
#include <string>
#include <atomic>
#include <filesystem>

#include "SNSplats.h"         // SceneNode (glm) + HostDeviceInterface.h + Splats.h
#include "SplatsManagement.h"
#include "CudaVirtualMemory.h"
#include "CURuntime.h"
#include "json/json.hpp"        // nlohmann::json — already a project dependency (SplatsyFilesLoader)

#ifdef SPLATSHOP_HAS_LIBTORCH
#include <torch/script.h>       // torch::jit::script::Module
#endif

/// 4DGS deformation configuration parsed from config.json.
struct Deform4DGSConfig {
    int shDegree = 3;
    int nGaussians = 0;
    std::string formatVersion = "1.0";

    /// Load a Deform4DGSConfig from a config.json file written by
    /// tools/export_4dgs_torchscript.py. Missing/unreadable files yield the
    /// struct defaults above. Defined inline (header-only) so the import UI in
    /// gui/motion.h can call it without pulling in the LibTorch-guarded .cpp.
    static Deform4DGSConfig loadFromFile(const std::string& jsonPath) {
        Deform4DGSConfig cfg;
        namespace fs = std::filesystem;
        if (jsonPath.empty() || !fs::exists(jsonPath)) {
            return cfg;
        }
        try {
            // readTextFile is from unsuck.hpp (already included transitively).
            std::string text = readTextFile(jsonPath);
            auto j = nlohmann::json::parse(text);
            if (j.contains("sh_degree")     && j["sh_degree"].is_number_integer())     cfg.shDegree     = j["sh_degree"].get<int>();
            if (j.contains("n_gaussians")   && j["n_gaussians"].is_number_integer())   cfg.nGaussians   = j["n_gaussians"].get<int>();
            if (j.contains("format_version")&& j["format_version"].is_string())        cfg.formatVersion= j["format_version"].get<std::string>();
        } catch (const std::exception& e) {
            println("Deform4DGSConfig: could not parse '{}': {}; using defaults", jsonPath, e.what());
        }
        return cfg;
    }
};

/// Device-backed deformation buffers for a SN4DGSSplats node.
/// These hold the deformed output of the LibTorch forward pass
/// and are read by the staging kernel instead of the canonical buffers.
struct Deform4DGSBuffer {
    // Deformed output (written by LibTorch, read by staging kernel).
    // Opacity is intentionally NOT here: Splatshop stores opacity inside
    // GaussianData.color (interleaved with rgb) and swapToDeformed() does not
    // swap a separate opacity pointer, so a deformed opacity buffer would never
    // be consumed. See SN4DGSSplats.cpp::runDeformation for the rationale.
    shared_ptr<CudaVirtualMemory> vm_deformedPosition  = nullptr;
    shared_ptr<CudaVirtualMemory> vm_deformedScale     = nullptr;
    shared_ptr<CudaVirtualMemory> vm_deformedRotation  = nullptr;

    CUdeviceptr cptr_deformedPosition = 0;
    CUdeviceptr cptr_deformedScale    = 0;
    CUdeviceptr cptr_deformedRotation = 0;

    void allocDevice(int64_t numSplats);
    void freeDevice();
};

struct SN4DGSSplats : public SNSplats {

    Deform4DGSConfig deformConfig;
    Deform4DGSBuffer deformBuffer;

    // The TorchScript deformation module. Loaded from deformation_model.pt.
#ifdef SPLATSHOP_HAS_LIBTORCH
    std::unique_ptr<torch::jit::script::Module> deformModule;
#else
    std::unique_ptr<void, void(*)(void*)> deformModule;
#endif

    // Current time (normalized, 0.0 to 1.0). The deformation is recomputed
    // whenever this changes or when canonical Gaussians are edited.
    float currentTime = 0.0f;

    // Flag: set to true after editing canonical Gaussians, forcing a
    // deformation recompute on the next frame.
    std::atomic<bool> needsRecompute{true};

    // Flag: deformation is active. When false, canonical Gaussians are
    // rendered as-is (useful for editing the rest pose).
    bool deformationEnabled = true;

    // --- Construction / destruction ---

    /// Construct from a loaded canonical Splats and a TorchScript model path.
    /// @param name         Node name
    /// @param splats       Canonical Gaussian data (already loaded)
    /// @param modelPath    Path to deformation_model.pt (TorchScript)
    /// @param config       Deformation configuration
    SN4DGSSplats(std::string name,
                 shared_ptr<Splats> splats,
                 const std::string& modelPath,
                 const Deform4DGSConfig& config);

    ~SN4DGSSplats();

    // --- Deformation ---

    /// Run the full deformation inference pipeline for the given time.
    /// This is a NO-OP if the time hasn't changed and no canonical edit
    /// has occurred (needsRecompute == false).
    /// @param normalizedTime  Time in [0, 1]
    /// @param stream          CUDA stream for LibTorch execution
    void deform(float normalizedTime, CUstream stream = 0);

    // --- Accessors for staging ---

    /// Swap dmng.data pointers to point to deformed buffers.
    /// Call before the staging kernel so it reads deformed data.
    /// Must be paired with swapToCanonical() after rendering.
    void swapToDeformed();

    /// Restore dmng.data pointers back to canonical buffers.
    void swapToCanonical();

    /// Get the pointer to deformed positions (for kernel_stageSplats).
    vec3* getDeformedPositions() const {
        return reinterpret_cast<vec3*>(deformBuffer.cptr_deformedPosition);
    }

    /// Get the pointer to deformed scales.
    vec3* getDeformedScales() const {
        return reinterpret_cast<vec3*>(deformBuffer.cptr_deformedScale);
    }

    /// Get the pointer to deformed rotation quaternions.
    vec4* getDeformedRotations() const {
        return reinterpret_cast<vec4*>(deformBuffer.cptr_deformedRotation);
    }

    // --- Overrides ---

    uint64_t getGpuMemoryUsage() override {
        return SNSplats::getGpuMemoryUsage();
    }

    std::string toString() override { return "SN4DGSSplats"; }

private:
    /// Internal: run a single LibTorch forward pass.
    void runDeformation(float time, CUstream stream);

    // Saved canonical pointers for swapToDeformed/swapToCanonical.
    vec3* canonicalPosition   = nullptr;
    vec3* canonicalScale      = nullptr;
    vec4* canonicalQuaternion = nullptr;
};
