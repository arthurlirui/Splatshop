#pragma once

// SNRiggedSplats: a Gaussian-splatting scene node driven by a skeleton
// (linear blend skinning) plus optional ARKit blendshapes for the face.
//
// It derives from SNSplats so it is picked up by the existing render pass
// (Scene::forEach<SNSplats> / process<SNSplats> use dynamic_cast, which also
// matches derived types). The per-frame skinning kernel (src/motion/skinning.cu)
// writes deformed position/scale/quaternion into the inherited dmng.data buffers
// right before rendering; the standard render kernels then read them unchanged.
//
// Rest-pose copies of position/scale/quaternion are kept on the device so the
// skinning kernel can recompute the deformed state every frame from the rest
// pose + current joint matrices + blendshape weights, without destroying the
// original splat data.

#include <memory>
#include <string>
#include <vector>

#include "SNSplats.h"          // brings in SceneNode.h (glm) + HostDeviceInterface.h + Splats.h
#include "SplatsManagement.h"
#include "CudaVirtualMemory.h"
#include "CURuntime.h"
#include "Splats.h"
#include "motion/MotionTypes.h"

// Skinning attributes: up to 4 bones per splat (LBS weights).
constexpr int MAX_BONES_PER_SPLAT = 4;

struct Skeleton {
	std::vector<std::string> jointNames;
	std::vector<int> parents;               // parent index per joint, -1 for root
	std::vector<glm::mat4> inverseBindMatrices; // IBP per joint (maps rest pose to joint-local)
};

// Device-backed rig data attached to a SNRiggedSplats node.
struct RigAsset {
	Skeleton skeleton;

	int blendshapeCount = 0; // 0 if the model has no facial blendshapes

	// Per-splat skinning data (device). boneIndices: MAX_BONES_PER_SPLAT x uint16,
	// boneWeights: MAX_BONES_PER_SPLAT x float (already normalized).
	shared_ptr<CudaVirtualMemory> vm_boneIndices = nullptr;
	shared_ptr<CudaVirtualMemory> vm_boneWeights = nullptr;
	CUdeviceptr cptr_boneIndices = 0;
	CUdeviceptr cptr_boneWeights = 0;

	// Blendshape deltas stored as per-splat per-bs {dx,dy,dz, dRotX,dRotY,dRotZ,dRotW, dsx,dsy,dsz}
	// (10 floats per splat per blendshape). Only allocated if blendshapeCount > 0.
	// Layout: [bs][splat][10 floats].
	shared_ptr<CudaVirtualMemory> vm_blendshapeDeltas = nullptr;
	CUdeviceptr cptr_blendshapeDeltas = 0;

	void allocDevice(int64_t numSplats);
	void freeDevice();
};

struct SNRiggedSplats : public SNSplats {

	RigAsset rig;

	// Rest-pose copies on the device. The skinning kernel reads from these and
	// writes the deformed result into dmng.data.position/scale/quaternion.
	shared_ptr<CudaVirtualMemory> vm_restPosition = nullptr;
	shared_ptr<CudaVirtualMemory> vm_restScale = nullptr;
	shared_ptr<CudaVirtualMemory> vm_restQuaternion = nullptr;
	CUdeviceptr cptr_restPosition = 0;
	CUdeviceptr cptr_restScale = 0;
	CUdeviceptr cptr_restQuaternion = 0;

	// Current-frame skinning matrices (M_skin = M_global * IBP), one mat4 per
	// joint, uploaded from the host each frame by RiggedHumanController.
	shared_ptr<CudaVirtualMemory> vm_skinningMatrices = nullptr;
	CUdeviceptr cptr_skinningMatrices = 0;

	// Current pose / face state (host). The controller updates these and the
	// derived skinning matrices; the node marks itself dirty until the skinning
	// kernel has been dispatched for the frame.
	motion::SkeletonPose currentPose;
	motion::FaceData currentFace;
	bool poseDirty = true;
	bool skinningEnabled = true; // can be disabled to edit the rest pose directly

	SNRiggedSplats(std::string name, shared_ptr<Splats> splats)
		: SNSplats(name, splats)
	{
		if(splats){
			currentPose.reset(0); // sized when a rig is attached
		}
	}

	~SNRiggedSplats(){
		destroyRig();
	}

	// Allocate rest copies and skinning-matrix buffer to match the given splat
	// count / joint count, and upload the rest pose from the host Splats buffers.
	// Called once the splat data has finished loading and the rig is attached.
	void initRestAndRig(int64_t numSplats, int numJoints);

	void destroyRig();

	// Copy the current dmng.data position/scale/quaternion back into the rest
	// buffers. Use this after a destructive transform-bake edit so subsequent
	// skinning restarts from the edited rest pose.
	void syncRestFromCurrent();

	uint64_t getGpuMemoryUsage() override {
		return SNSplats::getGpuMemoryUsage();
	}

	std::string toString() override { return "SNRiggedSplats"; }
};
