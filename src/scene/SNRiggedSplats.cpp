#include "SNRiggedSplats.h"

#include <cstring>

void RigAsset::allocDevice(int64_t numSplats) {
	freeDevice();

	vm_boneIndices = CURuntime::allocVirtual(format("rig boneIndices [{}]", "SNRiggedSplats"));
	vm_boneWeights = CURuntime::allocVirtual(format("rig boneWeights [{}]", "SNRiggedSplats"));
	vm_boneIndices->commit(sizeof(uint16_t) * MAX_BONES_PER_SPLAT * numSplats, true);
	vm_boneWeights->commit(sizeof(float)   * MAX_BONES_PER_SPLAT * numSplats, true);
	cptr_boneIndices = vm_boneIndices->cptr;
	cptr_boneWeights = vm_boneWeights->cptr;

	if(blendshapeCount > 0){
		vm_blendshapeDeltas = CURuntime::allocVirtual(format("rig blendshape deltas [{}]", "SNRiggedSplats"));
		constexpr int floatsPerSplatPerBs = 10; // dpos(3)+drot(4)+dscale(3)
		vm_blendshapeDeltas->commit(sizeof(float) * floatsPerSplatPerBs * blendshapeCount * numSplats, true);
		cptr_blendshapeDeltas = vm_blendshapeDeltas->cptr;
	}
}

void RigAsset::freeDevice() {
	CURuntime::free(vm_boneIndices);
	CURuntime::free(vm_boneWeights);
	CURuntime::free(vm_blendshapeDeltas);
	vm_boneIndices = nullptr;
	vm_boneWeights = nullptr;
	vm_blendshapeDeltas = nullptr;
	cptr_boneIndices = 0;
	cptr_boneWeights = 0;
	cptr_blendshapeDeltas = 0;
}

void SNRiggedSplats::initRestAndRig(int64_t numSplats, int numJoints) {
	destroyRig();

	vm_restPosition  = CURuntime::allocVirtual(format("[{}] rest position",  name));
	vm_restScale     = CURuntime::allocVirtual(format("[{}] rest scale",     name));
	vm_restQuaternion= CURuntime::allocVirtual(format("[{}] rest quaternion",name));
	vm_restPosition  ->commit(sizeof(vec3) * numSplats, true);
	vm_restScale     ->commit(sizeof(vec3) * numSplats, true);
	vm_restQuaternion->commit(sizeof(vec4) * numSplats, true);
	cptr_restPosition  = vm_restPosition->cptr;
	cptr_restScale     = vm_restScale->cptr;
	cptr_restQuaternion= vm_restQuaternion->cptr;

	// Upload rest pose from the host Splats buffers (already populated by the
	// loader). These are the source-of-truth positions the skinning kernel reads
	// from every frame.
	if(splats && splats->position && splats->scale && splats->rotation){
		CURuntime::check(cuMemcpyHtoD(cptr_restPosition,
			splats->position->ptr, sizeof(vec3) * numSplats));
		CURuntime::check(cuMemcpyHtoD(cptr_restScale,
			splats->scale->ptr, sizeof(vec3) * numSplats));
		CURuntime::check(cuMemcpyHtoD(cptr_restQuaternion,
			splats->rotation->ptr, sizeof(vec4) * numSplats));
	}

	if(numJoints > 0){
		vm_skinningMatrices = CURuntime::allocVirtual(format("[{}] skinning matrices", name));
		vm_skinningMatrices->commit(sizeof(glm::mat4) * numJoints, true);
		cptr_skinningMatrices = vm_skinningMatrices->cptr;

		// Initialize skinning matrices to identity (rest pose = no deformation).
		std::vector<glm::mat4> identityMats(numJoints, glm::mat4(1.0f));
		CURuntime::check(cuMemcpyHtoD(cptr_skinningMatrices,
			identityMats.data(), sizeof(glm::mat4) * numJoints));

		currentPose.reset(numJoints);
	}

	rig.allocDevice(numSplats);
	poseDirty = true;
}

void SNRiggedSplats::destroyRig() {
	CURuntime::free(vm_restPosition);
	CURuntime::free(vm_restScale);
	CURuntime::free(vm_restQuaternion);
	CURuntime::free(vm_skinningMatrices);
	vm_restPosition = nullptr;
	vm_restScale = nullptr;
	vm_restQuaternion = nullptr;
	vm_skinningMatrices = nullptr;
	cptr_restPosition = 0;
	cptr_restScale = 0;
	cptr_restQuaternion = 0;
	cptr_skinningMatrices = 0;
	rig.freeDevice();
}

void SNRiggedSplats::syncRestFromCurrent() {
	if(!splats || dmng.data.count == 0) return;
	int64_t n = dmng.data.count;
	CURuntime::check(cuMemcpyDtoD(cptr_restPosition,   (CUdeviceptr)dmng.data.position,  sizeof(vec3) * n));
	CURuntime::check(cuMemcpyDtoD(cptr_restScale,      (CUdeviceptr)dmng.data.scale,     sizeof(vec3) * n));
	CURuntime::check(cuMemcpyDtoD(cptr_restQuaternion, (CUdeviceptr)dmng.data.quaternion, sizeof(vec4) * n));
	poseDirty = true;
}
