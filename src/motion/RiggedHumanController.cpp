#include "RiggedHumanController.h"

#include <cstring>

#include "../SplatEditor.h"
#include "MotionTypes.h"

namespace motion {

SNRiggedSplats* RiggedHumanController::resolve(Scene& scene, NodeID id) {
	SNRiggedSplats* found = nullptr;
	scene.root->traverse([&](SceneNode* node){
		if(!found && node->ID == id) {
			found = dynamic_cast<SNRiggedSplats*>(node);
		}
	});
	return found;
}

bool RiggedHumanController::setPose(Scene& scene, NodeID id, const SkeletonPose& pose) {
	SNRiggedSplats* node = resolve(scene, id);
	if(!node) return false;
	if(pose.joints.size() != node->currentPose.joints.size()) return false;
	node->currentPose = pose;
	node->poseDirty = true;
	return true;
}

bool RiggedHumanController::setJointPose(Scene& scene, NodeID id, int jointIndex, const JointPose& jp) {
	SNRiggedSplats* node = resolve(scene, id);
	if(!node) return false;
	if(jointIndex < 0 || jointIndex >= (int)node->currentPose.joints.size()) return false;
	node->currentPose.joints[jointIndex] = jp;
	node->poseDirty = true;
	return true;
}

bool RiggedHumanController::setFace(Scene& scene, NodeID id, const FaceData& face) {
	SNRiggedSplats* node = resolve(scene, id);
	if(!node || node->rig.blendshapeCount <= 0) return false;
	node->currentFace = face;
	node->poseDirty = true;
	return true;
}

bool RiggedHumanController::setBlendshape(Scene& scene, NodeID id, int index, float weight) {
	SNRiggedSplats* node = resolve(scene, id);
	if(!node || node->rig.blendshapeCount <= 0) return false;
	if(index < 0 || index >= BLENDSHAPE_COUNT) return false;
	node->currentFace.weights[index] = weight;
	node->poseDirty = true;
	return true;
}

void RiggedHumanController::computeSkinningMatrices(SNRiggedSplats* node,
                                                    std::vector<glm::mat4>& outSkin) const {
	const Skeleton& sk = node->rig.skeleton;
	int n = (int)sk.parents.size();
	outSkin.assign(n, glm::mat4(1.0f));
	if(n == 0) return;

	// Global matrices via parent accumulation.
	std::vector<glm::mat4> globalMats(n, glm::mat4(1.0f));
	for(int i = 0; i < n; i++) {
		glm::mat4 localM = node->currentPose.joints[i].localMatrix();
		int parent = sk.parents[i];
		if(parent >= 0) {
			globalMats[i] = globalMats[parent] * localM;
		} else {
			globalMats[i] = localM;
		}
	}

	// Skin matrix = global * inverseBind.
	for(int i = 0; i < n; i++) {
		if(i < (int)sk.inverseBindMatrices.size()) {
			outSkin[i] = globalMats[i] * sk.inverseBindMatrices[i];
		} else {
			outSkin[i] = globalMats[i];
		}
	}
}

void RiggedHumanController::update(Scene& scene) {
	scene.forEach<SNRiggedSplats>([&](SNRiggedSplats* node){
		if(!node->poseDirty) return;
		if(!node->skinningEnabled) { node->poseDirty = false; return; }

		std::vector<glm::mat4> skinMats;
		computeSkinningMatrices(node, skinMats);
		if(!skinMats.empty() && node->cptr_skinningMatrices) {
			CURuntime::check(cuMemcpyHtoD(node->cptr_skinningMatrices,
				skinMats.data(), sizeof(glm::mat4) * skinMats.size()));
		}
		// Keep poseDirty true until the skinning kernel has actually run.
	});
}

void RiggedHumanController::dispatchSkinning(Scene& scene) {
	SplatEditor* editor = SplatEditor::instance;
	if(!editor || !editor->prog_skinning) return;

	scene.forEach<SNRiggedSplats>([&](SNRiggedSplats* node){
		if(!node->skinningEnabled) { node->poseDirty = false; return; }
		if(!node->poseDirty) return;
		if(node->dmng.data.count == 0) { node->poseDirty = false; return; }
		if(!node->cptr_restPosition || !node->cptr_skinningMatrices) { node->poseDirty = false; return; }

		int numJoints = (int)node->rig.skeleton.parents.size();
		uint32_t splatCount = node->dmng.data.count;
		int blendshapeCount = node->rig.blendshapeCount;

		// Kernel arguments. The skinning kernel writes deformed position/scale/
		// quaternion into the node's dmng.data buffers in place.
		CUdeviceptr outPosition  = (CUdeviceptr)node->dmng.data.position;
		CUdeviceptr outScale     = (CUdeviceptr)node->dmng.data.scale;
		CUdeviceptr outQuaternion= (CUdeviceptr)node->dmng.data.quaternion;
		CUdeviceptr restPosition = node->cptr_restPosition;
		CUdeviceptr restScale    = node->cptr_restScale;
		CUdeviceptr restQuat     = node->cptr_restQuaternion;
		CUdeviceptr boneIndices  = node->rig.cptr_boneIndices;
		CUdeviceptr boneWeights  = node->rig.cptr_boneWeights;
		CUdeviceptr skinMats     = node->cptr_skinningMatrices;
		CUdeviceptr bsDeltas     = node->rig.cptr_blendshapeDeltas;
		float* faceWeights       = const_cast<float*>(node->currentFace.weights);

		void* args[] = {
			&splatCount,
			&numJoints,
			&blendshapeCount,
			&outPosition, &outScale, &outQuaternion,
			&restPosition, &restScale, &restQuat,
			&boneIndices, &boneWeights,
			&skinMats,
			&bsDeltas, faceWeights,
		};

		editor->prog_skinning->launch("kernel_skin_splats", args, splatCount, editor->mainstream);
		node->poseDirty = false;
	});
}

} // namespace motion
