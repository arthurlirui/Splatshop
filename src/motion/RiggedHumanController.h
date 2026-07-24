#pragma once

// RiggedHumanController: drives SNRiggedSplats nodes with skeletal pose and
// facial blendshape input.
//
// Per-frame flow (called from SplatEditor::update() / draw()):
//   1. setPose()/setFace() write the desired pose/face onto a node and mark it
//      poseDirty.
//   2. update(scene) accumulates each dirty node's local joint poses into
//      global joint matrices (M_global[i] = M_global[parent] * M_local[i]),
//      computes skinning matrices M_skin[i] = M_global[i] * IBP[i], and uploads
//      them to the node's cptr_skinningMatrices device buffer.
//   3. dispatchSkinning(scene) launches the CUDA skinning kernel for each dirty
//      node, which reads rest pose + skinning matrices + blendshape weights and
//      writes deformed position/scale/quaternion into dmng.data.
//
// All host-side; the only CUDA call here is cuMemcpyHtoD for the matrix buffer.

#include <vector>

#include "../scene/SceneNode.h"
#include "../scene/Scene.h"
#include "../scene/SNRiggedSplats.h"
#include "MotionTypes.h"

namespace motion {

class RiggedHumanController {
public:
	// Set the full skeleton pose for a node (one JointPose per joint).
	static bool setPose(Scene& scene, NodeID id, const SkeletonPose& pose);

	// Set a single joint's local pose (index must be valid for the node's rig).
	static bool setJointPose(Scene& scene, NodeID id, int jointIndex, const JointPose& jp);

	// Set facial blendshape weights (ARKit 52).
	static bool setFace(Scene& scene, NodeID id, const FaceData& face);

	static bool setBlendshape(Scene& scene, NodeID id, int index, float weight);

	// Resolve a node ID to a SNRiggedSplats; returns nullptr if not rigged.
	static SNRiggedSplats* resolve(Scene& scene, NodeID id);

	// Accumulate local poses into skinning matrices and upload to device.
	// Called once per frame before dispatchSkinning.
	void update(Scene& scene);

	// Launch the skinning CUDA kernel for every dirty rigged node.
	// Uses SplatEditor::instance->prog_skinning.
	void dispatchSkinning(Scene& scene);

private:
	// Compute M_global and M_skin for a node from its currentPose.
	void computeSkinningMatrices(SNRiggedSplats* node, std::vector<glm::mat4>& outSkin) const;
};

} // namespace motion
