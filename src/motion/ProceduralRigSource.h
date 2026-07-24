#pragma once

// Procedural rig generation for testing the skinning pipeline without a real
// rigged asset. Produces a simple 5-joint skeleton stacked along the Y axis
// (root -> spine -> chest -> neck -> head) and assigns each splat to its
// nearest joint by height, with all weight on that single bone. Blendshapes
// are left empty (blendshapeCount = 0).
//
// This is a stand-in until the real asset format (GLTF skin + blendshape
// deltas) is decided and a IRigAssetSource loader is implemented.

#include <memory>
#include <vector>

#include "../scene/SNRiggedSplats.h"
#include "MotionTypes.h"

namespace motion {

struct ProceduralRigSource {
	// Build a 5-joint skeleton and per-splat single-bone weights for the given
	// node, allocate the rest-pose device copies + skinning matrix buffer, and
	// upload everything. Requires that the node's splats have finished loading
	// (splats->numSplatsLoaded == splats->numSplats).
	static void build(shared_ptr<SNRiggedSplats> node);
};

} // namespace motion
