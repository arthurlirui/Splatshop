#include "ProceduralRigSource.h"

#include <algorithm>
#include <cstring>

namespace motion {

void ProceduralRigSource::build(shared_ptr<SNRiggedSplats> node) {
	if(!node || !node->splats) return;
	int64_t numSplats = node->splats->numSplats;
	if(numSplats <= 0) return;

	// --- Build a 5-joint skeleton stacked along Y -------------------
	// Joint heights are chosen relative to the splat AABB's Y range so the
	// skeleton spans the model regardless of its absolute size.
	float minY = node->splats->min.y;
	float maxY = node->splats->max.y;
	float range = std::max(maxY - minY, 1e-3f);

	auto yAt = [&](float frac) { return minY + frac * range; };

	// Joint rest-pose positions (local translation relative to parent).
	// Hierarchy: root(spine0) -> spine -> chest -> neck -> head
	struct JointDef { const char* name; int parent; float yFrac; };
	static const JointDef joints[5] = {
		{"root",  -1, 0.00f},
		{"spine",  0, 0.25f},
		{"chest",  1, 0.55f},
		{"neck",   2, 0.80f},
		{"head",   3, 0.95f},
	};

	Skeleton sk;
	sk.jointNames.clear();
	sk.parents.clear();
	sk.inverseBindMatrices.clear();

	std::vector<glm::mat4> globalBind(5, glm::mat4(1.0f));
	for(int i = 0; i < 5; i++) {
		sk.jointNames.push_back(joints[i].name);
		sk.parents.push_back(joints[i].parent);

		glm::vec3 localPos(0.0f);
		if(joints[i].parent >= 0) {
			localPos.y = yAt(joints[i].yFrac) - yAt(joints[joints[i].parent].yFrac);
		} else {
			localPos.y = yAt(joints[i].yFrac);
		}
		glm::mat4 localM = glm::translate(glm::mat4(1.0f), localPos);
		if(joints[i].parent >= 0) {
			globalBind[i] = globalBind[joints[i].parent] * localM;
		} else {
			globalBind[i] = localM;
		}
		// IBP = inverse(globalBind).
		sk.inverseBindMatrices.push_back(glm::inverse(globalBind[i]));
	}

	node->rig.skeleton = sk;
	node->rig.blendshapeCount = 0;

	// --- Allocate device rest copies + skinning matrices + rig buffers ---
	node->initRestAndRig(numSplats, 5);

	// --- Generate per-splat bone indices/weights (nearest joint by Y) -------
	// All weight on a single bone (the nearest joint). This is the crudest
	// possible skinning but sufficient to validate the pipeline.
	std::vector<uint16_t> boneIndices(MAX_BONES_PER_SPLAT * numSplats, 0);
	std::vector<float>    boneWeights(MAX_BONES_PER_SPLAT * numSplats, 0.0f);

	const float* posData = reinterpret_cast<const float*>(node->splats->position->data);
	for(int64_t s = 0; s < numSplats; s++) {
		float y = posData[s * 3 + 1];
		float frac = (y - minY) / range;
		// Nearest of the 5 joint fractions {0.0, 0.25, 0.55, 0.80, 0.95}.
		float jointFracs[5] = {0.0f, 0.25f, 0.55f, 0.80f, 0.95f};
		int best = 0;
		float bestDist = std::fabs(frac - jointFracs[0]);
		for(int j = 1; j < 5; j++) {
			float d = std::fabs(frac - jointFracs[j]);
			if(d < bestDist) { bestDist = d; best = j; }
		}
		int off = (int)(s * MAX_BONES_PER_SPLAT);
		boneIndices[off + 0] = (uint16_t)best;
		boneWeights[off + 0] = 1.0f;
		// Remaining 3 slots stay 0.
	}

	CURuntime::check(cuMemcpyHtoD(node->rig.cptr_boneIndices,
		boneIndices.data(), sizeof(uint16_t) * MAX_BONES_PER_SPLAT * numSplats));
	CURuntime::check(cuMemcpyHtoD(node->rig.cptr_boneWeights,
		boneWeights.data(), sizeof(float) * MAX_BONES_PER_SPLAT * numSplats));

	node->poseDirty = true;
}

} // namespace motion
