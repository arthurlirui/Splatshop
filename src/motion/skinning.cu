// Per-frame linear blend skinning (LBS) + blendshape deformation kernel for
// SNRiggedSplats nodes.
//
// For each splat, reads the rest-pose position/scale/quaternion and the
// per-splat bone indices/weights, applies the current-frame skinning matrices,
// optionally accumulates facial blendshape deltas, and writes the deformed
// result into the node's dmng.data position/scale/quaternion buffers. The
// standard render kernels then read those buffers unchanged.
//
// Launched from RiggedHumanController::dispatchSkinning every frame for each
// dirty rigged node.

#define CUB_DISABLE_BF16_SUPPORT

#define GLM_FORCE_CUDA
#define CUDA_VERSION 12000

namespace std {
	using size_t = ::size_t;
};

using namespace std;

#include "./libs/glm/glm/glm.hpp"
#include "./libs/glm/glm/gtc/matrix_transform.hpp"
#include "./libs/glm/glm/gtc/quaternion.hpp"

#include "utils.cuh"
#include "HostDeviceInterface.h"

namespace cg = cooperative_groups;

using glm::vec3;
using glm::vec4;
using glm::mat3;
using glm::mat4;
using glm::quat;

constexpr int MAX_BONES_PER_SPLAT = 4;
// Blendshape delta layout per splat per blendshape: 10 floats.
//   [0..2] position delta, [3..6] rotation delta (quaternion, w first),
//   [7..9] scale delta.
constexpr int BS_FLOATS_PER_SPLAT = 10;

extern "C" __global__
void kernel_skin_splats(
	uint32_t splatCount,
	int numJoints,
	int blendshapeCount,
	// out: deformed splat attributes (written in place into dmng.data)
	vec3* outPosition,
	vec3* outScale,
	vec4* outQuaternion,
	// in: rest-pose copies
	const vec3* restPosition,
	const vec3* restScale,
	const vec4* restQuaternion,
	// in: per-splat skinning data
	const uint16_t* boneIndices,   // [splat * MAX_BONES_PER_SPLAT + b]
	const float* boneWeights,      // [splat * MAX_BONES_PER_SPLAT + b]
	// in: per-frame skinning matrices (M_skin = M_global * IBP), one mat4 per joint
	const mat4* skinningMatrices,
	// in: facial blendshape deltas (optional; may be null)
	const float* blendshapeDeltas, // [bs * splatCount * BS_FLOATS_PER_SPLAT + splat * BS_FLOATS_PER_SPLAT + k]
	const float* faceWeights       // [blendshapeCount]
) {
	uint32_t splatIndex = cg::this_grid().thread_rank();
	if(splatIndex >= splatCount) return;

	// --- Linear blend skinning of position ---
	vec3 rp = restPosition[splatIndex];
	vec3 deformedPos(0.0f);

	// Weighted blend of rotation matrices extracted from each skinning matrix.
	// We accumulate a blended 3x3 rotation and a blended scale per splat.
	mat3 blendedRot(0.0f);
	vec3 blendedScale(0.0f);

	const uint16_t* bi = boneIndices + splatIndex * MAX_BONES_PER_SPLAT;
	const float*     bw = boneWeights + splatIndex * MAX_BONES_PER_SPLAT;

	for(int b = 0; b < MAX_BONES_PER_SPLAT; b++) {
		float w = bw[b];
		if(w <= 0.0f) continue;
		int joint = bi[b];
		if(joint < 0 || joint >= numJoints) continue;

		mat4 m = skinningMatrices[joint];
		// Position: m * restPos, weighted.
		deformedPos += w * vec3(m * vec4(rp, 1.0f));

		// Rotation/scale: extract the 3x3 upper-left of the skinning matrix.
		// (This blends rotation and scale jointly, which is the standard LBS
		// approximation for gaussian splats.)
		mat3 r3(
			m[0][0], m[0][1], m[0][2],
			m[1][0], m[1][1], m[1][2],
			m[2][0], m[2][1], m[2][2]
		);
		blendedRot += w * r3;
		blendedScale += w * vec3(
			glm::length(vec3(m[0][0], m[0][1], m[0][2])),
			glm::length(vec3(m[1][0], m[1][1], m[1][2])),
			glm::length(vec3(m[2][0], m[2][1], m[2][2]))
		);
	}

	outPosition[splatIndex] = deformedPos;

	// --- Scale: rest scale multiplied by the blended bone scale ---
	vec3 rs = restScale[splatIndex];
	outScale[splatIndex] = rs * blendedScale;

	// --- Rotation: transform the rest quaternion by the blended rotation ---
	// Device quaternion storage is vec4(.x=w, .y=x, .z=y, .w=z).
	vec4 rq = restQuaternion[splatIndex];
	quat qRest = quat(rq.x, rq.y, rq.z, rq.w); // (w, x, y, z)

	// Build a quaternion from the blended rotation matrix.
	// (Shear/non-orthonormal blends are approximated by the closest rotation.)
	quat qBlend = quat(1.0f, 0.0f, 0.0f, 0.0f);
	{
		mat3 r = blendedRot;
		float trace = r[0][0] + r[1][1] + r[2][2];
		if(trace > 0.0f) {
			float s = 0.5f / sqrt(trace + 1.0f);
			qBlend.w = 0.25f / s;
			qBlend.x = (r[1][2] - r[2][1]) * s;
			qBlend.y = (r[2][0] - r[0][2]) * s;
			qBlend.z = (r[0][1] - r[1][0]) * s;
		} else if(r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
			float s = 2.0f * sqrt(1.0f + r[0][0] - r[1][1] - r[2][2]);
			qBlend.w = (r[1][2] - r[2][1]) / s;
			qBlend.x = 0.25f * s;
			qBlend.y = (r[1][0] + r[0][1]) / s;
			qBlend.z = (r[2][0] + r[0][2]) / s;
		} else if(r[1][1] > r[2][2]) {
			float s = 2.0f * sqrt(1.0f + r[1][1] - r[0][0] - r[2][2]);
			qBlend.w = (r[2][0] - r[0][2]) / s;
			qBlend.x = (r[1][0] + r[0][1]) / s;
			qBlend.y = 0.25f * s;
			qBlend.z = (r[2][1] + r[1][2]) / s;
		} else {
			float s = 2.0f * sqrt(1.0f + r[2][2] - r[0][0] - r[1][1]);
			qBlend.w = (r[0][1] - r[1][0]) / s;
			qBlend.x = (r[2][0] + r[0][2]) / s;
			qBlend.y = (r[2][1] + r[1][2]) / s;
			qBlend.z = 0.25f * s;
		}
		float l = sqrt(qBlend.w * qBlend.w + qBlend.x * qBlend.x
		              + qBlend.y * qBlend.y + qBlend.z * qBlend.z);
		if(l > 0.0f) {
			float inv = 1.0f / l;
			qBlend.w *= inv; qBlend.x *= inv; qBlend.y *= inv; qBlend.z *= inv;
		}
	}

	quat qDeformed = qBlend * qRest;
	outQuaternion[splatIndex] = vec4(qDeformed.w, qDeformed.x, qDeformed.y, qDeformed.z);

	// --- Facial blendshape deltas (optional) ---
	if(blendshapeCount > 0 && blendshapeDeltas != nullptr && faceWeights != nullptr) {
		vec3 dPos(0.0f);
		quat dRot(0.0f, 0.0f, 0.0f, 0.0f);
		vec3 dScale(0.0f);
		for(int bs = 0; bs < blendshapeCount; bs++) {
			float w = faceWeights[bs];
			if(w == 0.0f) continue;
			const float* d = blendshapeDeltas
				+ (uint64_t)bs * splatCount * BS_FLOATS_PER_SPLAT
				+ (uint64_t)splatIndex * BS_FLOATS_PER_SPLAT;
			dPos += w * vec3(d[0], d[1], d[2]);
			// Rotation delta quaternion (w, x, y, z).
			quat dq = quat(d[3], d[4], d[5], d[6]);
			// Accumulate rotation deltas via normalized linear blend (small
			// rotations approximation; adequate for facial expression deltas).
			dRot.w += w * dq.w;
			dRot.x += w * dq.x;
			dRot.y += w * dq.y;
			dRot.z += w * dq.z;
			dScale += w * vec3(d[7], d[8], d[9]);
		}
		outPosition[splatIndex] = deformedPos + dPos;
		outScale[splatIndex] = rs * blendedScale + dScale;
		float l = sqrt(dRot.w * dRot.w + dRot.x * dRot.x
		              + dRot.y * dRot.y + dRot.z * dRot.z);
		if(l > 0.0f) {
			float inv = 1.0f / l;
			dRot.w *= inv; dRot.x *= inv; dRot.y *= inv; dRot.z *= inv;
			qDeformed = dRot * qDeformed;
			outQuaternion[splatIndex] = vec4(qDeformed.w, qDeformed.x, qDeformed.y, qDeformed.z);
		}
	}
}
