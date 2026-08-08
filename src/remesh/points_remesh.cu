
// Point-cloud remeshing / density optimization.
//
// Voxel-grid downsampling: each input point is assigned to a voxel of edge
// length h, points are sorted by their 32-bit packed voxel key, and every
// occupied voxel collapses to a single centroid point (mean position, mean
// color). This reduces point density to a uniform target spacing h while
// preserving geometric coverage, lowering VRAM and improving render speed.
//
// Pipeline (orchestrated from SplatEditor::remeshPointCloud):
//   1. kernel_compute_voxelKeys   — per point: key = pack(floor((p-min)/h))
//   2. GPUSorting::sort_32bit_keyvalue — sort point indices by voxel key
//   3. kernel_mark_seg_heads      — per sorted point: 1 if first in its voxel
//   4. GPUPrefixSums::dispatch    — exclusive prefix sum -> compact out index
//   5. kernel_clear_accum         — zero the per-output accumulators
//   6. kernel_accumulate          — atomicAdd each point into its voxel slot
//   7. kernel_normalize           — divide sums by count -> final centroid
//   8. kernel_compute_boundingbox_remesh — recompute output AABB
//
// The 32-bit key packs three 10-bit voxel coordinates (1024^3 = 1B voxels),
// which is ample for any practical voxel-grid downsample of real-world point
// clouds. The voxel grid is clamped per axis so out-of-range points fall into
// the boundary voxels instead of corrupting the key.

#define CUB_DISABLE_BF16_SUPPORT

#define GLM_FORCE_CUDA
#define CUDA_VERSION 12000

namespace std{
	using size_t = ::size_t;
};

#include "./libs/glm/glm/glm.hpp"
#include "./libs/glm/glm/gtc/matrix_transform.hpp"

#include "../utils.cuh"

#include "../HostDeviceInterface.h"

// 10 bits per axis -> 1024 voxels per axis, packed into 32 bits as (x<<20)|(y<<10)|z.
// Leaves 2 bits spare; enough for any realistic downsample of a point cloud.
constexpr uint32_t VOXEL_BITS = 10;
constexpr uint32_t VOXEL_MASK = (1u << VOXEL_BITS) - 1u;
constexpr uint32_t VOXEL_PER_AXIS = (1u << VOXEL_BITS);

// Pack integer voxel coordinates into a 32-bit key. Axis order x<<20 | y<<10 | z
// so the sort groups points by x-major then y then z — locality-preserving.
__device__ __inline__
uint32_t packVoxelKey(uint32_t vx, uint32_t vy, uint32_t vz){
	return (vx << (2 * VOXEL_BITS)) | (vy << VOXEL_BITS) | vz;
}

// Compute the voxel index along one axis, clamped to [0, VOXEL_PER_AXIS-1].
__device__ __inline__
uint32_t voxelCoord(float p, float min, float invH){
	int32_t c = int32_t((p - min) * invH);
	if (c < 0) c = 0;
	if (c >= int32_t(VOXEL_PER_AXIS)) c = int32_t(VOXEL_PER_AXIS) - 1;
	return uint32_t(c);
}

// Per point: compute the packed 32-bit voxel key and store (key, pointIndex).
// Keys/values feed GPUSorting::sort_32bit_keyvalue.
extern "C" __global__
void kernel_compute_voxelKeys(
	CommonLaunchArgs args,
	PointData in,
	uint32_t* keys,
	uint32_t* vals,
	vec3 min,
	float invH
){
	int index = cg::this_grid().thread_rank();
	if(index >= in.count) return;

	vec3 pos = in.position[index];

	uint32_t vx = voxelCoord(pos.x, min.x, invH);
	uint32_t vy = voxelCoord(pos.y, min.y, invH);
	uint32_t vz = voxelCoord(pos.z, min.z, invH);

	keys[index] = packVoxelKey(vx, vy, vz);
	vals[index] = uint32_t(index);
}

// Per sorted point: write 1 if this point starts a new voxel segment
// (i.e. its key differs from the previous sorted point's key), else 0.
// Point 0 is always a segment head. Output feeds an exclusive prefix sum.
extern "C" __global__
void kernel_mark_seg_heads(
	CommonLaunchArgs args,
	uint32_t* sortedKeys,
	uint32_t count,
	uint32_t* segHeads
){
	int index = cg::this_grid().thread_rank();
	if(index >= count) return;

	uint32_t key = sortedKeys[index];
	uint32_t prevKey = (index == 0) ? 0xFFFFFFFFu : sortedKeys[index - 1];
	segHeads[index] = (key != prevKey) ? 1u : 0u;
}

// Zero the per-output accumulators (position sums, per-channel color sums,
// and counts) so kernel_accumulate can atomicAdd into them safely.
// Launched over the output slot count.
extern "C" __global__
void kernel_clear_accum(
	CommonLaunchArgs args,
	PointData out,
	vec4* accColor   // RGBA as 4x float for atomicAdd-friendliness
){
	int index = cg::this_grid().thread_rank();
	if(index >= out.count) return;

	out.position[index] = vec3(0.0f);
	accColor[index] = vec4(0.0f);
	out.flags[index] = 0u;       // reused as per-voxel point count
}

// Per sorted input point: atomicAdd its position and color into the output
// slot indexed by the prefix-summed segment id (compactIndex). Because points
// sharing a voxel are contiguous after the sort, every point in a segment
// contributes to the same slot, yielding the voxel centroid after normalize.
extern "C" __global__
void kernel_accumulate(
	CommonLaunchArgs args,
	PointData in,
	uint32_t* sortedVals,
	uint32_t* compactIndex,
	uint32_t count,
	PointData out,
	vec4* accColor
){
	int index = cg::this_grid().thread_rank();
	if(index >= count) return;

	uint32_t srcPoint = sortedVals[index];
	uint32_t slot = compactIndex[index];

	vec3 pos = in.position[srcPoint];
	atomicAdd(&out.position[slot].x, pos.x);
	atomicAdd(&out.position[slot].y, pos.y);
	atomicAdd(&out.position[slot].z, pos.z);

	uint32_t color = in.color[srcPoint];
	vec4 rgba = vec4(
		float(color & 0xFFu),
		float((color >> 8) & 0xFFu),
		float((color >> 16) & 0xFFu),
		float((color >> 24) & 0xFFu)
	);
	atomicAdd(&accColor[slot].x, rgba.x);
	atomicAdd(&accColor[slot].y, rgba.y);
	atomicAdd(&accColor[slot].z, rgba.z);
	atomicAdd(&accColor[slot].w, rgba.w);

	atomicAdd(&out.flags[slot], 1u);
}

// Per output slot: divide accumulated sums by the voxel's point count to
// produce the final centroid position and averaged RGBA color. Slots with a
// zero count (shouldn't occur, but guard anyway) are left black/zero.
extern "C" __global__
void kernel_normalize(
	CommonLaunchArgs args,
	PointData out,
	vec4* accColor
){
	int index = cg::this_grid().thread_rank();
	if(index >= out.count) return;

	uint32_t cnt = out.flags[index];
	if(cnt == 0u) return;

	float invCount = 1.0f / float(cnt);

	out.position[index] = out.position[index] * invCount;

	vec4 rgba = accColor[index] * invCount;
	uint8_t r = uint8_t(clamp(rgba.x + 0.5f, 0.0f, 255.0f));
	uint8_t g = uint8_t(clamp(rgba.y + 0.5f, 0.0f, 255.0f));
	uint8_t b = uint8_t(clamp(rgba.z + 0.5f, 0.0f, 255.0f));
	uint8_t a = uint8_t(clamp(rgba.w + 0.5f, 0.0f, 255.0f));
	out.color[index] = uint32_t(r) | (uint32_t(g) << 8)
	                 | (uint32_t(b) << 16) | (uint32_t(a) << 24);

	// Reset flags to a clean (non-deleted, non-selected) state now that the
	// count reuse is done, so the renderer sees valid per-point flags.
	out.flags[index] = 0u;
}

// Recompute the output cloud's axis-aligned bounding box from its positions,
// mirroring kernel_compute_boundingbox in points.cu. Writes min/max directly.
extern "C" __global__
void kernel_compute_boundingbox_remesh(
	CommonLaunchArgs args,
	PointData model,
	vec3& min,
	vec3& max
){
	int index = cg::this_grid().thread_rank();
	if(index >= model.count) return;

	vec3 pos = vec3(model.transform * vec4(model.position[index], 1.0f));

	if(pos.x < min.x) atomicMinFloat(&min.x, pos.x);
	if(pos.y < min.y) atomicMinFloat(&min.y, pos.y);
	if(pos.z < min.z) atomicMinFloat(&min.z, pos.z);
	if(pos.x > max.x) atomicMaxFloat(&max.x, pos.x);
	if(pos.y > max.y) atomicMaxFloat(&max.y, pos.y);
	if(pos.z > max.z) atomicMaxFloat(&max.z, pos.z);
}
