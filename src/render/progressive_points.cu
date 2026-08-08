// Progressive real-time point-cloud rendering — CUDA kernels.
//
// Port of the technique from Schütz et al., "Progressive Real-Time Rendering
// of One Billion Points Without Hierarchical Acceleration Structures" (2019),
// reference implementation: https://github.com/arthurlirui/Skye (E:\Code\Skye),
// specifically modules/progressive/{distribute.cs, reproject.vs/fs, fill.vs/fs,
// create_vbo.cs, compute_fill*.cs}. Skye is OpenGL 4.5 + V8/JS; this file is a
// CUDA reimplementation so it integrates with Splatshop's CUDA-first renderer
// and reuses the existing (depth<<32)|color framebuffer convention.
//
// Per-frame pipeline (orchestrated in SplatEditor_draw.h):
//   1. clear_index            — zero the per-pixel index image
//   2. reproject              — draw last frame's visible points; write color
//                                to target.framebuffer and the packed global
//                                point ID to the index image
//   3. fill                   — draw a budget of fresh "random" points from the
//                                shuffled buffer, filling holes
//   4. create_vbo             — scan the index image, compact visible points
//                                into next frame's reproject buffer, write the
//                                indirect draw count
//
// The prime-permutation shuffle (kernel_progressive_distribute) runs once at
// load and is what makes a contiguous draw over the buffer sample the whole
// cloud uniformly, so per-frame cost is bounded by the fill budget.

#define CUB_DISABLE_BF16_SUPPORT

#define GLM_FORCE_CUDA
#define CUDA_VERSION 12000

namespace std{
	using size_t = ::size_t;
};

#include <cooperative_groups.h>
namespace cg = cooperative_groups;

#include "./libs/glm/glm/glm.hpp"
#include "./libs/glm/glm/gtc/matrix_transform.hpp"
#include "./libs/glm/glm/gtc/matrix_access.hpp"
#include "./libs/glm/glm/gtx/transform.hpp"
#include "./libs/glm/glm/gtc/quaternion.hpp"

#include "../utils.cuh"
#include "../HostDeviceInterface.h"
#include "../scene/ProgressivePointData.h"

// ProgressiveVertex and ProgressiveIndirectCommand are defined in
// scene/ProgressivePointData.h so the host side (SplatEditor_draw.h's
// ProgressiveTarget) can also see them.

// ─────────────────────────────────────────────────────────────────────────────
// Prime-congruence permutation — port of distribute.cs:permuteI.
// A bijection on [0, prime) when prime ≡ 3 (mod 4). Applying it twice gives a
// pseudo-random but unique target slot for every source index, so a contiguous
// draw range becomes a uniform random sample of the whole cloud.
//
// The intermediate n*n would overflow a signed int64 for large primes, so we
// compute in unsigned 64-bit, which is well-defined and portable (unlike
// __int128, which NVRTC only supports on Linux). This is safe because B2's
// clamp caps the cloud at PROGRESSIVE_MAX_BUFFERS * PROGRESSIVE_MAX_POINTS_PER_BUFFER
// ≈ 1.07e9 points, so prime ≤ ~1.07e9 and n*n ≤ ~1.14e18 < UINT64_MAX (1.84e19).
// ─────────────────────────────────────────────────────────────────────────────
__device__ __inline__
int64_t permuteI(int64_t number, int64_t prime) {
	if (number > prime) return number;

	uint64_t n = uint64_t(number);
	uint64_t p = uint64_t(prime);
	uint64_t q = n * n;
	uint64_t residue = q % p;

	if (number <= prime / 2) {
		return int64_t(residue);
	} else {
		return int64_t(p - residue);
	}
}

// Fetch a point from the correct chunk of a progressive cloud by global index.
// Port of the 8-way routing in distribute.cs:130 / create_vbo.cs:87.
__device__ __inline__
void fetchProgressivePoint(const ProgressivePointData& pc, uint32_t globalIndex,
                           vec3& outPos, uint32_t& outColor) {
	uint32_t mpb = uint32_t(pc.maxPointsPerBuffer);
	uint32_t b   = globalIndex / mpb;
	uint32_t li  = globalIndex - b * mpb;
	// Clamp to available buffers (clouds larger than 8 chunks are clamped at
	// init time, so this just guards the tail).
	if (b >= pc.numBuffers) b = pc.numBuffers - 1;
	outPos   = pc.position[b][li];
	outColor = pc.color[b][li];
}

// Write a point into the correct chunk by its (shuffled) target index.
__device__ __inline__
void storeProgressivePoint(ProgressivePointData& pc, uint32_t targetIndex,
                           vec3 pos, uint32_t color) {
	uint32_t mpb = uint32_t(pc.maxPointsPerBuffer);
	uint32_t b   = targetIndex / mpb;
	uint32_t li  = targetIndex - b * mpb;
	if (b >= pc.numBuffers) b = pc.numBuffers - 1;
	pc.position[b][li] = pos;
	pc.color[b][li]    = color;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 0 (load time): scatter canonical points into shuffled slots.
// Port of distribute.cs. One thread per canonical point. Reads from the node's
// PointData (position/color, laid out linearly) and writes to the progressive
// cloud's chunk buffers at the permuted target index. After this runs, drawing
// a contiguous range of the chunk buffers samples the whole cloud uniformly.
// ─────────────────────────────────────────────────────────────────────────────
extern "C" __global__
void kernel_progressive_distribute(
	CommonLaunchArgs args,
	PointData canonical,
	ProgressivePointData pc
){
	int index = cg::this_grid().thread_rank();
	if (index >= canonical.count) return;
	if (index >= int(pc.count))   return;

	vec3     pos   = canonical.position[index];
	uint32_t color = canonical.color[index];

	int64_t prime = int64_t(pc.prime);
	int64_t t = permuteI(int64_t(index), prime);
	t = permuteI(t, prime);
	uint32_t targetIndex = uint32_t(t);

	storeProgressivePoint(pc, targetIndex, pos, color);
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-pixel index image clear. Skye clears attachment 1 implicitly each frame;
// in CUDA we explicitly zero the index image before reproject+fill. A value of
// 0xFFFFFFFF means "no point".
// ─────────────────────────────────────────────────────────────────────────────
extern "C" __global__
void kernel_progressive_clear_index(
	CommonLaunchArgs args,
	RenderTarget target,
	uint32_t* indexImage
){
	int pixelID = cg::this_grid().thread_rank();
	int numPixels = target.width * target.height;
	if (pixelID >= numPixels) return;
	indexImage[pixelID] = 0xFFFFFFFFu;
}

// Write one point into the framebuffer + index image. Shared by reproject and
// fill. Uses Splatshop's (depth<<32)|color + atomicMin convention so it
// composes with HQS / splat rendering and the final GL blit unchanged. Writes
// the packed global point ID to the index image only if this point wins the
// depth test for that pixel (so create_vbo reads back the front-most point).
__device__ __inline__
void writeProgressivePoint(
	RenderTarget target, uint32_t* indexImage,
	vec3 pos, uint32_t color, uint32_t globalID, mat4 transform
){
	mat4 mvp = target.proj * target.view * transform;
	vec4 ndc = mvp * vec4(pos, 1.0f);
	if (ndc.w <= 0.0f) return;                 // behind camera

	vec2 imgCoords = vec2(
		(0.5f * (ndc.x / ndc.w) + 0.5f) * float(target.width),
		(0.5f * (ndc.y / ndc.w) + 0.5f) * float(target.height)
	);

	int px = int(imgCoords.x);
	int py = int(imgCoords.y);
	if (px < 0 || px >= target.width)  return;
	if (py < 0 || py >= target.height) return;

	int pixelID = px + py * target.width;

	uint32_t udepth = __float_as_uint(ndc.w);
	uint64_t pixel  = (uint64_t(udepth) << 32) | uint64_t(color);

	// Win-or-tie depth test, then stamp our ID. Using atomicMin on the packed
	// depth|color keeps behavior consistent with the rest of the renderer; we
	// only record the ID when our depth is the new minimum.
	uint64_t old = target.framebuffer[pixelID];
	if (pixel <= old) {
		atomicMin(&target.framebuffer[pixelID], pixel);
		// Best-effort ID stamp: the front-most point may be overwritten by a
		// closer one racing here, but create_vbo just needs *a* visible point
		// per pixel; ties favor whichever writes first.
		indexImage[pixelID] = globalID;
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 1: reproject last frame's visible points.
// Port of reproject.vs/fs. One thread per entry in the reproject buffer
// (`indirectCount` entries, built last frame by create_vbo). Each entry already
// carries its world-space position, color, and global point ID, so reproject is
// just a project + framebuffer write — cheap and view-dependent.
// ─────────────────────────────────────────────────────────────────────────────
extern "C" __global__
void kernel_progressive_reproject(
	CommonLaunchArgs args,
	RenderTarget target,
	uint32_t* indexImage,
	ProgressiveVertex* reprojectBuffer,
	ProgressiveIndirectCommand* indirect,
	mat4 transform
){
	int index = cg::this_grid().thread_rank();
	if (index >= int(indirect->count)) return;

	ProgressiveVertex v = reprojectBuffer[index];

	// Discard uninitialized slots (matches fill.vs:66 — a zero position with
	// zero color marks an empty compaction slot).
	if (v.color == 0u && v.ux == 0.0f && v.uy == 0.0f && v.uz == 0.0f) return;

	writeProgressivePoint(target, indexImage, vec3(v.ux, v.uy, v.uz), v.color, v.index, transform);
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 2: fill holes with a budget of fresh "random" points.
// Port of fill.vs/fs. Launches `budget` threads. Thread i reads the point at
// shuffled slot (fillOffset + i) % count, projects it, and writes it. Because
// the buffer is shuffled, advancing fillOffset uniformly samples the cloud; over
// a few frames every hole gets filled. The host advances fillOffset after launch.
//
// The multi-buffer routing (for >134M-point clouds) is handled by
// fetchProgressivePoint via the global index arithmetic.
// ─────────────────────────────────────────────────────────────────────────────
extern "C" __global__
void kernel_progressive_fill(
	CommonLaunchArgs args,
	RenderTarget target,
	uint32_t* indexImage,
	ProgressivePointData pc,
	uint32_t fillOffset,
	uint32_t budget,
	mat4 transform
){
	int i = cg::this_grid().thread_rank();
	if (i >= int(budget)) return;

	uint32_t count = pc.count;
	if (count == 0u) return;

	uint32_t globalIndex = (fillOffset + uint32_t(i)) % count;

	vec3     pos;
	uint32_t color;
	fetchProgressivePoint(pc, globalIndex, pos, color);

	// Discard uninitialized points (matches fill.vs:66). After distribute,
	// every slot within [0,count) is initialized, but defensive guard keeps
	// behavior safe if distribute hasn't fully covered the tail.
	if (color == 0u && pos.x == 0.0f && pos.y == 0.0f && pos.z == 0.0f) return;

	writeProgressivePoint(target, indexImage, pos, color, globalIndex, transform);
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 3: rebuild next frame's reproject buffer from the index image.
// Port of create_vbo.cs. One thread per framebuffer pixel (flattened 1D launch,
// since CudaModularProgram only exposes 1D grids; we recover (gx,gy) from the
// flat thread rank). If the pixel's index image entry is a real point ID,
// atomicAdd a compaction counter, fetch the point's LOCAL position + color from
// the progressive cloud, and store {localPos, color, globalID} into the reproject
// buffer at that slot. Storing LOCAL position lets reproject re-apply the node's
// current world transform next frame, which keeps reproject correct when the
// object moves between frames. The final counter value becomes indirect->count.
// ─────────────────────────────────────────────────────────────────────────────
extern "C" __global__
void kernel_progressive_create_vbo(
	CommonLaunchArgs args,
	RenderTarget target,
	uint32_t* indexImage,
	ProgressiveIndirectCommand* indirect,
	ProgressiveVertex* reprojectBuffer,
	ProgressivePointData pc
){
	// CudaModularProgram launches a 1D grid (Y/Z forced to 1), so recover the 2D
	// pixel coordinate from the flat thread rank. blockIdx.y/blockDim.y are
	// always 1/0 here — we deliberately do not read them.
	uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;

	int numPixels = target.width * target.height;
	if (int(tid) >= numPixels) return;

	int gx = int(tid) % target.width;
	int gy = int(tid) / target.width;
	int pixelID = gx + gy * target.width;   // == tid

	uint32_t globalID = indexImage[pixelID];
	if (globalID == 0xFFFFFFFFu) return;       // empty pixel
	if (globalID >= pc.count)    return;       // sanity

	vec3     pos;
	uint32_t color;
	fetchProgressivePoint(pc, globalID, pos, color);

	uint32_t slot = atomicAdd(&indirect->count, 1u);

	ProgressiveVertex v;
	v.ux    = pos.x;
	v.uy    = pos.y;
	v.uz    = pos.z;
	v.color = color;
	v.index = globalID;
	reprojectBuffer[slot] = v;
}

// ─────────────────────────────────────────────────────────────────────────────
// Reset the indirect command's count to zero (host calls this before
// create_vbo so the compaction counter starts fresh each frame).
// ─────────────────────────────────────────────────────────────────────────────
extern "C" __global__
void kernel_progressive_reset_indirect(
	CommonLaunchArgs args,
	ProgressiveIndirectCommand* indirect
){
	if (cg::this_grid().thread_rank() == 0) {
		indirect->count      = 0;
		indirect->primCount  = 0;
		indirect->firstIndex = 0;
		indirect->baseVertex = 0;
		indirect->baseInstance = 0;
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// Adaptive fill budget — port of compute_fill_fixed.cs.
// Skye uses GL timestamp queries to measure time already spent (reproject +
// fixed fill), estimate points-per-millisecond, and write the remaining-budget
// fill count for a target frame time. In CUDA we feed the measured reproject+
// fill duration (ms) from the host (via CUDA events) and the target frame time,
// and this kernel writes the additional fill count into `outRemaining`.
//
// This is the self-regulating real-time mechanism: if reproject+fixed fill were
// cheap, spend more points this frame; if they were expensive, spend fewer.
// ─────────────────────────────────────────────────────────────────────────────
struct AdaptiveFillInput {
	float elapsedMs;        // time spent in reproject + fixed fill so far
	float targetFrameMs;    // total frame-time budget (e.g. 16.6ms for 60fps)
	float pointsPerMs;      // measured throughput (points / ms)
	uint32_t numPoints;     // total points in cloud
	uint32_t minFill;       // floor: always fill at least this many
};

extern "C" __global__
void kernel_progressive_compute_fill(
	CommonLaunchArgs args,
	AdaptiveFillInput in,
	uint32_t* outRemaining
){
	if (cg::this_grid().thread_rank() != 0) return;

	float remainingMs = in.targetFrameMs - in.elapsedMs;
	if (remainingMs < 0.0f) remainingMs = 0.0f;

	float estimate = remainingMs * in.pointsPerMs;
	if (estimate < float(in.minFill)) estimate = float(in.minFill);

	uint32_t remaining = uint32_t(estimate);
	if (remaining > in.numPoints) remaining = in.numPoints;

	*outRemaining = remaining;
}
