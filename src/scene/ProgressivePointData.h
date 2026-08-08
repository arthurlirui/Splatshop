#pragma once

// Progressive (Skye-style) point-cloud rendering data.
//
// This implements the technique from Schütz et al., "Progressive Real-Time
// Rendering of One Billion Points Without Hierarchical Acceleration Structures"
// (TU Wien, 2019), referenced from https://github.com/arthurlirui/Skye
// (reference repo E:\Code\Skye). The idea:
//   - At load time, every point is scattered to a pseudo-random slot in the
//     vertex buffer using a prime-congruence permutation. A contiguous draw
//     over the buffer therefore uniformly samples the whole cloud.
//   - Each frame: reproject last frame's visible points (cheap), then fill a
//     budget of new "random" points to cover holes, then rebuild next frame's
//     reproject buffer from a per-pixel index image. Over a few frames the
//     image converges while per-frame cost stays bounded by the fill budget.
//
// The host side (this header) owns the shuffled device buffers and the prime.
// The device kernels live in src/render/progressive_points.cu. The per-target
// render state (reproject VBO, index image, indirect count) is cached in
// SplatEditor_draw.h's ProgressiveTarget, mirroring ConcurrentTarget.

#include <cstdint>

// The host-side owner (ProgressivePointCloud) needs <memory>, <vector>,
// <cmath> and CudaVirtualMemory.h. NVRTC has no host standard library on its
// include path and the .cu kernels only touch the POD ProgressivePointData
// struct below, so guard all host-only dependencies. The ProgressivePointCloud
// struct itself is also guarded further down.
#ifndef __CUDACC__
#include <cmath>
#include <memory>
#include <vector>
#include "CudaVirtualMemory.h"
#endif

#include "HostDeviceInterface.h"

// Matches Skye's MAX_POINTS_PER_BUFFER: the largest single GL buffer they
// upload at once. We keep the same constant so the multi-buffer routing logic
// in the kernels is a faithful port. CUDA virtual memory can grow beyond this,
// but chunking keeps the prime-permutation indices in uint32 range and matches
// the reference implementation's >134M-point handling.
inline constexpr uint64_t PROGRESSIVE_MAX_POINTS_PER_BUFFER = 134'000'000ull;

// Number of chunk buffers a single cloud may span (matches Skye's 8-way
// distribute.cs / create_vbo.cs routing). 8 * 134M ≈ 1.07 billion points.
inline constexpr uint32_t PROGRESSIVE_MAX_BUFFERS = 8;

// Reproject-buffer vertex: {position, color, globalPointID}. Matches Skye's
// VertexT in create_vbo.cs (20 bytes). create_vbo writes into this; reproject
// and fill read from it. Position is stored in the cloud's LOCAL space so that
// reproject can re-apply the node's current world transform each frame (this
// keeps reproject correct when the object moves between frames).
struct ProgressiveVertex {
	float    ux, uy, uz;
	uint32_t color;
	uint32_t index;
};

// Indirect draw command consumed by the reproject pass: how many points are in
// this frame's reproject buffer. Written by create_vbo (atomicAdd into `count`),
// read back by the host to launch reproject over exactly that many threads.
// CUDA has no glDrawArraysIndirect, so the host reads `count` back via a pinned
// mirror (ProgressiveTarget::hostIndirect); the 5-uint layout is kept for parity
// with Skye but only `count` is used.
struct ProgressiveIndirectCommand {
	uint32_t count;
	uint32_t primCount;
	uint32_t firstIndex;
	uint32_t baseVertex;
	uint32_t baseInstance;
};

// Device-side view of a progressive point cloud. Passed to the kernels.
// Mirrors Skye's per-VBO {vec3 position, uint32 color} layout, split into
// separate position/color arrays because that is how Splatshop's PointData /
// Points already store them; the kernels index them in lockstep.
struct ProgressivePointData {

	uint32_t count = 0;                 // total points in the cloud
	uint32_t numBuffers = 0;            // number of populated chunk buffers
	uint64_t maxPointsPerBuffer = PROGRESSIVE_MAX_POINTS_PER_BUFFER;

	// Per-chunk device pointers (up to PROGRESSIVE_MAX_BUFFERS). Uninitialized
	// chunks beyond numBuffers are never read by the kernels.
	vec3*     position[PROGRESSIVE_MAX_BUFFERS] = {};
	uint32_t* color[PROGRESSIVE_MAX_BUFFERS]    = {};

	// Prime used by the permutation. Stored as uint64 so the kernel's
	// (int64)n*n computation cannot overflow before the mod for any uint32 index.
	uint64_t prime = 0;

	// Live fill cursor (wraps around 0..count). Advanced by the fill kernel
	// indirectly (host reads/writes it each frame when launching fill).
	uint32_t fillOffset = 0;

	bool ready = false;                 // set once distribute() has completed
};

// Host-side owner of the shuffled chunk buffers for one SNPoints node.
// Allocates PROGRESSIVE_MAX_BUFFERS chunks up front (virtual memory is only
// physically committed on demand), then commit()s enough to hold `count` points.
#ifndef __CUDACC__
struct ProgressivePointCloud {

	shared_ptr<CudaVirtualMemory> vm_position[PROGRESSIVE_MAX_BUFFERS];
	shared_ptr<CudaVirtualMemory> vm_color[PROGRESSIVE_MAX_BUFFERS];

	uint32_t numBuffers = 0;            // populated chunk count
	uint64_t pointsPerBuffer = 0;       // points actually stored in each full chunk
	uint64_t count = 0;                 // total points
	uint64_t prime = 0;

	ProgressivePointData data{};

	// Largest prime p ≡ 3 (mod 4) with p <= n.
	// Port of Skye's previousPrimeCongruent3mod4 (ProgressiveLoader.h:45-74).
	// A prime congruent to 3 mod 4 makes the quadratic permutation a bijection
	// on [0, prime) (see preshing.com/20121224), which is what gives the
	// "contiguous draw = uniform random sample" property.
	static uint64_t largestPrimeCongruent3mod4(uint64_t n) {
		auto isPrime = [](uint64_t x) -> bool {
			if (x < 2) return false;
			if (x < 4) return true;          // 2, 3
			if (x % 2 == 0) return false;
			for (uint64_t i = 3; i * i <= x; i += 2) {
				if (x % i == 0) return false;
			}
			return true;
		};

		if (n < 3) return 2;
		// Walk down to the largest value ≡ 3 (mod 4) at or below n.
		uint64_t candidate = n - ((n - 3) % 4); // largest <= n with candidate%4 == 3
		while (true) {
			if (candidate <= 2) return 2;
			if (isPrime(candidate)) return candidate;
			if (candidate >= 4) candidate -= 4;
			else return 2;
		}
	}

	// Allocate chunk buffers and commit enough physical memory for `numPoints`.
	// After this returns, data.position[]/color[] point at the (still unshuffled)
	// chunks. The host then launches kernel_progressive_distribute to scatter the
	// canonical PointData into these slots.
	void init(uint64_t numPoints) {
		count = numPoints;
		numBuffers = uint32_t((numPoints + PROGRESSIVE_MAX_POINTS_PER_BUFFER - 1) / PROGRESSIVE_MAX_POINTS_PER_BUFFER);
		if (numBuffers == 0) numBuffers = 1;
		if (numBuffers > PROGRESSIVE_MAX_BUFFERS) {
			// The kernels route a global index to chunk b = index / mpb and clamp
			// b to numBuffers-1, but they do NOT recompute the local index — so
			// any index beyond numBuffers*mpb would read/write past the last
			// chunk. We therefore clamp `count` (and thus the prime and the fill
			// range) to what the allocated buffers can actually hold, rather than
			// only clamping numBuffers. This keeps the tail-chunk sizing below as
			// well in range.
			uint64_t maxSupported = uint64_t(PROGRESSIVE_MAX_BUFFERS) * PROGRESSIVE_MAX_POINTS_PER_BUFFER;
			println("WARNING: progressive point cloud has {} points, needs {} buffers but only {} ({} points) are supported; clamping point count.",
				numPoints, numBuffers, PROGRESSIVE_MAX_BUFFERS, maxSupported);
			numBuffers = PROGRESSIVE_MAX_BUFFERS;
			count = maxSupported;
			numPoints = maxSupported;
		}

		prime = largestPrimeCongruent3mod4(count);

		for (uint32_t i = 0; i < numBuffers; i++) {
			uint64_t ptsThis = (i + 1 < numBuffers) ? PROGRESSIVE_MAX_POINTS_PER_BUFFER
				: (count - uint64_t(i) * PROGRESSIVE_MAX_POINTS_PER_BUFFER);

			vm_position[i] = CudaVirtualMemory::create();
			vm_color[i]    = CudaVirtualMemory::create();

			vm_position[i]->commit(ptsThis * sizeof(vec3));
			vm_color[i]->commit(ptsThis * sizeof(uint32_t));

			data.position[i] = (vec3*)vm_position[i]->cptr;
			data.color[i]    = (uint32_t*)vm_color[i]->cptr;
		}

		data.count        = uint32_t(count);
		data.numBuffers   = numBuffers;
		data.maxPointsPerBuffer = PROGRESSIVE_MAX_POINTS_PER_BUFFER;
		data.prime        = prime;
		data.fillOffset   = 0;
		data.ready        = false;

		pointsPerBuffer = PROGRESSIVE_MAX_POINTS_PER_BUFFER;
	}

	uint64_t getGpuMemoryUsage() const {
		uint64_t usage = 0;
		for (uint32_t i = 0; i < numBuffers; i++) {
			if (vm_position[i]) usage += vm_position[i]->comitted;
			if (vm_color[i])    usage += vm_color[i]->comitted;
		}
		return usage;
	}
};
#endif // __CUDACC__
