#include "gui/ImguiPage.h"
#include "gui/guivr.h"
#include "scene/SN4DGSSplats.h"
#include "scene/ProgressivePointData.h"

struct ConcurrentTarget{
	RenderTarget target;
	CUstream mainstream;
	CUstream sidestream;

	CUevent cu_tilesComputed;

	shared_ptr<CudaVirtualMemory> virt_fb_depth;
	shared_ptr<CudaVirtualMemory> virt_fb_color;

	CUdeviceptr cptr_numVisibleSplats;
	CUdeviceptr cptr_numFragments;
	CUdeviceptr cptr_tiles;

	shared_ptr<CudaVirtualMemory> virt_numTilefragments_splatwise;
	shared_ptr<CudaVirtualMemory> virt_numTilefragments_splatwise_ordered;
	shared_ptr<CudaVirtualMemory> virt_tileIDs;
	shared_ptr<CudaVirtualMemory> virt_indices;
	shared_ptr<CudaVirtualMemory> virt_depth;
	shared_ptr<CudaVirtualMemory> virt_bounds;
	shared_ptr<CudaVirtualMemory> virt_ordering_splatdepth;
	shared_ptr<CudaVirtualMemory> virt_stagedata;

	uint32_t numVisibleSplats;
	uint32_t numFragments;
};

// Per-target progressive-render state, cached across frames like ConcurrentTarget.
// Mirrors Skye's render_progressive.js state (reproject VBO, index image, indirect
// command) but in CUDA: a compaction buffer of ProgressiveVertex, a per-pixel
// uint32 index image (0xFFFFFFFF = empty), and an indirect command struct.
struct ProgressiveTarget {
	RenderTarget target;
	shared_ptr<CudaVirtualMemory> virt_indexImage;        // uint32 per pixel
	shared_ptr<CudaVirtualMemory> virt_reprojectBuffer;   // ProgressiveVertex[]
	shared_ptr<CudaVirtualMemory> virt_indirect;          // ProgressiveIndirectCommand

	CUdeviceptr cptr_indexImage = 0;
	CUdeviceptr cptr_reprojectBuffer = 0;
	CUdeviceptr cptr_indirect = 0;

	// Host-visible mirror of the indirect command's `count`, so we can launch
	// the reproject kernel over exactly the right number of threads without a
	// device->host sync per frame (we read it back via a pinned staging copy).
	ProgressiveIndirectCommand* hostIndirect = nullptr;
	CUdeviceptr devStagingIndirect = 0;

	uint32_t reprojectCapacity = 0;   // max ProgressiveVertex entries allocated
	bool initialized = false;

	// Adaptive-budget throughput estimate (points/ms), updated each frame.
	float pointsPerMs = 0.0f;
	CUevent evFillStart = 0;
	CUevent evFillEnd = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Progressive point rendering — the Skye-style port.
//
// One ProgressiveTarget is cached per RenderTarget (keyed by framebuffer
// pointer + size), holding the reproject VBO, the per-pixel index image, and
// the indirect command. Per frame, for each ready SNPoints node:
//   clear_index -> reproject (last frame's visible pts) -> fill (budget of new
//   shuffled pts) -> create_vbo (compact visible pts into next frame's VBO).
//
// v1 follows Skye's single-active-cloud model: only the first ready node is
// rendered progressively; additional ready nodes are skipped here (they would
// need per-node reproject buffers — Phase 3). All points write into the shared
// target.framebuffer with the (depth<<32)|color + atomicMin convention.
// ─────────────────────────────────────────────────────────────────────────────
inline void drawpoints_progressive(
	vector<SNPoints*> nodes,
	RenderTarget target,
	CUstream mainstream
){
	if(nodes.size() == 0) return;

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto prog = editor->prog_progressive_points;
	if(prog == nullptr) return;

	SNPoints* node = nodes[0];
	ProgressivePointData pc = node->progressive.data;
	if(!pc.ready || pc.count == 0u) return;

	mat4 transform = node->transform_global;

	// Lazily grow / (re)allocate the per-target progressive cache. Keyed on the
	// framebuffer pointer + dimensions so a resized target gets a fresh buffer.
	static vector<ProgressiveTarget> cache;
	ProgressiveTarget* pt = nullptr;
	for(auto& c : cache){
		if(c.target.framebuffer == target.framebuffer &&
		   c.target.width == target.width && c.target.height == target.height){
			pt = &c;
			break;
		}
	}

	uint32_t numPixels = uint32_t(target.width) * uint32_t(target.height);
	// Cap the reproject buffer at one entry per pixel (a point can occupy at
	// most one pixel after dedup), rounded up for virtual-memory granularity.
	uint32_t reprojectCapacity = numPixels;

	if(pt == nullptr){
		ProgressiveTarget fresh;
		fresh.target = target;

		fresh.virt_indexImage = CudaVirtualMemory::create();
		fresh.virt_indexImage->commit(numPixels * sizeof(uint32_t));
		fresh.cptr_indexImage = fresh.virt_indexImage->cptr;

		fresh.virt_reprojectBuffer = CudaVirtualMemory::create();
		fresh.virt_reprojectBuffer->commit(reprojectCapacity * sizeof(ProgressiveVertex));
		fresh.cptr_reprojectBuffer = fresh.virt_reprojectBuffer->cptr;

		fresh.virt_indirect = CudaVirtualMemory::create();
		fresh.virt_indirect->commit(sizeof(ProgressiveIndirectCommand));
		fresh.cptr_indirect = fresh.virt_indirect->cptr;

		// Pinned host mirror for reading the indirect count back without a
		// full device->host copy each frame.
		cuMemAllocHost((void**)&fresh.hostIndirect, sizeof(ProgressiveIndirectCommand));
		fresh.hostIndirect->count = 0;

		cuEventCreate(&fresh.evFillStart, CU_EVENT_DEFAULT);
		cuEventCreate(&fresh.evFillEnd, CU_EVENT_DEFAULT);

		fresh.reprojectCapacity = reprojectCapacity;
		fresh.initialized = true;

		cache.push_back(std::move(fresh));
		pt = &cache.back();
	}

	// Grow buffers if the target grew.
	if(pt->reprojectCapacity < reprojectCapacity){
		pt->virt_reprojectBuffer->commit(reprojectCapacity * sizeof(ProgressiveVertex));
		pt->cptr_reprojectBuffer = pt->virt_reprojectBuffer->cptr;
		pt->reprojectCapacity = reprojectCapacity;
	}
	if(uint64_t(pt->virt_indexImage->comitted) < uint64_t(numPixels) * sizeof(uint32_t)){
		pt->virt_indexImage->commit(numPixels * sizeof(uint32_t));
		pt->cptr_indexImage = pt->virt_indexImage->cptr;
	}

	// Optional reset (from GUI "Reset progressive"): clear the reproject buffer
	// so the next frame rebuilds the image from scratch.
	if(settings.progressiveResetRequested){
		cuMemsetD32Async(pt->cptr_indirect, 0, sizeof(ProgressiveIndirectCommand) / 4, mainstream);
		pt->hostIndirect->count = 0;
		settings.progressiveResetRequested = false;
	}

	// Stage 0: clear the per-pixel index image (0xFFFFFFFF = empty).
	prog->launch("kernel_progressive_clear_index",
		{&editor->launchArgs, &target, &pt->cptr_indexImage}, numPixels, mainstream);

	// Read last frame's indirect count back to the host so we launch reproject
	// over exactly that many threads. This is a tiny (20-byte) async copy; we
	// synchronize on it before launching.
	cuMemcpyDtoHAsync(pt->hostIndirect, pt->cptr_indirect, sizeof(ProgressiveIndirectCommand), mainstream);
	cuStreamSynchronize(mainstream);
	uint32_t reprojectCount = pt->hostIndirect->count;
	if(reprojectCount > reprojectCapacity) reprojectCount = reprojectCapacity;

	// Stage 1: reproject last frame's visible points.
	if(reprojectCount > 0){
		prog->launch("kernel_progressive_reproject",
			{&editor->launchArgs, &target, &pt->cptr_indexImage,
			 &pt->cptr_reprojectBuffer, &pt->cptr_indirect, &transform},
			reprojectCount, mainstream);
	}

	// Stage 2: fill a budget of fresh shuffled points into holes.
	// (Phase 1 uses a fixed budget from the GUI slider. The adaptive-budget
	// kernel kernel_progressive_compute_fill — port of compute_fill.cs — is wired
	// in Phase 2 using evFillStart/evFillEnd to measure reproject+fill time and
	// self-regulate against progressiveTargetFrameMs.)
	uint32_t budget = settings.progressiveBudget;
	budget = std::min(budget, pc.count);

	prog->launch("kernel_progressive_fill",
		{&editor->launchArgs, &target, &pt->cptr_indexImage, &pc,
		 &pc.fillOffset, &budget, &transform},
		budget, mainstream);

	// Advance the fill cursor (wraps around the cloud). The kernel reads
	// pc.fillOffset by value, so we update the host copy and will re-upload it
	// via the by-value struct next frame. (pc is a host copy of the node's
	// ProgressivePointData; the node's own fillOffset is advanced in update().)
	node->progressive.data.fillOffset =
		(node->progressive.data.fillOffset + budget) % pc.count;

	// Stage 3: rebuild next frame's reproject buffer from the index image.
	// Reset the indirect count first so create_vbo's atomicAdd compaction starts
	// from zero.
	prog->launch("kernel_progressive_reset_indirect",
		{&editor->launchArgs, &pt->cptr_indirect}, 1, mainstream);

	// One thread per framebuffer pixel. CudaModularProgram only exposes 1D grids,
	// so flatten width*height into a 1D launch; the kernel recovers (gx,gy) from
	// the flat thread rank. create_vbo stores LOCAL positions (no transform) —
	// reproject applies the node's current world transform next frame.
	{
		uint32_t numPixelsCreate = uint32_t(target.width) * uint32_t(target.height);
		uint32_t blocksize = 256;
		uint32_t gridsize = (numPixelsCreate + blocksize - 1) / blocksize;
		prog->launch("kernel_progressive_create_vbo",
			{&editor->launchArgs, &target, &pt->cptr_indexImage,
			 &pt->cptr_indirect, &pt->cptr_reprojectBuffer, &pc},
			{.gridsize = gridsize, .blocksize = blocksize, .stream = mainstream});
	}
}

void dump_tile(ConcurrentTarget& target, uint32_t numTiles){

	auto editor = SplatEditor::instance;

	int width     = target.target.width;
	int height    = target.target.height;
	int numPixels = width * height;
	int tiles_x   = int(width + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
	int tiles_y   = int(height + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);

	vector<Tile> tiles(numTiles);
	cuMemcpyDtoH(tiles.data(), target.cptr_tiles, sizeof(Tile) * numTiles);

	stringstream ss;
	ss << format("numTiles: {}\n", numTiles);

	// Tile largestTile;
	// int largestTileID = 0;
	// int splatsInLargest = 0;
	// for(int tileID = 0; tileID < numTiles; tileID++){
	// 	int numSplats = tiles[tileID].lastIndex - tiles[tileID].firstIndex;
	// 	if(numSplats > splatsInLargest){
	// 		largestTileID = tileID;
	// 		splatsInLargest = numSplats;
	// 		largestTile = tiles[tileID];
	// 	}
	// }

	

	// ss << format("largest tile: {}, splats: {} \n", largestTileID, splatsInLargest);

	auto dumpTile = [&](string label, int tile_x, int tile_y){
		Tile debugTile = tiles[tile_x + tile_y * tiles_x];

		int numSplats = debugTile.lastIndex - debugTile.firstIndex + 1;
		vector<StageData> stagedatas(target.virt_stagedata->comitted / sizeof(StageData));
		vector<uint32_t> splatIndices(target.virt_indices->comitted / 4);
		vector<float> depths(target.virt_depth->comitted / 4);
		vector<uint32_t> ordering(target.virt_ordering_splatdepth->comitted / 4);

		cuMemcpyDtoH(stagedatas.data(), target.virt_stagedata->cptr, target.virt_stagedata->comitted);
		cuMemcpyDtoH(splatIndices.data(), target.virt_indices->cptr, target.virt_indices->comitted);
		cuMemcpyDtoH(depths.data(), target.virt_depth->cptr, target.virt_depth->comitted);
		cuMemcpyDtoH(ordering.data(), target.virt_ordering_splatdepth->cptr, target.virt_ordering_splatdepth->comitted);

		stringstream ssPointcloud;

		auto decode_basisvector_i16vec2 = [](glm::i16vec2 encoded) -> vec2 {
			constexpr float basisvector_encoding_factor = 20.0f;

			float length = float(encoded.y) / basisvector_encoding_factor;
			float angle = float(encoded.x) / 10'000.0f;

			float x = cos(angle);
			float y = sin(angle);

			return vec2{x, y} * length;
		};

		for(int i = 0; i < numSplats; i++){
			int splatIndex = splatIndices[debugTile.firstIndex + i];
			StageData stagedata = stagedatas[splatIndex];
			// float depth = depths[ordering[splatIndex]];
			// float depth = stagedata.depth;
			float depth = 16.0f * float(i) / float(numSplats);

			vec2 imgpos = vec2(stagedata.imgPos_encoded) / 10.0f;
			uint8_t* rgba = (uint8_t*)&stagedata.color;

			float x = imgpos.x;
			float y = imgpos.y;
			float z = depth;

			vec2 basisvector1 = decode_basisvector_i16vec2(stagedata.basisvector1_encoded);
			vec2 basisvector2 = decode_basisvector_i16vec2(stagedata.basisvector2_encoded);

			ssPointcloud << format("{:.3f}, {:.3f}, {:.3f}, {}, {}, {}, {}, {}, {}, {}, {}\n", 
				x, y, z, 
				rgba[0], rgba[1], rgba[2], rgba[3],
				basisvector1.x, basisvector1.y,
				basisvector2.x, basisvector2.y
				);
		}

		string filename = format("dump_{}_tile_{}_{}.csv", label, tile_x, tile_y);
		writeFile(filename.c_str(), ssPointcloud.str());
	};

	for(int tx = -1; tx <= 1; tx++)
	for(int ty = -1; ty <= 1; ty++)
	{
		dumpTile("garden", 60 + tx, 50 + ty);
	}
	


	for(int tileID = 0; tileID < numTiles; tileID++){
		Tile tile = tiles[tileID];
		int tile_x = tileID % tiles_x;
		int tile_y = tileID / tiles_x;

		ss << format("Tile {:3} / {:3}, first: {:8}, last: {:8}, count: {:6}\n", 
			tile_x, tile_y,
			tile.firstIndex, tile.lastIndex, tile.lastIndex - tile.firstIndex
		);
	}

	writeFile("./dump.txt", ss.str());


	editor->settings.requestDebugDump = false;
}


void drawsplats_perspectiveCorrect_concurrent(
	Scene* scene, 
	vector<ConcurrentTarget> targets
){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& state = editor->state;

	static CUevent event_start = 0;
	static CUevent event_end = 0;
	static double t_start;
	static bool initialized = false;
	if(!initialized){
		cuEventCreate(&event_start, CU_EVENT_DEFAULT);
		cuEventCreate(&event_end, CU_EVENT_DEFAULT);
		initialized = true;
	}

	if(Runtime::measureTimings){
		cuCtxSynchronize();
		t_start = now();
		cuEventRecord(event_start, targets[0].mainstream);
	}

	// staging of splats needs a lot of memory, 
	// so we still do this part sequentially, one target after the other
	for(auto& target : targets){

		cuMemsetD32Async(target.cptr_numVisibleSplats  , 0, 1, target.mainstream);
		cuMemsetD32Async(target.cptr_numFragments      , 0, 1, target.mainstream);

		// hm, can we know how much we need before we need it? 
		uint32_t numPotentiallyVisibleSplats = 0;
		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			numPotentiallyVisibleSplats += node->dmng.data.count;
		});
		target.virt_stagedata->commit(sizeof(StageData_perspectivecorrect) * numPotentiallyVisibleSplats);
		target.virt_bounds->commit(sizeof(glm::i16vec4) * numPotentiallyVisibleSplats);
		target.virt_ordering_splatdepth->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);
		target.virt_numTilefragments_splatwise->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);

		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			if(node->dmng.data.count == 0) return;

			node->dmng.data.transform = node->transform_global;

			mat4 world = scene->world->transform;
			GaussianData data = node->dmng.data;

			ColorCorrection colorCorrection;
			if(node->selected){
				colorCorrection = settings.colorCorrection;
			}
			
			void* args[] = {
				// in
				&editor->launchArgs,
				&target, 
				&colorCorrection, 
				&node->dmng.data, 
				// out
				&target.cptr_numVisibleSplats,
				&target.cptr_numFragments, 
				&target.virt_numTilefragments_splatwise->cptr,
				&target.virt_bounds->cptr,
				&target.virt_stagedata->cptr,
				&target.virt_ordering_splatdepth->cptr,
			};
			editor->prog_gaussians_rendering->launch("kernel_stageSplats_perspectivecorrect", args, data.count, target.mainstream);
		});

		

	}
	// cuCtxSynchronize();

	// Retrieve number of staged/visible splats and tile-fragments
	int i = 0;
	for(auto& target : targets){

		// cuMemcpyDtoHAsync target must be page-locked.
		// Apparently local variables count as page-locked memory?
		uint32_t numVisibleSplats;
		uint32_t numFragments;
		CURuntime::check(cuMemcpyDtoHAsync(&numVisibleSplats, target.cptr_numVisibleSplats, 4, target.mainstream));
		CURuntime::check(cuMemcpyDtoHAsync(&numFragments, target.cptr_numFragments, 4, target.mainstream));

		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);

		Runtime::numVisibleSplats += numVisibleSplats;
		Runtime::numVisibleFragments += numFragments;

		target.numFragments = numFragments;
		target.numVisibleSplats = numVisibleSplats;

		{
			string label = "numSplats";
			string value = format(getSaneLocale(), "{:L}", numVisibleSplats);
			Runtime::debugValueList.push_back({label, value});
		}
		{
			string label = "numFragments";
			string value = format(getSaneLocale(), "{:L}", numFragments);
			Runtime::debugValueList.push_back({label, value});
		}
		
	}

	// Create tile fragment arrays concurrently
	for(auto target : targets){

		// Compute prefix sum of tile fragment counters.
		GPUPrefixSumsCS::dispatch(target.numVisibleSplats, target.virt_numTilefragments_splatwise->cptr, target.mainstream);

		target.virt_tileIDs->commit(4 * target.numFragments);
		target.virt_indices->commit(4 * target.numFragments);

		// now create the tile fragment (StageData) array
		void* argsCreatefragmentArray[] = {
			// input
			&editor->launchArgs, &target,
			&target.numVisibleSplats, &target.virt_bounds->cptr,
			&target.virt_numTilefragments_splatwise->cptr,& target.cptr_numFragments,
			// output
			&target.virt_tileIDs->cptr, &target.virt_indices->cptr,
		};

		// Lot's of syncs - without them the 16bit sorting crashed upon loadig large splat models
		cuCtxSynchronize();
		editor->prog_gaussians_rendering->launch("kernel_createTilefragmentArray_perspectivecorrect", argsCreatefragmentArray, target.numVisibleSplats, target.mainstream);
		cuCtxSynchronize();
		// TODO: shouldn't tileIDs be actual 16bit instead of 32bit uints where only the lower 16 bits are read?
		GPUSorting::sort_16bitkey_32bitvalue(target.numFragments, target.virt_tileIDs->cptr, target.virt_indices->cptr, target.mainstream);
	}

	for(auto target : targets){

		int width   = target.target.width;
		int height  = target.target.height;
		int tiles_x = (width + TILE_SIZE_PERSPCORRECT - 1) / TILE_SIZE_PERSPCORRECT;
		int tiles_y = (height + TILE_SIZE_PERSPCORRECT - 1) / TILE_SIZE_PERSPCORRECT;
		uint32_t numTiles = tiles_x * tiles_y;
		constexpr uint32_t blockSize = TILE_SIZE_PERSPCORRECT * TILE_SIZE_PERSPCORRECT;
		int tileSize = TILE_SIZE_PERSPCORRECT;

		CURuntime::check(cuMemsetD8Async(target.cptr_tiles, 0, sizeof(Tile) * numTiles, target.mainstream));
		editor->prog_gaussians_rendering->launch(
			"kernel_computeTiles_method1", 
			{&editor->launchArgs, &target, &target.virt_tileIDs->cptr, &target.numFragments, &tileSize, &target.cptr_tiles}, 
			target.numFragments, target.mainstream
		);

		cuEventRecord(target.cu_tilesComputed, target.mainstream);

		// both streams use the computed tiles, so make sure the sidestream also waits until the mainstream computed the tiles.
		cuStreamWaitEvent(target.mainstream, target.cu_tilesComputed, 0);
		cuStreamWaitEvent(target.sidestream, target.cu_tilesComputed, 0);

		if(target.numFragments > 0){

			void* args_rendering[] = {
				&editor->launchArgs, &target.target, &target.cptr_tiles, &target.virt_indices->cptr, 
				&target.virt_stagedata->cptr,
			};

			editor->prog_gaussians_rendering->launch("kernel_render_gaussians_perspectivecorrect", args_rendering, {.gridsize = numTiles, .blocksize = blockSize, .stream = target.sidestream});

		}
	}

	// wait for all streams
	for(auto target : targets){
		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);
	}
	
	if(Runtime::measureTimings){
		cuCtxSynchronize();

		double seconds = now() - t_start;

		// float duration;
		// cuEventElapsedTime(&duration, event_start, event_end);

		Runtime::timings.add("[draw splats (host)]", seconds * 1000.0f);
	}
	
}

void drawsplats_3dgs_concurrent(
	Scene* scene, 
	vector<ConcurrentTarget> targets
){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& state = editor->state;

	static CUevent event_start = 0;
	static CUevent event_end = 0;
	static double t_start;
	static bool initialized = false;
	if(!initialized){
		cuEventCreate(&event_start, CU_EVENT_DEFAULT);
		cuEventCreate(&event_end, CU_EVENT_DEFAULT);
		initialized = true;
	}

	if(Runtime::measureTimings){
		cuCtxSynchronize();
		t_start = now();
		cuEventRecord(event_start, targets[0].mainstream);
	}

	// staging of splats needs a lot of memory, 
	// so we still do this part sequentially, one target after the other
	for(auto& target : targets){

		cuMemsetD32Async(target.cptr_numVisibleSplats  , 0, 1, target.mainstream);
		cuMemsetD32Async(target.cptr_numFragments      , 0, 1, target.mainstream);

		// hm, can we know how much we need before we need it? 
		uint32_t numPotentiallyVisibleSplats = 0;
		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			numPotentiallyVisibleSplats += node->dmng.data.count;
		});
		target.virt_stagedata->commit(sizeof(StageData) * numPotentiallyVisibleSplats);
		target.virt_depth->commit(sizeof(float) * numPotentiallyVisibleSplats);
		target.virt_ordering_splatdepth->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);
		target.virt_numTilefragments_splatwise->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);

		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			if(node->dmng.data.count == 0) return;

			node->dmng.data.transform = node->transform_global;

			mat4 world = scene->world->transform;
			GaussianData data = node->dmng.data;

			ColorCorrection colorCorrection;
			if(node->selected){
				colorCorrection = settings.colorCorrection;
			}
			
			void* args[] = {
				// in
				&editor->launchArgs,
				&target, 
				&colorCorrection, 
				&node->dmng.data, 
				// out
				&target.cptr_numVisibleSplats,
				&target.cptr_numFragments, 
				&target.virt_numTilefragments_splatwise->cptr,
				&target.virt_depth->cptr,
				&target.virt_stagedata->cptr,
				&target.virt_ordering_splatdepth->cptr,
			};
			editor->prog_gaussians_rendering->launch("kernel_stageSplats", args, data.count, target.mainstream);
		});

		

	}
	// cuCtxSynchronize();

	// Retrieve number of staged/visible splats and tile-fragments
	int i = 0;
	for(auto& target : targets){

		// cuMemcpyDtoHAsync target must be page-locked.
		// Apparently local variables count as page-locked memory?
		uint32_t numVisibleSplats;
		uint32_t numFragments;
		CURuntime::check(cuMemcpyDtoHAsync(&numVisibleSplats, target.cptr_numVisibleSplats, 4, target.mainstream));
		CURuntime::check(cuMemcpyDtoHAsync(&numFragments, target.cptr_numFragments, 4, target.mainstream));

		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);

		Runtime::numVisibleSplats += numVisibleSplats;
		Runtime::numVisibleFragments += numFragments;

		target.numFragments = numFragments;
		target.numVisibleSplats = numVisibleSplats;

		{
			string label = "numSplats";
			string value = format(getSaneLocale(), "{:L}", numVisibleSplats);
			Runtime::debugValueList.push_back({label, value});
		}
		{
			string label = "numFragments";
			string value = format(getSaneLocale(), "{:L}", numFragments);
			Runtime::debugValueList.push_back({label, value});
		}
		
	}

	for(auto target : targets){

		target.virt_tileIDs->commit(4 * target.numFragments);
		target.virt_numTilefragments_splatwise_ordered->commit(4 * target.numVisibleSplats);
		target.virt_indices->commit(4 * target.numFragments);

		// Unfortunately we can't sort simultaneously, since both use the same alternative sort buffers
		cuCtxSynchronize();

		// sort visible splats by depth (or rather, compute the order without applying it).
		// We have to provide radix-sort with intermediate memory for sorting, which must be separate for each concurrent target.
		GPUSorting::sort_32bit_keyvalue(target.numVisibleSplats, target.virt_depth->cptr, target.virt_ordering_splatdepth->cptr, 0, 0,target.mainstream);

		// Apply the ordering. Necessary because we don't have 64 bit sorting and do it in a 32bit sort, followed by another 16 bit sort.
		// The follow-up 16 bit sort needs its keys to be sorted by the preceeding 32 bit sort.
		void* argsApply[] = {
			&target.virt_numTilefragments_splatwise->cptr, 
			&target.virt_numTilefragments_splatwise_ordered->cptr, 
			&target.virt_ordering_splatdepth->cptr, 
			&target.numVisibleSplats
		};
		editor->prog_gaussians_rendering->launch("kernel_applyOrdering_u32", argsApply, target.numVisibleSplats, target.mainstream);

		// Compute prefix sum of tile fragment counters.
		GPUPrefixSumsCS::dispatch(target.numVisibleSplats, target.virt_numTilefragments_splatwise_ordered->cptr, target.mainstream);
	}

	// Create tile fragment arrays concurrently
	for(auto target : targets){
		// The prefix sum is stored in-place in cptr_numTilefragments_ordered
		CUdeviceptr cptr_prefixsum = target.virt_numTilefragments_splatwise_ordered->cptr;

		// now create the tile fragment (StageData) array
		void* argsCreatefragmentArray[] = {
			// input
			&editor->launchArgs, &target,
			&target.virt_ordering_splatdepth->cptr, &target.numVisibleSplats, &target.virt_stagedata->cptr, 
			&cptr_prefixsum,& target.cptr_numFragments,
			// output
			&target.virt_tileIDs->cptr, &target.virt_indices->cptr,
		};

		editor->prog_gaussians_rendering->launch("kernel_createTilefragmentArray", argsCreatefragmentArray, target.numVisibleSplats, target.mainstream);
	}

	// But sort one target after the other because both use the same alternative sort buffers
	for(auto target : targets){
		cuCtxSynchronize();
		// The prefix sum is stored in-place in cptr_numTilefragments_ordered
		CUdeviceptr cptr_prefixsum = target.virt_numTilefragments_splatwise_ordered->cptr;
		GPUSorting::sort_16bitkey_32bitvalue(target.numFragments, target.virt_tileIDs->cptr, target.virt_indices->cptr, target.mainstream);
	}

	for(auto target : targets){

		int width   = target.target.width;
		int height  = target.target.height;
		int tiles_x = int(width + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
		int tiles_y = int(height + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
		uint32_t numTiles = tiles_x * tiles_y;
		int tileSize = TILE_SIZE_3DGS;

		cuMemsetD8Async(target.cptr_tiles, 0, sizeof(Tile) * numTiles, target.mainstream);
		editor->prog_gaussians_rendering->launch(
			"kernel_computeTiles_method1", 
			{&editor->launchArgs, &target, &target.virt_tileIDs->cptr, &target.numFragments, &tileSize, &target.cptr_tiles}, 
			target.numFragments, target.mainstream
		);

		cuEventRecord(target.cu_tilesComputed, target.mainstream);

		// both streams use the computed tiles, so make sure the sidestream also waits until the mainstream computed the tiles.
		cuStreamWaitEvent(target.mainstream, target.cu_tilesComputed, 0);
		cuStreamWaitEvent(target.sidestream, target.cu_tilesComputed, 0);

		if(editor->settings.requestDebugDump){ 
			dump_tile(target, numTiles);
		}

		if(target.numFragments > 0){

			uint32_t pointsInTileThreshold = 0;

			void* args_rendering[] = {
				&editor->launchArgs, &target.target, &target.cptr_tiles, &target.virt_indices->cptr, 
				&target.virt_stagedata->cptr, &pointsInTileThreshold
			};

			if(settings.rendermode == RENDERMODE_HEATMAP){
				pointsInTileThreshold = 1'000'000;
				editor->prog_gaussians_rendering->launch("kernel_render_heatmap", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}else if(settings.showSolid){
				pointsInTileThreshold = 1'000'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians_solid", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}else if(settings.enableSplatCulling){
				pointsInTileThreshold = 10'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians_with_discard", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.mainstream});
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}else{
				pointsInTileThreshold = 1'000'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}
		}
	}

	// wait for all streams
	for(auto target : targets){
		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);
	}
	
	if(Runtime::measureTimings){
		cuEventRecord(event_end, 0);

		cuCtxSynchronize();

		double seconds = now() - t_start;

		float duration;
		cuEventElapsedTime(&duration, event_start, event_end);

		Runtime::timings.add("[draw splats (host)]", seconds * 1000.0f);
		Runtime::timings.add("[draw splats (device)]", duration);
	}
	
}

void drawsplats_3dgs_concurrent_bandwidth(
	Scene* scene, 
	vector<ConcurrentTarget> targets
){
	// Memory bandwidth performance study with simplified code paths, e.g. single target, no solid mode, etc.

	auto target = targets[0];

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& state = editor->state;

	static CUevent event_start = 0;
	static CUevent event_end = 0;
	static double t_start;
	static bool initialized = false;
	static CudaModularProgram* prog_render_bandwidth = nullptr;
	static shared_ptr<CudaVirtualMemory> virt_fluff_in = CURuntime::allocVirtual("bandwidth fluff in");
	static shared_ptr<CudaVirtualMemory> virt_fluff_out = CURuntime::allocVirtual("bandwidth fluff out");

	if(!initialized){
		cuEventCreate(&event_start, CU_EVENT_DEFAULT);
		cuEventCreate(&event_end, CU_EVENT_DEFAULT);
		prog_render_bandwidth = new CudaModularProgram({"./src/gaussians_rendering_bandwidth.cu"});
		
		// up to 50 floats for 30M splats
		virt_fluff_in->commit(30'000'000ll * 50ll * 4ll);
		virt_fluff_out->commit(30'000'000ll * 50ll * 4ll);
		cuMemsetD8(virt_fluff_in->cptr, 0, virt_fluff_in->comitted);

		initialized = true;
	}

	if(Runtime::measureTimings){
		cuCtxSynchronize();
		t_start = now();
		cuEventRecord(event_start, targets[0].mainstream);
	}

	// staging of splats needs a lot of memory, 
	// so we still do this part sequentially, one target after the other
	{

		cuMemsetD32Async(target.cptr_numVisibleSplats  , 0, 1, target.mainstream);
		cuMemsetD32Async(target.cptr_numFragments      , 0, 1, target.mainstream);

		// hm, can we know how much we need before we need it? 
		uint32_t numPotentiallyVisibleSplats = 0;
		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			numPotentiallyVisibleSplats += node->dmng.data.count;
		});
		target.virt_stagedata->commit(sizeof(StageData) * numPotentiallyVisibleSplats);
		target.virt_depth->commit(sizeof(float) * numPotentiallyVisibleSplats);
		target.virt_ordering_splatdepth->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);
		target.virt_numTilefragments_splatwise->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);

		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			if(node->dmng.data.count == 0) return;

			node->dmng.data.transform = node->transform_global;

			mat4 world = scene->world->transform;
			GaussianData data = node->dmng.data;

			ColorCorrection colorCorrection;
			if(node->selected){
				colorCorrection = settings.colorCorrection;
			}
			
			void* args[] = {
				// in
				&editor->launchArgs,
				&target, 
				&colorCorrection, 
				&node->dmng.data, 
				&virt_fluff_in->cptr,

				// out
				&target.cptr_numVisibleSplats,
				&target.cptr_numFragments, 
				&target.virt_numTilefragments_splatwise->cptr,
				&target.virt_depth->cptr,
				&target.virt_stagedata->cptr,
				&target.virt_ordering_splatdepth->cptr,
				&virt_fluff_out->cptr,
			};
			prog_render_bandwidth->launch("kernel_stageSplats", args, data.count, target.mainstream);
		});

		

	}
	// cuCtxSynchronize();

	// Retrieve number of staged/visible splats and tile-fragments
	int i = 0;
	for(auto& target : targets){

		// cuMemcpyDtoHAsync target must be page-locked.
		// Apparently local variables count as page-locked memory?
		uint32_t numVisibleSplats;
		uint32_t numFragments;
		CURuntime::check(cuMemcpyDtoHAsync(&numVisibleSplats, target.cptr_numVisibleSplats, 4, target.mainstream));
		CURuntime::check(cuMemcpyDtoHAsync(&numFragments, target.cptr_numFragments, 4, target.mainstream));

		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);

		Runtime::numVisibleSplats += numVisibleSplats;
		Runtime::numVisibleFragments += numFragments;

		target.numFragments = numFragments;
		target.numVisibleSplats = numVisibleSplats;

		{
			string label = "numSplats";
			string value = format(getSaneLocale(), "{:L}", numVisibleSplats);
			Runtime::debugValueList.push_back({label, value});
		}
		{
			string label = "numFragments";
			string value = format(getSaneLocale(), "{:L}", numFragments);
			Runtime::debugValueList.push_back({label, value});
		}
		
	}

	for(auto target : targets){

		target.virt_tileIDs->commit(4 * target.numFragments);
		target.virt_numTilefragments_splatwise_ordered->commit(4 * target.numFragments);
		target.virt_indices->commit(4 * target.numFragments);

		// Unfortunately we can't sort simultaneously, since both use the same alternative sort buffers
		cuCtxSynchronize();

		// sort visible splats by depth (or rather, compute the order without applying it).
		// We have to provide radix-sort with intermediate memory for sorting, which must be separate for each concurrent target.
		GPUSorting::sort_32bit_keyvalue(target.numVisibleSplats, target.virt_depth->cptr, target.virt_ordering_splatdepth->cptr, 0, 0,target.mainstream);

		// Apply the ordering. Necessary because we don't have 64 bit sorting and do it in a 32bit sort, followed by another 16 bit sort.
		// The follow-up 16 bit sort needs its keys to be sorted by the preceeding 32 bit sort.
		void* argsApply[] = {
			&target.virt_numTilefragments_splatwise->cptr, 
			&target.virt_numTilefragments_splatwise_ordered->cptr, 
			&target.virt_ordering_splatdepth->cptr, 
			&target.numVisibleSplats
		};
		editor->prog_gaussians_rendering->launch("kernel_applyOrdering_u32", argsApply, target.numVisibleSplats, target.mainstream);

		// Compute prefix sum of tile fragment counters.
		GPUPrefixSumsCS::dispatch(target.numVisibleSplats, target.virt_numTilefragments_splatwise_ordered->cptr, target.mainstream);
	}

	// Create tile fragment arrays concurrently
	for(auto target : targets){
		// The prefix sum is stored in-place in cptr_numTilefragments_ordered
		CUdeviceptr cptr_prefixsum = target.virt_numTilefragments_splatwise_ordered->cptr;

		// now create the tile fragment (StageData) array
		void* argsCreatefragmentArray[] = {
			// input
			&editor->launchArgs, &target,
			&target.virt_ordering_splatdepth->cptr, &target.numVisibleSplats, &target.virt_stagedata->cptr, 
			&cptr_prefixsum,& target.cptr_numFragments,
			// output
			&target.virt_tileIDs->cptr, &target.virt_indices->cptr,
		};

		editor->prog_gaussians_rendering->launch("kernel_createTilefragmentArray", argsCreatefragmentArray, target.numVisibleSplats, target.mainstream);
	}

	// But sort one target after the other because both use the same alternative sort buffers
	for(auto target : targets){
		cuCtxSynchronize();
		// The prefix sum is stored in-place in cptr_numTilefragments_ordered
		CUdeviceptr cptr_prefixsum = target.virt_numTilefragments_splatwise_ordered->cptr;
		GPUSorting::sort_16bitkey_32bitvalue(target.numFragments, target.virt_tileIDs->cptr, target.virt_indices->cptr, target.mainstream);
	}

	for(auto target : targets){

		int width   = target.target.width;
		int height  = target.target.height;
		int tiles_x = int(width + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
		int tiles_y = int(height + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
		uint32_t numTiles = tiles_x * tiles_y;
		int tileSize = TILE_SIZE_3DGS;

		cuMemsetD8Async(target.cptr_tiles, 0, sizeof(Tile) * numTiles, target.mainstream);
		editor->prog_gaussians_rendering->launch(
			"kernel_computeTiles_method1", 
			{&editor->launchArgs, &target, &target.virt_tileIDs->cptr, &target.numFragments, &tileSize, &target.cptr_tiles}, 
			target.numFragments, target.mainstream
		);

		cuEventRecord(target.cu_tilesComputed, target.mainstream);

		// both streams use the computed tiles, so make sure the sidestream also waits until the mainstream computed the tiles.
		cuStreamWaitEvent(target.mainstream, target.cu_tilesComputed, 0);
		cuStreamWaitEvent(target.sidestream, target.cu_tilesComputed, 0);

		if(editor->settings.requestDebugDump){ 
			dump_tile(target, numTiles);
		}

		if(target.numFragments > 0){

			uint32_t pointsInTileThreshold = 0;

			void* args_rendering[] = {
				&editor->launchArgs, &target.target, &target.cptr_tiles, &target.virt_indices->cptr, 
				&target.virt_stagedata->cptr, &pointsInTileThreshold
			};

			if(settings.showSolid){
				pointsInTileThreshold = 1'000'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians_solid", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}else if(settings.enableSplatCulling){
				pointsInTileThreshold = 10'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians_with_discard", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.mainstream});
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}else{
				pointsInTileThreshold = 1'000'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}
		}
	}

	// wait for all streams
	for(auto target : targets){
		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);
	}
	
	if(Runtime::measureTimings){
		cuEventRecord(event_end, 0);

		cuCtxSynchronize();

		double seconds = now() - t_start;

		float duration;
		cuEventElapsedTime(&duration, event_start, event_end);

		Runtime::timings.add("[draw splats (host)]", seconds * 1000.0f);
		Runtime::timings.add("[draw splats (device)]", duration);
	}
	
}

void drawsplats_3dgs_concurrent_fragintersections(
	Scene* scene, 
	vector<ConcurrentTarget> targets
){
	// Fragment intersection study with simplified code paths, e.g. single target, no solid mode, etc.

	auto target = targets[0];

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& state = editor->state;

	static CUevent event_start = 0;
	static CUevent event_end = 0;
	static double t_start;
	static bool initialized = false;
	static CudaModularProgram* prog_render_fragintersections = nullptr;

	if(!initialized){
		cuEventCreate(&event_start, CU_EVENT_DEFAULT);
		cuEventCreate(&event_end, CU_EVENT_DEFAULT);
		prog_render_fragintersections = new CudaModularProgram({"./src/gaussians_rendering_fragintersection_3dgs.cu"});

		initialized = true;
	}

	if(Runtime::measureTimings){
		cuCtxSynchronize();
		t_start = now();
		cuEventRecord(event_start, targets[0].mainstream);
	}

	// staging of splats needs a lot of memory, 
	// so we still do this part sequentially, one target after the other
	{

		cuMemsetD32Async(target.cptr_numVisibleSplats  , 0, 1, target.mainstream);
		cuMemsetD32Async(target.cptr_numFragments      , 0, 1, target.mainstream);

		// hm, can we know how much we need before we need it? 
		uint32_t numPotentiallyVisibleSplats = 0;
		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			numPotentiallyVisibleSplats += node->dmng.data.count;
		});
		target.virt_stagedata->commit(sizeof(StageData) * numPotentiallyVisibleSplats);
		target.virt_depth->commit(sizeof(float) * numPotentiallyVisibleSplats);
		target.virt_ordering_splatdepth->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);
		target.virt_numTilefragments_splatwise->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);

		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			if(node->dmng.data.count == 0) return;

			node->dmng.data.transform = node->transform_global;

			mat4 world = scene->world->transform;
			GaussianData data = node->dmng.data;

			ColorCorrection colorCorrection;
			if(node->selected){
				colorCorrection = settings.colorCorrection;
			}
			
			void* args[] = {
				// in
				&editor->launchArgs,
				&target, 
				&colorCorrection, 
				&node->dmng.data, 

				// out
				&target.cptr_numVisibleSplats,
				&target.cptr_numFragments, 
				&target.virt_numTilefragments_splatwise->cptr,
				&target.virt_depth->cptr,
				&target.virt_stagedata->cptr,
				&target.virt_ordering_splatdepth->cptr,
			};
			prog_render_fragintersections->launch("kernel_stageSplats", args, data.count, target.mainstream);
		});

		

	}
	// cuCtxSynchronize();

	// Retrieve number of staged/visible splats and tile-fragments
	int i = 0;
	for(auto& target : targets){

		// cuMemcpyDtoHAsync target must be page-locked.
		// Apparently local variables count as page-locked memory?
		uint32_t numVisibleSplats;
		uint32_t numFragments;
		CURuntime::check(cuMemcpyDtoHAsync(&numVisibleSplats, target.cptr_numVisibleSplats, 4, target.mainstream));
		CURuntime::check(cuMemcpyDtoHAsync(&numFragments, target.cptr_numFragments, 4, target.mainstream));

		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);

		Runtime::numVisibleSplats += numVisibleSplats;
		Runtime::numVisibleFragments += numFragments;

		target.numFragments = numFragments;
		target.numVisibleSplats = numVisibleSplats;

		{
			string label = "numSplats";
			string value = format(getSaneLocale(), "{:L}", numVisibleSplats);
			Runtime::debugValueList.push_back({label, value});
		}
		{
			string label = "numFragments";
			string value = format(getSaneLocale(), "{:L}", numFragments);
			Runtime::debugValueList.push_back({label, value});
		}
		
	}

	for(auto target : targets){

		target.virt_tileIDs->commit(4 * target.numFragments);
		target.virt_numTilefragments_splatwise_ordered->commit(4 * target.numFragments);
		target.virt_indices->commit(4 * target.numFragments);

		// Unfortunately we can't sort simultaneously, since both use the same alternative sort buffers
		cuCtxSynchronize();

		// sort visible splats by depth (or rather, compute the order without applying it).
		// We have to provide radix-sort with intermediate memory for sorting, which must be separate for each concurrent target.
		GPUSorting::sort_32bit_keyvalue(target.numVisibleSplats, target.virt_depth->cptr, target.virt_ordering_splatdepth->cptr, 0, 0,target.mainstream);

		// Apply the ordering. Necessary because we don't have 64 bit sorting and do it in a 32bit sort, followed by another 16 bit sort.
		// The follow-up 16 bit sort needs its keys to be sorted by the preceeding 32 bit sort.
		void* argsApply[] = {
			&target.virt_numTilefragments_splatwise->cptr, 
			&target.virt_numTilefragments_splatwise_ordered->cptr, 
			&target.virt_ordering_splatdepth->cptr, 
			&target.numVisibleSplats
		};
		editor->prog_gaussians_rendering->launch("kernel_applyOrdering_u32", argsApply, target.numVisibleSplats, target.mainstream);

		// Compute prefix sum of tile fragment counters.
		GPUPrefixSumsCS::dispatch(target.numVisibleSplats, target.virt_numTilefragments_splatwise_ordered->cptr, target.mainstream);
	}

	// Create tile fragment arrays concurrently
	for(auto target : targets){
		// The prefix sum is stored in-place in cptr_numTilefragments_ordered
		CUdeviceptr cptr_prefixsum = target.virt_numTilefragments_splatwise_ordered->cptr;

		// now create the tile fragment (StageData) array
		void* argsCreatefragmentArray[] = {
			// input
			&editor->launchArgs, &target,
			&target.virt_ordering_splatdepth->cptr, &target.numVisibleSplats, &target.virt_stagedata->cptr, 
			&cptr_prefixsum,& target.cptr_numFragments,
			// output
			&target.virt_tileIDs->cptr, &target.virt_indices->cptr,
		};

		prog_render_fragintersections->launch("kernel_createTilefragmentArray", argsCreatefragmentArray, target.numVisibleSplats, target.mainstream);
	}

	// But sort one target after the other because both use the same alternative sort buffers
	for(auto target : targets){
		cuCtxSynchronize();
		// The prefix sum is stored in-place in cptr_numTilefragments_ordered
		CUdeviceptr cptr_prefixsum = target.virt_numTilefragments_splatwise_ordered->cptr;
		GPUSorting::sort_16bitkey_32bitvalue(target.numFragments, target.virt_tileIDs->cptr, target.virt_indices->cptr, target.mainstream);
	}

	for(auto target : targets){

		int width   = target.target.width;
		int height  = target.target.height;
		int tiles_x = int(width + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
		int tiles_y = int(height + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
		uint32_t numTiles = tiles_x * tiles_y;
		int tileSize = TILE_SIZE_3DGS;

		cuMemsetD8Async(target.cptr_tiles, 0, sizeof(Tile) * numTiles, target.mainstream);
		editor->prog_gaussians_rendering->launch(
			"kernel_computeTiles_method1", 
			{&editor->launchArgs, &target, &target.virt_tileIDs->cptr, &target.numFragments, &tileSize, &target.cptr_tiles}, 
			target.numFragments, target.mainstream
		);

		cuEventRecord(target.cu_tilesComputed, target.mainstream);

		// both streams use the computed tiles, so make sure the sidestream also waits until the mainstream computed the tiles.
		cuStreamWaitEvent(target.mainstream, target.cu_tilesComputed, 0);
		cuStreamWaitEvent(target.sidestream, target.cu_tilesComputed, 0);

		if(editor->settings.requestDebugDump){ 
			dump_tile(target, numTiles);
		}

		if(target.numFragments > 0){

			uint32_t pointsInTileThreshold = 0;

			void* args_rendering[] = {
				&editor->launchArgs, &target.target, &target.cptr_tiles, &target.virt_indices->cptr, 
				&target.virt_stagedata->cptr, &pointsInTileThreshold
			};

			if(settings.rendermode == RENDERMODE_HEATMAP){
				pointsInTileThreshold = 1'000'000;
				editor->prog_gaussians_rendering->launch("kernel_render_heatmap", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}else if(settings.showSolid){
				pointsInTileThreshold = 1'000'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians_solid", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}else if(settings.enableSplatCulling){
				pointsInTileThreshold = 10'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians_with_discard", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.mainstream});
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}else{
				pointsInTileThreshold = 1'000'000;
				editor->prog_gaussians_rendering->launch("kernel_render_gaussians", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
			}

			
		}
	}

	// wait for all streams
	for(auto target : targets){
		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);
	}
	
	if(Runtime::measureTimings){
		cuEventRecord(event_end, 0);

		cuCtxSynchronize();

		double seconds = now() - t_start;

		float duration;
		cuEventElapsedTime(&duration, event_start, event_end);

		Runtime::timings.add("[draw splats (host)]", seconds * 1000.0f);
		Runtime::timings.add("[draw splats (device)]", duration);
	}
	
}

void drawsplats_3dgs_concurrent_soa(
	Scene* scene, 
	vector<ConcurrentTarget> targets
){
	// Structure-of-Array performance study with simplified code paths, e.g. single target, no solid mode, etc.

	auto target = targets[0];

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& state = editor->state;

	static CUevent event_start = 0;
	static CUevent event_end = 0;
	static double t_start;
	static bool initialized = false;
	static CudaModularProgram* prog_render_soa = nullptr;
	static shared_ptr<CudaVirtualMemory> virt_sd_basisvector1_encoded = CURuntime::allocVirtual("sd_basisvector1_encoded");
	static shared_ptr<CudaVirtualMemory> virt_sd_basisvector2_encoded = CURuntime::allocVirtual("sd_basisvector2_encoded");
	static shared_ptr<CudaVirtualMemory> virt_sd_imgPos_encoded = CURuntime::allocVirtual("sd_imgPos_encoded");
	static shared_ptr<CudaVirtualMemory> virt_sd_color = CURuntime::allocVirtual("sd_color");
	static shared_ptr<CudaVirtualMemory> virt_sd_flags = CURuntime::allocVirtual("sd_flags");
	static shared_ptr<CudaVirtualMemory> virt_sd_depth = CURuntime::allocVirtual("sd_depth");

	if(!initialized){
		cuEventCreate(&event_start, CU_EVENT_DEFAULT);
		cuEventCreate(&event_end, CU_EVENT_DEFAULT);

		prog_render_soa = new CudaModularProgram({"./src/gaussians_rendering_soa.cu"});
		initialized = true;
	}

	if(Runtime::measureTimings){
		cuCtxSynchronize();
		t_start = now();
		cuEventRecord(event_start, targets[0].mainstream);
	}

	// staging of splats needs a lot of memory, 
	// so we still do this part sequentially, one target after the other
	{

		cuMemsetD32Async(target.cptr_numVisibleSplats  , 0, 1, target.mainstream);
		cuMemsetD32Async(target.cptr_numFragments      , 0, 1, target.mainstream);

		// hm, can we know how much we need before we need it? 
		uint32_t numPotentiallyVisibleSplats = 0;
		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			numPotentiallyVisibleSplats += node->dmng.data.count;
		});
		
		// target.virt_stagedata->commit(sizeof(StageData) * numPotentiallyVisibleSplats);
		virt_sd_basisvector1_encoded->commit(sizeof(glm::i16vec2) * numPotentiallyVisibleSplats);
		virt_sd_basisvector2_encoded->commit(sizeof(glm::i16vec2) * numPotentiallyVisibleSplats);
		virt_sd_imgPos_encoded->commit(sizeof(glm::i16vec2) * numPotentiallyVisibleSplats);
		virt_sd_color->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);
		virt_sd_flags->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);
		virt_sd_depth->commit(sizeof(float) * numPotentiallyVisibleSplats);

		target.virt_depth->commit(sizeof(float) * numPotentiallyVisibleSplats);
		target.virt_ordering_splatdepth->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);
		target.virt_numTilefragments_splatwise->commit(sizeof(uint32_t) * numPotentiallyVisibleSplats);

		scene->process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			if(node->dmng.data.count == 0) return;

			node->dmng.data.transform = node->transform_global;

			mat4 world = scene->world->transform;
			GaussianData data = node->dmng.data;

			ColorCorrection colorCorrection;
			if(node->selected){
				colorCorrection = settings.colorCorrection;
			}
			
			void* args[] = {
				// in
				&editor->launchArgs,
				&target, 
				&colorCorrection, 
				&node->dmng.data, 
				// out
				&target.cptr_numVisibleSplats,
				&target.cptr_numFragments, 
				&target.virt_numTilefragments_splatwise->cptr,
				&target.virt_depth->cptr,

				// &target.virt_stagedata->cptr,
				&virt_sd_basisvector1_encoded->cptr,
				&virt_sd_basisvector2_encoded->cptr,
				&virt_sd_imgPos_encoded->cptr,
				&virt_sd_color->cptr,
				&virt_sd_flags->cptr,
				&virt_sd_depth->cptr,

				&target.virt_ordering_splatdepth->cptr,
			};
			
			prog_render_soa->launch("kernel_stageSplats", args, data.count, target.mainstream);
		});

		

	}
	// cuCtxSynchronize();

	// Retrieve number of staged/visible splats and tile-fragments
	int i = 0;
	{

		// cuMemcpyDtoHAsync target must be page-locked.
		// Apparently local variables count as page-locked memory?
		uint32_t numVisibleSplats;
		uint32_t numFragments;
		CURuntime::check(cuMemcpyDtoHAsync(&numVisibleSplats, target.cptr_numVisibleSplats, 4, target.mainstream));
		CURuntime::check(cuMemcpyDtoHAsync(&numFragments, target.cptr_numFragments, 4, target.mainstream));

		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);

		Runtime::numVisibleSplats += numVisibleSplats;
		Runtime::numVisibleFragments += numFragments;

		target.numFragments = numFragments;
		target.numVisibleSplats = numVisibleSplats;

		{
			string label = "numSplats";
			string value = format(getSaneLocale(), "{:L}", numVisibleSplats);
			Runtime::debugValueList.push_back({label, value});
		}
		{
			string label = "numFragments";
			string value = format(getSaneLocale(), "{:L}", numFragments);
			Runtime::debugValueList.push_back({label, value});
		}
		
	}

	{

		target.virt_tileIDs->commit(4 * target.numFragments);
		target.virt_numTilefragments_splatwise_ordered->commit(4 * target.numFragments);
		target.virt_indices->commit(4 * target.numFragments);

		// Unfortunately we can't sort simultaneously, since both use the same alternative sort buffers
		cuCtxSynchronize();

		// sort visible splats by depth (or rather, compute the order without applying it).
		// We have to provide radix-sort with intermediate memory for sorting, which must be separate for each concurrent target.
		GPUSorting::sort_32bit_keyvalue(target.numVisibleSplats, target.virt_depth->cptr, target.virt_ordering_splatdepth->cptr, 0, 0,target.mainstream);

		// Apply the ordering. Necessary because we don't have 64 bit sorting and do it in a 32bit sort, followed by another 16 bit sort.
		// The follow-up 16 bit sort needs its keys to be sorted by the preceeding 32 bit sort.
		void* argsApply[] = {
			&target.virt_numTilefragments_splatwise->cptr, 
			&target.virt_numTilefragments_splatwise_ordered->cptr, 
			&target.virt_ordering_splatdepth->cptr, 
			&target.numVisibleSplats
		};
		editor->prog_gaussians_rendering->launch("kernel_applyOrdering_u32", argsApply, target.numVisibleSplats, target.mainstream);

		// Compute prefix sum of tile fragment counters.
		GPUPrefixSumsCS::dispatch(target.numVisibleSplats, target.virt_numTilefragments_splatwise_ordered->cptr, target.mainstream);
	}

	// Create tile fragment arrays concurrently
	{
		// The prefix sum is stored in-place in cptr_numTilefragments_ordered
		CUdeviceptr cptr_prefixsum = target.virt_numTilefragments_splatwise_ordered->cptr;

		// now create the tile fragment (StageData) array
		void* argsCreatefragmentArray[] = {
			// input
			&editor->launchArgs, &target,
			&target.virt_ordering_splatdepth->cptr, &target.numVisibleSplats, 
			
			//&target.virt_stagedata->cptr, 
			&virt_sd_basisvector1_encoded->cptr,
			&virt_sd_basisvector2_encoded->cptr,
			&virt_sd_imgPos_encoded->cptr,
			&virt_sd_color->cptr,
			&virt_sd_flags->cptr,
			&virt_sd_depth->cptr,

			&cptr_prefixsum,& target.cptr_numFragments,
			// output
			&target.virt_tileIDs->cptr, &target.virt_indices->cptr,
		};

		prog_render_soa->launch("kernel_createTilefragmentArray", argsCreatefragmentArray, target.numVisibleSplats, target.mainstream);
	}

	// But sort one target after the other because both use the same alternative sort buffers
	{
		cuCtxSynchronize();
		// The prefix sum is stored in-place in cptr_numTilefragments_ordered
		CUdeviceptr cptr_prefixsum = target.virt_numTilefragments_splatwise_ordered->cptr;
		GPUSorting::sort_16bitkey_32bitvalue(target.numFragments, target.virt_tileIDs->cptr, target.virt_indices->cptr, target.mainstream);
	}

	{

		int width   = target.target.width;
		int height  = target.target.height;
		int tiles_x = int(width + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
		int tiles_y = int(height + TILE_SIZE_3DGS - 1) / int(TILE_SIZE_3DGS);
		uint32_t numTiles = tiles_x * tiles_y;
		int tileSize = TILE_SIZE_3DGS;

		cuMemsetD8Async(target.cptr_tiles, 0, sizeof(Tile) * numTiles, target.mainstream);
		editor->prog_gaussians_rendering->launch(
			"kernel_computeTiles_method1", 
			{&editor->launchArgs, &target, &target.virt_tileIDs->cptr, &target.numFragments, &tileSize, &target.cptr_tiles}, 
			target.numFragments, target.mainstream
		);

		cuEventRecord(target.cu_tilesComputed, target.mainstream);

		// both streams use the computed tiles, so make sure the sidestream also waits until the mainstream computed the tiles.
		cuStreamWaitEvent(target.mainstream, target.cu_tilesComputed, 0);
		cuStreamWaitEvent(target.sidestream, target.cu_tilesComputed, 0);

		if(target.numFragments > 0){

			uint32_t pointsInTileThreshold = 1'000'000;

			void* args_rendering[] = {
				&editor->launchArgs, &target.target, &target.cptr_tiles, &target.virt_indices->cptr, 
				
				//&target.virt_stagedata->cptr, 
				&virt_sd_basisvector1_encoded->cptr,
				&virt_sd_basisvector2_encoded->cptr,
				&virt_sd_imgPos_encoded->cptr,
				&virt_sd_color->cptr,
				&virt_sd_flags->cptr,
				&virt_sd_depth->cptr,
				
				&pointsInTileThreshold
			};

			

			prog_render_soa->launch("kernel_render_gaussians", args_rendering, {.gridsize = numTiles, .blocksize = 256, .stream = target.sidestream});
		}
	}

	// wait for all streams
	{
		cuStreamSynchronize(target.mainstream);
		cuStreamSynchronize(target.sidestream);
	}
	
	if(Runtime::measureTimings){
		cuEventRecord(event_end, 0);

		cuCtxSynchronize();

		double seconds = now() - t_start;

		float duration;
		cuEventElapsedTime(&duration, event_start, event_end);

		Runtime::timings.add("[draw splats (host)]", seconds * 1000.0f);
		Runtime::timings.add("[draw splats (device)]", duration);
	}
	
}


void SplatEditor::draw(Scene* scene, vector<RenderTarget> targets){

	cuCtxSynchronize();

	// Motion control: deform rigged splat nodes (LBS + blendshapes) before any
	// per-node staging launch, so the deformed position/scale/quaternion are
	// ready in dmng.data for the render kernels. No-op for non-rigged scenes.
	rigController.dispatchSkinning(*scene);

	// 4DGS dynamic scene: deform canonical Gaussians via HexPlane + MLP.
	// Runs the TorchScript deformation model for each SN4DGSSplats node,
	// writing the time-dependent position/scale/rotation into deformBuffer, then
	// swaps dmng.data to point at the deformed buffers so the staging kernels
	// (drawsplats_3dgs_concurrent*, below) read deformed Gaussians.
	std::vector<SN4DGSSplats*> deformedNodes4DGS;
	scene->forEach<SN4DGSSplats>([&](SN4DGSSplats* node) {
		if (!node->visible) return;
		if (node->dmng.data.count == 0) return;
		float t = timeline.playhead > 0.0
			? float(timeline.playhead / (timeline.duration > 0.0 ? timeline.duration : 1.0))
			: 0.0f;
		node->deform(t, mainstream);
		node->swapToDeformed();
		deformedNodes4DGS.push_back(node);
	});

	// deform() dispatches cuMemcpyDtoDAsync on this `mainstream`; the splat
	// staging kernels below run on each target's per-target `mainstream`
	// (a CU_STREAM_NON_BLOCKING stream created in the cache below). Without a
	// bridge, staging could race ahead of the deform copies. Record an event on
	// this stream after the deform dispatch; each per-target stream waits on it
	// before its first launch. `deformEvent` is 0 when no deform ran this frame,
	// so the per-target loop skips the wait.
	CUevent deformEvent = CUevent(0);
	if (!deformedNodes4DGS.empty()) {
		static CUevent deformDone = []{
			CUevent e;
			CURuntime::check(cuEventCreate(&e, CU_EVENT_DISABLE_TIMING));
			return e;
		}();
		cuEventRecord(deformDone, mainstream);
		deformEvent = deformDone;
	}

	// NOTE: swapToCanonical() is called AFTER all splat rendering, at the very
	// end of draw() — restoring here would un-swap before staging reads the
	// deformed buffers, defeating the deformation. See the end of this function.

	// Stuff that only needs to be done once for all targets
	vector<SNPoints*> nodes;
	scene->process<SNPoints>([&](SNPoints* node){
		if(!node->visible) return;
		nodes.push_back(node);
	});

	scene->forEach<SNTriangles>([&](SNTriangles* node) {
		if (!node->visible) return;

		node->data.transform = node->transform_global;

		Runtime::numRenderedTriangles += node->data.count;

		TriangleQueueItem item;
		item.geometry = node->data;
		item.material = node->material;

		triangleQueue.push_back(item);
	});

	// Cache concurrency stuff that needs to be dedicated to each target.
	static vector<ConcurrentTarget> cache;
	while(cache.size() < targets.size()){
		CUstream mainstream, sidestream;
		CURuntime::check(cuStreamCreate(&mainstream, CU_STREAM_NON_BLOCKING));
		CURuntime::check(cuStreamCreate(&sidestream, CU_STREAM_NON_BLOCKING));

		CUevent cu_tilesComputed;
		cuEventCreate(&cu_tilesComputed, CU_EVENT_DEFAULT);

		int MAX_WIDTH = 4096;
		int MAX_HEIGHT = 4096;
		int tileSize = 8; // use smallest potential tile size to allocate enough memory for number of tiles
		int MAX_TILES_X = MAX_WIDTH / tileSize;
		int MAX_TILES_Y = MAX_HEIGHT / tileSize;

		ConcurrentTarget concurrent;
		concurrent.mainstream = mainstream;
		concurrent.sidestream = sidestream;
		concurrent.cu_tilesComputed = cu_tilesComputed;
		concurrent.virt_fb_depth = CURuntime::allocVirtual("fb_depth");
		concurrent.virt_fb_color = CURuntime::allocVirtual("fb_color");

		concurrent.cptr_numVisibleSplats           = CURuntime::alloc("numVisibleSplats", 8);
		concurrent.cptr_numFragments               = CURuntime::alloc("numFragments", 8);
		concurrent.cptr_tiles                      = CURuntime::alloc("tiles", sizeof(Tile) * MAX_TILES_X * MAX_TILES_Y);
		
		concurrent.virt_tileIDs                            = CURuntime::allocVirtual("sm1_tileIDs");
		concurrent.virt_indices                            = CURuntime::allocVirtual("sm1_indices");
		concurrent.virt_depth                              = CURuntime::allocVirtual("sm1_depth");
		concurrent.virt_bounds                             = CURuntime::allocVirtual("sm1_bounds");
		concurrent.virt_stagedata                          = CURuntime::allocVirtual("sm1_stagedata");
		concurrent.virt_ordering_splatdepth                = CURuntime::allocVirtual("sm1_ordering_splatdepth ");
		concurrent.virt_numTilefragments_splatwise         = CURuntime::allocVirtual("sm1_numTilefragments_splatwise");
		concurrent.virt_numTilefragments_splatwise_ordered = CURuntime::allocVirtual("sm1_numTilefragments_splatwise_ordered");

		cache.push_back(concurrent);
	}

	// Augment given targets by (cached) concurrent auxiliary stuff
	vector<ConcurrentTarget> concurrentTargets;
	for(int i = 0; i < targets.size(); i++){

		ConcurrentTarget concurrent = cache[i];
		concurrent.target = targets[i];

		concurrentTargets.push_back(concurrent);
	}

	if(ovr->isActive()){
		makeAssetsVR(imn_assets->page);
		makeBrushesVR(imn_brushes->page);
		makeLayersVR(imn_layers->page);
		makePaintingVR(imn_painting->page);
	}

	// NOTE: triangle&point rendering is actually still sequential. 
	// Currently needs to be sequential due to global device counters in kernel.
	// TODO: Make triangle and point rendering concurrent for all targets. 
	for(ConcurrentTarget concurrent : concurrentTargets){

		RenderTarget target = concurrent.target;
		CUstream mainstream = concurrent.mainstream;
		CUstream sidestream = concurrent.sidestream;

		// Wait for this frame's 4DGS deformation copies (dispatched on the
		// editor's mainstream above) before any per-target launch reads dmng.data.
		// deformEvent is 0 when no deform ran this frame → skip the wait.
		if (deformEvent != CUevent(0)) {
			cuStreamWaitEvent(mainstream, deformEvent, CU_EVENT_WAIT_DEFAULT);
			cuStreamWaitEvent(sidestream, deformEvent, CU_EVENT_WAIT_DEFAULT);
		}

		prog_gaussians_rendering->launch("kernel_clearFramebuffer", {&launchArgs, &target}, target.width * target.height, mainstream);

		if(nodes.size() > 0)
		{ // RENDER POINTS

			// Split nodes into progressive (shuffled & ready) and HQS (everything
			// else). Each path writes into target.framebuffer using the shared
			// (depth<<32)|color + atomicMin convention, so they compose with the
			// clear above and with each other. The HQS path additionally uses
			// the per-target fb_depth/fb_color scratch buffers.
			vector<SNPoints*> hqsNodes;
			vector<SNPoints*> progressiveNodes;
			for(SNPoints* node : nodes){
				if(settings.pointRenderer == POINTRENDERER_PROGRESSIVE && node->progressive.data.ready){
					progressiveNodes.push_back(node);
				}else{
					hqsNodes.push_back(node);
				}
			}

			if(hqsNodes.size() > 0)
			{ // RENDER POINTS - HQS (existing path)

				shared_ptr<CudaVirtualMemory> virt_fb_depth = concurrent.virt_fb_depth;
				shared_ptr<CudaVirtualMemory> virt_fb_color = concurrent.virt_fb_color;

				virt_fb_depth->commit(target.width * target.height * 4);
				virt_fb_color->commit(target.width * target.height * 16);

				uint32_t INF = 0x7f800000;
				cuMemsetD32Async(virt_fb_depth->cptr, INF, target.width * target.height, mainstream);
				cuMemsetD32Async(virt_fb_color->cptr, 0, 4 * target.width * target.height, mainstream);

				float pointSize = 0.5f;

				// depthmap
				for(SNPoints* node : hqsNodes){
					prog_points->launch("kernel_hqs_depth", {&launchArgs, &node->manager.data, &target, &virt_fb_depth->cptr, &virt_fb_color->cptr, &pointSize}, node->manager.data.count, mainstream);
				}

				// colors
				for(SNPoints* node : hqsNodes){
					// Apply the live color-correction preview only to the
					// selected node, mirroring the splat render-time preview
					// path in scene->process<SNSplats> above. An unselected
					// node gets a default-constructed ColorCorrection, which
					// applyColorCorrection leaves unchanged.
					ColorCorrection cc;
					if(node->selected){
						cc = settings.colorCorrection;
					}
					prog_points->launch("kernel_hqs_color", {&launchArgs, &node->manager.data, &target, &virt_fb_depth->cptr, &virt_fb_color->cptr, &pointSize, &cc}, node->manager.data.count, mainstream);
				}

				// normalize and transfer to target.framebuffer
				uint32_t numPixels = target.width * target.height;
				prog_points->launch("kernel_hqs_normalize", {&launchArgs, &target, &virt_fb_depth->cptr, &virt_fb_color->cptr}, numPixels, mainstream);
			}

			if(progressiveNodes.size() > 0)
			{ // RENDER POINTS - PROGRESSIVE (Skye-style port)
				drawpoints_progressive(progressiveNodes, target, mainstream);
			}
		}

		{ // RENDER TRIANGLES
			
			if (triangleQueue.size() > 0)
			{
				static CUdeviceptr cptr_queue_geometry = 0;
				static CUdeviceptr cptr_queue_material = 0;

				if (cptr_queue_geometry == 0) {
					cptr_queue_geometry = CURuntime::alloc("queue_geometry", 100'000 * sizeof(TriangleData));
					cptr_queue_material = CURuntime::alloc("queue_material", 100'000 * sizeof(TriangleMaterial));
				}

				vector<TriangleData> queue_data;
				vector<TriangleMaterial> queue_material;
				for (auto item : triangleQueue) {
					queue_data.push_back(item.geometry);
					queue_material.push_back(item.material);
				}

				cuMemcpyHtoDAsync(cptr_queue_geometry, queue_data.data(), queue_data.size() * sizeof(TriangleData), mainstream);
				cuMemcpyHtoDAsync(cptr_queue_material, queue_material.data(), queue_material.size() * sizeof(TriangleMaterial), mainstream);

				TriangleModelQueue queue;
				queue.count = queue_data.size();
				queue.geometries = (TriangleData*)cptr_queue_geometry;
				queue.materials = (TriangleMaterial*)cptr_queue_material;

				OptionalLaunchSettings settings = { 0 };
				settings.blocksize = 64;
				settings.stream = mainstream;
				prog_triangles->launchCooperative("kernel_drawTriangleQueue", { &launchArgs, &queue, &target }, settings);
			}
		}

		if(ovr->isActive())
		{
			

			auto glmapping_assets = mapCudaGl(imn_assets->page->framebuffer->colorAttachments[0]);
			auto glmapping_brushes = mapCudaGl(imn_brushes->page->framebuffer->colorAttachments[0]);
			auto glmapping_layers = mapCudaGl(imn_layers->page->framebuffer->colorAttachments[0]);
			auto glmapping_painting = mapCudaGl(imn_painting->page->framebuffer->colorAttachments[0]);

			{ // DRAW VR MENU ASSETS		
				imn_assets->mesh->data.transform = imn_assets->transform;
				imn_assets->mesh->material.texture.data = nullptr;
				imn_assets->mesh->material.texture.surface = -1;
				imn_assets->mesh->material.texture.cutexture = glmapping_assets.texture;
				imn_assets->mesh->material.texture.width = imn_assets->page->width;
				imn_assets->mesh->material.texture.height = imn_assets->page->height;
				imn_assets->mesh->material.mode = MATERIAL_MODE_TEXTURED;
				prog_triangles->launchCooperative("kernel_drawTriangles", {&launchArgs, &imn_assets->mesh->data, &imn_assets->mesh->material, &target}, {.stream = mainstream});

				Runtime::numRenderedTriangles += imn_assets->mesh->data.count;
			}

			{ // DRAW VR MENU BRUSHES
				imn_brushes->mesh->data.transform = imn_brushes->transform;
				imn_brushes->mesh->material.texture.data = nullptr;
				imn_brushes->mesh->material.texture.surface = -1;
				imn_brushes->mesh->material.texture.cutexture = glmapping_brushes.texture;
				imn_brushes->mesh->material.texture.width = imn_brushes->page->width;
				imn_brushes->mesh->material.texture.height = imn_brushes->page->height;
				imn_brushes->mesh->material.mode = MATERIAL_MODE_TEXTURED;
				prog_triangles->launchCooperative("kernel_drawTriangles", {&launchArgs, &imn_brushes->mesh->data, &imn_brushes->mesh->material, &target}, {.stream = mainstream});

				Runtime::numRenderedTriangles += imn_brushes->mesh->data.count;
			}

			{ // DRAW VR MENU LAYERS
				shared_ptr<ImguiNode> node = imn_layers;

				node->mesh->data.transform = node->transform;
				node->mesh->material.texture.data = nullptr;
				node->mesh->material.texture.surface = -1;
				node->mesh->material.texture.cutexture = glmapping_layers.texture;
				node->mesh->material.texture.width = node->page->width;
				node->mesh->material.texture.height = node->page->height;
				node->mesh->material.mode = MATERIAL_MODE_TEXTURED;
				prog_triangles->launchCooperative("kernel_drawTriangles", {&launchArgs, &node->mesh->data, &node->mesh->material, &target}, {.stream = mainstream});

				Runtime::numRenderedTriangles += node->mesh->data.count;
			}

			{ // DRAW VR MENU PAINTING
				shared_ptr<ImguiNode> node = imn_painting;

				node->mesh->data.transform = node->transform;
				node->mesh->material.texture.data = nullptr;
				node->mesh->material.texture.surface = -1;
				node->mesh->material.texture.cutexture = glmapping_painting.texture;
				node->mesh->material.texture.width = node->page->width;
				node->mesh->material.texture.height = node->page->height;
				node->mesh->material.mode = MATERIAL_MODE_TEXTURED;
				prog_triangles->launchCooperative("kernel_drawTriangles", {&launchArgs, &node->mesh->data, &node->mesh->material, &target}, {.stream = mainstream});

				Runtime::numRenderedTriangles += node->mesh->data.count;
			}

			glmapping_assets.unmap();
			glmapping_brushes.unmap();
			glmapping_layers.unmap();
			glmapping_painting.unmap();
		}

		// cuCtxSynchronize();

		{ // DRAW DEVICE LINES

			static CUdeviceptr cptr_numProcessedLines_0 = CURuntime::alloc("processed lines counter", 8);
			static CUdeviceptr cptr_numProcessedLines_1 = CURuntime::alloc("processed lines counter", 8);

			cuMemsetD32Async(cptr_numProcessedLines_0, 0, 2, mainstream);
			cuMemsetD32Async(cptr_numProcessedLines_1, 0, 2, mainstream);

			OptionalLaunchSettings settings = {0};
			settings.blocksize = 64;
			settings.stream = mainstream;
			prog_lines->launchCooperative("kernel_drawLines", {&launchArgs, &target, &cptr_lines, &cptr_numLines, &cptr_numProcessedLines_0}, settings);
			prog_lines->launchCooperative("kernel_drawLines", {&launchArgs, &target, &virt_lines_host->cptr, &cptr_numLines_host, &cptr_numProcessedLines_1}, settings);
		}

		cuCtxSynchronize();

		if(settings.enableEDL){
			prog_helpers->launch("kernel_applyEyeDomeLighting", {&launchArgs, &target}, target.width * target.height, mainstream);
		}

		cuEventRecord(event_edl_applied, mainstream);
		cuStreamWaitEvent(mainstream, event_edl_applied, CU_EVENT_WAIT_DEFAULT);
		cuStreamWaitEvent(sidestream, event_edl_applied, CU_EVENT_WAIT_DEFAULT);

		if(Runtime::measureTimings){
			cuEventRecord(event_mainstream, 0);
		}
	}

	cuCtxSynchronize();

	if(settings.splatRenderer == SPLATRENDERER_3DGS){

		// if(settings.intersectionMode && INTERSECTION_3DGS || settings.intersectionMode ==INTERSECTION_TIGHTBB){
		// 	drawsplats_3dgs_concurrent_fragintersections(scene, concurrentTargets);
		// }else 
		if(settings.renderSoA){
			drawsplats_3dgs_concurrent_soa(scene, concurrentTargets);
		}else if(settings.renderBandwidth){
			drawsplats_3dgs_concurrent_bandwidth(scene, concurrentTargets);
		}else if(settings.renderFragIntersections){
			drawsplats_3dgs_concurrent_fragintersections(scene, concurrentTargets);
		}else{
			drawsplats_3dgs_concurrent(scene, concurrentTargets);
		}

	}else if(settings.splatRenderer == SPLATRENDERER_PERSPECTIVE_CORRECT){
		drawsplats_perspectiveCorrect_concurrent(scene, concurrentTargets);
	}

	// All splat rendering (drawsplats_3dgs_concurrent*, above) has now consumed
	// the deformed dmng.data for any SN4DGSSplats nodes. Restore canonical
	// pointers so the next frame's deform() writes into the canonical buffers
	// and so non-render reads (editing, stats, GPU-mem reports) see canonical
	// data. This MUST come after the splat render dispatch, not after the deform
	// dispatch — otherwise staging would read canonical (rest-pose) Gaussians.
	for (auto* node : deformedNodes4DGS) {
		node->swapToCanonical();
	}
}