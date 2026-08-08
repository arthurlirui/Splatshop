
#define CUB_DISABLE_BF16_SUPPORT

#define GLM_FORCE_CUDA
#define CUDA_VERSION 12000

namespace std{
	using size_t = ::size_t;
};


#include <curand_kernel.h>

#include "./libs/glm/glm/glm.hpp"
#include "./libs/glm/glm/gtc/matrix_transform.hpp"
#include "./libs/glm/glm/gtc/matrix_access.hpp"
#include "./libs/glm/glm/gtx/transform.hpp"
#include "./libs/glm/glm/gtc/quaternion.hpp"

#include "../utils.cuh"

// #include "./include/OrbitControls.h"
// #include "./include/DesktopVRControls.h"

#include "../HostDeviceInterface.h"

// from: https://developer.nvidia.com/blog/faster-parallel-reductions-kepler/
__device__ __inline__ 
int warpReduceMax(float val) {

	for (int offset = warpSize / 2; offset > 0; offset /= 2){
		val = max(val, __shfl_down_sync(0xFFFFFFFF, val, offset, 32));
	}

	return val;
}

__device__ __inline__ 
int warpReduceMin(float val) {

	for (int offset = warpSize / 2; offset > 0; offset /= 2){
		val = min(val, __shfl_down_sync(0xFFFFFFFF, val, offset, 32));
	}

	return val;
}

extern "C" __global__
void kernel_drawPoints(CommonLaunchArgs args, PointData points, RenderTarget target){

	auto grid = cg::this_grid();

	auto uniforms = args.uniforms;

	vec3 origin = vec3(inverse(target.view) * vec4(0.0, 0.0, 0.0, 1.0));

	int index = grid.thread_rank();

	if(index >= points.count) return;

	vec3 pos = points.position[index];
	uint32_t flags = points.flags[index];
	uint32_t color = points.color[index];
	uint8_t* rgba = (uint8_t*)&color;

	bool isSelected = flags & FLAGS_SELECTED;
	bool isErased = flags & FLAGS_DELETED;

	if(isSelected){ 
		// color = 0x000000ff;
		rgba[0] = clamp(rgba[0] * 1.5f + 50.1f, 0.0f, 255.0f);
	}
	if(isErased) return;

	mat4 transform = target.proj * target.view * points.transform;
	vec4 ndc = transform * vec4(pos, 1.0f);
	vec2 imgCoords = vec2(
		(0.5f * (ndc.x / ndc.w) + 0.5f) * target.width,
		(0.5f * (ndc.y / ndc.w) + 0.5f) * target.height
	);

	if(ndc.w < 0.0f) return;

	float size = 0.5f;

	for(float dx = -size; dx <= size; dx += 1.0f)
	for(float dy = -size; dy <= size; dy += 1.0f)
	{
		vec2 coord = imgCoords + vec2{dx, dy};

		if(coord.x < 0.0f || coord.x >= target.width) continue;
		if(coord.y < 0.0f || coord.y >= target.height) continue;

		int pixelID = int(coord.x) + int(coord.y) * target.width;

		uint32_t udepth = __float_as_uint(ndc.w);
		// uint32_t udepth = __float_as_uint(2.0f);
		uint64_t pixel = (uint64_t(udepth) << 32) | color;

		uint64_t oldPixel = target.framebuffer[pixelID];

		if(pixel < oldPixel){
			atomicMin(&target.framebuffer[pixelID], pixel);
		}
	}

}

extern "C" __global__
void kernel_compute_boundingbox(CommonLaunchArgs args, PointData model, vec3& min, vec3& max){

	int index = cg::this_grid().thread_rank();

	if(index >= model.count) return;

	vec3 pos = vec3(model.transform * vec4(model.position[index], 1.0f));

	// should probably do warp&block level reduce before atomic min/max
	// the ifs are hacky but still improve perf from 15 to 4 ms
	if(pos.x < min.x) atomicMinFloat(&min.x, pos.x);
	if(pos.y < min.y) atomicMinFloat(&min.y, pos.y);
	if(pos.z < min.z) atomicMinFloat(&min.z, pos.z);
	if(pos.x > max.x) atomicMaxFloat(&max.x, pos.x);
	if(pos.y > max.y) atomicMaxFloat(&max.y, pos.y);
	if(pos.z > max.z) atomicMaxFloat(&max.z, pos.z);

}

extern "C" __global__
void kernel_update_model(CommonLaunchArgs args, RenderTarget target, PointData model){

	auto grid = cg::this_grid();

	auto uniforms = args.uniforms;
	// auto controls = args.controls;
	auto& mouse = args.mouseEvents;
	auto keys = args.keys;
	// auto desktopVrControls = args.desktopVrControls;
	auto state = args.state;
	auto& keyEvents = args.keyEvents;

	uint64_t t_start = nanotime();

	// if(grid.thread_rank() == 0){
	// 	if(state->counter == 0){
	// 		state->vr_transform = mat4(1.0f);
	// 		state->vr_controller_neutral = mat4(1.0f);
	// 	}
	// }

	// bool noInput = mouse.action == 0 && mouse.button == 0 && keys->mods == 0;
	bool thereIsInput = mouse.buttonPressed != 0 || mouse.action != 0 || mouse.button != 0 || keys->mods != 0;
	thereIsInput = thereIsInput || keys->isCtrlDown();
	thereIsInput = thereIsInput || keys->isAltDown();
	thereIsInput = thereIsInput || keys->isShiftDown();
	thereIsInput = thereIsInput || keyEvents.numEvents > 0;
	thereIsInput = thereIsInput || mouse.isMiddleDown;
	thereIsInput = thereIsInput || args.brush.mode != BRUSHMODE::NONE;

	grid.sync();

	// prepare rays
	vec3 origin = vec3(inverse(target.view) * vec4(0.0, 0.0, 0.0, 1.0));

	vec4 dir_00_projspace = vec4{-1.0f, -1.0f, 1.0f, 1.0f};
	vec4 dir_01_projspace = vec4{-1.0f,  1.0f, 1.0f, 1.0f};
	vec4 dir_10_projspace = vec4{ 1.0f, -1.0f, 1.0f, 1.0f};
	vec4 dir_11_projspace = vec4{ 1.0f,  1.0f, 1.0f, 1.0f};

	float right = 1.0f / target.proj[0][0];
	float up = 1.0f / target.proj[1][1];
	vec4 dir_00_worldspace = inverse(target.view) * vec4(-right, -up, -1.0f, 1.0f);
	vec4 dir_01_worldspace = inverse(target.view) * vec4(-right,  up, -1.0f, 1.0f);
	vec4 dir_10_worldspace = inverse(target.view) * vec4( right, -up, -1.0f, 1.0f);
	vec4 dir_11_worldspace = inverse(target.view) * vec4( right,  up, -1.0f, 1.0f);

	auto getRayDir = [&](float u, float v){
		float A_00 = (1.0f - u) * (1.0f - v);
		float A_01 = (1.0f - u) *         v;
		float A_10 =         u  * (1.0f - v);
		float A_11 =         u  *         v;

		vec3 dir = (
			A_00 * dir_00_worldspace + 
			A_01 * dir_01_worldspace + 
			A_10 * dir_10_worldspace + 
			A_11 * dir_11_worldspace - vec4(origin, 1.0));
		dir = normalize(dir);

		return dir;
	};

	grid.sync();

	vec3 brushPos = args.state->hovered_pos;
	float distToPos = length(origin - brushPos);
	float brushRadius = distToPos / 15.0f;

	// handle selection / deselection
	mat4 transform = target.proj * target.view;

	float u = float(mouse.pos_x) / target.width;
	float v = float(mouse.pos_y) / target.height;
	vec3 dir = getRayDir(u, v);

	if(thereIsInput)
	process(model.count, [&](int index){
		uint32_t flag = model.flags[index];
		vec3 pos = model.position[index];
		vec3 worldPos = vec3(model.transform * vec4(pos, 1.0f));

		vec3 camToPoint = worldPos - origin;

		float t = glm::dot(dir, camToPoint);

		vec3 pointOnLine = origin + t * dir;
		float pointToLineDistance = length(pointOnLine - worldPos);

		vec4 ndc = transform * vec4(worldPos, 1.0f);
		vec2 imgCoords = vec2(
			(0.5f * (ndc.x / ndc.w) + 0.5f) * target.width,
			(0.5f * (ndc.y / ndc.w) + 0.5f) * target.height
		);

		vec2 mpos = vec2(mouse.pos_x, mouse.pos_y);
		float d = length(imgCoords - mpos);

		if(t < 0.0f){
			d = Infinity;
		}

		bool isHighlighted = d < args.brush.size;
		if(isHighlighted){
			flag = flag | FLAGS_HIGHLIGHTED;
		}else{
			flag = flag & (~FLAGS_HIGHLIGHTED);
		}

		if(args.brush.mode == BRUSHMODE::SELECT){
			
			bool isSelected = isHighlighted && mouse.isLeftDown;

			if(isSelected){
				flag = flag | FLAGS_SELECTED;
			}
		}else if(args.brush.mode == BRUSHMODE::REMOVE_FLAGS){
			bool isSelected = isHighlighted && mouse.isLeftDown;

			if(isSelected){
				flag = flag & (~0b1001);
			}
		}else if(args.brush.mode == BRUSHMODE::ERASE){
			bool isSelected = isHighlighted && mouse.isLeftDown;

			if(isHighlighted){
				flag = flag | 0b010;
			}else{
				flag = flag & (~0b010);
			}

			if(isSelected){
				flag = flag | 0b1000;
			}
		}

		model.flags[index] = flag;

		// // TRANSLATION
		// if(mouse.isMiddleDown && (flag & FLAGS_SELECTED)){

		// 	vec2 diff = {
		// 		mouse.pos_x - args.mouseEvents_prev.pos_x,
		// 		mouse.pos_y - args.mouseEvents_prev.pos_y
		// 	};

		// 	auto ux = -diff.x / 1000.0f;
		// 	auto uy = diff.y / 1000.0f;

		// 	float x = -ux * controls.radius;
		// 	float y = uy * controls.radius;
		// 	float z = 0.0f;

		// 	mat4 world = controls.world;

		// 	vec3 _pos     = vec3(world * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
		// 	vec3 _right   = vec3(world * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
		// 	vec3 _forward = vec3(world * glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
		// 	vec3 _up      = vec3(world * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

		// 	_right   = glm::normalize(_right   - _pos) * x;
		// 	_forward = glm::normalize(_forward - _pos) * y;
		// 	_up      = glm::normalize(_up      - _pos) * (-z);

		// 	model.position[index] += _right + _forward + _up;
			
		// }
	});

}


extern "C" __global__
void kernel_hqs_depth(
	CommonLaunchArgs args, PointData points, 
	RenderTarget target, 
	uint32_t* fb_depth,
	uint32_t* fb_color,
	float pointSize
){
	auto grid = cg::this_grid();
	int index = grid.thread_rank();

	if(index >= points.count) return;

	vec3 pos = points.position[index];
	uint32_t flags = points.flags[index];
	uint32_t color = points.color[index];
	uint8_t* rgba = (uint8_t*)&color;

	bool isSelected = flags & FLAGS_SELECTED;
	bool isErased = flags & FLAGS_DELETED;

	if(isSelected){ 
		// color = 0x000000ff;
		rgba[0] = clamp(rgba[0] * 1.5f + 50.1f, 0.0f, 255.0f);
	}
	if(isErased) return;

	mat4 transform = target.proj * target.view * points.transform;
	vec4 ndc = transform * vec4(pos, 1.0f);
	vec2 imgCoords = vec2(
		(0.5f * (ndc.x / ndc.w) + 0.5f) * target.width,
		(0.5f * (ndc.y / ndc.w) + 0.5f) * target.height
	);

	if(ndc.w < 0.0f) return;

	// Host-controlled splat radius (in pixels). The main HQS path passes
	// 0.5f; the Orbbec Point Cloud panel passes settings.orbbecPCPointSize.
	for(float dx = -pointSize; dx <= pointSize; dx += 1.0f)
	for(float dy = -pointSize; dy <= pointSize; dy += 1.0f)
	{
		vec2 coord = imgCoords + vec2{dx, dy};

		if(coord.x < 0.0f || coord.x >= target.width) continue;
		if(coord.y < 0.0f || coord.y >= target.height) continue;

		int pixelID = int(coord.x) + int(coord.y) * target.width;

		uint32_t udepth = __float_as_uint(ndc.w);

		uint64_t old = fb_depth[pixelID];

		if(udepth < old){
			atomicMin(&fb_depth[pixelID], udepth);
		}
	}

}

extern "C" __global__
void kernel_hqs_color(
	CommonLaunchArgs args, PointData points,
	RenderTarget target,
	uint32_t* fb_depth,
	uint32_t* fb_colors,
	float pointSize,
	ColorCorrection colorCorrection
){
	auto grid = cg::this_grid();
	int index = grid.thread_rank();

	if(index >= points.count) return;

	vec3 pos = points.position[index];
	uint32_t flags = points.flags[index];
	uint32_t color = points.color[index];
	uint8_t* rgba = (uint8_t*)&color;

	bool isSelected = flags & FLAGS_SELECTED;
	bool isErased = flags & FLAGS_DELETED;

	if(isSelected){
		// color = 0x000000ff;
		rgba[0] = clamp(rgba[0] * 1.5f + 50.1f, 0.0f, 255.0f);
	}
	if(isErased) return;

	// Apply the live color-correction settings (brightness/contrast/gamma/
	// hue/saturation/lightness) to this point's color. The host passes the
	// settings.colorCorrection values for selected nodes (mirrors the splat
	// render-time preview path); for unselected nodes the host passes a
	// default-constructed ColorCorrection, which applyColorCorrection leaves
	// unchanged (brightness=0, contrast=0, gamma=1, hue=0, ...).
	{
		vec4 c = vec4(float(rgba[0]) / 255.0f,
		              float(rgba[1]) / 255.0f,
		              float(rgba[2]) / 255.0f, 1.0f);
		c = applyColorCorrection(c, colorCorrection);
		rgba[0] = (uint8_t)clamp(c.r * 255.0f, 0.0f, 255.0f);
		rgba[1] = (uint8_t)clamp(c.g * 255.0f, 0.0f, 255.0f);
		rgba[2] = (uint8_t)clamp(c.b * 255.0f, 0.0f, 255.0f);
	}

	mat4 transform = target.proj * target.view * points.transform;
	vec4 ndc = transform * vec4(pos, 1.0f);
	vec2 imgCoords = vec2(
		(0.5f * (ndc.x / ndc.w) + 0.5f) * target.width,
		(0.5f * (ndc.y / ndc.w) + 0.5f) * target.height
	);

	if(ndc.w < 0.0f) return;

	// Host-controlled splat radius (see kernel_hqs_depth above).
	for(float dx = -pointSize; dx <= pointSize; dx += 1.0f)
	for(float dy = -pointSize; dy <= pointSize; dy += 1.0f)
	{
		vec2 coord = imgCoords + vec2{dx, dy};

		if(coord.x < 0.0f || coord.x >= target.width) continue;
		if(coord.y < 0.0f || coord.y >= target.height) continue;

		int pixelID = int(coord.x) + int(coord.y) * target.width;

		float depth = ndc.w;
		float referenceDepth = __uint_as_float(fb_depth[pixelID]) * 1.01f;

		if(depth <= referenceDepth){
			atomicAdd(&fb_colors[4 * pixelID + 0], rgba[0]);
			atomicAdd(&fb_colors[4 * pixelID + 1], rgba[1]);
			atomicAdd(&fb_colors[4 * pixelID + 2], rgba[2]);
			atomicAdd(&fb_colors[4 * pixelID + 3], 1);
		}
	}

}

extern "C" __global__
void kernel_hqs_normalize(
	CommonLaunchArgs args,
	RenderTarget target, 
	uint32_t* fb_depth,
	uint32_t* fb_colors
){
	auto grid = cg::this_grid();
	int pixelID = grid.thread_rank();

	int numPixels = target.width * target.height;

	if(pixelID >= numPixels) return;
	
	uint32_t udepth = fb_depth[pixelID];
	float depth = __uint_as_float(udepth);
	uint32_t R = fb_colors[4 * pixelID + 0];
	uint32_t G = fb_colors[4 * pixelID + 1];
	uint32_t B = fb_colors[4 * pixelID + 2];
	uint32_t count = fb_colors[4 * pixelID + 3];

	uint32_t color = 0;
	uint8_t* rgba = (uint8_t*)&color;

	rgba[0] = R / count;
	rgba[1] = G / count;
	rgba[2] = B / count;
	rgba[3] = 255;

	uint32_t referenceDepth = __uint_as_float(target.framebuffer[pixelID] >> 32);

	if(depth < referenceDepth * 1.01f){
		uint64_t pixel = uint64_t(depth) << 32 | uint64_t(color);
		target.framebuffer[pixelID] = pixel;
	}

}