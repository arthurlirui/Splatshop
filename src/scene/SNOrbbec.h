#pragma once

// SNOrbbec.h
//
// A live RGBD point-cloud scene node backed by an Orbbec camera.
//
// SNOrbbec derives from SNPoints so it reuses the existing point rendering
// path (HQS renderer) and the PointDataManager device buffers. Unlike the
// file loaders, which append once to a growing cloud, this node represents
// a *recurring* per-frame cloud: every captured frame replaces the whole
// point set. The host fills `points->position` / `points->color` from the
// latest RGBDFrame (see SplatEditor_update.h) and the update loop uploads
// the entire buffer each frame.
//
// The actual Orbbec SDK interaction (pipeline, frame polling) lives in
// OrbbecCapture; this node only holds the produced point data and the
// signalling state used by the main-thread upload branch.

#include <atomic>
#include <algorithm>
#include <cstring>

#include "SNPoints.h"

#ifdef SPLATSHOP_HAS_ORBBEC
#include "camera/OrbbecTypes.h"
#endif

struct SNOrbbec : public SNPoints {

	// Set by the main thread when a new RGBDFrame has been copied into
	// points->position / points->color; cleared once the GPU upload for
	// that frame is dispatched. The upload branch in SplatEditor_update.h
	// watches this flag.
	std::atomic<bool> frameReady{false};

	// Frame index of the last point cloud fed into loadPointCloud(), used by
	// the GUI to skip re-uploading when getLatestPointCloud() returns the
	// same frame across multiple GUI ticks.
	uint64_t lastFrameIndex = 0;

	// Fixed point capacity (color_width * depth_height). The host buffers
	// are pre-allocated to this size on the first frame so we can reuse
	// them without reallocation.
	int64_t capacity = 0;

	// Self-contained orbit camera for the Orbbec Point Cloud display panel
	// (see SplatEditor_render.h / gui/orbbec_preview.h). This is independent
	// of GLRenderer::camera so the panel can orbit the live stream without
	// disturbing the main 3D viewport.
	float pcYaw = 0.0f;
	float pcPitch = 0.0f;
	float pcRadius = 2.0f;
	glm::vec3 pcTarget = glm::vec3(0.0f);
	bool pcCameraInited = false;          // set once the first AABB frame is framed

	// Drag state machine for the panel. Tracking down/up rather than relying
	// on ImGui::IsMouseDragging() avoids the "stuck dragging" bug where the
	// cursor leaves the panel while a button is held and the cloud keeps
	// spinning until the user clicks inside again.
	bool pcRotating = false;              // left button is held (rotate)
	bool pcPanning  = false;              // right/middle button held (pan)
	float pcLastMouseX = 0.0f;            // last cursor pos for delta calc
	float pcLastMouseY = 0.0f;

	SNOrbbec(string name) : SNPoints(name) {
		this->points = make_shared<Points>();
		this->points->name = name;
		this->points->bytesPerPoint = 16;
		this->points->headerSize = 0;
		this->manager.data.transform = points->world;
	}

	uint64_t getGpuMemoryUsage() override {
		return manager.getGpuMemoryUsage();
	}

	Box3 getBoundingBox() override {
		return {manager.data.min, manager.data.max};
	}

#ifdef SPLATSHOP_HAS_ORBBEC
	// Copy a freshly captured RGB point-cloud frame into the host buffers.
	// `xyzrgb` is the OB_FORMAT_RGB_POINT buffer: an array of OBColorPoint
	// {float x, y, z, r, g, b}. The per-point stride and field offsets are
	// derived from the SDK's own OBColorPoint definition via sizeof/offsetof,
	// so a layout change in a future SDK is picked up automatically.
	// Called on the main thread.
	void loadPointCloud(const uint8_t* xyzrgb, int64_t pointCount) {
		if (pointCount <= 0) return;

		constexpr int64_t stride   = sizeof(OBColorPoint);
		constexpr int64_t posBytes = sizeof(float) * 3;
		constexpr int64_t rOff     = offsetof(OBColorPoint, r);
		constexpr int64_t gOff     = offsetof(OBColorPoint, g);
		constexpr int64_t bOff     = offsetof(OBColorPoint, b);

		// (Re)allocate host buffers when the capacity changes.
		if (capacity != pointCount) {
			points->position = make_shared<Buffer>(posBytes * pointCount);
			points->color    = make_shared<Buffer>(4ll * pointCount);
			capacity = pointCount;
			points->numPoints       = pointCount;
			points->numPointsLoaded = pointCount;
		}

		auto* dstPos = reinterpret_cast<uint8_t*>(points->position->data);
		auto* dstCol = reinterpret_cast<uint8_t*>(points->color->data);
		const uint8_t* src = xyzrgb;
		for (int64_t i = 0; i < pointCount; ++i) {
			const uint8_t* p = src + stride * i;
			// position: 3 floats (x, y, z)
			std::memcpy(dstPos + posBytes * i, p, posBytes);
			// color: r,g,b are floats in [0,1]; convert to 8-bit + 0xff alpha.
			float rf, gf, bf;
			std::memcpy(&rf, p + rOff, sizeof(float));
			std::memcpy(&gf, p + gOff, sizeof(float));
			std::memcpy(&bf, p + bOff, sizeof(float));
			uint8_t rgba[4] = {
				(uint8_t)std::min(255.f, std::max(0.f, rf * 255.f)),
				(uint8_t)std::min(255.f, std::max(0.f, gf * 255.f)),
				(uint8_t)std::min(255.f, std::max(0.f, bf * 255.f)),
				0xff };
			std::memcpy(dstCol + 4ll * i, rgba, 4);
		}

		frameReady.store(true);
	}
#endif // SPLATSHOP_HAS_ORBBEC
};
