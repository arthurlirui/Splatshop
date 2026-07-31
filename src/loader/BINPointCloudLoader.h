#pragma once

// .bin point-cloud loader — Skye fast path.
//
// Skye's tools/las2bin.js converts a LAS file into a raw 16-bytes-per-point
// binary: {float x, float y, float z, uint8 r, uint8 g, uint8 b, uint8 a}.
// Coordinates are already de-scaled/de-offset and shifted so the cloud sits near
// the origin (las2bin subtracts the source min). This format needs no parsing,
// so it streams at near-disk bandwidth (Skye reports ~100M points/s from NVMe).
//
// Reference: E:\Code\Skye\tools\las2bin.js and modules\progressive\
// ProgressiveBINLoader.h. We do not depend on Skye at build time; this is a
// native Splatshop loader that produces the standard shared_ptr<Points> that
// SNPoints consumes, matching the existing LASLoader.h structure.

#include <cmath>
#include <filesystem>
#include <print>
#include <memory>
#include <string>
#include <thread>

#include "unsuck.hpp"
#include "Points.h"

#include <glm/gtx/quaternion.hpp>

using namespace std;
namespace fs = std::filesystem;

struct BINPointCloudLoader {

	// .bin record layout from las2bin.js: 16 bytes, little-endian.
	// No alignas: the natural layout is already 16 bytes with no padding, and we
	// read it via Buffer::get (memcpy, alignment-agnostic).
	struct BinPoint {
		float    x, y, z;
		uint8_t  r, g, b, a;
	};
	static_assert(sizeof(BinPoint) == 16);

	static shared_ptr<Points> load(string path) {

		if (!fs::exists(path)) {
			println("BINPointCloudLoader: file not found: {}", path);
			return nullptr;
		}

		uint64_t fileSize = fs::file_size(path);
		uint64_t numPoints = fileSize / sizeof(BinPoint);

		println("BINPointCloudLoader: {} points ({} bytes)", numPoints, fileSize);

		shared_ptr<Points> points = make_shared<Points>();
		points->name         = fs::path(path).filename().string();
		points->numPoints    = int64_t(numPoints);
		points->bytesPerPoint = 16;
		points->headerSize    = 0;
		points->numPointsLoaded = 0;

		// Allocate host buffers in the layout PointData expects (separate
		// position (vec3) / color (uint32) arrays), then stream-convert the
		// interleaved .bin records into them on a background thread — same
		// incremental-upload contract as LASLoader.h, so SplatEditor_update.h
		// can upload chunks to the device as they arrive.
		points->position = make_shared<Buffer>(numPoints * 12);
		points->color    = make_shared<Buffer>(numPoints * 4);

		// No stored min in the .bin header; world = identity and we scan the
		// cloud's own min/max for the bounding box / camera focus.
		points->world = glm::mat4(1.0f);

		auto t = jthread([path, points, numPoints]() {
			double tStart = now();

			constexpr int batchSize = 500'000;            // points per disk read
			Buffer raw(batchSize * sizeof(BinPoint));

			for (uint64_t i = 0; i < numPoints; i++) {
				if (i % batchSize == 0) {
					uint64_t start = i * sizeof(BinPoint);
					uint64_t remaining = (numPoints - i) * sizeof(BinPoint);
					uint64_t bytes = min(remaining, uint64_t(batchSize * sizeof(BinPoint)));
					readBinaryFile(path, start, bytes, raw.data);
				}

				uint64_t srcOffset = (i % batchSize) * sizeof(BinPoint);
				BinPoint p = raw.get<BinPoint>(srcOffset);

				points->position->set(p.x, 12 * i + 0);
				points->position->set(p.y, 12 * i + 4);
				points->position->set(p.z, 12 * i + 8);

				// las2bin leaves alpha = 0; force opaque so points are visible.
				uint8_t a = (p.a == 0) ? uint8_t(255) : p.a;
				points->color->set<uint8_t>(p.r, 4 * i + 0);
				points->color->set<uint8_t>(p.g, 4 * i + 1);
				points->color->set<uint8_t>(p.b, 4 * i + 2);
				points->color->set<uint8_t>(a,   4 * i + 3);

				// Track world-space bounds for camera focus / culling.
				points->min.x = min(points->min.x, p.x);
				points->min.y = min(points->min.y, p.y);
				points->min.z = min(points->min.z, p.z);
				points->max.x = max(points->max.x, p.x);
				points->max.y = max(points->max.y, p.y);
				points->max.z = max(points->max.z, p.z);

				points->numPointsLoaded++;
			}

			double seconds = now() - tStart;
			println("BINPointCloudLoader: loaded {:L} points in {:.3f}s", points->numPointsLoaded, seconds);
		});

		t.detach();

		return points;
	}
};
