#pragma once

// Point-cloud PLY loader.
//
// Distinct from GSPlyLoader (which reads 3D-Gaussian-Splatting PLYs carrying
// scale/rotation/SH attributes). This loader reads plain point-cloud PLYs whose
// vertex element has at least {x, y, z} and optionally {r/red, g/green, b/blue}
// in 8-bit or 16-bit form, in ASCII or binary_little_endian format. Produces the
// standard shared_ptr<Points> consumed by SNPoints, matching LASLoader.h's
// incremental-upload contract.
//
// Detection from a drag&drop: a .ply is treated as a *point cloud* (routed here)
// when it has no scale_*/rot_*/f_dc_*/f_rest_* properties; otherwise the existing
// GSPlyLoader path handles it as gaussian splats. See main.cpp drop callback.

#include <cmath>
#include <iostream>
#include <filesystem>
#include <print>
#include <format>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "unsuck.hpp"
#include "Points.h"

#include <glm/gtx/quaternion.hpp>

using namespace std;
namespace fs = std::filesystem;

struct PointCloudPlyHeader {
	int64_t numVertices = 0;
	int64_t headerSize  = 0;          // byte offset to start of data section
	bool isAscii        = false;

	// Byte offsets of each property within one vertex record (binary) or column
	// indices (ascii). -1 means "absent".
	int64_t offX = -1, offY = -1, offZ = -1;
	int64_t offR = -1, offG = -1, offB = -1;
	int64_t bytesPerVertex = 0;
	bool colorIs16bit = false;        // uint16 rgb vs uint8 rgb

	// Splat-PLY detection: true if the file has gaussian-splat properties.
	bool isSplatPly = false;
};

struct PointCloudPlyLoader {

	// Map a PLY property type name to its byte size.
	static int typeSize(const string& type) {
		if (type == "char"   || type == "uchar"  || type == "int8"   || type == "uint8")  return 1;
		if (type == "short"  || type == "ushort" || type == "int16"  || type == "uint16") return 2;
		if (type == "int"    || type == "uint"   || type == "int32"  || type == "uint32" || type == "float" || type == "float32") return 4;
		if (type == "double" || type == "float64") return 8;
		return 0;
	}

	static PointCloudPlyHeader readHeader(const string& path) {
		PointCloudPlyHeader h;

		auto data = readBinaryFile(path, 0, 64'000);
		if (!data) return h;

		// Work on a NUL-terminated copy for line parsing.
		string text(data->data_char, data->size);
		size_t endPos = text.find("end_header");
		if (endPos == string::npos) return h;
		size_t dataStart = text.find('\n', endPos);
		if (dataStart == string::npos) return h;
		h.headerSize = int64_t(dataStart) + 1;

		// Parse line by line up to end_header.
		istringstream ss(text.substr(0, endPos));
		string line;
		bool inVertex = false;
		int64_t offset = 0;
		int colIndex = 0;

		while (getline(ss, line)) {
			// Trim trailing \r.
			if (!line.empty() && line.back() == '\r') line.pop_back();

			if (line.rfind("format", 0) == 0) {
				h.isAscii = (line.find("ascii") != string::npos);
			} else if (line.rfind("element vertex", 0) == 0) {
				istringstream ls(line);
				string tok;
				ls >> tok >> tok >> h.numVertices;
				inVertex = true;
				offset = 0;
				colIndex = 0;
			} else if (line.rfind("element", 0) == 0) {
				// A different element (e.g. faces) starts -> vertex element done.
				inVertex = false;
			} else if (inVertex && line.rfind("property", 0) == 0) {
				istringstream ls(line);
				string tok, type, name;
				ls >> tok >> type >> name;
				int sz = typeSize(type);

				// Splat-PLY detection.
				if (name == "scale_0" || name == "rot_0" || name == "f_dc_0" || name == "f_rest_0") {
					h.isSplatPly = true;
				}

				int64_t myOffset = h.isAscii ? int64_t(colIndex) : offset;
				if      (name == "x" || name == "X") h.offX = myOffset;
				else if (name == "y" || name == "Y") h.offY = myOffset;
				else if (name == "z" || name == "Z") h.offZ = myOffset;
				else if (name == "r" || name == "red")   { h.offR = myOffset; h.colorIs16bit = (sz == 2); }
				else if (name == "g" || name == "green") { h.offG = myOffset; h.colorIs16bit = (sz == 2); }
				else if (name == "b" || name == "blue")  { h.offB = myOffset; h.colorIs16bit = (sz == 2); }

				if (!h.isAscii) offset += sz;
				colIndex++;
			}
		}

		h.bytesPerVertex = h.isAscii ? 0 : offset;
		return h;
	}

	static shared_ptr<Points> load(string path) {
		auto header = readHeader(path);
		if (header.numVertices == 0 || header.offX < 0 || header.offY < 0 || header.offZ < 0) {
			println("PointCloudPlyLoader: not a valid point-cloud PLY (missing x/y/z): {}", path);
			return nullptr;
		}
		if (header.isSplatPly) {
			println("PointCloudPlyLoader: '{}' looks like a gaussian-splat PLY; use the splat loader instead.", path);
			return nullptr;
		}

		println("PointCloudPlyLoader: {} vertices, ascii={}, color16bit={}", header.numVertices, header.isAscii, header.colorIs16bit);

		shared_ptr<Points> points = make_shared<Points>();
		points->name          = fs::path(path).filename().string();
		points->numPoints     = header.numVertices;
		points->bytesPerPoint = int(header.bytesPerVertex);
		points->headerSize    = int(header.headerSize);
		points->numPointsLoaded = 0;
		points->world         = glm::mat4(1.0f);

		points->position = make_shared<Buffer>(header.numVertices * 12);
		points->color    = make_shared<Buffer>(header.numVertices * 4);

		auto t = jthread([path, points, header]() {
			double tStart = now();
			int64_t n = header.numVertices;

			if (header.isAscii) {
				// Read the whole file (PLY ASCII point clouds are usually small).
				auto all = readBinaryFile(path, header.headerSize);
				if (!all) return;
				istringstream ss(string(all->data_char, all->size));
				for (int64_t i = 0; i < n; i++) {
					float x = 0, y = 0, z = 0;
					int r = 255, g = 255, b = 255;
					// Read all columns; pick the ones we care about by index.
					// We assume column order matches header order. Build a quick
					// read by tokenizing the line.
					string line;
					if (!getline(ss, line)) break;
					istringstream ls(line);
					vector<float> vals;
					string tok;
					while (ls >> tok) vals.push_back(stof(tok));

					auto getv = [&](int64_t idx) -> float { return (idx >= 0 && idx < int64_t(vals.size())) ? vals[idx] : 0.0f; };
					x = getv(header.offX); y = getv(header.offY); z = getv(header.offZ);
					if (header.colorIs16bit) {
						r = int(getv(header.offR)); g = int(getv(header.offG)); b = int(getv(header.offB));
						r = (r > 255) ? r / 256 : r;
						g = (g > 255) ? g / 256 : g;
						b = (b > 255) ? b / 256 : b;
					} else if (header.offR >= 0) {
						r = int(getv(header.offR)); g = int(getv(header.offG)); b = int(getv(header.offB));
					}
					storePoint(points, i, x, y, z, uint8_t(r), uint8_t(g), uint8_t(b));
				}
			} else {
				// Binary: stream in batches exactly like LASLoader.
				constexpr int batchSize = 500'000;
				Buffer raw(batchSize * header.bytesPerVertex);
				int cs = header.colorIs16bit ? 2 : 1;

				for (int64_t i = 0; i < n; i++) {
					if (i % batchSize == 0) {
						uint64_t start = header.headerSize + uint64_t(i) * header.bytesPerVertex;
						uint64_t remaining = (uint64_t(n) - i) * header.bytesPerVertex;
						uint64_t bytes = min(remaining, uint64_t(batchSize) * header.bytesPerVertex);
						readBinaryFile(path, start, bytes, raw.data);
					}
					int64_t src = (i % batchSize) * header.bytesPerVertex;
					float x = raw.get<float>(src + header.offX);
					float y = raw.get<float>(src + header.offY);
					float z = raw.get<float>(src + header.offZ);

					uint8_t r = 255, g = 255, b = 255;
					if (header.offR >= 0) {
						if (header.colorIs16bit) {
							uint16_t rv = raw.get<uint16_t>(src + header.offR);
							uint16_t gv = raw.get<uint16_t>(src + header.offG);
							uint16_t bv = raw.get<uint16_t>(src + header.offB);
							r = uint8_t(rv > 255 ? rv / 256 : rv);
							g = uint8_t(gv > 255 ? gv / 256 : gv);
							b = uint8_t(bv > 255 ? bv / 256 : bv);
						} else {
							r = raw.get<uint8_t>(src + header.offR);
							g = raw.get<uint8_t>(src + header.offG);
							b = raw.get<uint8_t>(src + header.offB);
						}
					}
					storePoint(points, i, x, y, z, r, g, b);
				}
			}

			double seconds = now() - tStart;
			println("PointCloudPlyLoader: loaded {:L} points in {:.3f}s", points->numPointsLoaded, seconds);
		});

		t.detach();
		return points;
	}

  private:
	static void storePoint(const shared_ptr<Points>& points, int64_t i,
	                       float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
		points->position->set(x, 12 * i + 0);
		points->position->set(y, 12 * i + 4);
		points->position->set(z, 12 * i + 8);
		points->color->set<uint8_t>(r,     4 * i + 0);
		points->color->set<uint8_t>(g,     4 * i + 1);
		points->color->set<uint8_t>(b,     4 * i + 2);
		points->color->set<uint8_t>(255,   4 * i + 3);

		points->min.x = min(points->min.x, x);
		points->min.y = min(points->min.y, y);
		points->min.z = min(points->min.z, z);
		points->max.x = max(points->max.x, x);
		points->max.y = max(points->max.y, y);
		points->max.z = max(points->max.z, z);

		points->numPointsLoaded++;
	}
};
