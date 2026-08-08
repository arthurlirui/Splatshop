#pragma once


#include <cmath>
#include <iostream>
#include <filesystem>
#include <print>
#include <format>
#include <memory>
#include <string>
#include <thread>

#include "unsuck.hpp"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/matrix_decompose.hpp>

#include "Points.h"

using namespace std;
using glm::vec3;
using glm::vec4;
using glm::translate;

namespace fs = std::filesystem;


struct LasHeader{
	int versionMajor           = 0;
	int versionMinor           = 0;
	int headerSize             = 0;
	uint64_t offsetToPointData = 0;
	int format                 = 0;
	uint64_t bytesPerPoint     = 0;
	uint64_t numPoints         = 0;
	vec3 scale;
	vec3 offset;
	vec3 min;
	vec3 max;
};

struct LasLoader{

	static LasHeader readHeader(string path){

		auto buffer = readBinaryFile(path, 0, 375);

		LasHeader header;

		header.versionMajor      = buffer->get<uint8_t>(24);
		header.versionMinor      = buffer->get<uint8_t>(25);
		header.headerSize        = buffer->get<uint16_t>(94);
		header.offsetToPointData = buffer->get<uint32_t>(96);
		header.format            = buffer->get<uint8_t>(104);
		header.bytesPerPoint     = buffer->get<uint16_t>(105);
		if(header.versionMajor == 1 && header.versionMinor <= 2){
			header.numPoints     = buffer->get<uint32_t>(107); 
		}else{
			header.numPoints     = buffer->get<uint64_t>(247); 
		}

		header.scale = {
			buffer->get<double>(131),
			buffer->get<double>(139),
			buffer->get<double>(147),
		};
		header.offset = {
			buffer->get<double>(155),
			buffer->get<double>(163),
			buffer->get<double>(171),
		};
		header.min = {
			buffer->get<double>(187),
			buffer->get<double>(203),
			buffer->get<double>(219),
		};
		header.max = {
			buffer->get<double>(179),
			buffer->get<double>(195),
			buffer->get<double>(211),
		};

		return header;
	}


	static shared_ptr<Points> load(string path){

		auto header = readHeader(path);

		println("numPoints:         {}", header.numPoints);
		println("bytes per point:   {}", header.bytesPerPoint);

		shared_ptr<Points> points = make_shared<Points>();

		points->name = fs::path(path).filename().string();

		points->numPointsLoaded   = 0;
		points->numPoints         = header.numPoints;
		points->bytesPerPoint     = header.bytesPerPoint;
		points->headerSize        = header.headerSize;
		
		points->position = make_shared<Buffer>(header.numPoints * 12);
		points->color    = make_shared<Buffer>(header.numPoints * 4);

		
		points->world = translate(header.min);
		
		auto t = jthread([header, path, points](){

			double t_start = now();

			int batchSize = 100'000;
			Buffer buffer(batchSize * header.bytesPerPoint);

			int rgbOffset = 0;
			if(header.format == 2) rgbOffset = 20;
			if(header.format == 3) rgbOffset = 28;
			if(header.format == 5) rgbOffset = 28;
			if(header.format == 7) rgbOffset = 30;

			int intensityOffset = 12;

			for(uint64_t i = 0; i < header.numPoints; i++){

				if(i % batchSize == 0){
					uint64_t start = header.offsetToPointData + i * header.bytesPerPoint;
					uint64_t remainingBytes = (header.numPoints - i) * header.bytesPerPoint;
					uint64_t batchByteSize = std::min(remainingBytes, uint64_t(batchSize * header.bytesPerPoint));
					readBinaryFile(path, start, batchByteSize, buffer.data);
				}

				uint64_t srcOffset = (i % batchSize) * header.bytesPerPoint;

				{ // POSITION
					// auto pos = glm::vec3(
					// 	buffer.get<float>(srcOffset + 0),
					// 	buffer.get<float>(srcOffset + 4),
					// 	buffer.get<float>(srcOffset + 8)
					// );

					vec3 pos = {
						double(buffer.get<int32_t>(srcOffset + 0)) * header.scale.x + header.offset.x - header.min.x,
						double(buffer.get<int32_t>(srcOffset + 4)) * header.scale.y + header.offset.y - header.min.y,
						double(buffer.get<int32_t>(srcOffset + 8)) * header.scale.z + header.offset.z - header.min.z,
					};

					points->min.x = min(points->min.x, pos.x + header.min.x);
					points->min.y = min(points->min.y, pos.y + header.min.y);
					points->min.z = min(points->min.z, pos.z + header.min.z);
					points->max.x = max(points->max.x, pos.x + header.min.x);
					points->max.y = max(points->max.y, pos.y + header.min.y);
					points->max.z = max(points->max.z, pos.z + header.min.z);

					// splats->mean.x += pos.x;
					// splats->mean.y += pos.y;
					// splats->mean.z += pos.z;
					
					points->position->set(pos.x, 12 * i + 0);
					points->position->set(pos.y, 12 * i + 4);
					points->position->set(pos.z, 12 * i + 8);
				}

				if(rgbOffset > 0)
				{ // COLOR
					uint16_t r = buffer.get<uint16_t>(srcOffset + rgbOffset + 0);
					uint16_t g = buffer.get<uint16_t>(srcOffset + rgbOffset + 2);
					uint16_t b = buffer.get<uint16_t>(srcOffset + rgbOffset + 4);

					r = r > 255 ? r / 256 : r;
					g = g > 255 ? g / 256 : g;
					b = b > 255 ? b / 256 : b;

					points->color->set<uint8_t>(uint8_t(r), 4 * i + 0);
					points->color->set<uint8_t>(uint8_t(g), 4 * i + 1);
					points->color->set<uint8_t>(uint8_t(b), 4 * i + 2);
				}else{
					points->color->set<uint8_t>(255, 4 * i + 0);
					points->color->set<uint8_t>(255, 4 * i + 1);
					points->color->set<uint8_t>(255, 4 * i + 2);
				}

				points->numPointsLoaded++;
			}

			double t_end = now();
			double seconds = t_end - t_start;

			println("loaded {:L} points in {:.3f} seconds.", points->numPointsLoaded.load(), seconds);
		});

		t.detach();

		return points;
	}


};