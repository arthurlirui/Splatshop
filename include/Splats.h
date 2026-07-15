#pragma once


#include <cmath>
#include <iostream>
#include <print>
#include <format>
#include <memory>
#include <string>
#include <atomic>

#include "unsuck.hpp"

#include <glm/gtx/quaternion.hpp>

using namespace std;

struct Splats {

	string name;

	int64_t numSplats;
	int64_t bytesPerSplat;
	int64_t headerSize;
	int64_t numSHCoefficients;
	int64_t shDegree;

	// incremented concurrently by loader threads; use atomic to avoid data races.
	std::atomic<int64_t> numSplatsLoaded;

	glm::mat4 world;

	vec3 min = {Infinity, Infinity, Infinity};
	vec3 max = {-Infinity, -Infinity, -Infinity};

	shared_ptr<Buffer> position   = nullptr;
	shared_ptr<Buffer> scale      = nullptr;
	shared_ptr<Buffer> rotation   = nullptr;
	shared_ptr<Buffer> color      = nullptr;
	shared_ptr<Buffer> SHs        = nullptr;
	shared_ptr<Buffer> flags      = nullptr;

	static shared_ptr<Splats> createSphere(){

		int segments = 24;
		int numSplats = segments * segments;

		shared_ptr<Splats> splats = make_shared<Splats>();
		splats->name = "sphere";
		splats->numSplats = numSplats;
		splats->numSplatsLoaded = numSplats;
		splats->position = make_shared<Buffer>(12 * numSplats);
		splats->scale    = make_shared<Buffer>(12 * numSplats);
		splats->rotation = make_shared<Buffer>(16 * numSplats);
		splats->color    = make_shared<Buffer>(16 * numSplats);

		for(int i = 0; i < segments; i++)
		for(int j = 0; j < segments; j++)
		{
			int splatIndex = i + segments * j;

			float u = float(i) / float(segments);
			float v = float(j) / float(segments);

			float rxy = cos(3.1415f * (2.0f * v - 1.0f));
			float x = rxy * cos(2.0f * 3.1415 * u);
			float y = rxy * sin(2.0f * 3.1415 * u);
			float z = sin(3.1415f * (2.0f * v - 1.0f));

			splats->position->set<float>(x, 12 * splatIndex + 0);
			splats->position->set<float>(y, 12 * splatIndex + 4);
			splats->position->set<float>(z, 12 * splatIndex + 8);

			splats->scale->set<float>(0.01f, 12 * splatIndex + 0);
			splats->scale->set<float>(0.01f, 12 * splatIndex + 4);
			splats->scale->set<float>(0.01f, 12 * splatIndex + 8);

			splats->rotation->set<float>(1.0f, 16 * splatIndex +  0);
			splats->rotation->set<float>(0.0f, 16 * splatIndex +  4);
			splats->rotation->set<float>(0.0f, 16 * splatIndex +  8);
			splats->rotation->set<float>(0.0f, 16 * splatIndex + 12);

			splats->color->set<float>(1.0f, 16 * splatIndex +  0);
			splats->color->set<float>(0.0f, 16 * splatIndex +  4);
			splats->color->set<float>(0.0f, 16 * splatIndex +  8);
			splats->color->set<float>(1.0f, 16 * splatIndex + 12);
		}

		return splats;
	}

};