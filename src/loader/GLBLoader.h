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

#include "json/json.hpp"

#include "ImageLoader.h"

// #if !defined(STB_IMAGE_WRITE_IMPLEMENTATION)
// 	#define STB_IMAGE_WRITE_IMPLEMENTATION
// 	#define STB_IMAGE_IMPLEMENTATION
// 	#include "stb/stb_image.h"
// 	#include "stb/stb_image_write.h"
// #endif

using namespace std;

namespace fs = std::filesystem;
using json = nlohmann::json;
using glm::ivec2;


struct GLB{

	vector<vec3> positions;
	vector<vec2> uvs;
	// vector<vec3> normals;

	ivec2 textureSize;
	shared_ptr<Buffer> texture;

};

struct GLBLoader{

	static GLB load(string path){

		auto buffer = readBinaryFile(path);

		if(!buffer){
			println("ERROR: failed to load GLB file '{}'", path);
			GLB empty;
			empty.textureSize = {0, 0};
			return empty;
		}

		uint32_t magic = buffer->get<uint32_t>(0);
		uint32_t version = buffer->get<uint32_t>(4);
		uint32_t length = buffer->get<uint32_t>(8);
		uint32_t chunkSize = buffer->get<uint32_t>(12);
		uint32_t chunkType = buffer->get<uint32_t>(16);

		println("magic: {}", magic);
		println("version: {}", version);
		println("length: {}", length);
		println("chunkSize: {}", chunkSize);
		println("chunkType: {}", chunkType);

		string strJson = string((const char*)(buffer->ptr + 20), chunkSize);
		auto js = json::parse(strJson);

		string dmp = js.dump(4);
		println("{}", dmp);


		println("views: {}", js["bufferViews"].size());

		int64_t binaryChunkOffset = 12 + 8 + chunkSize + 8;
		int64_t binaryChunkSize = buffer->get<uint32_t>(20 + chunkSize);

		for (int i = 0; i < js["bufferViews"].size(); i++) {
			int64_t byteOffset = binaryChunkOffset  + static_cast<int64_t>(js["bufferViews"][i]["byteOffset"]);
			int64_t byteLength = js["bufferViews"][i]["byteLength"];
			println("view {}, byteOffset: {}, byteLength: {}", i, byteOffset, byteLength);
		}

		auto glbMesh = js["meshes"][0];
		auto glbPrimitive = glbMesh["primitives"][0];

		vector<vec3> positions;
		vector<vec2> uvs;

		vector<uint32_t> indices;
		if(glbPrimitive.contains("indices")){
			int accessorRef = glbPrimitive["indices"];
			int64_t byteOffset = binaryChunkOffset + static_cast<int64_t>(js["bufferViews"][accessorRef]["byteOffset"]);
			int64_t byteLength = js["bufferViews"][accessorRef]["byteLength"];

			int64_t numIndices = byteLength / 4;
			indices.reserve(numIndices);

			for(int i = 0; i < numIndices; i++){
				uint32_t index = buffer->get<uint32_t>(byteOffset + 4 * i);
				indices.push_back(index);
			}

			println("has indices. byteOffset: {}, byteLength: {}", byteOffset, byteLength);
		} 

		for (auto& element : glbPrimitive["attributes"].items()) {

			string attributeName = element.key();

			int accessorRef = glbPrimitive["attributes"][attributeName];
			int64_t byteOffset = binaryChunkOffset  + static_cast<int64_t>(js["bufferViews"][accessorRef]["byteOffset"]);
			int64_t byteLength = js["bufferViews"][accessorRef]["byteLength"];

			if(attributeName == "POSITION"){
				for(int i = 0; i < indices.size(); i++){
					int64_t index = indices[i];
					vec3 pos = buffer->get<vec3>(byteOffset + 12 * index);
					positions.push_back(pos);
				}
			}else if(attributeName == "TEXCOORD_0"){
				for(int i = 0; i < indices.size(); i++){
					int64_t index = indices[i];
					vec2 uv = buffer->get<vec2>(byteOffset + 8 * index);
					uvs.push_back(uv);
				}
			}
		}

		while(uvs.size() < positions.size()){
			vec2 uv = {0.5f, 0.5f};

			uvs.push_back(uv);
		}

		shared_ptr<Buffer> texture = make_shared<Buffer>(4 * 512 * 512);
		for(int x = 0; x < 512; x++)
		for(int y = 0; y < 512; y++)
		{
			uint8_t r = x / 2;
			uint8_t g = y / 2;
			uint8_t b = 100;

			int texelID = x + y * 512;

			texture->set<uint8_t>(  r, 4 * texelID + 0);
			texture->set<uint8_t>(  g, 4 * texelID + 1);
			texture->set<uint8_t>(  b, 4 * texelID + 2);
			texture->set<uint8_t>(255, 4 * texelID + 3);
		}

		GLB glb;
		glb.positions = positions;
		glb.uvs = uvs;
		glb.textureSize = {512, 512};
		glb.texture = texture;

		
		if(js.contains("images") && js["images"].size() > 0){
			auto js_image = js["images"][0];
			int bufferviewIndex = js_image["bufferView"];
			int64_t byteOffset = binaryChunkOffset  + static_cast<int64_t>(js["bufferViews"][bufferviewIndex]["byteOffset"]);
			int64_t byteLength = static_cast<int64_t>(js["bufferViews"][bufferviewIndex]["byteLength"]);
			
			int width;
			int height;
			int n;
			uint8_t* data = stbi_load_from_memory(buffer->ptr + byteOffset, byteLength, &width, &height, &n, 4);

			if(data != nullptr){
				glb.textureSize = { width, height };
				glb.texture = make_shared<Buffer>(width * height * 4);

				memcpy(glb.texture->data, data, width * height * 4);

				stbi_image_free(data);
			}else{
				println("WARNING: stbi_load_from_memory failed for GLB image, using fallback texture");
			}
		}

		return glb;
	}

	static void load(string path, function<void(GLB)> onMeshParsed){

		auto buffer = readBinaryFile(path);

		if(!buffer){
			println("ERROR: failed to load GLB file '{}'", path);
			return;
		}

		uint32_t magic = buffer->get<uint32_t>(0);
		uint32_t version = buffer->get<uint32_t>(4);
		uint32_t length = buffer->get<uint32_t>(8);
		uint32_t chunkSize = buffer->get<uint32_t>(12);
		uint32_t chunkType = buffer->get<uint32_t>(16);

		println("magic: {}", magic);
		println("version: {}", version);
		println("length: {}", length);
		println("chunkSize: {}", chunkSize);
		println("chunkType: {}", chunkType);

		string strJson = string((const char*)(buffer->ptr + 20), chunkSize);
		auto js = json::parse(strJson);

		string dmp = js.dump(4);
		println("{}", dmp);


		println("views: {}", js["bufferViews"].size());

		int64_t binaryChunkOffset = 12 + 8 + chunkSize + 8;
		int64_t binaryChunkSize = buffer->get<uint32_t>(20 + chunkSize);

		for (int i = 0; i < js["bufferViews"].size(); i++) {
			int64_t byteOffset = binaryChunkOffset  + static_cast<int64_t>(js["bufferViews"][i]["byteOffset"]);
			int64_t byteLength = js["bufferViews"][i]["byteLength"];
			println("view {}, byteOffset: {}, byteLength: {}", i, byteOffset, byteLength);
		}

		int numMeshes = js["meshes"].size();
		
		for(int meshIndex = 0; meshIndex < numMeshes; meshIndex++){
			auto glbMesh = js["meshes"][meshIndex];
			// auto glbPrimitive = glbMesh["primitives"][0];

			for (auto glbPrimitive : glbMesh["primitives"]) {
				vector<vec3> positions;
				vector<vec2> uvs;

				vector<uint32_t> indices;
				if(glbPrimitive.contains("indices")){
					int accessorRef = glbPrimitive["indices"];
					int64_t byteOffset = binaryChunkOffset + static_cast<int64_t>(js["bufferViews"][accessorRef]["byteOffset"]);
					int64_t byteLength = js["bufferViews"][accessorRef]["byteLength"];

					int64_t numIndices = byteLength / 4;
					indices.reserve(numIndices);

					for(int i = 0; i < numIndices; i++){
						uint32_t index = buffer->get<uint32_t>(byteOffset + 4 * i);
						indices.push_back(index);
					}

					println("has indices. byteOffset: {}, byteLength: {}", byteOffset, byteLength);
				} 

				for (auto& element : glbPrimitive["attributes"].items()) {

					string attributeName = element.key();

					int accessorRef = glbPrimitive["attributes"][attributeName];
					int64_t byteOffset = binaryChunkOffset  + static_cast<int64_t>(js["bufferViews"][accessorRef]["byteOffset"]);
					int64_t byteLength = js["bufferViews"][accessorRef]["byteLength"];

					if(attributeName == "POSITION"){
						for(int i = 0; i < indices.size(); i++){
							int64_t index = indices[i];
							vec3 pos = buffer->get<vec3>(byteOffset + 12 * index);
							positions.push_back(pos);
						}
					}else if(attributeName == "TEXCOORD_0"){
						for(int i = 0; i < indices.size(); i++){
							int64_t index = indices[i];
							vec2 uv = buffer->get<vec2>(byteOffset + 8 * index);
							uvs.push_back(uv);
						}
					}
				}

				while(uvs.size() < positions.size()){
					vec2 uv = {0.5f, 0.5f};

					uvs.push_back(uv);
				}

				shared_ptr<Buffer> texture = make_shared<Buffer>(4 * 512 * 512);
				for(int x = 0; x < 512; x++)
				for(int y = 0; y < 512; y++)
				{
					uint8_t r = x / 2;
					uint8_t g = y / 2;
					uint8_t b = 100;

					int texelID = x + y * 512;

					texture->set<uint8_t>(  r, 4 * texelID + 0);
					texture->set<uint8_t>(  g, 4 * texelID + 1);
					texture->set<uint8_t>(  b, 4 * texelID + 2);
					texture->set<uint8_t>(255, 4 * texelID + 3);
				}

				GLB glb;
				glb.positions = positions;
				glb.uvs = uvs;
				glb.textureSize = {512, 512};
				glb.texture = texture;

				int imageIndex = -1;
				if(glbPrimitive.contains("material") && js.contains("materials")){
					int materialIndex = glbPrimitive["material"];
					auto material = js["materials"][materialIndex];

					if(material.contains("pbrMetallicRoughness")){
						auto whatever = material["pbrMetallicRoughness"];
						if(whatever.contains("baseColorTexture")){
							auto baseColorTexture = whatever["baseColorTexture"];
							imageIndex = baseColorTexture["index"];
						}
					}
				}


				
				if(imageIndex >= 0 && js.contains("images") && imageIndex < static_cast<int>(js["images"].size()))
				{
					auto js_image = js["images"][imageIndex];
					int bufferviewIndex = js_image["bufferView"];
					int64_t byteOffset = binaryChunkOffset  + static_cast<int64_t>(js["bufferViews"][bufferviewIndex]["byteOffset"]);
					int64_t byteLength = static_cast<int64_t>(js["bufferViews"][bufferviewIndex]["byteLength"]);
					
					int width;
					int height;
					int n;
					uint8_t* data = stbi_load_from_memory(buffer->ptr + byteOffset, byteLength, &width, &height, &n, 4);

					if(data != nullptr){
						glb.textureSize = { width, height };
						glb.texture = make_shared<Buffer>(width * height * 4);

						memcpy(glb.texture->data, data, width * height * 4);

						stbi_image_free(data);
					}else{
						println("WARNING: stbi_load_from_memory failed for GLB image, using fallback texture");
					}
				}

				onMeshParsed(glb);
			}

			
		}

	}


};