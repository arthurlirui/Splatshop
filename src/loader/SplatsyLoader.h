#pragma once


#include <cmath>
#include <iostream>
#include <print>
#include <format>
#include <memory>
#include <string>
#include <sstream>

#include "unsuck.hpp"

#include <glm/gtx/quaternion.hpp>
#include "json/json.hpp"

#include "Splats.h"
#include "./scene/SceneNode.h"
#include "./scene/SNSplats.h"
#include "./scene/Scene.h"
#include "AssetLibrary.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

using namespace std;


struct SplatsyLoader{

	static shared_ptr<SNSplats> loadSNSplats(string path, json j_node, Scene& scene){

		int64_t count = j_node["count"];
		string name = j_node["name"];

		json j_position = j_node["attributes"]["position"];
		json j_scale    = j_node["attributes"]["scale"];
		json j_rotation = j_node["attributes"]["rotation"];
		json j_color    = j_node["attributes"]["color"];

		shared_ptr<Splats> splats = make_shared<Splats>();
		splats->name = name;
		splats->numSplats = count;
		splats->numSplatsLoaded = count;
		splats->position = readBinaryFile(path, j_position["byteOffset"], j_position["byteLength"]);
		splats->scale    = readBinaryFile(path, j_scale["byteOffset"], j_scale["byteLength"]);
		splats->rotation = readBinaryFile(path, j_rotation["byteOffset"], j_rotation["byteLength"]);
		splats->color    = readBinaryFile(path, j_color["byteOffset"], j_color["byteLength"]);

		mat4 transform = mat4(1.0f);
		if(j_node.contains("transform")){
			for(int i = 0; i < 16; i++){
				float value = j_node["transform"][i];
				((float*)&transform)[i] = value;
			}
		}

		shared_ptr<SNSplats> node = make_shared<SNSplats>(name, splats);
		node->dmng.data.transform = transform;

		// scene.root->children.push_back(node);
		return node;

	}

	static void load(string path, Scene& scene, OrbitControls& controls){

		auto headerSizeBuffer = readBinaryFile(path, 0, 4);
		if(!headerSizeBuffer){
			println("ERROR: failed to read header size from '{}'", path);
			return;
		}
		int64_t headerSize = headerSizeBuffer->get<int32_t>(0);

		int64_t filesize = fs::file_size(path);
		if(headerSize <= 0 || headerSize > filesize){
			println("ERROR: invalid header size {} in '{}'", headerSize, path);
			return;
		}
		auto jsonBuffer = readBinaryFile(path, filesize - headerSize, headerSize);
		if(!jsonBuffer){
			println("ERROR: failed to read JSON header from '{}'", path);
			return;
		}
		string strJson = string((const char*)(jsonBuffer->data), headerSize);

		println("Loaded Json: '{}'", strJson);

		auto js = json::parse(strJson);

		json j_scene = js["scene"];
		for(json j_node : j_scene){
			string type = j_node["type"];

			if(type == "SNSplats"){
				auto node = loadSNSplats(path, j_node, scene);
				scene.world->children.push_back(node);
			}
		}

		if(js.contains("assets")){
			for(json j_node : js["assets"]){
				string type = j_node["type"];

				if(type == "SNSplats"){
					auto node = loadSNSplats(path, j_node, scene);
					node->dmng.data.writeDepth = false;
					node->hidden = true;
					AssetLibrary::assets.push_back(node);
				}
			}
		}

		if(js.contains("camera")){
			json j_camera = js["camera"];
			controls.yaw = j_camera["yaw"];
			controls.pitch = j_camera["pitch"];
			controls.radius = j_camera["radius"];
			controls.target.x = j_camera["target"][0];
			controls.target.y = j_camera["target"][1];
			controls.target.z = j_camera["target"][2];
		}

	
	}

};