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
#include "GSPlyLoader.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

using namespace std;


struct SplatsyFilesLoader{

	

	static shared_ptr<SNSplats> loadSNSplats(string nodePath, Scene& scene){
		
		shared_ptr<Splats> splats = GSPlyLoader::load(nodePath);

		mat4 transform = mat4(1.0f);
		// if(j_node.contains("transform")){
		// 	for(int i = 0; i < 16; i++){
		// 		float value = j_node["transform"][i];
		// 		((float*)&transform)[i] = value;
		// 	}
		// }

		fs::path path = fs::path(nodePath);
		string name = path.filename().replace_extension("").string();
		shared_ptr<SNSplats> node = make_shared<SNSplats>(name, splats);
		// node->dmng.data.transform = transform;

		return node;
	}

	static int load(string path, Scene& scene, OrbitControls& controls){
		
		string filename = fs::path(path).filename().string();

		string jsonPath = path;
		string directory = path;

		if(fs::is_directory(path)){
			if(fs::exists(path + "/scene.json")){
				jsonPath = path + "/scene.json";
			}else{
				return 123;
			}
			directory = path;
		}else{
			directory = fs::path(path).parent_path().string();
		}


		string strJson = readTextFile(jsonPath);
		auto js = json::parse(strJson);

		// json j_scene = js["scene"];
		// for(json j_node : j_scene){
		// 	string type = j_node["type"];
			
		// 	if(type == "SNSplats"){
		// 		string name = j_node["name"];
		// 		string nodePath = format("{}/splats/{}.ply", directory, name);

		// 		auto node = loadSNSplats(nodePath, j_node, scene);
		// 		scene.world->children.push_back(node);
		// 	}
		// }

		vector<string> splatsfiles;
		vector<string> assetsfiles;
		if(fs::exists(directory + "/splats")){
			splatsfiles = listFiles(directory + "/splats");
		}
		if(fs::exists(directory + "/assets")){
			assetsfiles = listFiles(directory + "/assets");
		}

		for(string path : splatsfiles){
			if(iEndsWith(path, ".ply")){
				auto node = loadSNSplats(path, scene);
				scene.world->children.push_back(node);
			}
		}

		for(string path : assetsfiles){
			if(iEndsWith(path, ".ply")){
				auto node = loadSNSplats(path, scene);
				node->dmng.data.writeDepth = false;
				node->hidden = true;
				AssetLibrary::assets.push_back(node);
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

		int i = 0;
		scene.process<SNSplats>([&](SNSplats* node){
			node->selected = (i == 0);
			i++;
		});

		return 0;
	}

};