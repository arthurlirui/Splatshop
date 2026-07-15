#pragma once

#include <string>
#include <format>
#include "cuda.h"
#include "cuda_runtime.h"

#include <glm/glm.hpp>
#include <glm/common.hpp>

#include "GLRenderer.h"
#include "HostDeviceInterface.h"

using namespace std;
using glm::ivec2;
using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::mat3;
using glm::mat4;
using glm::quat;
using glm::dot;
using glm::transpose;
using glm::inverse;

typedef int64_t NodeID;

struct SceneNode{
	
	inline static NodeID counter = 0;
	int64_t ID;
	string name;

	// local transform of each matrix
	mat4 transform = mat4(1.0f);

	// global transform with transformations of parents applied to it.
	// updatated via scene.updateTransformations()
	mat4 transform_global = mat4(1.0f);


	bool visible = true;   // visible as in rendered
	bool locked = false;
	bool hidden = false;   // hidden from layers user interface
	bool selected = false;
	bool hovered = false;
	shared_ptr<GLTexture> thumbnail = nullptr;
	bool dirty = false;
	
	Box3 aabb;
	CUdeviceptr cptr_aabb = 0;
	void* h_aabb_pinned = nullptr;
	
	union{
		uint32_t color = 0x000000ff;
		uint8_t rgba[4];
	};

	vector<shared_ptr<SceneNode>> children;

	SceneNode(){
		ID = counter;
		this->name = format("node_{}", counter);

		counter++;
	}
	
	SceneNode(string name){
		ID = counter;
		this->name = name;

		counter++;
	}

	// Virtual destructor is required so that derived class destructors
	// (e.g. SNSplats::~SNSplats which frees GPU virtual memory) are invoked
	// when a node is destroyed via a SceneNode* pointer or shared_ptr<SceneNode>.
	virtual ~SceneNode(){
		children.clear();
	}

	void traverse(function<void(SceneNode*)> callback){
		callback(this);

		for(auto child : children){
			child->traverse(callback);
		}
	}

	virtual string toString(){
		return "SceneNode";
	}

	virtual uint64_t getGpuMemoryUsage(){
		return 0;
	}

	virtual Box3 getBoundingBox(){
		Box3 box;
		return box;
	}

	SceneNode* find(string name){

		SceneNode* found = nullptr;

		this->traverse([&](SceneNode* node){
			if(node->name == name){
				found = node;
			}
		});

		return found;
	}

	void remove(SceneNode* toRemove){
		for(int i = 0; i < children.size(); i++){
			if(children[i].get() == toRemove) {
				children.erase(children.begin() + i);
				return;
			}
		}
	}

};