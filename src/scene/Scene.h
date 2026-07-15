#pragma once

#include <functional>
#include <deque>

using namespace std;

constexpr int MAX_SCENENODES = 100'000;

struct Scene {

	shared_ptr<SceneNode> root = nullptr;
	shared_ptr<SceneNode> world = nullptr;
	shared_ptr<SceneNode> vr = nullptr;
	shared_ptr<SceneNode> placingContainer = nullptr;
	 
	// Extra transformation of the scene.
	// Meant for VR, where we'd like to move, rotate and even scale the scene as part of the navigation.
	// mat4 transform = mat4(1.0f);

	Scene() {
		root = make_shared<SceneNode>("root");
		world = make_shared<SceneNode>("world");
		vr = make_shared<SceneNode>("vr");
		placingContainer = make_shared<SceneNode>("placingContainer");

		root->children.push_back(world);
		root->children.push_back(vr);
		root->children.push_back(placingContainer);
	}

	void process(string name, function<void(SceneNode*)> f){

		SceneNode* node = this->find(name);

		if(node){
			f(node);
		}
	}

	SceneNode* find(string name){

		SceneNode* found = nullptr;

		root->traverse([&](SceneNode* node){
			if(!found && node->name == name){
				found = node;
			}
		});

		return found;
	}

	SceneNode* find(const function<bool(SceneNode*)>& callback){

		SceneNode* found = nullptr;

		root->traverse([&](SceneNode* node){
			if(!found && callback(node)){
				found = node;
			}
		});

		return found;
	}

	// void traverse(const function<void(shared_ptr<SceneNode>)>& callback){



	// }

	void erase(shared_ptr<SceneNode> toErase){

		deque<shared_ptr<SceneNode>> queue;
		queue.push_back(root);

		while(!queue.empty()){

			shared_ptr<SceneNode> node = queue.front();
			queue.pop_front();

			int childToErase = -1;

			for(int i = 0; i < node->children.size(); i++){

				shared_ptr<SceneNode> child = node->children[i];

				if(child == toErase){
					childToErase = i;
				}
			}

			if(childToErase != -1){
				node->children.erase(node->children.begin() + childToErase);
				return;
			}

			for(shared_ptr<SceneNode> child : node->children){
				queue.push_back(child);
			}
		}

	}

	// void erase(const function<bool(SceneNode*)>& match){

	// 	root->traverse([&](SceneNode* node){
			
	// 		vector<int> childrenToErase;
			
	// 		for(int i = 0; i < node->children.size(); i++){
				
	// 			SceneNode* child = node->children[i].get();

	// 			if(match(child)){
	// 				childrenToErase.push_back(i);
	// 			}
	// 		}

	// 		for(int i = childrenToErase.size() - 1; i >= 0; i--){
	// 			int childIndex = childrenToErase[i];
	// 			node->children.erase(node->children.begin() + childIndex);
	// 		}

	// 	});
	// }

	void remove(shared_ptr<SceneNode> node){
		int tmpIndex = -1;
		for(int i = 0; i < root->children.size(); i++){
			if(root->children[i] == node){
				tmpIndex = i;
			}
		}

		if(tmpIndex != -1){
			root->children.erase(root->children.begin() + tmpIndex);
		}
	}

	template<typename T>
	void process(const function<void(T*)>& callback){
		this->forEach<T>(callback);
	}

	template<typename T>
	void forEach(const function<void(T*)>& callback){

		root->traverse([&](SceneNode* node){
			if(dynamic_cast<T*>(node) != nullptr){

				T* sn = dynamic_cast<T*>(node);
				callback(sn);
			}
		});
	}

	// Visit all nodes in scene graph
	template<typename T>
	void process(const function<void(shared_ptr<T>)>& callback){
		deque<shared_ptr<SceneNode>> queue;
		queue.push_back(root);

		while(!queue.empty()){

			shared_ptr<SceneNode> node = queue.front();
			queue.pop_front();


			if(dynamic_pointer_cast<T>(node) != nullptr){

				shared_ptr<T> sn = dynamic_pointer_cast<T>(node);
				callback(sn);
			}

			for(shared_ptr<SceneNode> child : node->children){
				queue.push_back(child);
			}
		}
	}

	void updateTransformations(){

		double t_start = now();

		function<void(SceneNode*, SceneNode*)> traverse;

		traverse = [&](SceneNode* parent, SceneNode* node){
			if (parent) {
				mat4 transform = parent->transform_global * node->transform;
				node->transform_global = transform;
			} else {
				node->transform_global = node->transform;
			}

			for(auto child : node->children){
				traverse(node, child.get());
			}

		};

		traverse(nullptr, root.get());
	}

	void deselectAllNodes(){
		process<SceneNode>([&](SceneNode* node){
			node->selected = false;
		});
	}

};