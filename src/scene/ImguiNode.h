#pragma once

#include <string>
#include <format>
#include "cuda.h"
#include "cuda_runtime.h"

#include <glm/glm.hpp>
#include <glm/common.hpp>

#include "GLRenderer.h"
#include "HostDeviceInterface.h"
#include "gui/ImguiPage.h"
#include "SNTriangles.h"

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


struct ImguiNode : public SceneNode {

	ImguiPage* page = nullptr;
	SNTriangles* mesh = nullptr;

	ImguiNode(string name)
		: SceneNode(name)
	{
		page = new ImguiPage();
		mesh = new SNTriangles(name);

		auto plane = Mesh::createPlane(512);
		mesh->set(plane->position, plane->uv);

		this->hidden = true;
		this->visible = true;
	}

	~ImguiNode(){
		println("Destroying ImguiNode node {}", name);

		// Free raw-pointer owned resources to prevent memory leaks.
		delete page;
		page = nullptr;

		delete mesh;
		mesh = nullptr;
	}

};