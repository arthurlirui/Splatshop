#pragma once

// MotionController: single-body rigid motion control interface.
//
// Each scene object (a SceneNode) is driven by writing its local transform
// (node->transform). The scene graph propagates parent->child transforms every
// frame via Scene::updateTransformations(), so changing node->transform here is
// picked up automatically on the next frame without touching any render kernel.
//
// Callers identify objects by SceneNode::ID (a stable int64_t assigned at node
// creation). The controller looks the node up through the Scene graph.

#include <functional>
#include <memory>

#include "../scene/Scene.h"
#include "../tween.h"
#include "MotionTypes.h"

namespace motion {

class MotionController {
public:
	// Resolve a node ID to a live SceneNode. Returns nullptr if not found.
	static SceneNode* resolve(Scene& scene, NodeID id) {
		SceneNode* found = nullptr;
		scene.root->traverse([&](SceneNode* node){
			if(!found && node->ID == id) found = node;
		});
		return found;
	}

	// --- Absolute setters -------------------------------------------------

	// Set the full local transform (translation, rotation, scale) of a node.
	static bool setTransform(Scene& scene, NodeID id, const TransformSample& sample) {
		SceneNode* node = resolve(scene, id);
		if(!node) return false;
		node->transform = sample.toMatrix();
		return true;
	}

	static bool setTranslation(Scene& scene, NodeID id, const vec3& t) {
		SceneNode* node = resolve(scene, id);
		if(!node) return false;
		TransformSample s = TransformSample::fromMatrix(node->transform);
		s.translation = t;
		node->transform = s.toMatrix();
		return true;
	}

	static bool setRotation(Scene& scene, NodeID id, const quat& r) {
		SceneNode* node = resolve(scene, id);
		if(!node) return false;
		TransformSample s = TransformSample::fromMatrix(node->transform);
		s.rotation = r;
		node->transform = s.toMatrix();
		return true;
	}

	static bool setScale(Scene& scene, NodeID id, const vec3& sc) {
		SceneNode* node = resolve(scene, id);
		if(!node) return false;
		TransformSample s = TransformSample::fromMatrix(node->transform);
		s.scale = sc;
		node->transform = s.toMatrix();
		return true;
	}

	// --- Relative mutators ------------------------------------------------

	static bool translate(Scene& scene, NodeID id, const vec3& delta) {
		SceneNode* node = resolve(scene, id);
		if(!node) return false;
		TransformSample s = TransformSample::fromMatrix(node->transform);
		s.translation += delta;
		node->transform = s.toMatrix();
		return true;
	}

	// Rotate about the node's local origin (pre-multiply rotation).
	static bool rotate(Scene& scene, NodeID id, const quat& delta) {
		SceneNode* node = resolve(scene, id);
		if(!node) return false;
		TransformSample s = TransformSample::fromMatrix(node->transform);
		s.rotation = delta * s.rotation;
		node->transform = s.toMatrix();
		return true;
	}

	static bool scaleBy(Scene& scene, NodeID id, const vec3& factor) {
		SceneNode* node = resolve(scene, id);
		if(!node) return false;
		TransformSample s = TransformSample::fromMatrix(node->transform);
		s.scale *= factor;
		node->transform = s.toMatrix();
		return true;
	}

	// --- Getter -----------------------------------------------------------

	static bool getTransform(Scene& scene, NodeID id, TransformSample& out) {
		SceneNode* node = resolve(scene, id);
		if(!node) return false;
		out = TransformSample::fromMatrix(node->transform);
		return true;
	}

	// --- Animated transition ---------------------------------------------
	//
	// Smoothly interpolate from the node's current transform to `target` over
	// `durationSeconds`, driven by the existing TWEEN system. This is the basic
	// building block for keyframe-free programmatic motion.

	static bool setTransformAnimated(Scene& scene, NodeID id,
	                                 const TransformSample& target,
	                                 double durationSeconds,
	                                 EaseMode ease = EaseMode::EaseInOut)
	{
		SceneNode* node = resolve(scene, id);
		if(!node || durationSeconds <= 0.0) {
			if(node) node->transform = target.toMatrix();
			return node != nullptr;
		}

		TransformSample start = TransformSample::fromMatrix(node->transform);
		// Hold a pointer copy; the node is expected to outlive this short tween.
		SceneNode* nodePtr = node;
		TWEEN::animate(durationSeconds, [=](double u){
			float f = applyEase(static_cast<float>(u), ease);
			TransformSample interp;
			interp.translation = mix(start.translation, target.translation, f);
			interp.rotation = slerpQuat(start.rotation, target.rotation, f);
			interp.scale = mix(start.scale, target.scale, f);
			nodePtr->transform = interp.toMatrix();
		});
		return true;
	}
};

} // namespace motion
