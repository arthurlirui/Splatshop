#pragma once

// Timeline: a lightweight keyframe-driven animation player.
//
// Tracks hold a sorted list of keyframes keyed by time (seconds). Each frame
// the Timeline advances its internal clock and, for each track, samples the
// interpolated value at the current playhead time and pushes it to the matching
// controller (MotionController for rigid transforms).
//
// The Timeline is intentionally host-only and CPU-driven; it writes node
// transforms via MotionController, which the existing render pipeline picks up
// through Scene::updateTransformations().
//
// External input: loadTracksFromJSON() reads a simple schema (see .cpp) so
// animations authored outside the editor can be played back.

#include <vector>
#include <string>
#include <memory>

#include "../scene/SceneNode.h"
#include "../scene/Scene.h"
#include "../tween.h"
#include "MotionTypes.h"
#include "MotionController.h"

namespace motion {

enum class InterpMode {
	Step,
	Linear,
	Slerp,   // quaternion-aware rotation + linear translation/scale
};

struct TransformKeyframe {
	double time = 0.0;
	TransformSample sample;
};

struct TransformTrack {
	NodeID target = -1;
	InterpMode mode = InterpMode::Slerp;
	std::vector<TransformKeyframe> frames;
};

class Timeline {
public:
	bool playing = false;
	bool loop = false;
	double playhead = 0.0;       // seconds
	double duration = 0.0;       // seconds, derived from tracks

	std::vector<TransformTrack> tracks;

	Timeline() = default;

	void clear() {
		tracks.clear();
		playhead = 0.0;
		duration = 0.0;
	}

	void addTrack(TransformTrack track) {
		tracks.push_back(std::move(track));
		recomputeDuration();
	}

	// Advance the playhead by dt seconds and apply sampled transforms to the scene.
	void update(Scene& scene, double dt);

	// Load rigid-transform tracks from a JSON file. Schema (see .cpp for details):
	// { "duration": <seconds>, "tracks": [ { "nodeName": "..", "mode": "slerp",
	//   "frames": [ { "t": 0.0, "translation":[x,y,z], "rotation":[x,y,z,w],
	//                 "scale":[x,y,z] }, ... ] } ] }
	bool loadTracksFromJSON(Scene& scene, const std::string& path);

private:

	void recomputeDuration();
};

TransformSample sampleTrack(const TransformTrack& track, double t);

} // namespace motion
