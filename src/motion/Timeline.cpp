#include "Timeline.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <format>

#include "json/json.hpp"

using json = nlohmann::json;

namespace motion {

// Sample a single track at time t. Keyframes are assumed sorted by time.
TransformSample sampleTrack(const TransformTrack& track, double t) {
	if(track.frames.empty()) return TransformSample::identity();

	// Clamp / wrap to frame range.
	if(t <= track.frames.front().time) return track.frames.front().sample;
	if(t >= track.frames.back().time)  return track.frames.back().sample;

	// Find the surrounding keyframe pair.
	size_t i = 0;
	while(i + 1 < track.frames.size() && track.frames[i + 1].time < t) ++i;
	const TransformKeyframe& a = track.frames[i];
	const TransformKeyframe& b = track.frames[i + 1];

	double span = b.time - a.time;
	float f = span > 0.0 ? static_cast<float>((t - a.time) / span) : 0.0f;

	switch(track.mode) {
		case InterpMode::Step:
			return a.sample;
		case InterpMode::Linear: {
			TransformSample out;
			out.translation = mix(a.sample.translation, b.sample.translation, f);
			// Linear interpolation of quaternions is cheap but not normalized;
			// keep it as an explicit option.
			quat q;
			q.x = mix(a.sample.rotation.x, b.sample.rotation.x, f);
			q.y = mix(a.sample.rotation.y, b.sample.rotation.y, f);
			q.z = mix(a.sample.rotation.z, b.sample.rotation.z, f);
			q.w = mix(a.sample.rotation.w, b.sample.rotation.w, f);
			out.rotation = quatNormalize(q);
			out.scale = mix(a.sample.scale, b.sample.scale, f);
			return out;
		}
		case InterpMode::Slerp:
		default: {
			TransformSample out;
			out.translation = mix(a.sample.translation, b.sample.translation, f);
			out.rotation = slerpQuat(a.sample.rotation, b.sample.rotation, f);
			out.scale = mix(a.sample.scale, b.sample.scale, f);
			return out;
		}
	}
}

void Timeline::recomputeDuration() {
	double maxT = 0.0;
	for(const auto& track : tracks) {
		if(!track.frames.empty())
			maxT = std::max(maxT, track.frames.back().time);
	}
	duration = maxT;
}

void Timeline::update(Scene& scene, double dt) {
	if(!playing || tracks.empty()) return;

	playhead += dt;
	if(playhead >= duration) {
		if(loop) {
			playhead = std::fmod(playhead, duration);
		} else {
			playhead = duration;
			playing = false;
		}
	}

	for(const auto& track : tracks) {
		TransformSample s = sampleTrack(track, playhead);
		MotionController::setTransform(scene, track.target, s);
	}
}

// --- JSON loading ---------------------------------------------------------

static InterpMode parseMode(const std::string& s) {
	if(s == "step")   return InterpMode::Step;
	if(s == "linear") return InterpMode::Linear;
	return InterpMode::Slerp;
}

static vec3 readVec3(const json& a) {
	return vec3(a.at(0).get<float>(), a.at(1).get<float>(), a.at(2).get<float>());
}

static quat readQuat(const json& a) {
	// Stored as [x, y, z, w].
	return quat(a.at(3).get<float>(), a.at(0).get<float>(),
	            a.at(1).get<float>(), a.at(2).get<float>());
}

bool Timeline::loadTracksFromJSON(Scene& scene, const std::string& path) {
	std::ifstream in(path);
	if(!in.is_open()) {
		std::println("Timeline: could not open {}", path);
		return false;
	}

	json root;
	try {
		in >> root;
	} catch(const std::exception& e) {
		std::println("Timeline: JSON parse error in {}: {}", path, e.what());
		return false;
	}

	tracks.clear();
	playhead = 0.0;

	if(!root.contains("tracks") || !root["tracks"].is_array()) {
		std::println("Timeline: missing \"tracks\" array in {}", path);
		return false;
	}

	for(const auto& jt : root["tracks"]) {
		TransformTrack track;
		std::string nodeName = jt.value("nodeName", "");
		SceneNode* node = scene.find(nodeName);
		if(!node) {
			std::println("Timeline: node \"{}\" not found, skipping track", nodeName);
			continue;
		}
		track.target = node->ID;
		track.mode = parseMode(jt.value("mode", "slerp"));

		if(!jt.contains("frames") || !jt["frames"].is_array()) continue;
		for(const auto& jf : jt["frames"]) {
			TransformKeyframe kf;
			kf.time = jf.value("t", 0.0);
			if(jf.contains("translation")) kf.sample.translation = readVec3(jf["translation"]);
			if(jf.contains("rotation"))    kf.sample.rotation = readQuat(jf["rotation"]);
			if(jf.contains("scale"))       kf.sample.scale = readVec3(jf["scale"]);
			track.frames.push_back(std::move(kf));
		}
		std::sort(track.frames.begin(), track.frames.end(),
		          [](const TransformKeyframe& a, const TransformKeyframe& b){
		              return a.time < b.time;
		          });
		tracks.push_back(std::move(track));
	}

	recomputeDuration();
	if(root.contains("duration")) duration = root["duration"].get<double>();
	return true;
}

} // namespace motion
