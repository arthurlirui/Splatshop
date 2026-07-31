
#include <cfloat>

#include "../motion/MotionTypes.h"
#include "../motion/MotionController.h"
#include "../motion/Timeline.h"
#include "../motion/RiggedHumanController.h"
#include "../motion/ProceduralRigSource.h"
#include "../scene/SNRiggedSplats.h"
#include "../scene/SN4DGSSplats.h"

using namespace motion;

// ARKit 52 blendshape names (canonical order), used to label the facial sliders.
static const char* kBsNames[BLENDSHAPE_COUNT] = {
	"browDownLeft","browDownRight","browInnerUp","browOuterUpLeft","browOuterUpRight",
	"cheekPuff","cheekSquintLeft","cheekSquintRight","eyeBlinkLeft","eyeBlinkRight",
	"eyeLookDownLeft","eyeLookDownRight","eyeLookInLeft","eyeLookInRight","eyeLookOutLeft",
	"eyeLookOutRight","eyeLookUpLeft","eyeLookUpRight","eyeSquintLeft","eyeSquintRight",
	"eyeWideLeft","eyeWideRight","jawForward","jawLeft","jawOpen","jawRight",
	"mouthClose","mouthDimpleLeft","mouthDimpleRight","mouthFrownLeft","mouthFrownRight",
	"mouthFunnel","mouthLeft","mouthLowerDownLeft","mouthLowerDownRight","mouthPressLeft",
	"mouthPressRight","mouthPucker","mouthRight","mouthRollLower","mouthRollUpper",
	"mouthShrugLower","mouthShrugUpper","mouthSmileLeft","mouthSmileRight","mouthStretchLeft",
	"mouthStretchRight","mouthUpperUpLeft","mouthUpperUpRight","noseSneerLeft","noseSneerRight",
	"tongueOut"
};

// Motion control panel.
//
// Exposes the MotionController / Timeline through an ImGui window:
//   - Numeric translation / rotation / scale editing for the currently selected
//     node, driving its local transform via MotionController.
//   - A small set of test-animation buttons that animate the selection.
//   - A basic timeline player (play / pause / loop / scrub / load JSON) backed
//     by editor->timeline.
//
// Human rig controls (joint sliders + ARKit 52 blendshape sliders) are added in
// a later step, once SNRiggedSplats / RiggedHumanController are in place.
void SplatEditor::makeMotionGUI(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& scene = editor->scene;

	if(!settings.showMotion) return;

	int width = 440;
	ImGui::SetNextWindowPos(ImVec2(40, 57 + 16));
	ImGui::SetNextWindowSize(ImVec2(width, 520));
	ImGui::SetNextWindowBgAlpha(1.0f);

	static bool open = true;
	ImGuiWindowFlags flags = ImGuiWindowFlags_NoBringToFrontOnFocus;
	if(ImGui::Begin("Motion", &open, flags)){

		shared_ptr<SceneNode> node = editor->getSelectedNode();
		if(!node){
			ImGui::TextDisabled("Select a node to control its motion.");
		}else{
			ImGui::Text("Node: %s (ID %lld)", node->name.c_str(), (long long)node->ID);

			TransformSample sample;
			MotionController::getTransform(scene, node->ID, sample);

			bool changed = false;

			// --- Translation -------------------------------------------------
			if(ImGui::CollapsingHeader("Translation", ImGuiTreeNodeFlags_DefaultOpen)){
				if(ImGui::DragFloat3("pos", &sample.translation.x, 0.05f, -FLT_MAX, FLT_MAX, "%.3f"))
					changed = true;
			}

			// --- Rotation (Euler degrees, editable) --------------------------
			if(ImGui::CollapsingHeader("Rotation", ImGuiTreeNodeFlags_DefaultOpen)){
				vec3 e = quatToEulerXYZ(sample.rotation) * 57.2957795130823f; // rad->deg
				if(ImGui::DragFloat3("rot (deg)", &e.x, 0.5f, -360.0f, 360.0f, "%.2f")){
					sample.rotation = quatFromEulerXYZ(e * 0.0174532925199433f); // deg->rad
					changed = true;
				}
			}

			// --- Scale -------------------------------------------------------
			if(ImGui::CollapsingHeader("Scale", ImGuiTreeNodeFlags_DefaultOpen)){
				if(ImGui::DragFloat3("scale", &sample.scale.x, 0.01f, 0.0001f, 1000.0f, "%.3f"))
					changed = true;
				ImGui::SameLine();
				if(ImGui::Button("Uniform##scaleuniform")){
					float u = (sample.scale.x + sample.scale.y + sample.scale.z) / 3.0f;
					sample.scale = vec3(u);
					changed = true;
				}
			}

			if(changed){
				MotionController::setTransform(scene, node->ID, sample);
			}

			ImGui::Separator();

			// --- Test animations --------------------------------------------
			constexpr float DEG2RAD = 0.0174532925199433f;
			if(ImGui::Button("Orbit (3s)")){
				TransformSample start = sample;
				TransformSample target = sample;
				target.translation += vec3(2.0f, 0.0f, 0.0f);
				target.rotation = quatAxisAngle(vec3(0,1,0), 90.0f * DEG2RAD);
				MotionController::setTransformAnimated(scene, node->ID, target, 3.0, EaseMode::EaseInOut);
			}
			ImGui::SameLine();
			if(ImGui::Button("Lift & spin (2s)")){
				TransformSample target = sample;
				target.translation += vec3(0.0f, 1.5f, 0.0f);
				target.rotation = quatAxisAngle(vec3(0,1,0), 45.0f * DEG2RAD);
				MotionController::setTransformAnimated(scene, node->ID, target, 2.0, EaseMode::EaseOut);
			}
			ImGui::SameLine();
			if(ImGui::Button("Reset")){
				MotionController::setTransform(scene, node->ID, TransformSample::identity());
			}
		}

		ImGui::Separator();

		// --- Timeline player ---------------------------------------------
		if(ImGui::CollapsingHeader("Timeline", ImGuiTreeNodeFlags_DefaultOpen)){
			auto& tl = editor->timeline;
			if(ImGui::Button(tl.playing ? "Pause" : "Play")){
				tl.playing = !tl.playing;
			}
			ImGui::SameLine();
			ImGui::Checkbox("Loop", &tl.loop);
			ImGui::SameLine();
			if(ImGui::Button("Stop")){
				tl.playing = false;
				tl.playhead = 0.0;
			}

			float ph = static_cast<float>(tl.playhead);
			float dur = static_cast<float>(tl.duration > 0.0 ? tl.duration : 1.0);
			if(ImGui::SliderFloat("playhead", &ph, 0.0f, dur, "%.2f s")){
				tl.playhead = static_cast<double>(ph);
				// Scrub: apply sampled transforms immediately without advancing.
				// (Timeline::update only runs when playing, so we apply manually here.)
				for(const auto& track : tl.tracks){
					if(track.target < 0) continue;
					MotionController::setTransform(scene, track.target, sampleTrack(track, tl.playhead));
				}
			}
			ImGui::Text("Duration: %.2f s   Tracks: %d", tl.duration, (int)tl.tracks.size());

			if(ImGui::Button("Load JSON")){
				static char pathBuf[512] = "";
				ImGui::SameLine();
				ImGui::PushItemWidth(280);
				ImGui::InputText("##timelinepath", pathBuf, sizeof(pathBuf));
				ImGui::PopItemWidth();
				if(strlen(pathBuf) > 0){
					tl.loadTracksFromJSON(scene, pathBuf);
				}
			}
		}

			ImGui::Separator();

			// --- 4DGS Dynamic Scene Player --------------------------------
			if(ImGui::CollapsingHeader("4DGS Dynamic Scene", ImGuiTreeNodeFlags_DefaultOpen)){
				// Check if any SN4DGSSplats nodes exist in the scene
				bool has4DGS = false;
				size_t num4DGS = 0;
				scene.forEach<SN4DGSSplats>([&](SN4DGSSplats*) {
					has4DGS = true;
					num4DGS++;
				});

				if(!has4DGS){
					ImGui::TextDisabled("No 4DGS nodes in the scene.");
					ImGui::TextDisabled("Import a 4DGS model (canonical.ply + deformation_model.pt).");
				}else{
					ImGui::Text("4DGS nodes: %zu", num4DGS);

					auto& tl = editor->timeline;

					// Playback controls
					if(ImGui::Button(tl.playing ? "Pause##4dgs" : "Play##4dgs")){
						tl.playing = !tl.playing;
					}
					ImGui::SameLine();
					if(ImGui::Button("Stop##4dgs")){
						tl.playing = false;
						tl.playhead = 0.0;
					}

					// Time slider (normalized 0.0 to 1.0)
					float ph = static_cast<float>(tl.playhead);
					if(ImGui::SliderFloat("time##4dgs", &ph, 0.0f, 1.0f, "t=%.3f")){
						tl.playhead = static_cast<double>(ph);
					}

					// Playback speed
					static float speed = 1.0f;
					ImGui::SliderFloat("speed##4dgs", &speed, 0.1f, 4.0f, "%.1fx");

					// Info
					ImGui::TextDisabled("Cycle: 0.0 → 1.0 → 0.0 (bounce loop)");

					// Enable/disable deformation on selected 4DGS node
					shared_ptr<SceneNode> sel = editor->getSelectedNode();
					SN4DGSSplats* node4dgs = sel ? dynamic_cast<SN4DGSSplats*>(sel.get()) : nullptr;
					if(node4dgs){
						ImGui::Separator();
						ImGui::Text("Selected: %s", node4dgs->name.c_str());
						bool enabled = node4dgs->deformationEnabled;
						if(ImGui::Checkbox("Deformation active", &enabled)){
							node4dgs->deformationEnabled = enabled;
							if(enabled) node4dgs->needsRecompute.store(true);
						}
						if(ImGui::Button("Force recompute##4dgsforce")){
							node4dgs->needsRecompute.store(true);
						}
					}
				}
			}

			ImGui::Separator();

			// --- Human rig controls ------------------------------------------
		// If the selected node is a SNRiggedSplats, expose per-joint local pose
		// sliders and ARKit 52 blendshape sliders. Otherwise offer to convert a
		// plain SNSplats selection into a procedural test rig.
		if(ImGui::CollapsingHeader("Human Rig", ImGuiTreeNodeFlags_DefaultOpen)){
			shared_ptr<SceneNode> sel = editor->getSelectedNode();
			SNRiggedSplats* rigNode = sel ? dynamic_cast<SNRiggedSplats*>(sel.get()) : nullptr;

			if(!rigNode){
				ImGui::TextDisabled("Select a rigged splat node, or create one below.");
				if(sel && dynamic_cast<SNSplats*>(sel.get())){
					if(ImGui::Button("Attach procedural test rig")){
						// Replace the selected SNSplats with a SNRiggedSplats wrapping
						// the same splat data, then build the procedural skeleton.
						auto snSplats = dynamic_pointer_cast<SNSplats>(sel);
						auto rigged = make_shared<SNRiggedSplats>(snSplats->name + "_rigged", snSplats->splats);
						rigged->dmng.data.numSHCoefficients = snSplats->dmng.data.numSHCoefficients;
						rigged->dmng.data.shDegree = snSplats->dmng.data.shDegree;
						rigged->transform = snSplats->transform;
						// Swap the node in the scene graph.
						scene.swapNode(sel, rigged);
						ProceduralRigSource::build(rigged);
						editor->setSelectedNode(rigged.get());
					}
				}
			} else {
				ImGui::Checkbox("Skinning enabled", &rigNode->skinningEnabled);
				ImGui::SameLine();
				if(ImGui::Button("Reset pose")){
					rigNode->currentPose.reset(rigNode->currentPose.joints.size());
					rigNode->poseDirty = true;
				}

				// Joint local rotation sliders (Euler degrees, per joint).
				if(ImGui::TreeNode("Joints")){
					for(size_t j = 0; j < rigNode->currentPose.joints.size(); j++){
						const std::string& jn = rigNode->rig.skeleton.jointNames.size() > j
							? rigNode->rig.skeleton.jointNames[j] : std::to_string(j);
						vec3 e = quatToEulerXYZ(rigNode->currentPose.joints[j].rotation) * 57.2957795130823f;
						if(ImGui::DragFloat3(jn.c_str(), &e.x, 0.5f, -180.0f, 180.0f, "%.1f")){
							JointPose jp = rigNode->currentPose.joints[j];
							jp.rotation = quatFromEulerXYZ(e * 0.0174532925199433f);
							RiggedHumanController::setJointPose(scene, rigNode->ID, (int)j, jp);
						}
					}
					ImGui::TreePop();
				}

				// ARKit 52 blendshape sliders.
				if(rigNode->rig.blendshapeCount > 0){
					if(ImGui::TreeNode("Face (ARKit 52)")){
						for(int b = 0; b < BLENDSHAPE_COUNT && b < rigNode->rig.blendshapeCount; b++){
							float w = rigNode->currentFace.weights[b];
							if(ImGui::SliderFloat(kBsNames[b], &w, 0.0f, 1.0f, "%.2f")){
								RiggedHumanController::setBlendshape(scene, rigNode->ID, b, w);
							}
						}
						ImGui::TreePop();
					}
				} else {
					ImGui::TextDisabled("No facial blendshapes on this model.");
				}
			}
		}
	}
	ImGui::End();
}
