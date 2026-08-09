
// Orbbec RGBD camera control panel.
//
// Exposes device selection, stream configuration, live camera-parameter
// control (exposure / gain / white balance / mirror / laser / alignment),
// and an optional real-time RGBD point-cloud preview rendered through the
// existing SNPoints / HQS pipeline.
//
// All SDK access goes through orbbec::OrbbecCapture; this header pulls in
// only OrbbecTypes.h (no libobsensor headers). When SPLATSHOP_HAS_ORBBEC is
// undefined makeOrbbecGUI() compiles to an empty stub.
#include "SplatEditor.h"

#ifdef SPLATSHOP_HAS_ORBBEC
#include <algorithm>
#include <ctime>
#include "../camera/OrbbecCapture.h"
#include "../scene/SNOrbbec.h"

using std::string;
using orbbec::OrbbecCapture;
using orbbec::StreamConfig;
using orbbec::CameraParams;
using orbbec::DeviceInfo;
using orbbec::AlignMode;

void SplatEditor::makeOrbbecGUI(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& scene = editor->scene;

	if(!settings.showOrbbec) return;

	ImGui::SetNextWindowSize(ImVec2(420, 620), ImGuiCond_FirstUseEver);
	if(ImGui::Begin("Orbbec RGBD Camera", &settings.showOrbbec)){

		// Lazily create the capture object on first show.
		if(!editor->orbbecCapture){
			editor->orbbecCapture = std::make_shared<OrbbecCapture>();
		}
		auto& cap = editor->orbbecCapture;

		// ==================================================================
		// Device section
		// ==================================================================
		if(ImGui::CollapsingHeader("Device", ImGuiTreeNodeFlags_DefaultOpen)){
			if(ImGui::Button("Refresh")) {
				editor->orbbecDevices = OrbbecCapture::enumerateDevices();
				editor->orbbecSelectedDevice = 0;
			}
			ImGui::SameLine();
			if(editor->orbbecDevices.empty()){
				ImGui::TextDisabled("no devices");
			} else {
				// Build a combo of device names with SN for disambiguation.
				auto& devs = editor->orbbecDevices;
				auto cur = (editor->orbbecSelectedDevice >= 0 &&
				            editor->orbbecSelectedDevice < (int)devs.size())
				           ? editor->orbbecSelectedDevice : 0;

				// Preview label: "name (SN: xxxxx)"
				auto devLabel = [&](int i) -> string {
					auto& d = devs[i];
					return d.serialNumber.empty()
						? format("{} (UID: {})", d.name, d.uid)
						: format("{} (SN: {})", d.name, d.serialNumber);
				};
				string previewStr = devLabel(cur);
				if(ImGui::BeginCombo("##orbbecdev", previewStr.c_str())){
					for(int i = 0; i < (int)devs.size(); ++i){
						bool sel = (i == cur);
						string lbl = devLabel(i);
						if(ImGui::Selectable(lbl.c_str(), sel)){
							editor->orbbecSelectedDevice = i;
						}
						if(sel) ImGui::SetItemDefaultFocus();
					}
					ImGui::EndCombo();
				}

				const auto& d = devs[cur];
				ImGui::Text("Serial: %s", d.serialNumber.c_str());
				ImGui::Text("UID:    %s", d.uid.c_str());
				ImGui::Text("Link:   %s  (PID 0x%04X)", d.connectionType.c_str(), d.pid);
			}

			ImGui::Dummy(ImVec2(0, 4));
			bool open = cap->isOpen();
			if(ImGui::Button(open ? "Close Device" : "Open Device")){
				if(open){
					cap->close();
					editor->snOrbbec = nullptr;
					// Reset cached state so the next device starts fresh.
					editor->orbbecStreamCfgLoaded = false;
					editor->orbbecParamsLoaded = false;
					editor->orbbecColorProfiles.clear();
					editor->orbbecDepthProfiles.clear();
				} else if(!editor->orbbecDevices.empty()){
					int idx = std::clamp(editor->orbbecSelectedDevice, 0,
					                     (int)editor->orbbecDevices.size() - 1);
					const auto& d = editor->orbbecDevices[idx];
					// Prefer the serial; fall back to uid.
					string id = d.serialNumber.empty() ? d.uid : d.serialNumber;
					// Reset cached config before opening the new device.
					editor->orbbecStreamCfgLoaded = false;
					editor->orbbecParamsLoaded = false;
					editor->orbbecColorProfiles.clear();
					editor->orbbecDepthProfiles.clear();
					editor->orbbecCfgColor = StreamConfig{};
					editor->orbbecCfgDepth = StreamConfig{};
					editor->orbbecParams = CameraParams{};
					cap->open(id);
					// Auto-load supported profiles for the newly opened device.
					if(cap->isOpen()){
						editor->orbbecColorProfiles = cap->getSupportedProfiles(2);
						editor->orbbecDepthProfiles = cap->getSupportedProfiles(3);
						editor->orbbecColorProfileIdx = 0;
						editor->orbbecDepthProfileIdx = 0;
					}
				}
			}

			ImGui::SameLine();
			bool streaming = cap->isStreaming();
			// ImGui 1.81 has no BeginDisabled; gate the button on `open`.
			if(open){
				if(ImGui::Button(streaming ? "Stop" : "Start")){
					if(streaming) cap->stop();
					else          cap->start();
				}
			} else {
				ImGui::TextDisabled("(open a device to start)");
			}
		}

		// ==================================================================
		// Stream configuration
		// ==================================================================
		if(ImGui::CollapsingHeader("Streams")){
			auto& sc = editor->orbbecCfgColor;
			auto& sd = editor->orbbecCfgDepth;

			// Load current config once after device open.
			if(cap->isOpen() && !editor->orbbecStreamCfgLoaded){
				cap->getStreamConfig(sc, sd);
				editor->orbbecStreamCfgLoaded = true;
			}
			if(!cap->isOpen()) editor->orbbecStreamCfgLoaded = false;

			// Helper: build a human-readable label for a StreamConfig profile.
			auto profileLabel = [](const StreamConfig& s) -> string {
				return format("{}x{} @ {}fps fmt={}", s.width, s.height, s.fps, s.format);
			};

			// --- Color profile combo ---
			ImGui::Text("Color");
			ImGui::Checkbox("enable color", &sc.enable);
			if(cap->isOpen() && !editor->orbbecColorProfiles.empty()){
				// Build preview from current config (or first profile).
				string colorPreview = profileLabel(sc);
				if(ImGui::BeginCombo("##color_profile", colorPreview.c_str())){
					for(int i = 0; i < (int)editor->orbbecColorProfiles.size(); ++i){
						auto& prof = editor->orbbecColorProfiles[i];
						string lbl = profileLabel(prof);
						bool sel = (prof.width == sc.width && prof.height == sc.height &&
						            prof.fps == sc.fps && prof.format == sc.format);
						if(ImGui::Selectable(lbl.c_str(), sel)){
							sc.width  = prof.width;
							sc.height = prof.height;
							sc.fps    = prof.fps;
							sc.format = prof.format;
							editor->orbbecColorProfileIdx = i;
						}
						if(sel) ImGui::SetItemDefaultFocus();
					}
					ImGui::EndCombo();
				}
			}
			// Allow manual override (Custom).
			ImGui::PushID("color_manual");
			int cw = sc.width, ch = sc.height, cf = sc.fps;
			ImGui::InputInt("width",  &cw);  sc.width  = cw;
			ImGui::InputInt("height", &ch);  sc.height = ch;
			ImGui::InputInt("fps",    &cf);  sc.fps    = cf;
			ImGui::InputInt("format", &sc.format);
			ImGui::PopID();

			ImGui::Separator();

			// --- Depth profile combo ---
			ImGui::Text("Depth");
			ImGui::Checkbox("enable depth", &sd.enable);
			if(cap->isOpen() && !editor->orbbecDepthProfiles.empty()){
				string depthPreview = profileLabel(sd);
				if(ImGui::BeginCombo("##depth_profile", depthPreview.c_str())){
					for(int i = 0; i < (int)editor->orbbecDepthProfiles.size(); ++i){
						auto& prof = editor->orbbecDepthProfiles[i];
						string lbl = profileLabel(prof);
						bool sel = (prof.width == sd.width && prof.height == sd.height &&
						            prof.fps == sd.fps && prof.format == sd.format);
						if(ImGui::Selectable(lbl.c_str(), sel)){
							sd.width  = prof.width;
							sd.height = prof.height;
							sd.fps    = prof.fps;
							sd.format = prof.format;
							editor->orbbecDepthProfileIdx = i;
						}
						if(sel) ImGui::SetItemDefaultFocus();
					}
					ImGui::EndCombo();
				}
			}
			ImGui::PushID("depth_manual");
			int dw = sd.width, dh = sd.height, df = sd.fps;
			ImGui::InputInt("width",  &dw);  sd.width  = dw;
			ImGui::InputInt("height", &dh);  sd.height = dh;
			ImGui::InputInt("fps",    &df);  sd.fps    = df;
			ImGui::InputInt("format", &sd.format);
			ImGui::PopID();

			ImGui::Separator();

			// --- Alignment mode (stream-level config, not a device property) ---
			const char* alignModes[] = { "Disable", "D2C HW", "D2C SW", "C2D SW" };
			int alignIdx = static_cast<int>(editor->orbbecParams.alignMode);
			ImGui::Combo("Align mode", &alignIdx, alignModes, IM_ARRAYSIZE(alignModes));
			editor->orbbecParams.alignMode = static_cast<AlignMode>(alignIdx);

			ImGui::Checkbox("Frame sync", &editor->orbbecParams.frameSync);
			ImGui::Checkbox("Aggregate all required", &editor->orbbecParams.aggregateAllRequired);

			ImGui::Separator();

			if(ImGui::Button("Apply (restart)")){
				cap->setStreamConfig(sc, sd);
				cap->applyCameraParams(editor->orbbecParams);
				if(cap->isStreaming()){ cap->stop(); cap->start(); }
			}
		}

		// ==================================================================
		// Depth denoising (SDK post-processing filters)
		// ==================================================================
		if(ImGui::CollapsingHeader("Depth Denoising")){
			auto& p = editor->orbbecParams;

			// Live parameter updates: any widget change in this section
			// immediately pushes the new params to the capture thread via
			// applyDepthFilterParams(), so the next frame already reflects
			// the change — no stream restart needed.
			bool changed = false;

			// --- Compare toggle ---
			ImGui::Checkbox("Compare Before/After", &settings.orbbecDenoiseCompare);
			ImGui::SameLine();
			ImGui::TextDisabled("(params apply instantly)");

			// --- Hardware Noise Removal (LutNoiseRemovalFilter, SPLIT/LUT) ---
			ImGui::Separator();
			changed |= ImGui::Checkbox("Hardware Noise Removal", &p.hwNoiseRemovalEnabled);
			if(p.hwNoiseRemovalEnabled){
				ImGui::Indent();
				changed |= ImGui::InputInt("max LUT",   &p.hwNoiseMaxLut);
				changed |= ImGui::InputInt("min diff",  &p.hwNoiseMinDiff);
				ImGui::Unindent();
			}

			// --- Software Noise Removal (NoiseRemovalFilter, OVERALL) ---
			ImGui::Separator();
			changed |= ImGui::Checkbox("Software Noise Removal", &p.denoiseFilterEnabled);
			if(p.denoiseFilterEnabled){
				ImGui::Indent();
				changed |= ImGui::InputInt("max size",   &p.denoiseMaxSize);
				changed |= ImGui::InputInt("disp diff",  &p.denoiseDispDiff);
				ImGui::Unindent();
			}

			// --- Spatial Advanced ---
			ImGui::Separator();
			changed |= ImGui::Checkbox("Spatial Filter", &p.spatialFilterEnabled);
			if(p.spatialFilterEnabled){
				ImGui::Indent();
				changed |= ImGui::SliderFloat("alpha",     &p.spatialAlpha,  0.f, 1.f, "%.2f");
				changed |= ImGui::InputInt("radius",       &p.spatialRadius);
				changed |= ImGui::InputInt("magnitude",    &p.spatialMagnitude);
				changed |= ImGui::InputInt("disp diff",    &p.spatialDispDiff);
				ImGui::Unindent();
			}

			// --- Temporal ---
			ImGui::Separator();
			changed |= ImGui::Checkbox("Temporal Filter", &p.temporalFilterEnabled);
			if(p.temporalFilterEnabled){
				ImGui::Indent();
				changed |= ImGui::SliderFloat("weight",     &p.temporalWeight,    0.f, 1.f, "%.2f");
				changed |= ImGui::SliderFloat("diff scale", &p.temporalDiffScale, 0.f, 1.f, "%.3f");
				ImGui::Unindent();
			}

			// --- Distance Filter (host-side pixel clamp, post-denoise) ---
			ImGui::Separator();
			changed |= ImGui::Checkbox("Distance Filter", &p.depthDistFilterEnabled);
			if(p.depthDistFilterEnabled){
				ImGui::Indent();
				changed |= ImGui::SliderFloat("Min distance (mm)", &p.depthDistMinMm,
				                              0.f, 10000.f, "%.0f");
				changed |= ImGui::SliderFloat("Max distance (mm)", &p.depthDistMaxMm,
				                              0.f, 10000.f, "%.0f");
				// Keep min <= max so the clamp range is always valid.
				if(p.depthDistMinMm > p.depthDistMaxMm){
					p.depthDistMinMm = p.depthDistMaxMm;
				}
				ImGui::Unindent();
			}

			if(changed && cap->isStreaming()){
				cap->applyDepthFilterParams(p);
			}
		}

		// ==================================================================
		// Camera parameters
		// ==================================================================
		if(ImGui::CollapsingHeader("Camera Parameters")){
			auto& p = editor->orbbecParams;
			// Pull current values from the device once it's open.
			if(cap->isOpen() && !editor->orbbecParamsLoaded){
				p = cap->getCameraParams();
				editor->orbbecParamsLoaded = true;
			}
			if(!cap->isOpen()) editor->orbbecParamsLoaded = false;

			ImGui::TextDisabled("(-1 = device default; click Apply to write)");

			ImGui::Checkbox("Color auto-exposure",    &p.colorAutoExposure);
			ImGui::Checkbox("Color auto white balance", &p.colorAutoWhiteBalance);
			ImGui::Checkbox("Color mirror",           &p.colorMirror);
			ImGui::Checkbox("Depth auto-exposure",    &p.depthAutoExposure);
			ImGui::Checkbox("Depth mirror",           &p.depthMirror);
			ImGui::Checkbox("Laser on",               &p.laserOn);
			ImGui::Checkbox("LDP on",                 &p.ldpOn);

			ImGui::Separator();
			ImGui::InputInt("Color exposure",     &p.colorExposure);
			ImGui::InputInt("Color gain",         &p.colorGain);
			ImGui::InputInt("Color white balance",&p.colorWhiteBalance);
			ImGui::InputInt("Color brightness",   &p.colorBrightness);
			ImGui::InputInt("Color saturation",   &p.colorSaturation);
			ImGui::InputInt("Color contrast",     &p.colorContrast);
			ImGui::InputInt("Color gamma",        &p.colorGamma);
			ImGui::InputInt("Depth exposure",     &p.depthExposure);
			ImGui::InputInt("Depth gain",         &p.depthGain);
			ImGui::InputInt("Depth precision",    &p.depthPrecisionLevel);
			ImGui::InputInt("Min depth (mm)",     &p.minDepth);
			ImGui::InputInt("Max depth (mm)",     &p.maxDepth);
			ImGui::InputInt("IR exposure",        &p.irExposure);
			ImGui::InputInt("IR gain",            &p.irGain);

			ImGui::Separator();
			if(ImGui::Button("Apply to device")){
				cap->applyCameraParams(p);
			}
			ImGui::SameLine();
			if(ImGui::Button("Read back")){
				p = cap->getCameraParams();
			}

			ImGui::Separator();
			// --- Save / Load camera params (device control + filters +
			//     stream config) to a JSON file under calibration/. ---
			if(ImGui::Button("Save Params")){
				if(!editor->orbbecDevices.empty()){
					int idx = std::clamp(editor->orbbecSelectedDevice, 0,
					                     (int)editor->orbbecDevices.size() - 1);
					const auto& d = editor->orbbecDevices[idx];
					int refW = editor->orbbecCfgColor.width  > 0 ? editor->orbbecCfgColor.width  : 640;
					int refH = editor->orbbecCfgColor.height > 0 ? editor->orbbecCfgColor.height : 480;
					std::string id = d.serialNumber.empty() ? d.uid : d.serialNumber;
					std::string path = orbbec::CalibrationStore::cameraParamsPathFor(id, refW, refH);
					orbbec::CameraParamsFile cpf;
					cpf.deviceSerial = id;
					cpf.deviceName   = d.name;
					cpf.params       = p;
					cpf.colorCfg     = editor->orbbecCfgColor;
					cpf.depthCfg     = editor->orbbecCfgDepth;
					// ISO-8601 timestamp (simple, no external dep).
					{
						auto t = std::time(nullptr);
						std::tm tm{};
						#ifdef _WIN32
						localtime_s(&tm, &t);
						#else
						localtime_r(&t, &tm);
						#endif
						char buf[32];
						std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
						cpf.timestamp = buf;
					}
					if(orbbec::CalibrationStore::saveCameraParams(cpf, path)){
						editor->orbbecParamsSaveMsg = format("saved: {}", path);
					} else {
						editor->orbbecParamsSaveMsg = "save failed";
					}
				} else {
					editor->orbbecParamsSaveMsg = "no device selected";
				}
			}
			ImGui::SameLine();
			if(ImGui::Button("Load Params")){
				if(!editor->orbbecDevices.empty()){
					int idx = std::clamp(editor->orbbecSelectedDevice, 0,
					                     (int)editor->orbbecDevices.size() - 1);
					const auto& d = editor->orbbecDevices[idx];
					int refW = editor->orbbecCfgColor.width  > 0 ? editor->orbbecCfgColor.width  : 640;
					int refH = editor->orbbecCfgColor.height > 0 ? editor->orbbecCfgColor.height : 480;
					std::string id = d.serialNumber.empty() ? d.uid : d.serialNumber;
					orbbec::CameraParamsFile cpf;
					if(orbbec::CalibrationStore::loadCameraParamsForDevice(cpf, id, refW, refH)){
						editor->orbbecParams     = cpf.params;
						editor->orbbecCfgColor   = cpf.colorCfg;
						editor->orbbecCfgDepth   = cpf.depthCfg;
						editor->orbbecParamsSaveMsg = format("loaded: {}", cpf.filePath);
						// Push the loaded params to the device / filters live.
						p = cpf.params;
						cap->setStreamConfig(cpf.colorCfg, cpf.depthCfg);
						if(cap->isOpen())  cap->applyCameraParams(p);
						if(cap->isStreaming()) cap->applyDepthFilterParams(p);
					} else {
						editor->orbbecParamsSaveMsg = "no saved params file for this device";
					}
				} else {
					editor->orbbecParamsSaveMsg = "no device selected";
				}
			}
			if(!editor->orbbecParamsSaveMsg.empty()){
				ImGui::SameLine();
				ImGui::TextDisabled("%s", editor->orbbecParamsSaveMsg.c_str());
			}
		}

		// ==================================================================
		// Point cloud preview
		// ==================================================================
		if(ImGui::CollapsingHeader("Point Cloud")){
			auto& p = editor->orbbecParams;
			bool pcChanged = false;

			bool pc = cap->isPointCloudEnabled();
			if(ImGui::Checkbox("显示点云 (Generate RGB point cloud)", &pc)){
				cap->setPointCloudEnabled(pc);
			}

			ImGui::Separator();

			// Point cloud alignment direction: D2C (depth→color resolution)
			// or C2D (color→depth resolution). C2D keeps the cloud in the
			// depth sensor's coordinate system.
			const char* pcAlignModes[] = { "D2C (depth->color)", "C2D (color->depth)" };
			int pcAlignIdx = (p.pointCloudAlignMode == AlignMode(3)) ? 1 : 0;
			if(ImGui::Combo("Point cloud alignment", &pcAlignIdx, pcAlignModes, IM_ARRAYSIZE(pcAlignModes))){
				p.pointCloudAlignMode = (pcAlignIdx == 1) ? AlignMode(3) : AlignMode(2);
				pcChanged = true;
			}

			// Route the denoised depth frame back into the point-cloud path
			// so the cloud reflects the active depth denoising filters.
			pcChanged |= ImGui::Checkbox("Use denoised depth for point cloud",
			                             &p.pointCloudUseDenoisedDepth);

			if(pcChanged && cap->isStreaming()){
				cap->applyDepthFilterParams(p);
			}

			ImGui::Separator();

			// The button toggles the preview node's presence; its label
			// reflects whether the node currently exists (not the generation
			// checkbox above).
			bool hasNode = editor->snOrbbec != nullptr;
			if(ImGui::Button(hasNode ? "Hide preview" : "Show preview")){
				if(hasNode){
					scene.world->remove(editor->snOrbbec.get());
					editor->snOrbbec = nullptr;
				} else if(pc){
					editor->snOrbbec = std::make_shared<SNOrbbec>("Orbbec Live Cloud");
					editor->snOrbbec->lastFrameIndex = 0;
					scene.world->children.push_back(editor->snOrbbec);
				}
			}

			// Feed the latest point cloud into the scene node each frame,
			// but skip the full re-copy/upload when the frame index is
			// unchanged from the last GUI tick.
			if(editor->snOrbbec && cap->isPointCloudEnabled()){
				auto frame = cap->getLatestPointCloud();
				if(frame && frame->colorData && frame->colorWidth > 0 &&
				   frame->frameIndex != editor->snOrbbec->lastFrameIndex){
					editor->snOrbbec->lastFrameIndex = frame->frameIndex;
					editor->snOrbbec->loadPointCloud(
						(const uint8_t*)frame->colorData->data, frame->colorWidth);
				}
			}
		}

		// ==================================================================
		// Status
		// ==================================================================
		if(ImGui::CollapsingHeader("Status", ImGuiTreeNodeFlags_DefaultOpen)){
			ImGui::Text("Streaming: %s", cap->isStreaming() ? "yes" : "no");
			ImGui::Text("FPS: %.1f", cap->getMeasuredFps());
			auto f = cap->getLatestFrame();
			if(f){
				ImGui::Text("Color: %dx%d (fmt %d)", f->colorWidth, f->colorHeight, f->colorFormat);
				ImGui::Text("Depth: %dx%d (fmt %d, scale %.4f)",
				            f->depthWidth, f->depthHeight, f->depthFormat, f->depthScale);
				ImGui::Text("Frame #: %llu", (unsigned long long)f->frameIndex);
			} else {
				ImGui::TextDisabled("no frame yet");
			}
			auto ci = cap->getColorIntrinsics();
			ImGui::Text("Color intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d",
			            ci.fx, ci.fy, ci.cx, ci.cy, ci.w, ci.h);
		}
	}
	ImGui::End();
}

#else // !SPLATSHOP_HAS_ORBBEC

// Stub so drawGUI() can call this unconditionally.
void SplatEditor::makeOrbbecGUI() {}

#endif // SPLATSHOP_HAS_ORBBEC
