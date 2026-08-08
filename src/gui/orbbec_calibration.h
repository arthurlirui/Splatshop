
// Orbbec lens calibration panel.
//
// Real-time chessboard capture → cv::calibrateCamera → 5-parameter
// Brown-Conrady distortion (k1,k2,p1,p2,k3) + intrinsics. Results are
// persisted to JSON and optionally applied to the Orbbec preview (live
// undistortion) and the RGB point cloud (calibrated intrinsics).
//
// Requires both the OrbbecSDK (SPLATSHOP_HAS_ORBBEC) and OpenCV
// (SPLATSHOP_HAS_OPENCV). Without either, compiles to an empty stub.
#include "SplatEditor.h"

#if defined(SPLATSHOP_HAS_ORBBEC) && defined(SPLATSHOP_HAS_OPENCV)

#include <algorithm>
#include <cstring>

#include "../camera/OrbbecCapture.h"
#include "../calibration/Calibrator.h"
#include "../calibration/CalibrationStore.h"
#include "Calibration.h"

using std::string;
using orbbec::OrbbecCapture;
using orbbec::Calibrator;
using orbbec::CalibStream;
using orbbec::DeviceCalibration;
using orbbec::StreamCalibration;
using orbbec::CalibrationStore;

void SplatEditor::makeOrbbecCalibrationGUI(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;

	if(!settings.showOrbbecCalibration) return;

	// Lazily create the calibrator.
	if(!editor->orbbecCalibrator){
		editor->orbbecCalibrator = std::make_shared<Calibrator>();
	}
	auto& cal = editor->orbbecCalibrator;
	auto& cap = editor->orbbecCapture;

	ImGui::SetNextWindowSize(ImVec2(460, 720), ImGuiCond_FirstUseEver);
	if(ImGui::Begin("Orbbec Calibration", &settings.showOrbbecCalibration)){

		if(!cap || !cap->isOpen()){
			ImGui::TextDisabled("Open an Orbbec device first (Orbbec panel).");
			ImGui::End();
			return;
		}

		// ==================================================================
		// Chessboard settings
		// ==================================================================
		if(ImGui::CollapsingHeader("Chessboard", ImGuiTreeNodeFlags_DefaultOpen)){
			int cols = cal->chessCols();
			int rows = cal->chessRows();
			float sq = cal->squareSizeMm();
			ImGui::InputInt("Inner corners (cols)", &cols);
			ImGui::InputInt("Inner corners (rows)", &rows);
			ImGui::InputFloat("Square size (mm)", &sq, 0.1f, 1.0f, "%.2f");
			if(ImGui::Button("Apply spec")){
				cal->setChessboard(cols, rows, sq);
			}
			ImGui::SameLine();
			ImGui::TextDisabled("(changing the spec clears samples)");

			ImGui::Separator();
			const char* streams[] = { "Color", "IR", "Depth" };
			ImGui::Combo("Target stream", &editor->orbbecCalibTargetStream, streams, IM_ARRAYSIZE(streams));
			ImGui::TextDisabled("Depth uses IR's distortion (same sensor)");
		}

		// ==================================================================
		// Live capture + overlay
		// ==================================================================
		if(ImGui::CollapsingHeader("Live Capture", ImGuiTreeNodeFlags_DefaultOpen)){
			auto frame = cap->getLatestFrame();
			if(!frame){
				ImGui::TextDisabled("no frame yet (start streaming)");
			} else {
				// Pick the source image for the selected stream.
				const uint8_t* src = nullptr;
				int w = 0, h = 0, ch = 1;
				if(editor->orbbecCalibTargetStream == 0 && frame->colorData){
					// Color: most formats are 3-channel after SDK decode.
					// Treat unknown as RGB(3). The Calibrator converts to
					// gray internally.
					src = (const uint8_t*)frame->colorData->data;
					w = frame->colorWidth; h = frame->colorHeight; ch = 3;
				} else if(editor->orbbecCalibTargetStream == 1 && frame->irData){
					src = (const uint8_t*)frame->irData->data;
					w = frame->irWidth; h = frame->irHeight; ch = 1;
				} else if(editor->orbbecCalibTargetStream == 2 && frame->irData){
					// Depth: calibrate via the co-located IR image (the
					// chessboard is visible in IR, not in the depth map).
					src = (const uint8_t*)frame->irData->data;
					w = frame->irWidth; h = frame->irHeight; ch = 1;
				}

				if(src && w > 0 && h > 0){
					// Run detection every frame for the overlay. We do NOT
					// store here — storage happens on Capture / auto-capture.
					cal->detectOnly(src, w, h, ch);

					// Build a BGR8 overlay with the detected corners drawn.
					int ow = 0, oh = 0;
					cal->buildOverlay(src, w, h, ch,
					                  editor->orbbecCalibOverlayScratch, ow, oh);

					// Upload to the overlay texture.
					GLuint& tex = editor->orbbecCalibTexOverlay;
					if(tex == 0 || editor->orbbecCalibTexOverlayW != ow ||
					   editor->orbbecCalibTexOverlayH != oh){
						if(tex == 0) glGenTextures(1, &tex);
						glBindTexture(GL_TEXTURE_2D, tex);
						glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
						glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
						glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
						glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
						glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, ow, oh, 0,
						             GL_RGB, GL_UNSIGNED_BYTE, nullptr);
						editor->orbbecCalibTexOverlayW = ow;
						editor->orbbecCalibTexOverlayH = oh;
					}
					glBindTexture(GL_TEXTURE_2D, tex);
					// Calibrator produces BGR8; upload with GL_BGR so the
					// driver swaps R/B into the RGBA8 texture.
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, ow, oh,
					                GL_BGR, GL_UNSIGNED_BYTE,
					                editor->orbbecCalibOverlayScratch.data());

					// Aspect-fit draw.
					float regionW = ImGui::GetContentRegionAvail().x;
					float drawW = (float)ow, drawH = (float)oh;
					float scale = regionW / drawW;
					if(drawH * scale > ImGui::GetContentRegionAvail().y - 40)
						scale = (ImGui::GetContentRegionAvail().y - 40) / drawH;
					drawW *= scale; drawH *= scale;
					ImGui::Image((ImTextureID)(void*)(intptr_t)tex,
					             ImVec2(drawW, drawH));

					if(cal->lastDetectionOk()){
						ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f),
						                   "Valid pose  %d corners",
						                   (int)cal->lastCorners().size());
					} else {
						ImGui::TextDisabled("no chessboard detected");
					}
				} else {
					ImGui::TextDisabled("selected stream has no data");
					if(editor->orbbecCalibTargetStream >= 1){
						ImGui::TextDisabled("(enable IR stream in Orbbec panel)");
					}
				}
			}

			ImGui::Separator();
			ImGui::Text("Samples: %d  (aim for >= 15)", cal->sampleCount());

			static bool autoCapture = false;
			ImGui::Checkbox("Auto-capture", &autoCapture);
			ImGui::SameLine();
			if(ImGui::Button("Capture")){
				auto frame = cap->getLatestFrame();
				if(frame){
					const uint8_t* src = nullptr; int w=0,h=0,ch=1;
					if(editor->orbbecCalibTargetStream == 0 && frame->colorData){
						src=(const uint8_t*)frame->colorData->data;
						w=frame->colorWidth;h=frame->colorHeight;ch=3;
					} else if(editor->orbbecCalibTargetStream >= 1 && frame->irData){
						src=(const uint8_t*)frame->irData->data;
						w=frame->irWidth;h=frame->irHeight;ch=1;
					}
					if(src) cal->detectAndAddFrame(src, w, h, ch, /*poseDiverse=*/true);
				}
			}
			ImGui::SameLine();
			if(ImGui::Button("Remove last")) cal->removeLastSample();
			ImGui::SameLine();
			if(ImGui::Button("Clear")) cal->clear();

			// Auto-capture: grab a sample whenever a valid, diverse pose is
			// seen (the Calibrator's pose-diversity check skips duplicates).
			if(autoCapture){
				auto frame = cap->getLatestFrame();
				if(frame && cal->lastDetectionOk()){
					const uint8_t* src = nullptr; int w=0,h=0,ch=1;
					if(editor->orbbecCalibTargetStream == 0 && frame->colorData){
						src=(const uint8_t*)frame->colorData->data;
						w=frame->colorWidth;h=frame->colorHeight;ch=3;
					} else if(editor->orbbecCalibTargetStream >= 1 && frame->irData){
						src=(const uint8_t*)frame->irData->data;
						w=frame->irWidth;h=frame->irHeight;ch=1;
					}
					if(src) cal->detectAndAddFrame(src, w, h, ch, /*poseDiverse=*/true);
				}
			}
		}

		// ==================================================================
		// Run calibration
		// ==================================================================
		if(ImGui::CollapsingHeader("Run Calibration", ImGuiTreeNodeFlags_DefaultOpen)){
			static bool fixAspect = false;
			static float alpha = 0.f;
			ImGui::Checkbox("Fix aspect ratio (fx=fy)", &fixAspect);
			ImGui::SliderFloat("Undistort alpha", &alpha, 0.f, 1.f, "%.2f");
			ImGui::TextDisabled("(alpha: 0=crop to valid pixels, 1=keep all)");

			ImGui::Separator();
			if(ImGui::Button("Calibrate")){
				StreamCalibration out;
				double rms = cal->runCalibration(out, fixAspect, alpha);
				if(rms >= 0.0){
					// Route the result into the active device calibration.
					auto& dc = editor->orbbecActiveCalibration;
					dc.deviceSerial = editor->orbbecDevices.empty() ? "" :
						editor->orbbecDevices[editor->orbbecSelectedDevice].serialNumber;
					dc.deviceName   = editor->orbbecDevices.empty() ? "" :
						editor->orbbecDevices[editor->orbbecSelectedDevice].name;
					if(editor->orbbecCalibTargetStream == 0)      dc.color = out;
					else if(editor->orbbecCalibTargetStream == 1) dc.ir    = out;
					else                                          dc.ir    = out; // depth via IR
					// Depth shares IR's intrinsics/distortion by default.
					if(editor->orbbecCalibTargetStream == 1 || editor->orbbecCalibTargetStream == 2)
						dc.depth = out;
					println("Orbbec Calibration: RMS = {:.3f} px ({} images)",
					        rms, out.usedImageCount);
				} else {
					println("Orbbec Calibration: failed (need >= 3 samples)");
				}
			}

			auto& dc = editor->orbbecActiveCalibration;
			auto printStream = [](const char* name, const StreamCalibration& s){
				ImGui::Separator();
				ImGui::Text("%s", name);
				if(!s.valid){ ImGui::TextDisabled("  not calibrated"); return; }
				ImGui::Text("  fx=%.2f fy=%.2f cx=%.2f cy=%.2f  %dx%d",
				            s.intrinsics.fx, s.intrinsics.fy,
				            s.intrinsics.cx, s.intrinsics.cy,
				            s.intrinsics.w,  s.intrinsics.h);
				ImGui::Text("  k1=%.4f k2=%.4f p1=%.4f p2=%.4f k3=%.4f",
				            s.distortion.k1, s.distortion.k2,
				            s.distortion.p1, s.distortion.p2, s.distortion.k3);
				ImGui::Text("  RMS=%.3f px  N=%d  %s",
				            s.rmsReprojectionError, s.usedImageCount,
				            s.timestamp.c_str());
			};
			printStream("Color", dc.color);
			printStream("IR",    dc.ir);
			printStream("Depth", dc.depth);

			// Per-view errors (outlier inspection).
			const auto& errs = cal->perViewErrors();
			if(!errs.empty()){
				ImGui::Separator();
				ImGui::Text("Per-view RMS (px):");
				for(size_t i = 0; i < errs.size(); ++i){
					bool bad = errs[i] > 1.0;
					if(bad) ImGui::TextColored(ImVec4(1.0f,0.4f,0.4f,1.0f),
					                           "  [%d] %.3f", (int)i, errs[i]);
					else    ImGui::Text("  [%d] %.3f", (int)i, errs[i]);
				}
			}
		}

		// ==================================================================
		// Persistence + apply
		// ==================================================================
		if(ImGui::CollapsingHeader("Persistence & Apply")){
			auto& dc = editor->orbbecActiveCalibration;

			ImGui::TextDisabled("software-side only (not written to firmware)");

			// Save to default path.
			if(ImGui::Button("Save (default)")){
				int refW = dc.color.valid ? dc.color.intrinsics.w :
				          (dc.ir.valid ? dc.ir.intrinsics.w : 0);
				int refH = dc.color.valid ? dc.color.intrinsics.h :
				          (dc.ir.valid ? dc.ir.intrinsics.h : 0);
				string path;
				if(CalibrationStore::saveDefault(dc, refW, refH, path)){
					editor->orbbecCalibSavePath = path;
					dc.filePath = path;
				}
			}
			ImGui::SameLine();
			if(ImGui::Button("Load (default)")){
				string serial = dc.deviceSerial;
				int refW = cap->getColorIntrinsics().w;
				int refH = cap->getColorIntrinsics().h;
				if(refW == 0){ refW = cap->getIrIntrinsics().w; refH = cap->getIrIntrinsics().h; }
				if(CalibrationStore::loadForDevice(dc, serial, refW, refH)){
					editor->orbbecCalibLoadPath = dc.filePath;
				}
			}

			ImGui::Separator();
			ImGui::Text("Save path: %s", editor->orbbecCalibSavePath.c_str());
			ImGui::Text("Load path: %s", editor->orbbecCalibLoadPath.c_str());

			ImGui::Separator();
			if(ImGui::Button("Apply to capture (use as intrinsics)")){
				cap->setExternalCalibration(dc);
			}
			ImGui::SameLine();
			if(ImGui::Button("Clear external")){
				cap->clearExternalCalibration();
			}
			ImGui::Text("External calibration: %s",
			            cap->hasExternalCalibration() ? "active" : "none");

			ImGui::Separator();
			ImGui::Text("Undistort preview (Orbbec Preview panel):");
			const char* undistModes[] = { "Off", "On", "Compare (before/after)" };
			ImGui::SetNextItemWidth(200);
			ImGui::Combo("##undistmode", &settings.orbbecUndistortMode,
			             undistModes, IM_ARRAYSIZE(undistModes));
			ImGui::Checkbox("Use calibrated intrinsics for point cloud",
			                &settings.orbbecUseCalibratedIntrinsics);
		}
	}
	ImGui::End();
}

#else // !ORBBEC || !OPENCV

void SplatEditor::makeOrbbecCalibrationGUI() {}

#endif
