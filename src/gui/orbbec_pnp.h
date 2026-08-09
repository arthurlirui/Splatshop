
// Orbbec extrinsic calibration (solvePnP) + depth correction panel.
//
// Detects a chessboard in the live IR frame, solves the board's pose
// relative to the camera (Rodrigues rvec + tvec in mm), and uses the
// pose-derived ground-truth distance to fit a linear depth correction
//   depth_true_mm = a * depth_measured_mm + b.
//
// The solved pose + fit are stored on editor->orbbecActiveCalibration
// (ExtrinsicPose / DepthCorrection) and persisted with the existing
// CalibrationStore. The depth-correction On/Off toggle that affects the
// preview lives in settings.orbbecDepthCorrectMode (consumed in
// orbbec_preview.h).
//
// Requires both the OrbbecSDK (SPLATSHOP_HAS_ORBBEC) and OpenCV
// (SPLATSHOP_HAS_OPENCV). Without either, compiles to an empty stub.
#include "SplatEditor.h"

#if defined(SPLATSHOP_HAS_ORBBEC) && defined(SPLATSHOP_HAS_OPENCV)

#include <algorithm>
#include <cstring>

#include "../camera/OrbbecCapture.h"
#include "../calibration/ExtrinsicCalibrator.h"
#include "../calibration/CalibrationStore.h"
#include "Calibration.h"

using std::string;
using orbbec::OrbbecCapture;
using orbbec::ExtrinsicCalibrator;
using orbbec::Intrinsics;
using orbbec::DistortionCoeffs;
using orbbec::DeviceCalibration;
using orbbec::CalibrationStore;

void SplatEditor::makeOrbbecPnPGUI(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;

	if(!settings.showOrbbecPnP) return;

	// Lazily create the extrinsic calibrator.
	if(!editor->orbbecExtCalibrator){
		editor->orbbecExtCalibrator = std::make_shared<ExtrinsicCalibrator>();
	}
	auto& extCal = editor->orbbecExtCalibrator;
	auto& cap = editor->orbbecCapture;

	ImGui::SetNextWindowSize(ImVec2(460, 720), ImGuiCond_FirstUseEver);
	if(ImGui::Begin("Orbbec Extrinsic (PnP) & Depth Correction", &settings.showOrbbecPnP)){

		if(!cap || !cap->isOpen()){
			ImGui::TextDisabled("Open an Orbbec device first (Orbbec panel).");
			ImGui::End();
			return;
		}

		// ==================================================================
		// Chessboard + live pose
		// ==================================================================
		if(ImGui::CollapsingHeader("Board & Pose", ImGuiTreeNodeFlags_DefaultOpen)){
			int cols = extCal->chessCols();
			int rows = extCal->chessRows();
			float sq = extCal->squareSizeMm();
			ImGui::InputInt("Inner corners (cols)", &cols);
			ImGui::InputInt("Inner corners (rows)", &rows);
			ImGui::InputFloat("Square size (mm)", &sq, 0.1f, 1.0f, "%.2f");
			if(ImGui::Button("Apply spec")){
				extCal->setChessboard(cols, rows, sq);
			}
			ImGui::SameLine();
			ImGui::TextDisabled("(changing the spec clears depth samples)");

			ImGui::Separator();
			// Intrinsics source: calibrated IR (preferred) vs device IR.
			ImGui::Checkbox("Use calibrated IR intrinsics",
			                &editor->orbbecPnPUseCalibIntrinsics);
			ImGui::SameLine();
			bool haveCalibIR = editor->orbbecActiveCalibration.ir.valid;
			ImGui::TextDisabled(haveCalibIR ? "(calibration present)"
			                                : "(no calibration - using device)");

			ImGui::Separator();
			auto frame = cap->getLatestFrame();
			if(!frame || !frame->irData){
				ImGui::TextDisabled("no IR frame - enable the IR stream in the Orbbec panel");
			} else {
				const uint8_t* src = (const uint8_t*)frame->irData->data;
				int w = frame->irWidth, h = frame->irHeight, ch = 1;

				// Pick the intrinsics/distortion to solve with.
				Intrinsics K;
				DistortionCoeffs D;
				if(editor->orbbecPnPUseCalibIntrinsics && haveCalibIR){
					K = editor->orbbecActiveCalibration.ir.intrinsics;
					D = editor->orbbecActiveCalibration.ir.distortion;
				} else {
					K = cap->getIrIntrinsics();
					D = cap->getIrDistortion();
				}

				// Solve the pose every frame for the overlay + live readout.
				orbbec::ExtrinsicPose pose;
				bool ok = extCal->solvePose(src, w, h, ch, K, D, pose);
				if(ok){
					editor->orbbecActiveCalibration.extrinsic = pose;
				}

				// Build + upload the BGR8 overlay (corners + pose axes).
				int ow = 0, oh = 0;
				extCal->buildOverlay(src, w, h, ch,
				                     editor->orbbecPnPOverlayScratch, ow, oh, K, D);

				GLuint& tex = editor->orbbecPnPTexOverlay;
				if(tex == 0 || editor->orbbecPnPTexOverlayW != ow ||
				   editor->orbbecPnPTexOverlayH != oh){
					if(tex == 0) glGenTextures(1, &tex);
					glBindTexture(GL_TEXTURE_2D, tex);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, ow, oh, 0,
					             GL_RGB, GL_UNSIGNED_BYTE, nullptr);
					editor->orbbecPnPTexOverlayW = ow;
					editor->orbbecPnPTexOverlayH = oh;
				}
				glBindTexture(GL_TEXTURE_2D, tex);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, ow, oh,
				                GL_BGR, GL_UNSIGNED_BYTE,
				                editor->orbbecPnPOverlayScratch.data());

				float regionW = ImGui::GetContentRegionAvail().x;
				float drawW = (float)ow, drawH = (float)oh;
				float scale = regionW / drawW;
				if(drawH * scale > ImGui::GetContentRegionAvail().y - 40)
					scale = (ImGui::GetContentRegionAvail().y - 40) / drawH;
				drawW *= scale; drawH *= scale;
				ImGui::Image((ImTextureID)(void*)(intptr_t)tex, ImVec2(drawW, drawH));

				if(ok){
					ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f),
					                   "Valid pose  dist=%.1f mm (%.3f m)",
					                   pose.distanceMm, pose.distanceMm / 1000.f);
					ImGui::Text("rvec=[%.3f, %.3f, %.3f]  tvec=[%.1f, %.1f, %.1f] mm",
					            pose.rvec[0], pose.rvec[1], pose.rvec[2],
					            pose.tvec[0], pose.tvec[1], pose.tvec[2]);
				} else {
					ImGui::TextDisabled("no chessboard detected");
				}
			}
		}

		// ==================================================================
		// Depth correction
		// ==================================================================
		if(ImGui::CollapsingHeader("Depth Correction", ImGuiTreeNodeFlags_DefaultOpen)){
			ImGui::TextDisabled("fit depth_true = a * depth_measured + b  (mm)");

			ImGui::Separator();
			auto frame = cap->getLatestFrame();
			bool canCapture = frame && frame->irData && frame->depthData &&
			                  extCal->lastDetectionOk();
			if(!canCapture){
				ImGui::TextDisabled("detect the board first to capture a sample");
			}

			if(ImGui::Button("Capture Sample") && canCapture){
				// Ground truth = solved pose distance.
				float trueMm = editor->orbbecActiveCalibration.extrinsic.distanceMm;
				// Measured depth = mean of the valid depth pixels inside the
				// detected corner bounding box (the board region), in mm.
				int w = frame->depthWidth, h = frame->depthHeight;
				const uint16_t* depth16 = (const uint16_t*)frame->depthData->data;
				float dscale = frame->depthScale;
				// The IR/depth frames share the depth sensor's resolution in
				// the C2D path; if they differ, fall back to the image centre.
				int x0 = 0, y0 = 0, x1 = w, y1 = h;
				if(frame->irWidth == w && frame->irHeight == h){
					// Use the corner bounding box from the last detection.
					// extCal->lastPose() carries no corners, so approximate
					// with the central 60% region as a robust default.
					x0 = w / 5; y0 = h / 5;
					x1 = w - w / 5; y1 = h - h / 5;
				}
				double sum = 0.0;
				int cnt = 0;
				for(int y = y0; y < y1; ++y){
					for(int x = x0; x < x1; ++x){
						uint16_t d = depth16[y * w + x];
						if(d == 0) continue;
						sum += (double)d * dscale;
						++cnt;
					}
				}
				if(cnt > 0 && trueMm > 0.f){
					float measuredMm = (float)(sum / cnt);
					extCal->addSample(trueMm, measuredMm);
					println("Orbbec PnP: sample  true={:.1f} mm  measured={:.1f} mm  (diff {:+.1f})",
					        trueMm, measuredMm, measuredMm - trueMm);
				}
			}
			ImGui::SameLine();
			if(ImGui::Button("Remove last")) extCal->removeLastSample();
			ImGui::SameLine();
			if(ImGui::Button("Clear")) extCal->clearSamples();

			ImGui::Text("Samples: %d  (aim for >= 4 over a range of distances)",
			            extCal->sampleCount());

			// Sample table.
			const auto& samples = extCal->samples();
			if(!samples.empty()){
				ImGui::Separator();
				ImGui::Text("  #   true(mm)   measured(mm)   diff(mm)");
				for(int i = 0; i < (int)samples.size(); ++i){
					float t = samples[i].first, m = samples[i].second;
					ImGui::Text("  [%d]  %8.1f   %10.1f   %+7.1f", i, t, m, m - t);
				}
			}

			ImGui::Separator();
			if(ImGui::Button("Fit a, b")){
				float a, b; double rms;
				if(extCal->fitDepthCorrection(a, b, rms)){
					auto& dc = editor->orbbecActiveCalibration.depthCorrection;
					dc.a = a; dc.b = b; dc.rmsMm = rms;
					dc.sampleCount = extCal->sampleCount();
					dc.valid = true;
				} else {
					auto& dc = editor->orbbecActiveCalibration.depthCorrection;
					dc.valid = false;
				}
			}

			auto& dc = editor->orbbecActiveCalibration.depthCorrection;
			if(dc.valid){
				ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f),
				                   "depth' = %.6f * depth %+.3f mm   RMS=%.3f mm  N=%d",
				                   dc.a, dc.b, dc.rmsMm, dc.sampleCount);
			} else {
				ImGui::TextDisabled("not fitted (capture >= 2 samples then Fit)");
			}

			// Manual fine-tune sliders (kept valid so the preview reacts live).
			ImGui::Separator();
			ImGui::Text("Manual tweak (applies to preview when correction is On):");
			float aTweak = dc.valid ? dc.a : 1.f;
			float bTweak = dc.valid ? dc.b : 0.f;
			bool chgA = ImGui::SliderFloat("a (scale)", &aTweak, 0.5f, 1.5f, "%.6f");
			bool chgB = ImGui::SliderFloat("b (offset mm)", &bTweak, -200.f, 200.f, "%.3f");
			if(chgA || chgB){
				dc.a = aTweak; dc.b = bTweak;
				dc.valid = true;
			}
		}

		// ==================================================================
		// Persistence + preview toggle
		// ==================================================================
		if(ImGui::CollapsingHeader("Persistence & Preview")){
			auto& dc = editor->orbbecActiveCalibration;

			ImGui::TextDisabled("pose + depth correction saved with the lens calibration");

			if(ImGui::Button("Save (default)")){
				int refW = dc.ir.valid ? dc.ir.intrinsics.w : cap->getIrIntrinsics().w;
				int refH = dc.ir.valid ? dc.ir.intrinsics.h : cap->getIrIntrinsics().h;
				string path;
				if(CalibrationStore::saveDefault(dc, refW, refH, path)){
					editor->orbbecCalibSavePath = path;
					dc.filePath = path;
				}
			}
			ImGui::SameLine();
			if(ImGui::Button("Load (default)")){
				string serial = dc.deviceSerial;
				int refW = cap->getIrIntrinsics().w;
				int refH = cap->getIrIntrinsics().h;
				if(CalibrationStore::loadForDevice(dc, serial, refW, refH)){
					editor->orbbecCalibLoadPath = dc.filePath;
				}
			}

			ImGui::Separator();
			ImGui::Text("Save path: %s", editor->orbbecCalibSavePath.c_str());
			ImGui::Text("Load path: %s", editor->orbbecCalibLoadPath.c_str());

			ImGui::Separator();
			ImGui::Text("Depth correction in Orbbec Preview:");
			const char* dcorModes[] = { "Off", "On" };
			ImGui::SetNextItemWidth(120);
			ImGui::Combo("##dcorpreview", &settings.orbbecDepthCorrectMode,
			             dcorModes, IM_ARRAYSIZE(dcorModes));
			ImGui::SameLine();
			ImGui::TextDisabled("(open the Orbbec Preview panel to see it)");
		}
	}
	ImGui::End();
}

#else // !ORBBEC || !OPENCV

void SplatEditor::makeOrbbecPnPGUI() {}

#endif
