
// Orbbec RGBD real-time preview panel.
//
// Displays the live RGB video stream and the Depth stream (with a colormap)
// as 2D textures inside an ImGui window. The 3D point-cloud preview is not
// rendered here - it reuses the existing SNOrbbec scene node in the main 3D
// viewport (toggled from the Orbbec control panel). This panel only shows
// RGB + Depth 2D images plus point-cloud statistics.
//
// Implementation is pure GL + ImGui: RGBDFrame pixel data is uploaded to GL
// textures via glTexSubImage2D each frame and drawn with ImGui::Image. No
// external display dependency (OpenCV / Open3D) is required.
//
// When SPLATSHOP_HAS_ORBBEC is undefined makeOrbbecPreviewGUI() compiles to
// an empty stub.
#include "SplatEditor.h"

#ifdef SPLATSHOP_HAS_ORBBEC

#include "GL/glew.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "stb/stb_image.h"

#include "../camera/OrbbecCapture.h"
#include "../camera/OrbbecTypes.h"
#if defined(SPLATSHOP_HAS_OPENCV)
#include "../calibration/Calibrator.h"
#include "Calibration.h"
#endif

using std::vector;
using orbbec::OrbbecCapture;
using orbbec::RGBDFrame;

// ---------------------------------------------------------------------------
// Colormap lookup tables (256 entries, RGB).
//
// Generated analytically so there is no external data dependency. Each
// function fills a 256x3 byte array. The depth pixel (uint16, in mm) is
// normalized to [0,255] via a user-adjustable max-distance, then used as
// an index into the LUT.
// ---------------------------------------------------------------------------
namespace {

enum ColormapType { CM_TURBO = 0, CM_JET = 1, CM_GRAY = 2, CM_INFERNO = 3 };
constexpr int CM_COUNT = 4;

// Turbo colormap - polynomial approximation from Google's Turbo paper
// (Mikhailov 2019). Gives a perceptually uniform rainbow.
void genTurboLUT(uint8_t* lut) {
	for (int i = 0; i < 256; ++i) {
		float t = float(i) / 255.f;
		float r, g, b;
		// Red
		r = 0.13572138f + t * (4.61539260f + t * (-42.66032258f +
			t * (132.13108234f + t * (-152.94239396f + t * 59.28637943f))));
		// Green
		g = 0.09140171f + t * (2.19418839f + t * (4.84296658f +
			t * (-14.18503333f + t * (4.27729857f + t * 2.82956604f))));
		// Blue
		b = 0.10667330f + t * (12.64194608f + t * (-60.58204836f +
			t * (110.36276771f + t * (-89.99010972f + t * 27.64882782f))));
		lut[i*3+0] = (uint8_t)std::clamp(r * 255.f + 0.5f, 0.f, 255.f);
		lut[i*3+1] = (uint8_t)std::clamp(g * 255.f + 0.5f, 0.f, 255.f);
		lut[i*3+2] = (uint8_t)std::clamp(b * 255.f + 0.5f, 0.f, 255.f);
	}
}

// Jet colormap - classic MATLAB jet.
void genJetLUT(uint8_t* lut) {
	for (int i = 0; i < 256; ++i) {
		float v = float(i) / 255.f;
		float r = std::clamp(1.5f - std::abs(4.f * v - 3.f), 0.f, 1.f);
		float g = std::clamp(1.5f - std::abs(4.f * v - 2.f), 0.f, 1.f);
		float b = std::clamp(1.5f - std::abs(4.f * v - 1.f), 0.f, 1.f);
		lut[i*3+0] = (uint8_t)(r * 255.f + 0.5f);
		lut[i*3+1] = (uint8_t)(g * 255.f + 0.5f);
		lut[i*3+2] = (uint8_t)(b * 255.f + 0.5f);
	}
}

// Grayscale - simple identity.
void genGrayLUT(uint8_t* lut) {
	for (int i = 0; i < 256; ++i) {
		lut[i*3+0] = (uint8_t)i;
		lut[i*3+1] = (uint8_t)i;
		lut[i*3+2] = (uint8_t)i;
	}
}

// Inferno colormap - polynomial approximation (from matplotlib's inferno).
void genInfernoLUT(uint8_t* lut) {
	for (int i = 0; i < 256; ++i) {
		float t = float(i) / 255.f;
		float r = std::clamp(2.0f * t, 0.f, 1.f);
		float g = std::clamp(t * t * 1.3f, 0.f, 1.f);
		float b = std::clamp(0.5f + 0.5f * std::sin(3.14159f * t), 0.f, 1.f);
		lut[i*3+0] = (uint8_t)(r * 255.f + 0.5f);
		lut[i*3+1] = (uint8_t)(g * 255.f + 0.5f);
		lut[i*3+2] = (uint8_t)(b * 255.f + 0.5f);
	}
}

// Fill the selected colormap LUT into `lut` (must be 768 bytes).
void fillLUT(int type, uint8_t* lut) {
	switch (type) {
		case CM_TURBO:   genTurboLUT(lut);   break;
		case CM_JET:     genJetLUT(lut);     break;
		case CM_GRAY:    genGrayLUT(lut);    break;
		case CM_INFERNO: genInfernoLUT(lut); break;
		default:         genTurboLUT(lut);   break;
	}
}

// OBFormat constants come from the SDK headers (included transitively via
// OrbbecTypes.h -> <libobsensor/h/ObTypes.h>). We use them directly:
//   OB_FORMAT_YUYV=0, OB_FORMAT_MJPG=5, OB_FORMAT_Y16=8, OB_FORMAT_RGB=22

} // namespace

// ---------------------------------------------------------------------------
// Preview state held on SplatEditor (declared in SplatEditor.h under the
// #ifdef SPLATSHOP_HAS_ORBBEC block). The texture handles and scratch
// buffers are lazily allocated on first use.
// ---------------------------------------------------------------------------

void SplatEditor::makeOrbbecPreviewGUI() {

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;

	if (!settings.showOrbbecPreview) return;

	ImGui::SetNextWindowSize(ImVec2(680, 560), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Orbbec Preview", &settings.showOrbbecPreview)) {

		auto& cap = editor->orbbecCapture;
		if (!cap) {
			ImGui::TextDisabled("open the Orbbec control panel and connect a device first");
			ImGui::End();
			return;
		}

		// --- Toolbar ---
		if (ImGui::Button(settings.orbbecPreviewPaused ? "Resume" : "Pause")) {
			settings.orbbecPreviewPaused = !settings.orbbecPreviewPaused;
		}
		ImGui::SameLine();
		ImGui::Checkbox("Auto-fit", &settings.orbbecPreviewAutofit);
		ImGui::SameLine();
		ImGui::SetNextItemWidth(150);
		const char* cmNames[] = { "Turbo", "Jet", "Gray", "Inferno" };
		ImGui::Combo("Depth colormap", &settings.orbbecDepthColormap, cmNames, IM_ARRAYSIZE(cmNames));
		ImGui::SameLine();
		ImGui::SetNextItemWidth(100);
		ImGui::SliderFloat("Max dist (m)", &settings.orbbecDepthMaxMeters, 0.5f, 20.f, "%.1f");
#if defined(SPLATSHOP_HAS_OPENCV)
		ImGui::SameLine();
		ImGui::SetNextItemWidth(110);
		const char* undistModes[] = { "Undistort: Off", "Undistort: On", "Undistort: Compare" };
		ImGui::Combo("##undistmode", &settings.orbbecUndistortMode, undistModes, IM_ARRAYSIZE(undistModes));
#endif

		ImGui::Separator();

		// --- Fetch the latest frame ---
		std::shared_ptr<RGBDFrame> frame;
		if (!settings.orbbecPreviewPaused) {
			frame = cap->getLatestFrame();
			if (frame) editor->orbbecPreviewFrameIndex = frame->frameIndex;
		}
		// When paused, reuse the last-held frame pointer (orbbecPreviewHeldFrame).
		if (settings.orbbecPreviewPaused && editor->orbbecPreviewHeldFrame) {
			frame = editor->orbbecPreviewHeldFrame;
		} else if (frame) {
			editor->orbbecPreviewHeldFrame = frame;
		}

		if (!frame || (!frame->colorData && !frame->depthData)) {
			ImGui::TextDisabled("no frame yet - start streaming from the Orbbec panel");
			ImGui::End();
			return;
		}

		// Determine display sizes. In compare mode the two streams
		// (raw vs undistorted) stack vertically, so each gets the full
		// width; otherwise they sit side by side at half width each.
		float avail = ImGui::GetContentRegionAvail().x;
		float gap = 8.f;
		float halfW = (avail - gap) * 0.5f;
#if defined(SPLATSHOP_HAS_OPENCV)
		bool compareMode = (settings.orbbecUndistortMode == 2);
		bool undistOn    = (settings.orbbecUndistortMode >= 1);
#else
		bool compareMode = false;
		bool undistOn    = false;
#endif

		// ================================================================
		// RGB image
		// ================================================================
		// Helper: convert the raw color frame to an upload-ready byte
		// buffer (RGB/BGRA/YUYV/MJPG → a contiguous source buffer). Fills
		// outData / outGlFormat / outBpp / outStbDecoded (true if the
		// returned pointer must be freed via stbi_image_free). Returns
		// false on an unsupported/failed format (the caller has already
		// printed a message).
		auto prepareColor = [](const std::shared_ptr<RGBDFrame>& f,
		                       vector<uint8_t>& scratch,
		                       const uint8_t*& outData, GLenum& outGlFormat,
		                       int& outBpp, bool& outStbDecoded) -> bool {
			outData = nullptr; outGlFormat = GL_RGB; outBpp = 3; outStbDecoded = false;
			if (!f->colorData || f->colorWidth <= 0) return false;
			int fmt = f->colorFormat;
			int w = f->colorWidth, h = f->colorHeight;
			const uint8_t* src = (const uint8_t*)f->colorData->data;
			int64_t pixCount = (int64_t)w * h;
			if (fmt == OB_FORMAT_RGB) {
				outData = src; outGlFormat = GL_RGB; outBpp = 3;
			} else if (fmt == OB_FORMAT_BGR) {
				if ((int64_t)scratch.size() < pixCount * 3) scratch.resize(pixCount * 3);
				uint8_t* d = scratch.data();
				for (int64_t i = 0; i < pixCount; ++i) {
					d[i*3+0] = src[i*3+2]; d[i*3+1] = src[i*3+1]; d[i*3+2] = src[i*3+0];
				}
				outData = d; outGlFormat = GL_RGB; outBpp = 3;
			} else if (fmt == OB_FORMAT_BGRA) {
				outData = src; outGlFormat = GL_BGRA; outBpp = 4;
			} else if (fmt == OB_FORMAT_RGBA) {
				outData = src; outGlFormat = GL_RGBA; outBpp = 4;
			} else if (fmt == OB_FORMAT_YUYV) {
				if ((int64_t)scratch.size() < pixCount * 3) scratch.resize(pixCount * 3);
				uint8_t* d = scratch.data();
				for (int64_t i = 0; i < pixCount; i += 2) {
					int idx = (int)(i * 2);
					int y0 = src[idx], u = src[idx + 1] - 128;
					int y1 = src[idx + 2], v = src[idx + 3] - 128;
					d[i*3+0]   = (uint8_t)std::clamp(y0 + (int)(v * 1.402f), 0, 255);
					d[i*3+1]   = (uint8_t)std::clamp(y0 - (int)(u * 0.344f) - (int)(v * 0.714f), 0, 255);
					d[i*3+2]   = (uint8_t)std::clamp(y0 + (int)(u * 1.772f), 0, 255);
					if (i + 1 < pixCount) {
						d[(i+1)*3+0] = (uint8_t)std::clamp(y1 + (int)(v * 1.402f), 0, 255);
						d[(i+1)*3+1] = (uint8_t)std::clamp(y1 - (int)(u * 0.344f) - (int)(v * 0.714f), 0, 255);
						d[(i+1)*3+2] = (uint8_t)std::clamp(y1 + (int)(u * 1.772f), 0, 255);
					}
				}
				outData = d; outGlFormat = GL_RGB; outBpp = 3;
			} else if (fmt == OB_FORMAT_MJPG) {
				int dw = 0, dh = 0, dn = 0;
				uint8_t* decoded = stbi_load_from_memory(
					src, (uint32_t)f->colorData->size, &dw, &dh, &dn, 3);
				if (decoded && dw == w && dh == h) {
					outData = decoded; outGlFormat = GL_RGB; outBpp = 3; outStbDecoded = true;
				} else {
					if (decoded) stbi_image_free(decoded);
					return false;
				}
			} else {
				return false;
			}
			return true;
		};

		// Helper: upload a color byte buffer to a GL texture and draw it
		// with ImGui::Image, preserving aspect ratio.
		auto drawColor = [&settings](const char* label, const uint8_t* data, int w, int h,
		                             GLenum glFormat, int bpp, float panelW,
		                             GLuint& tex, int& texW, int& texH, int& texBpp) {
			ImGui::BeginChild(label, ImVec2(panelW, 0), true);
			ImGui::Text("%s", label + 2); // skip "##" prefix
			if (!data) { ImGui::TextDisabled("no data"); ImGui::EndChild(); return; }
			if (tex == 0 || texW != w || texH != h || texBpp != bpp) {
				if (tex == 0) glGenTextures(1, &tex);
				glBindTexture(GL_TEXTURE_2D, tex);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, glFormat, GL_UNSIGNED_BYTE, nullptr);
				texW = w; texH = h; texBpp = bpp;
			}
			glBindTexture(GL_TEXTURE_2D, tex);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, glFormat, GL_UNSIGNED_BYTE, data);
			float drawW = (float)w, drawH = (float)h;
			if (settings.orbbecPreviewAutofit) {
				float regionW = ImGui::GetContentRegionAvail().x;
				float regionH = ImGui::GetContentRegionAvail().y - 20;
				float s = std::min(regionW / drawW, regionH / drawH);
				drawW *= s; drawH *= s;
			}
			ImGui::Image((ImTextureID)(void*)(intptr_t)tex, ImVec2(drawW, drawH));
			ImGui::Text("%dx%d  bpp=%d", w, h, bpp);
			ImGui::EndChild();
		};

		if (frame->colorData && frame->colorWidth > 0 && frame->colorHeight > 0) {
			// Prepare the raw (pre-undistortion) upload buffer.
			const uint8_t* rawData = nullptr;
			GLenum rawFmt = GL_RGB; int rawBpp = 3; bool stbDecoded = false;
			if (!prepareColor(frame, editor->orbbecColorScratch,
			                  rawData, rawFmt, rawBpp, stbDecoded)) {
				ImGui::TextDisabled("unsupported color format = %d", frame->colorFormat);
			} else {
				int w = frame->colorWidth, h = frame->colorHeight;
#if defined(SPLATSHOP_HAS_OPENCV)
				bool canUndistort = undistOn &&
				    editor->orbbecActiveCalibration.color.valid &&
				    editor->orbbecActiveCalibration.color.intrinsics.w == w &&
				    editor->orbbecActiveCalibration.color.intrinsics.h == h;
				const uint8_t* undistData = nullptr;
				GLenum undistFmt = rawFmt; int undistBpp = rawBpp;
				if (canUndistort) {
					auto& cal = editor->orbbecCalibrator;
					if (cal) {
						cal->ensureUndistortMaps(editor->orbbecActiveCalibration.color,
						                         w, h, 0.f);
						if (cal->hasUndistortMaps(w, h)) {
							size_t need = (size_t)w * h * rawBpp;
							editor->orbbecColorScratchUndist.resize(need);
							cal->undistort(rawData, editor->orbbecColorScratchUndist.data(),
							               w, h, rawBpp, /*isDepth=*/false);
							undistData = editor->orbbecColorScratchUndist.data();
						}
					}
				}

				if (compareMode && undistData) {
					// Side-by-side: raw (left) vs undistorted (right).
					float qw = (ImGui::GetContentRegionAvail().x - gap) * 0.5f;
					drawColor("##orbbec_rgb_raw", rawData, w, h, rawFmt, rawBpp, qw,
					          editor->orbbecTexColor, editor->orbbecTexColorW,
					          editor->orbbecTexColorH, editor->orbbecTexColorBpp);
					ImGui::SameLine();
					drawColor("##orbbec_rgb_undist", undistData, w, h, undistFmt, undistBpp, qw,
					          editor->orbbecTexColorUndist, editor->orbbecTexColorUndistW,
					          editor->orbbecTexColorUndistH, editor->orbbecTexColorUndistBpp);
				} else if (undistOn && undistData) {
					// On: show only the undistorted image.
					drawColor("##orbbec_rgb", undistData, w, h, undistFmt, undistBpp, halfW,
					          editor->orbbecTexColorUndist, editor->orbbecTexColorUndistW,
					          editor->orbbecTexColorUndistH, editor->orbbecTexColorUndistBpp);
				} else {
					// Off (or undistort unavailable): show the raw image.
					drawColor("##orbbec_rgb", rawData, w, h, rawFmt, rawBpp, halfW,
					          editor->orbbecTexColor, editor->orbbecTexColorW,
					          editor->orbbecTexColorH, editor->orbbecTexColorBpp);
				}
#else
				drawColor("##orbbec_rgb", rawData, w, h, rawFmt, rawBpp, halfW,
				          editor->orbbecTexColor, editor->orbbecTexColorW,
				          editor->orbbecTexColorH, editor->orbbecTexColorBpp);
#endif
				if (stbDecoded) stbi_image_free((void*)rawData);
			}
		} else {
			ImGui::BeginChild("##orbbec_rgb", ImVec2(halfW, 0), true);
			ImGui::Text("RGB");
			ImGui::TextDisabled("no color frame");
			ImGui::EndChild();
		}

	depth_section:
		ImGui::SameLine();

		// ================================================================
		// Depth image (with colormap)
		//
		// In compare mode, show raw (pre-filter) and denoised depth
		// side-by-side. Otherwise show a single denoised depth image.
		// ================================================================

		// Helper lambda: render a depth image from an RGBDFrame into a GL
		// texture + ImGui::Image. Reuses the shared colormap LUT.
		// `forceUndistort`: when true, the depth map is software-undistorted
		// (using the IR stream's calibration, nearest-neighbour) before the
		// colormap pass. When false the raw depth is shown as-is.
		auto renderDepth = [&settings, editor](
			const char* label, const std::shared_ptr<RGBDFrame>& df,
			GLuint& tex, int& texW, int& texH,
			vector<uint8_t>& scratch, float panelW, bool forceUndistort)
		{
			ImGui::BeginChild(label, ImVec2(panelW, 0), true);
			ImGui::Text("%s", label + 2); // skip "##" prefix
			if (df && df->depthData && df->depthWidth > 0 && df->depthHeight > 0) {
				int w = df->depthWidth;
				int h = df->depthHeight;
				float scale = df->depthScale;

				// Lazy-create or resize the GL texture.
				if (tex == 0 || texW != w || texH != h) {
					if (tex == 0) glGenTextures(1, &tex);
					glBindTexture(GL_TEXTURE_2D, tex);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
					texW = w;
					texH = h;
				}

				// Ensure the LUT is generated.
				if (editor->orbbecDepthLUT.empty() ||
				    editor->orbbecDepthLUTType != settings.orbbecDepthColormap) {
					editor->orbbecDepthLUT.resize(768);
					fillLUT(settings.orbbecDepthColormap, editor->orbbecDepthLUT.data());
					editor->orbbecDepthLUTType = settings.orbbecDepthColormap;
				}

				// Normalize + colormap.
				int64_t pixCount = (int64_t)w * h;
				if ((int64_t)scratch.size() < pixCount * 3)
					scratch.resize(pixCount * 3);
				const uint16_t* depth16 = (const uint16_t*)df->depthData->data;
#if defined(SPLATSHOP_HAS_OPENCV)
				// Software undistortion of the depth map (nearest-neighbour so
				// no synthetic depth values are introduced). Uses the IR
				// stream's calibration — the depth sensor and IR are co-located.
				// Uses a per-editor member buffer (not thread_local) so the raw
				// and undistorted renders in compare mode don't clobber each
				// other's intermediate storage.
				if (forceUndistort &&
				    editor->orbbecActiveCalibration.ir.valid &&
				    editor->orbbecActiveCalibration.ir.intrinsics.w == w &&
				    editor->orbbecActiveCalibration.ir.intrinsics.h == h) {
					auto& cal = editor->orbbecCalibrator;
					if (cal) {
						cal->ensureUndistortMaps(editor->orbbecActiveCalibration.ir,
						                         w, h, 0.f);
						if (cal->hasUndistortMaps(w, h)) {
							editor->orbbecDepthUndist16.resize((size_t)w * h);
							cal->undistortDepth(depth16, editor->orbbecDepthUndist16.data(),
							                    w, h);
							depth16 = editor->orbbecDepthUndist16.data();
						}
					}
				}
#endif
				uint8_t* dst = scratch.data();
				const uint8_t* lut = editor->orbbecDepthLUT.data();
				float maxMm = settings.orbbecDepthMaxMeters * 1000.f;
				for (int64_t i = 0; i < pixCount; ++i) {
					float mm = (float)depth16[i] * scale;
					int idx = (mm <= 0.f) ? 0 : (int)(mm / maxMm * 255.f + 0.5f);
					if (idx > 255) idx = 255;
					if (idx < 0) idx = 0;
					dst[i*3+0] = lut[idx*3+0];
					dst[i*3+1] = lut[idx*3+1];
					dst[i*3+2] = lut[idx*3+2];
				}

				glBindTexture(GL_TEXTURE_2D, tex);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, dst);

				float drawW = (float)w, drawH = (float)h;
				if (settings.orbbecPreviewAutofit) {
					float regionW = ImGui::GetContentRegionAvail().x;
					float regionH = ImGui::GetContentRegionAvail().y - 20;
					float s = std::min(regionW / drawW, regionH / drawH);
					drawW *= s;
					drawH *= s;
				}
				ImGui::Image((ImTextureID)(void*)(intptr_t)tex, ImVec2(drawW, drawH));
				ImGui::Text("%dx%d  scale=%.4fmm", w, h, scale);
			} else {
				ImGui::TextDisabled("no depth frame");
			}
			ImGui::EndChild();
		};

#if defined(SPLATSHOP_HAS_OPENCV)
		// Undistortion compare mode takes precedence over the denoise compare
		// mode (both can't stack cleanly in one panel width).
		if (compareMode && editor->orbbecActiveCalibration.ir.valid) {
			// Side-by-side: raw depth (left) vs undistorted depth (right).
			float qw = (ImGui::GetContentRegionAvail().x - gap) * 0.5f;
			renderDepth("##orbbec_depth_raw", frame,
			            editor->orbbecTexDepth,
			            editor->orbbecTexDepthW, editor->orbbecTexDepthH,
			            editor->orbbecDepthScratch, qw, /*forceUndistort=*/false);
			ImGui::SameLine();
			renderDepth("##orbbec_depth_undist", frame,
			            editor->orbbecTexDepthUndist,
			            editor->orbbecTexDepthUndistW, editor->orbbecTexDepthUndistH,
			            editor->orbbecDepthScratchUndist, qw, /*forceUndistort=*/true);
		} else
#endif
		if (settings.orbbecDenoiseCompare) {
			// Side-by-side: raw (pre-filter) vs denoised depth.
			auto rawFrame = cap->getLatestRawFrame();
			float quarterW = (ImGui::GetContentRegionAvail().x - gap) * 0.5f;
			renderDepth("##orbbec_depth_raw", rawFrame,
			            editor->orbbecTexDepthRaw,
			            editor->orbbecTexDepthRawW, editor->orbbecTexDepthRawH,
			            editor->orbbecDepthScratchRaw, quarterW, /*forceUndistort=*/false);
			ImGui::SameLine();
			renderDepth("##orbbec_depth_denoised", frame,
			            editor->orbbecTexDepth,
			            editor->orbbecTexDepthW, editor->orbbecTexDepthH,
			            editor->orbbecDepthScratch, quarterW,
#if defined(SPLATSHOP_HAS_OPENCV)
			            /*forceUndistort=*/undistOn
#else
			            /*forceUndistort=*/false
#endif
			);
		} else {
			renderDepth("##orbbec_depth", frame,
			            editor->orbbecTexDepth,
			            editor->orbbecTexDepthW, editor->orbbecTexDepthH,
			            editor->orbbecDepthScratch, halfW,
#if defined(SPLATSHOP_HAS_OPENCV)
			            /*forceUndistort=*/undistOn
#else
			            /*forceUndistort=*/false
#endif
			);
		}

		// ================================================================
		// Point cloud statistics (3D rendering available via the dedicated
		// "Orbbec Point Cloud" panel, toggled from the "PC View" menu button)
		// ================================================================
		ImGui::Separator();
		if (cap->isPointCloudEnabled()) {
			auto pc = cap->getLatestPointCloud();
			if (pc && pc->colorData && pc->colorWidth > 0) {
				ImGui::Text("Point cloud: %d points  (open \"PC View\" for the 3D window)",
				            pc->colorWidth);
			} else {
				ImGui::TextDisabled("point cloud enabled, waiting for first cloud...");
			}
		} else {
			ImGui::TextDisabled("point cloud not enabled (toggle in Orbbec panel)");
		}
		ImGui::SameLine();
		ImGui::Text("FPS: %.1f", cap->getMeasuredFps());
#if defined(SPLATSHOP_HAS_OPENCV)
		ImGui::Separator();
		if (settings.orbbecUseCalibratedIntrinsics && cap->hasExternalCalibration()) {
			ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f),
			                   "Calibrated intrinsics active");
			ImGui::TextDisabled("(SDK point cloud uses device intrinsics; "
			                    "use undistort preview for software correction)");
		} else {
			ImGui::TextDisabled("Intrinsics: device default");
		}
#endif
	}
	ImGui::End();
}

// ---------------------------------------------------------------------------
// Orbbec live point-cloud display panel.
//
// A dedicated ImGui window that shows the streaming SNOrbbec cloud as a 3D
// image. The cloud is rendered into a small RenderTarget (640x480) by the
// HQS point renderer in SplatEditor::render() and blitted to
// orbbecTexPointCloud; this panel simply displays that texture with
// ImGui::Image and provides a self-contained orbit camera (drag to rotate,
// wheel to zoom) plus auto-fit-to-AABB. It does not touch GLRenderer::camera.
// ---------------------------------------------------------------------------
void SplatEditor::makeOrbbecPointCloudGUI() {

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;

	if (!settings.showOrbbecPointCloud) return;

	ImGui::SetNextWindowSize(ImVec2(680, 560), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Orbbec Point Cloud", &settings.showOrbbecPointCloud)) {

		auto& cap = editor->orbbecCapture;
		auto& node = editor->snOrbbec;

		// --- Toolbar ---
		if (ImGui::Button(settings.orbbecPCPaused ? "Resume" : "Pause")) {
			settings.orbbecPCPaused = !settings.orbbecPCPaused;
		}
		ImGui::SameLine();
		ImGui::Checkbox("Auto-fit", &settings.orbbecPCAutoFit);
		ImGui::SameLine();
		ImGui::SetNextItemWidth(120);
		ImGui::SliderFloat("Point size", &settings.orbbecPCPointSize, 0.0f, 4.0f, "%.1f");
		ImGui::SameLine();
		if (cap) {
			ImGui::Text("FPS: %.1f", cap->getMeasuredFps());
		}

		ImGui::Separator();

		if (!cap) {
			ImGui::TextDisabled("open the Orbbec control panel and connect a device first");
			ImGui::End();
			return;
		}

		// Lazily create the SNOrbbec scene node the panel renders from, so the
		// user gets a live 3D stream as soon as they enable "Generate RGB
		// point cloud". Mirrors the "Show preview" button in the Orbbec panel.
		if (!node && cap->isPointCloudEnabled()) {
			editor->snOrbbec = std::make_shared<SNOrbbec>("Orbbec Live Cloud");
			editor->snOrbbec->lastFrameIndex = 0;
			editor->snOrbbec->pcCameraInited = false;
			scene.world->children.push_back(editor->snOrbbec);
			node = editor->snOrbbec;
		}

		// Feed the latest cloud into the node (same logic as the Orbbec panel,
		// so the panel works even if the user never opened that one).
		if (node && cap->isPointCloudEnabled() && !settings.orbbecPCPaused) {
			auto frame = cap->getLatestPointCloud();
			if (frame && frame->colorData && frame->colorWidth > 0 &&
			    frame->frameIndex != node->lastFrameIndex) {
				node->lastFrameIndex = frame->frameIndex;
				node->loadPointCloud(
					(const uint8_t*)frame->colorData->data, frame->colorWidth);
			}
		}

		if (!node || node->manager.data.count == 0) {
			ImGui::TextDisabled(
				"no cloud - enable \"Generate RGB point cloud\" in the Orbbec panel");
			ImGui::End();
			return;
		}

		if (!cap->isPointCloudEnabled()) {
			ImGui::TextDisabled("point cloud generation is disabled");
			ImGui::End();
			return;
		}

		// --- 3D image ---
		GLuint tex = editor->orbbecTexPointCloud;
		int texW = editor->orbbecTexPointCloudW;
		int texH = editor->orbbecTexPointCloudH;
		if (tex == 0 || texW <= 0 || texH <= 0) {
			ImGui::TextDisabled("waiting for first rendered frame...");
			ImGui::End();
			return;
		}

		// Aspect-preserving fit into the available region.
		float availW = ImGui::GetContentRegionAvail().x;
		float availH = ImGui::GetContentRegionAvail().y - 36; // leave room for status
		float drawW = (float)texW, drawH = (float)texH;
		float scale = std::min(availW / drawW, availH / drawH);
		if (scale < 1.0f) { drawW *= scale; drawH *= scale; }

		// Capture mouse input over the image for the self-contained orbit
		// camera. Drag = rotate yaw/pitch, wheel = zoom radius.
		ImGui::Image((ImTextureID)(void*)(intptr_t)tex, ImVec2(drawW, drawH));
		bool hovered = ImGui::IsItemHovered();

		if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
			ImVec2 delta = ImGui::GetIO().MouseDelta;
			node->pcYaw   -= delta.x / 300.0f;
			node->pcPitch += delta.y / 300.0f;
			node->pcPitch = std::clamp(node->pcPitch, -1.5534f, 1.5534f);
			settings.orbbecPCAutoFit = false; // user took manual control
		}
		if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
			float w = ImGui::GetIO().MouseWheel;
			node->pcRadius *= (w > 0) ? (1.0f / 1.1f) : 1.1f;
			node->pcRadius = std::clamp(node->pcRadius, 0.05f, 100.0f);
			settings.orbbecPCAutoFit = false;
		}

		// --- Status line ---
		PointData& pd = node->manager.data;
		ImGui::Text("points: %u   size: %.2fx%.2fx%.2f m   tex: %dx%d",
		            pd.count,
		            pd.max.x - pd.min.x, pd.max.y - pd.min.y, pd.max.z - pd.min.z,
		            texW, texH);
	}
	ImGui::End();
}

#else // !SPLATSHOP_HAS_ORBBEC

void SplatEditor::makeOrbbecPreviewGUI() {}
void SplatEditor::makeOrbbecPointCloudGUI() {}

#endif // SPLATSHOP_HAS_ORBBEC
