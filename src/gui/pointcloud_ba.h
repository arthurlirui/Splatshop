// Bundle-Adjustment-style point-cloud refinement panel.
//
// Drives the optim::PointCloudBA optimizer on a selected SNPointCloudBA node.
// See docs/ba_research.md for the algorithm rationale (differentiable
// Gaussian-splatting optimization of point position + color, paradigm B).
//
// Prototype workflow:
//   1. Select a point-cloud node (SNPoints / SNPointCloudBA) in the layers.
//   2. Press "Capture target from view" — reads the current rendered
//      framebuffer (virt_framebuffer) back to host as HxWx3 uint8 RGB and
//      feeds it to the node's BA target. The desktop camera intrinsics are
//      derived from GLRenderer::camera (fovy + aspect + viewport).
//   3. Toggle "Optimize" — the per-frame update pass runs `stepsPerFrame`
//      AdamW iterations against the target and writes refined pos/color
//      back into the node's device buffers, so the renderer updates live.
//   4. Watch the loss curve descend.
#include "SplatEditor.h"
#include "../scene/SNPoints.h"
#include "../scene/SNPointCloudBA.h"

#include "GLRenderer.h"

#include <vector>
#include <cstring>

using std::string;

// Tiny bytes formatter (unsuck's formatNumber is integer-only).
static string fmtBytes2(uint64_t b){
	double v = double(b);
	const char* unit = "B";
	if (v >= 1024.0) { v /= 1024.0; unit = "KB"; }
	if (v >= 1024.0) { v /= 1024.0; unit = "MB"; }
	if (v >= 1024.0) { v /= 1024.0; unit = "GB"; }
	char buf[64];
	snprintf(buf, sizeof(buf), "%.2f %s", v, unit);
	return string(buf);
}

void SplatEditor::makePointCloudBAGUI(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;

	if(!settings.showPointCloudBA) return;

	int width = 440;
	ImGui::SetNextWindowPos(ImVec2(40, 57 + 48), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(width, 460), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowBgAlpha(1.0f);

	static bool open = true;
	ImGuiWindowFlags flags = ImGuiWindowFlags_NoBringToFrontOnFocus;
	if(ImGui::Begin("Bundle Adjustment", &open, flags)){

		ImGui::TextDisabled("Differentiable Gaussian-splatting refinement of point\n"
		                    "position + color (GPU, libtorch autograd). See\n"
		                    "docs/ba_research.md for the algorithm survey.");
		ImGui::Separator();

		shared_ptr<SceneNode> sel = editor->getSelectedNode();
		SNPointCloudBA* node = sel ? dynamic_cast<SNPointCloudBA*>(sel.get()) : nullptr;

		if(!node){
			ImGui::TextDisabled("Select a Bundle-Adjustment point-cloud node.");
			if(ImGui::Button("Convert selected point cloud to BA node")){
				shared_ptr<SceneNode> s = editor->getSelectedNode();
				SNPoints* src = s ? dynamic_cast<SNPoints*>(s.get()) : nullptr;
			if(src){
				// Create a new BA node sharing the same host Points, then
				// deep-copy the device buffers (position/color/flags) from the
				// source into the BA node's own manager. We must NOT borrow
				// src's raw pointers: those point at memory owned by src's
				// PointDataManager (vm_position/color/flags), and swapNode()
				// below drops the last reference to src, freeing that memory.
				// A deep device-to-device copy keeps baNode self-contained.
				auto baNode = make_shared<SNPointCloudBA>(src->name + "_BA");
				baNode->points = src->points;
				baNode->manager.data.transform = src->manager.data.transform;
				int64_t n = src->points->numPointsLoaded.load();
				if (n > 0) {
					baNode->manager.commit(n);
					baNode->manager.data.count = src->manager.data.count;
					baNode->manager.data.numUploaded = src->manager.data.numUploaded;
					// Deep-copy the device buffers into baNode's own (already
					// committed) virtual memory.
					CUstream stream = editor->mainstream;
					CURuntime::check(cuMemcpyDtoDAsync(
						reinterpret_cast<CUdeviceptr>(baNode->manager.data.position),
						reinterpret_cast<CUdeviceptr>(src->manager.data.position),
						size_t(n) * sizeof(vec3), stream));
					CURuntime::check(cuMemcpyDtoDAsync(
						reinterpret_cast<CUdeviceptr>(baNode->manager.data.color),
						reinterpret_cast<CUdeviceptr>(src->manager.data.color),
						size_t(n) * sizeof(uint32_t), stream));
					CURuntime::check(cuMemcpyDtoDAsync(
						reinterpret_cast<CUdeviceptr>(baNode->manager.data.flags),
						reinterpret_cast<CUdeviceptr>(src->manager.data.flags),
						size_t(n) * sizeof(uint32_t), stream));
					baNode->manager.data.min = src->manager.data.min;
					baNode->manager.data.max = src->manager.data.max;
				}
				editor->scene.swapNode(s, baNode);
				editor->setSelectedNode(baNode.get());
				println("Converted '{}' to a Bundle-Adjustment node.", src->name);
			}
			}
		}else{
			ImGui::Text("Node: %s", node->name.c_str());
			auto& pd = node->manager.data;
			ImGui::Text("Points:  %s / %s",
				formatNumber(pd.numUploaded).c_str(),
				formatNumber(int64_t(node->points->numPoints)).c_str());
			ImGui::Text("VRAM:    %s", fmtBytes2(node->getGpuMemoryUsage()).c_str());

			ImGui::Separator();
			ImGui::TextDisabled("Target frame");
			ImGui::Text("Target:  %d x %d", node->targetW, node->targetH);

			// --- Capture target from the current rendered view ---
			if(ImGui::Button("Capture target from view")){
				int W = GLRenderer::width;
				int H = GLRenderer::height;
				if (W > 0 && H > 0 && editor->virt_framebuffer->cptr != 0) {
					// The framebuffer is W*H uint64: low 32 bits = RGBA8
					// (R<<0 | G<<8 | B<<16 | A<<24). Read back + unpack to RGB.
					std::vector<uint64_t> fb(size_t(W) * H);
					CURuntime::check(cuStreamSynchronize(editor->mainstream));
					CURuntime::check(cuMemcpyDtoH(fb.data(),
						editor->virt_framebuffer->cptr,
						size_t(W) * H * sizeof(uint64_t)));
					std::vector<uint8_t> rgb(size_t(W) * H * 3);
					for (size_t i = 0; i < fb.size(); ++i) {
						uint32_t c = uint32_t(fb[i] & 0xffffffffu);
						rgb[i * 3 + 0] = uint8_t((c >>  0) & 0xff);
						rgb[i * 3 + 1] = uint8_t((c >>  8) & 0xff);
						rgb[i * 3 + 2] = uint8_t((c >> 16) & 0xff);
					}
					// Derive pinhole intrinsics from the desktop camera.
					// proj[0][0] = f/aspect = fx_ndc; fx_pixels = fx_ndc * W/2.
					// proj[1][1] = f         = fy_ndc; fy_pixels = fy_ndc * H/2.
					// Principal point at the image centre.
					auto& cam = *GLRenderer::camera;
					float f = 1.0f / float(std::tan(glm::radians(cam.fovy) / 2.0f));
					float aspect = float(W) / float(H);
					float fx = (f / aspect) * float(W) * 0.5f;
					float fy = (f)          * float(H) * 0.5f;
					float cx = float(W) * 0.5f;
					float cy = float(H) * 0.5f;
					// World->view for the BA camera. The point cloud lives in
					// world space; the differentiable renderer applies `view`.
					glm::mat4 view = glm::mat4(cam.view);
					std::vector<float> view4x4(16);
					for (int r = 0; r < 4; ++r)
						for (int c = 0; c < 4; ++c)
							view4x4[r * 4 + c] = view[c][r]; // glm is col-major
					node->setTargetFrame(rgb.data(), W, H,
					                     fx, fy, cx, cy, view4x4.data());
					println("BA: captured target {}x{} (fx={}, fy={})", W, H, fx, fy);
				}
			}

			ImGui::Separator();
			ImGui::TextDisabled("Optimization");

			auto& cfg = node->ba.config;
			ImGui::Checkbox("Optimize position", &cfg.optimizePosition);
			ImGui::SameLine();
			ImGui::Checkbox("Optimize color", &cfg.optimizeColor);

			ImGui::SliderFloat("lr position", &cfg.lrPosition, 1e-5f, 1e-2f, "%.5f");
			ImGui::SliderFloat("lr color",    &cfg.lrColor,    1e-4f, 1e-1f, "%.4f");
			ImGui::SliderFloat("init scale",  &cfg.initScale,  1e-4f, 5e-2f, "%.4f");
			ImGui::SliderInt  ("steps / frame", &cfg.stepsPerFrame, 1, 64);
			ImGui::SliderInt  ("max steps",     &cfg.maxSteps,    100, 20000);
			ImGui::SliderFloat("L1 weight",   &cfg.lambdaL1, 0.0f, 1.0f, "%.2f");
			ImGui::SliderFloat("SSIM weight", &cfg.lambdaSSIM, 0.0f, 1.0f, "%.2f");

			ImGui::Separator();

			if(!node->ba.status.running && !node->optimizeEnabled){
				if(ImGui::Button("Start optimize")){
					if (node->targetW > 0 && node->targetH > 0) {
						// Re-init from the current target + cloud, then enable.
						node->initRequested.store(true);
						node->optimizeEnabled = true;
					} else {
						println("BA: capture a target frame first.");
					}
				}
			}else if(node->optimizeEnabled){
				if(ImGui::Button("Stop")){
					node->stopOptimize();
				}
			}else{
				ImGui::TextDisabled("Reached max steps — reset to run again.");
				if(ImGui::Button("Reset")){
					node->ba.reset();
				}
			}

			ImGui::Separator();
			ImGui::TextDisabled("Status");
			auto& st = node->ba.status;
			ImGui::Text("Step:    %d / %d", st.step, cfg.maxSteps);
			ImGui::Text("Loss:    %.5f", st.loss);
			ImGui::Text("  L1:    %.5f", st.lossL1);
			ImGui::Text("  SSIM:  %.5f", st.lossSSIM);
			ImGui::Text("Running: %s", st.running ? "yes" : "no");
		}
	}
	ImGui::End();
}
