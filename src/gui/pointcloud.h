
// Point-cloud rendering panel.
//
// Exposes Splatshop's two point renderers through an ImGui window:
//   - HQS (the existing "draw all points every frame" path), and
//   - Progressive (the Skye-style port: reproject + budgeted hole-fill + VBO
//     rebuild via a per-pixel index image; converges over frames with bounded
//     per-frame cost).
//
// Also hosts a small "Load Point Cloud..." dialog reachable from the File menu,
// supporting the three formats wired into the drop callback: .las, .bin (Skye
// fast path), and point-cloud .ply.
#include <cstring>

#include "SplatEditor.h"
#include "../scene/SNPoints.h"
#include "../loader/LASLoader.h"
#include "../loader/BINPointCloudLoader.h"
#include "../loader/PointCloudPlyLoader.h"

using std::string;

// Tiny bytes formatter (unsuck's formatNumber is integer-only; we want KB/MB/GB).
static string fmtBytes(uint64_t b){
	double v = double(b);
	const char* unit = "B";
	if (v >= 1024.0) { v /= 1024.0; unit = "KB"; }
	if (v >= 1024.0) { v /= 1024.0; unit = "MB"; }
	if (v >= 1024.0) { v /= 1024.0; unit = "GB"; }
	char buf[64];
	snprintf(buf, sizeof(buf), "%.2f %s", v, unit);
	return string(buf);
}

void SplatEditor::makePointCloudGUI(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& scene = editor->scene;

	// --- Load Point Cloud dialog (opened from File ▸ Load Point Cloud...) ----
	if(settings.showPointCloudLoadDialog){
		ImGui::SetNextWindowSize(ImVec2(520, 160), ImGuiCond_FirstUseEver);
		static char pathBuf[512] = "";
		static int  formatIdx = 0; // 0=.las 1=.bin 2=.ply
		const char* formatNames[] = { "LAS (.las)", "BIN (.bin) — Skye fast path", "PLY (.ply) — point cloud" };

		ImGui::OpenPopup("Load Point Cloud##pcdlg");
		if(ImGui::BeginPopupModal("Load Point Cloud##pcdlg", &settings.showPointCloudLoadDialog, ImGuiWindowFlags_NoResize)){

			ImGui::TextDisabled("Path to the point-cloud file:");
			ImGui::PushItemWidth(-1);
			ImGui::InputText("##pcpath", pathBuf, sizeof(pathBuf));
			ImGui::PopItemWidth();

			ImGui::Combo("Format", &formatIdx, formatNames, IM_ARRAYSIZE(formatNames));

			ImGui::Separator();

			if(ImGui::Button("Load", ImVec2(120, 0))){
				string path(pathBuf);
				shared_ptr<Points> points;
				if(formatIdx == 0)      points = LasLoader::load(path);
				else if(formatIdx == 1) points = BINPointCloudLoader::load(path);
				else                    points = PointCloudPlyLoader::load(path);

				if(points){
					shared_ptr<SNPoints> node = make_shared<SNPoints>(points->name, points);
					scene.world->children.push_back(node);
					editor->setSelectedNode(node.get());
					Runtime::controls->focus(points->min, points->max, 1.0f);
				}
				settings.showPointCloudLoadDialog = false;
			}
			ImGui::SameLine();
			if(ImGui::Button("Cancel", ImVec2(120, 0))){
				settings.showPointCloudLoadDialog = false;
			}

			ImGui::EndPopup();
		}
	}

	if(!settings.showPointCloud) return;

	int width = 420;
	ImGui::SetNextWindowPos(ImVec2(40, 57 + 16), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(width, 420), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowBgAlpha(1.0f);

	static bool open = true;
	ImGuiWindowFlags flags = ImGuiWindowFlags_NoBringToFrontOnFocus;
	if(ImGui::Begin("Point Cloud", &open, flags)){

		// --- Renderer selection ------------------------------------------------
		ImGui::TextDisabled("Renderer");
		bool isHqs        = settings.pointRenderer == POINTRENDERER_HQS;
		bool isProgressive = settings.pointRenderer == POINTRENDERER_PROGRESSIVE;
		if(ImGui::Checkbox("HQS (draw all points)", &isHqs) && isHqs){
			settings.pointRenderer = POINTRENDERER_HQS;
		}
		ImGui::SameLine();
		if(ImGui::Checkbox("Progressive (Skye)", &isProgressive) && isProgressive){
			settings.pointRenderer = POINTRENDERER_PROGRESSIVE;
		}
		ImGui::TextDisabled("HQS draws every point each frame; Progressive converges over\n"
		                    "frames with a bounded per-frame fill budget.");

		ImGui::Separator();

		// --- Progressive controls (only relevant when Progressive is active) ---
		if(ImGui::CollapsingHeader("Progressive", ImGuiTreeNodeFlags_DefaultOpen)){

			// Fixed fill budget (points filled per frame). Skye uses 1–30M; the
			// higher the budget, the faster the image converges, at the cost of
			// per-frame time. Capped at 30M to keep the slider sane.
			uint32_t budget = settings.progressiveBudget;
			ImGui::Text("Fill budget: %s points/frame", formatNumber(budget).c_str());
			static const uint32_t budgetMin = 10'000;
			static const uint32_t budgetMax = 30'000'000;
			if(ImGui::SliderScalar("##pcbudget", ImGuiDataType_U32, &budget, &budgetMin, &budgetMax, "%u")){
				settings.progressiveBudget = budget;
			}

			// Point size (currently informational — the progressive kernels write
			// 1px points; this is wired for the Phase-3 point-size pass).
			ImGui::SliderFloat("Point size", &settings.progressivePointSize, 0.5f, 4.0f, "%.1f");

			// Adaptive budget (Phase 2): self-regulate the fill count against a
			// target frame time using CUDA-event-measured reproject+fill time.
			ImGui::Checkbox("Adaptive budget", &settings.progressiveAdaptiveBudget);
			if(settings.progressiveAdaptiveBudget){
				ImGui::SameLine();
				ImGui::PushItemWidth(120);
				ImGui::SliderFloat("target ms", &settings.progressiveTargetFrameMs, 4.0f, 33.3f, "%.1f");
				ImGui::PopItemWidth();
			}

			// Reset: drop the reproject buffer so the next frame rebuilds the
			// image from scratch (useful after large camera moves).
			if(ImGui::Button("Reset progressive")){
				settings.progressiveResetRequested = true;
			}
		}

		ImGui::Separator();

		// --- Per-cloud stats for the selected SNPoints ------------------------
		shared_ptr<SceneNode> sel = editor->getSelectedNode();
		SNPoints* node = sel ? dynamic_cast<SNPoints*>(sel.get()) : nullptr;

		if(!node){
			ImGui::TextDisabled("Select a point-cloud node to see its stats.");
		}else{
			ImGui::Text("Node: %s", node->name.c_str());

			auto& pd = node->manager.data;
			ImGui::Text("Loaded:   %s / %s points",
				formatNumber(pd.numUploaded).c_str(),
				formatNumber(int64_t(node->points->numPoints)).c_str());

			if(pd.count > 0u){
				ImGui::Text("Bounds:   (%.1f, %.1f, %.1f) → (%.1f, %.1f, %.1f)",
					pd.min.x, pd.min.y, pd.min.z, pd.max.x, pd.max.y, pd.max.z);
			}

			ImGui::Text("VRAM:     %s", fmtBytes(node->getGpuMemoryUsage()).c_str());

			if(node->progressiveInitialized){
				ImGui::Separator();
				ImGui::TextDisabled("Progressive (shuffled) buffers");
				ImGui::Text("Points:   %s", formatNumber(int64_t(node->progressive.count)).c_str());
				ImGui::Text("Prime:    %llu", (unsigned long long)node->progressive.prime);
				ImGui::Text("Buffers:  %u chunk(s) of up to %llu pts",
					node->progressive.numBuffers,
					(unsigned long long)node->progressive.pointsPerBuffer);
				ImGui::Text("Fill ofs: %s", formatNumber(int64_t(node->progressive.data.fillOffset)).c_str());
				ImGui::Text("VRAM:     %s", fmtBytes(node->progressive.getGpuMemoryUsage()).c_str());
				ImGui::Text("Ready:    %s", node->progressive.data.ready ? "yes" : "no (shuffling…)");
			}else if(settings.pointRenderer == POINTRENDERER_PROGRESSIVE){
				ImGui::TextDisabled("Progressive buffers not yet built.");
				ImGui::TextDisabled("(Built automatically once the cloud finishes uploading.)");
			}
		}
	}
	ImGui::End();
}
