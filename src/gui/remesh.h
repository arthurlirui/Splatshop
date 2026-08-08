
// Point-cloud remeshing / density optimization panel.
//
// Voxel-grid downsampling UI: pick a target spacing h, preview the expected
// output point count, and run SplatEditor::remeshPointCloud, which produces a
// new SNPoints node (one centroid per occupied voxel) while preserving the
// source cloud for comparison. Reachable via the toolbar/menubar toggle
// (settings.showRemesh).
#include "SplatEditor.h"
#include "../scene/SNPoints.h"

using std::string;

void SplatEditor::makeRemeshGUI(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;

	if(!settings.showRemesh) return;

	int width = 420;
	ImGui::SetNextWindowPos(ImVec2(40, 57 + 32), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(width, 300), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowBgAlpha(1.0f);

	static bool open = true;
	ImGuiWindowFlags flags = ImGuiWindowFlags_NoBringToFrontOnFocus;
	if(ImGui::Begin("Remesh / Density", &open, flags)){

		ImGui::TextDisabled("Voxel-grid downsampling");
		ImGui::TextDisabled("Collapses every occupied voxel of edge h to its centroid,\n"
		                    "producing a uniformly-spaced cloud. Source is preserved.");

		ImGui::Separator();

		shared_ptr<SceneNode> sel = editor->getSelectedNode();
		SNPoints* node = sel ? dynamic_cast<SNPoints*>(sel.get()) : nullptr;

		if(!node){
			ImGui::TextDisabled("Select a point-cloud node to remesh.");
		}else{
			auto& pd = node->manager.data;
			ImGui::Text("Node: %s", node->name.c_str());
			ImGui::Text("Input:   %s points", formatNumber(int64_t(pd.count)).c_str());
			if(pd.count > 0u){
				ImGui::Text("Bounds:  (%.1f, %.1f, %.1f) -> (%.1f, %.1f, %.1f)",
					pd.min.x, pd.min.y, pd.min.z, pd.max.x, pd.max.y, pd.max.z);
			}

			// Suggest a default voxel size from the bounding-box diagonal the
			// first time this cloud is seen (diagonal / 512 is a reasonable
			// starting density), but never clobber a user-tuned value.
			vec3 ext = pd.max - pd.min;
			float diag = sqrt(ext.x * ext.x + ext.y * ext.y + ext.z * ext.z);
			static SNPoints* lastNode = nullptr;
			if(lastNode != node){
				if(diag > 0.0f){
					settings.remeshVoxelSize = diag / 512.0f;
				}
				lastNode = node;
			}

			ImGui::Separator();

			// Voxel size slider (log-scale feels more natural for spacing).
			float h = settings.remeshVoxelSize;
			ImGui::Text("Voxel size h: %.5f", h);
			ImGui::PushItemWidth(-1);
			if(ImGui::SliderFloat("##remesh_h", &h, 0.0001f, 2.0f, "%.5f", ImGuiSliderFlags_Logarithmic)){
				settings.remeshVoxelSize = h;
			}
			ImGui::PopItemWidth();

			// Live estimate of the output point count (upper bound: volume / h^3,
			// clamped to input count — actual is the number of occupied voxels).
			if(h > 0.0f && pd.count > 0u){
				float vol = ext.x * ext.y * ext.z;
				double est = double(vol) / double(h * h * h);
				if(est > double(pd.count)) est = double(pd.count);
				ImGui::TextDisabled("Estimated output: ~%s points (upper bound)",
					formatNumber(int64_t(est)).c_str());
			}

			ImGui::Separator();

			// Adaptive fill is reserved for Phase-2 (splitting sparse regions).
			static bool adaptiveFill = false;
			ImGui::Checkbox("Adaptive fill (Phase 2 — not yet implemented)", &adaptiveFill);

			if(ImGui::Button("Remesh", ImVec2(120, 0))){
				double t0 = now();
				shared_ptr<SNPoints> out = editor->remeshPointCloud(node, settings.remeshVoxelSize, adaptiveFill);
				double dt = now() - t0;
				if(out){
					println("Remesh completed in {:.2f} s -> '{}'", dt, out->name);
				}else{
					println("Remesh failed (see console).");
				}
			}
			ImGui::SameLine();
			ImGui::TextDisabled("(creates a new node)");
		}
	}
	ImGui::End();
}
