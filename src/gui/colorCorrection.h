
void SplatEditor::makeColorCorrectionGui(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& state = editor->state;

	if(settings.showColorCorrection){
	// if (ImGui::CollapsingHeader("colorCorrection Colors", ImGuiTreeNodeFlags_DefaultOpen)) {
		// ImGui::Text("colorCorrection Colors");

		ImVec2 windowSize = {320, 200};
		ImGui::SetNextWindowPos({
			(GLRenderer::width - windowSize.x) / 2, 
			(GLRenderer::height - windowSize.y) / 2, }, 
			ImGuiCond_Once);
		ImGui::SetNextWindowSize(windowSize, ImGuiCond_Once);
		
		// static float brightness = 0.0f;
		// static float contrast = 0.0f;
		// static float gamma = 1.0f;

		if(ImGui::Begin("Color Correction")){
			ImGui::SliderFloat("Brightness", &settings.colorCorrection.brightness, -150.0f, 150.0f, "%.0f");
			ImGui::SliderFloat("Contrast", &settings.colorCorrection.contrast, -100.0f, 100.0f, "%.0f");
			ImGui::SliderFloat("Gamma", &settings.colorCorrection.gamma, 0.01f, 3.0f, "%.2f");

			ImGui::Separator();

			ImGui::SliderFloat("Hue", &settings.colorCorrection.hue, -160.0f, 160.0f, "%.0f");
			ImGui::SliderFloat("Saturation", &settings.colorCorrection.saturation, -1.0f, 1.0f, "%.2f");
			ImGui::SliderFloat("Lightness", &settings.colorCorrection.lightness, -1.0f, 1.0f, "%.2f");

			ImVec2 buttonSize(100.f, 0.f);


			ImGui::PushItemWidth(-1.0f);
			if(ImGui::Button("Apply", buttonSize)){
				shared_ptr<SceneNode> selected = getSelectedNode();

				// Bake path: only Gaussian splat nodes have an in-place color
				// bake kernel (kernel_apply_colorCorrection rewrites
				// GaussianData.color). Point nodes (SNPoints / SNOrbbec) are
				// transient/streaming and have no equivalent bake, so for them
				// Apply just resets the sliders - the live render-time preview
				// (see SplatEditor_draw.h HQS color launch) already applies the
				// settings to selected point nodes each frame.
				bool matchedSplat = false;
				onTypeMatch<SNSplats>(selected, [editor, &matchedSplat](shared_ptr<SNSplats> node){
					editor->apply(node->dmng.data, editor->settings.colorCorrection);
					matchedSplat = true;
				});

				settings.colorCorrection = ColorCorrection();
				(void)matchedSplat;
			}

			// ImGui::SameLine();
			ImGui::SameLine(ImGui::GetWindowWidth() - 100);

			if(ImGui::Button("Revert", buttonSize)){
				settings.colorCorrection = ColorCorrection();
			}
		}
		ImGui::End();


	}


}