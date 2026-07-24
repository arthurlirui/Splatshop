
void SplatEditor::makeMenubar(){

	auto editor = SplatEditor::instance;
	auto& settings = editor->settings;
	auto& state = editor->state;
	auto& scene = editor->scene;

	auto startHighlightButtonIf = [&](bool condition){
		ImGuiStyle* style = &ImGui::GetStyle();
		ImVec4* colors = style->Colors;
		ImVec4 color = colors[ImGuiCol_Button];

		if(condition){
			color = ImVec4(0.06f, 0.53f, 0.98f, 1.00f);
			ImGui::PushStyleColor(ImGuiCol_Button, color);
		}else{
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.0f, 0.0f, 0.0f, 0.0f});
		}
	};

	auto endHighlightButtonIf = [&](){
		ImGui::PopStyleColor(1);
	};

	
	// ImGui::PushStyleColor(ImGuiCol_MenuBarBg, ImVec4(1.0f, 1.0f, 1.0f, 0.0f));

	if (ImGui::BeginMainMenuBar()){

		if (ImGui::BeginMenu("File")){

			// if (ImGui::MenuItem("New", "")) {
			// 	editor->resetEditor();
			// }

			// ImGui::Separator();


			// if (ImGui::MenuItem("Load", "")) {

			// }
			if (ImGui::MenuItem("Save", "")) {
				settings.showFileSaveDialog = true;
			}

			// ImGui::Separator();

			// if (ImGui::MenuItem("Quit", "Esc")) {
			// 	exit(54243);
			// }
			
			ImGui::EndMenu();
		}

		startHighlightButtonIf(settings.showToolbar);
		if(ImGui::Button("Toolbar")){
			settings.showToolbar = !settings.showToolbar;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(settings.showDevStuff);
		if(ImGui::Button("Dev&Debug")){
			settings.showDevStuff = !settings.showDevStuff;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(settings.showKernelInfos);
		if(ImGui::Button("Dev&Kernels")){
			settings.showKernelInfos = !settings.showKernelInfos;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(editor->settings.showMemoryInfos);
		if(ImGui::Button("Memory")){
			editor->settings.showMemoryInfos = !editor->settings.showMemoryInfos;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(editor->settings.showTimingInfos);
		if(ImGui::Button("Timings")){
			editor->settings.showTimingInfos = !editor->settings.showTimingInfos;
			Runtime::measureTimings = editor->settings.showTimingInfos;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(editor->settings.showEditing);
		if(ImGui::Button("Editing")){
			editor->settings.showEditing = !editor->settings.showEditing;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(editor->settings.showMotion);
		if(ImGui::Button("Motion")){
			editor->settings.showMotion = !editor->settings.showMotion;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(editor->settings.showColorCorrection);
		if(ImGui::Button("Color Correction")){
			editor->settings.showColorCorrection = !editor->settings.showColorCorrection;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(editor->settings.showStats);
		ImGui::SameLine();
		if(ImGui::Button("Stats")){
			editor->settings.showStats = !editor->settings.showStats;
		}
		endHighlightButtonIf();

		startHighlightButtonIf(editor->settings.showGettingStarted);
		ImGui::SameLine();
		if(ImGui::Button("Getting Started")){
			editor->settings.showGettingStarted = !editor->settings.showGettingStarted;
		}
		endHighlightButtonIf();

		ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 65);

		if(ImGui::MenuItem("Hide GUI", "")){
			settings.hideGUI = !settings.hideGUI;
		}

		ImGui::EndMainMenuBar();
	}

	// ImGui::PopStyleColor();

}