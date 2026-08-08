
void SplatEditor::render(){

	cuStreamSynchronize(0);
	cuStreamSynchronize(mainstream);

	if(GLRenderer::width * GLRenderer::height == 0){
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		return;
	}

	{ // adjust framebuffer size
		GLRenderer::view.framebuffer->setSize(GLRenderer::width, GLRenderer::height);

		uint64_t requiredBytes = GLRenderer::width * GLRenderer::height * 8;
		virt_framebuffer->commit(requiredBytes);
	}

	glBindFramebuffer(GL_FRAMEBUFFER, GLRenderer::view.framebuffer->handle);

	static CUevent event_render_start;
	static CUevent event_render_end;
	static bool initialized = false;
	if(!initialized){
		cuEventCreate(&event_render_start, CU_EVENT_DEFAULT);
		cuEventCreate(&event_render_end, CU_EVENT_DEFAULT);
		initialized = true;
	}
	
	if(ovr->isActive() || viewmode == VIEWMODE_REMOTE_STEREO)
	{ // RENDER VR

		if(Runtime::measureTimings){
			cuEventRecord(event_render_start, 0);
		}

		// Per-eye poses/projection are assembled in SplatEditor_update.h and
		// stored in viewLeft/viewRight. For local VR we still need the HMD/eye
		// poses and recommended target size from OpenVR here; for the remote
		// path they are already in viewLeft/viewRight and the resolution comes
		// from remoteStereo.
		int width;
		int height;
		if(viewmode == VIEWMODE_REMOTE_STEREO) {
			// width/height are written by cmd_vr_pose under remoteStereo.mutex;
			// read them under the same lock to avoid a data race.
			std::lock_guard<std::mutex> lk(remoteStereo.mutex);
			width  = remoteStereo.width;
			height = remoteStereo.height;
		} else {
			auto size = ovr->getRecommmendedRenderTargetSize();
			width  = size[0];
			height = size[1];
		}

		Runtime::debugValues["vr resolution"] = format("{} x {}", width, height);

		if(!settings.enableOverlapped)
		{ // LEFT

			virt_framebuffer->commit(width * height * 8);

			RenderTarget target;
			target.width = width;
			target.height = height;
			target.framebuffer = (uint64_t*)virt_framebuffer->cptr;
			target.indexbuffer = nullptr;
			target.view = mat4(viewLeft.view);// * scene.transform;
			target.proj = viewLeft.proj;
			target.VP = viewLeft.VP;

			vector<RenderTarget> targets = { target };
			draw(&scene, targets);

			Rectangle targetViewport;
			targetViewport.x = 0;
			targetViewport.y = 0;
			targetViewport.width = width;
			targetViewport.height = height;

			auto glMapping = mapCudaGl(viewLeft.framebuffer->colorAttachments[0]);
			prog_gaussians_rendering->launch("kernel_blit_opengl", 
				{&launchArgs, &target, &glMapping.surface, &targetViewport}, 
				targetViewport.width * targetViewport.height);
			glMapping.unmap();
		}

		if(!settings.enableOverlapped)
		{ // RIGHT
			virt_framebuffer->commit(width * height * 8);

			RenderTarget target;
			target.width = width;
			target.height = height;
			target.framebuffer = (uint64_t*)virt_framebuffer->cptr;
			target.indexbuffer = nullptr;
			target.view = mat4(viewRight.view); // * scene.transform;
			target.proj = viewRight.proj;
			target.VP = viewRight.VP;

			vector<RenderTarget> targets = { target };
			draw(&scene, targets);

			Rectangle targetViewport;
			targetViewport.x = 0;
			targetViewport.y = 0;
			targetViewport.width = width;
			targetViewport.height = height;

			auto glMapping = mapCudaGl(viewRight.framebuffer->colorAttachments[0]);
			prog_gaussians_rendering->launch("kernel_blit_opengl", 
				{&launchArgs, &target, &glMapping.surface, &targetViewport}, 
				targetViewport.width * targetViewport.height);
			glMapping.unmap();
		}

		if(settings.enableOverlapped)
		{ // LEFT & RIGHT
			shared_ptr<CudaVirtualMemory> virt_framebuffer_left = virt_framebuffer;
			static shared_ptr<CudaVirtualMemory> virt_framebuffer_right = CURuntime::allocVirtual("framebuffer_right");

			virt_framebuffer_left->commit(width * height * 8);
			virt_framebuffer_right->commit(width * height * 8);

			RenderTarget target_left;
			target_left.width = width;
			target_left.height = height;
			target_left.framebuffer = (uint64_t*)virt_framebuffer_left->cptr;
			target_left.indexbuffer = nullptr;
			target_left.view = mat4(viewLeft.view); 
			target_left.proj = viewLeft.proj;
			target_left.VP = viewLeft.VP;
			target_left.isLeft = true;
			target_left.isRight = false;

			RenderTarget target_right;
			target_right.width = width;
			target_right.height = height;
			target_right.framebuffer = (uint64_t*)virt_framebuffer_right->cptr;
			target_right.indexbuffer = nullptr;
			target_right.view = mat4(viewRight.view); 
			target_right.proj = viewRight.proj;
			target_right.VP = viewRight.VP;
			target_right.isLeft = false;
			target_right.isRight = true;

			vector<RenderTarget> targets = { target_left, target_right };
			draw(&scene, targets);

			Rectangle targetViewport;
			targetViewport.x = 0;
			targetViewport.y = 0;
			targetViewport.width = width;
			targetViewport.height = height;

			auto glMapping_left = mapCudaGl(viewLeft.framebuffer->colorAttachments[0]);
			auto glMapping_right = mapCudaGl(viewRight.framebuffer->colorAttachments[0]);

			prog_gaussians_rendering->launch("kernel_blit_opengl", 
				{&launchArgs, &target_left, &glMapping_left.surface, &targetViewport}, 
				targetViewport.width * targetViewport.height);
			prog_gaussians_rendering->launch("kernel_blit_opengl", 
				{&launchArgs, &target_right, &glMapping_right.surface, &targetViewport}, 
				targetViewport.width * targetViewport.height);

			glMapping_left.unmap();
			glMapping_right.unmap();

		}

		// blit vr framebuffers to desktop framebuffer
		glBlitNamedFramebuffer(viewLeft.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
			0, 0, width, height,
			0, 0, GLRenderer::view.framebuffer->width / 2, GLRenderer::view.framebuffer->height,
			GL_COLOR_BUFFER_BIT, GL_LINEAR
		);
		
		glBlitNamedFramebuffer(viewRight.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
			0, 0, width, height,
			GLRenderer::view.framebuffer->width / 2, 0, GLRenderer::view.framebuffer->width, GLRenderer::view.framebuffer->height,
			GL_COLOR_BUFFER_BIT, GL_LINEAR
		);

		// Publish the rendered stereo eyes to the VR remote-stereo video
		// channel (WebSocket frame server). Cheap no-op when no client is
		// subscribed. Skipped for local OpenVR (which submits to the HMD
		// directly below) unless someone is also watching the stream.
		if(viewmode == VIEWMODE_REMOTE_STEREO) {
			remote::FrameStreamer::instance().captureFrame(
				viewLeft.framebuffer->colorAttachments[0]->handle,
				viewRight.framebuffer->colorAttachments[0]->handle,
				width, height);
		}

		if(Runtime::measureTimings){
			cuEventRecord(event_render_end, 0);

			cuCtxSynchronize();

			float duration;
			cuEventElapsedTime(&duration, event_render_start, event_render_end);

			Runtime::timings.add("[render - VR]", duration);
		}

		// DEBUG - INSET
		// int insetSize = 64;
		// glBlitNamedFramebuffer(viewRight.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
		// 	Runtime::mousePosition.x - insetSize / 2, Runtime::mousePosition.y - insetSize / 2, 
		// 	Runtime::mousePosition.x + insetSize / 2, Runtime::mousePosition.y + insetSize / 2,
		// 	500, 0, 500 + 1024, 0 + 1024,
		// 	GL_COLOR_BUFFER_BIT, GL_NEAREST
		// );

	}else{

		// RENDER DESKTOP

		if(Runtime::measureTimings){
			cuEventRecord(event_render_start, 0);
		}

		cuStreamSynchronize(0);

		// { // MAIN
		// 	RenderTarget target;
		// 	target.width = GLRenderer::width;
		// 	target.height = GLRenderer::height;
		// 	target.framebuffer = (uint64_t*)virt_framebuffer->cptr;
		// 	target.indexbuffer = nullptr;
		// 	target.view = mat4(GLRenderer::camera->view);
		// 	target.proj = GLRenderer::camera->proj;
			
		// 	draw(&scene, GLRenderer::view, target, mainstream, sidestream);

		// 	prog_gaussians_editing->launch("kernel_computeHoveredObject", {&launchArgs, &target}, 256, mainstream);

		// 	auto glMapping = mapCudaGl(GLRenderer::view.framebuffer->colorAttachments[0]);
		// 	prog_gaussians_rendering->launch("kernel_toOpenGL", {&launchArgs, &target, &glMapping.surface}, GLRenderer::width * GLRenderer::height, mainstream);
		// 	glMapping.unmap();
		// }

		if(settings.enableStereoFramebufferTest && settings.enableOverlapped){ 
			// render to multiple targets concurrently
			
			static shared_ptr<CudaVirtualMemory> virt_framebuffer_secondary = CURuntime::allocVirtual("secondary framebuffer (stereo concurrency test)");
			virt_framebuffer_secondary->commit(virt_framebuffer->comitted);

			RenderTarget target;
			target.width = GLRenderer::width;
			target.height = GLRenderer::height;
			target.framebuffer = (uint64_t*)virt_framebuffer->cptr;
			target.indexbuffer = nullptr;
			target.view = mat4(GLRenderer::camera->view);
			target.proj = GLRenderer::camera->proj;
			target.VP = GLRenderer::camera->VP;

			RenderTarget secondary = target;
			secondary.framebuffer = (uint64_t*)virt_framebuffer_secondary->cptr;
			
			vector<RenderTarget> targets = { target, secondary};
			draw(&scene, targets);

			Rectangle targetViewport_left;
			targetViewport_left.x = 0;
			targetViewport_left.y = 0;
			targetViewport_left.width = GLRenderer::width / 2;
			targetViewport_left.height = GLRenderer::height / 2;

			Rectangle targetViewport_right;
			targetViewport_right.x = GLRenderer::width / 2;
			targetViewport_right.y = 0;
			targetViewport_right.width = GLRenderer::width / 2;
			targetViewport_right.height = GLRenderer::height / 2;

			auto glMapping = mapCudaGl(GLRenderer::view.framebuffer->colorAttachments[0]);

			prog_gaussians_rendering->launch("kernel_blit_opengl", 
				{&launchArgs, &target, &glMapping.surface, &targetViewport_left}, 
				targetViewport_left.width * targetViewport_left.height);

			prog_gaussians_rendering->launch("kernel_blit_opengl", 
				{&launchArgs, &secondary, &glMapping.surface, &targetViewport_right}, 
				targetViewport_right.width * targetViewport_right.height);

			glMapping.unmap();
		}else if(settings.enableStereoFramebufferTest && !settings.enableOverlapped){ 

			{ // LEFT
				RenderTarget target;
				target.width = GLRenderer::width;
				target.height = GLRenderer::height;
				target.framebuffer = (uint64_t*)virt_framebuffer->cptr;
				target.indexbuffer = nullptr;
				target.view = mat4(GLRenderer::camera->view);
				target.proj = GLRenderer::camera->proj;
				target.VP = GLRenderer::camera->VP;

				draw(&scene, {target});

				Rectangle viewport;
				viewport.x = 0;
				viewport.y = 500;
				viewport.width = GLRenderer::width / 2;
				viewport.height = GLRenderer::height / 2;

				auto glMapping = mapCudaGl(GLRenderer::view.framebuffer->colorAttachments[0]);

				prog_gaussians_rendering->launch("kernel_blit_opengl", 
					{&launchArgs, &target, &glMapping.surface, &viewport}, 
					viewport.width * viewport.height);

				glMapping.unmap();
			}

			{ // RIGHT
				RenderTarget target;
				target.width = GLRenderer::width;
				target.height = GLRenderer::height;
				target.framebuffer = (uint64_t*)virt_framebuffer->cptr;
				target.indexbuffer = nullptr;
				target.view = mat4(GLRenderer::camera->view);
				target.proj = GLRenderer::camera->proj;
				target.VP = GLRenderer::camera->VP;

				draw(&scene, {target});

				Rectangle viewport;
				viewport.x = GLRenderer::width / 2;
				viewport.y = 500;
				viewport.width = GLRenderer::width / 2;
				viewport.height = GLRenderer::height / 2;

				auto glMapping = mapCudaGl(GLRenderer::view.framebuffer->colorAttachments[0]);

				prog_gaussians_rendering->launch("kernel_blit_opengl", 
					{&launchArgs, &target, &glMapping.surface, &viewport}, 
					viewport.width * viewport.height);

				glMapping.unmap();
			}



		}else{
			// standard single-target-rendering
			RenderTarget target;
			target.width = GLRenderer::width;
			target.height = GLRenderer::height;
			target.framebuffer = (uint64_t*)virt_framebuffer->cptr;
			target.indexbuffer = nullptr;
			target.view = mat4(GLRenderer::camera->view);
			target.proj = GLRenderer::camera->proj;
			target.VP = GLRenderer::camera->VP;
			
			vector<RenderTarget> targets = { target };
			draw(&scene, targets);

			prog_gaussians_editing->launch("kernel_computeHoveredObject", {&launchArgs, &target}, 256, mainstream);

			auto glMapping = mapCudaGl(GLRenderer::view.framebuffer->colorAttachments[0]);
			prog_gaussians_rendering->launch("kernel_toOpenGL", {&launchArgs, &target, &glMapping.surface}, GLRenderer::width * GLRenderer::height, mainstream);
			glMapping.unmap();
		}

#ifdef SPLATSHOP_HAS_ORBBEC
		// ----------------------------------------------------------------
		// Orbbec live point-cloud display panel.
		// Renders the streaming SNOrbbec cloud into a small dedicated
		// RenderTarget via the HQS point renderer (the same kernels used in
		// SplatEditor_draw.h), then blits it to a GL texture the panel shows
		// with ImGui::Image. This runs on `mainstream` (the editor's stream)
		// after the main viewport blit, so it never disturbs the main image.
		// ----------------------------------------------------------------
		if(settings.showOrbbecPointCloud && snOrbbec){
			auto& node = snOrbbec;
			PointData& pd = node->manager.data;

			if(pd.count > 0 && pd.position != nullptr && pd.color != nullptr){
				// Panel render resolution (independent of the main window).
				const int pcW = 640;
				const int pcH = 480;

				// Lazily allocate the panel's CUDA framebuffer and HQS
				// scratch buffers (sized to pcW*pcH, not the main viewport).
				static shared_ptr<CudaVirtualMemory> pcFB =
					CURuntime::allocVirtual("orbbec_pc_framebuffer");
				static shared_ptr<CudaVirtualMemory> pcFbDepth =
					CURuntime::allocVirtual("orbbec_pc_fb_depth");
				static shared_ptr<CudaVirtualMemory> pcFbColor =
					CURuntime::allocVirtual("orbbec_pc_fb_color");
				// Lazily create the GL texture the blit writes into.
				static shared_ptr<GLTexture> pcTex;
				if(!pcTex || pcTex->width != pcW || pcTex->height != pcH){
					pcTex = GLTexture::create(pcW, pcH, GL_RGBA8);
				}

				int64_t numPixels = (int64_t)pcW * pcH;
				pcFB->commit(numPixels * 8);
				pcFbDepth->commit(numPixels * 4);
				pcFbColor->commit(numPixels * 16);

				// --- Orbit camera for the panel (self-contained) ---
				// Mirror OrbitControls::update(): world = T(target) *
				// RotYaw * RotPitch * flip * T(0,0,radius), view = inverse.
				// Auto-fit recomputes target/radius from the cloud AABB so
				// the streaming cloud stays framed.
				vec3 bmin = pd.min;
				vec3 bmax = pd.max;
				vec3 center = (bmin + bmax) * 0.5f;
				float ext = glm::length(bmax - bmin);
				if(settings.orbbecPCAutoFit || !node->pcCameraInited){
					node->pcTarget = center;
					node->pcRadius = (ext > 1e-6f) ? (ext * 1.2f + 0.1f) : 1.0f;
					node->pcCameraInited = true;
				}

				glm::vec3 up(0.0f, 0.0f, 1.0f);
				glm::vec3 right(1.0f, 0.0f, 0.0f);
				glm::mat4 rotYaw = glm::rotate(node->pcYaw, up);
				glm::mat4 rotPitch = glm::rotate(node->pcPitch, right);
				glm::mat4 flip(1.0f, 0.0f, 0.0f, 0.0f,
				               0.0f, 0.0f, 1.0f, 0.0f,
				               0.0f, -1.0f, 0.0f, 0.0f,
				               0.0f, 0.0f, 0.0f, 1.0f);
				glm::mat4 Tcam = glm::translate(glm::vec3(0.0f, 0.0f, node->pcRadius));
				glm::mat4 Ttgt = glm::translate(node->pcTarget);
				glm::mat4 world = Ttgt * rotYaw * rotPitch * flip * Tcam;
			glm::mat4 view = glm::inverse(world);
			float aspect = float(pcW) / float(pcH);
			// Use the project's own projection (matches the main viewport;
			// also avoids a glm::perspective include-order pitfall here).
			glm::mat4 proj = Camera::createProjectionMatrix(0.05f, glm::radians(50.0f), aspect);

				RenderTarget pcTarget;
				pcTarget.width = pcW;
				pcTarget.height = pcH;
				pcTarget.framebuffer = (uint64_t*)pcFB->cptr;
				pcTarget.indexbuffer = nullptr;
				pcTarget.view = mat4(view);
				pcTarget.proj = mat4(proj);
				pcTarget.VP = mat4(proj * view);

				// Clear the panel framebuffer + HQS scratch buffers.
				prog_gaussians_rendering->launch("kernel_clearFramebuffer",
					{&launchArgs, &pcTarget}, pcW * pcH, mainstream);
				uint32_t INF = 0x7f800000;
				cuMemsetD32Async(pcFbDepth->cptr, INF, pcW * pcH, mainstream);
				cuMemsetD32Async(pcFbColor->cptr, 0, 4 * pcW * pcH, mainstream);

				// Render the cloud: depth pass, color pass (with the live
				// color correction so the panel matches the main viewport),
				// then normalize into pcTarget.framebuffer. The point's own
				// transform is identity for SNOrbbec, so use pd.transform.
				pd.transform = node->transform_global;
				float pointSize = settings.orbbecPCPointSize;

				prog_points->launch("kernel_hqs_depth",
					{&launchArgs, &pd, &pcTarget, &pcFbDepth->cptr, &pcFbColor->cptr, &pointSize},
					pd.count, mainstream);

				ColorCorrection cc;
				if(node->selected) cc = settings.colorCorrection;
				prog_points->launch("kernel_hqs_color",
					{&launchArgs, &pd, &pcTarget, &pcFbDepth->cptr, &pcFbColor->cptr, &pointSize, &cc},
					pd.count, mainstream);

				prog_points->launch("kernel_hqs_normalize",
					{&launchArgs, &pcTarget, &pcFbDepth->cptr, &pcFbColor->cptr},
					pcW * pcH, mainstream);

				// Blit the panel framebuffer to its GL texture.
				auto pcMapping = mapCudaGl(pcTex);
				prog_gaussians_rendering->launch("kernel_toOpenGL",
					{&launchArgs, &pcTarget, &pcMapping.surface}, pcW * pcH, mainstream);
				pcMapping.unmap();

				orbbecTexPointCloud = pcTex->handle;
				orbbecTexPointCloudW = pcW;
				orbbecTexPointCloudH = pcH;
			}
		}
#endif




		if(Runtime::measureTimings){
			cuEventRecord(event_render_end, 0);

			cuCtxSynchronize();

			float duration;
			cuEventElapsedTime(&duration, event_render_start, event_render_end);

			Runtime::timings.add("[render - desktop]", duration);
		}

		if(settings.showInset)
		{ // BLIT AN INSET
			cuCtxSynchronize();
			
			int insetSize = 32;
			int factor = 16;
			ivec2 center = {16 * 60 + 8, 16 * 50 + 8};
			// int start = {16 * 60 - 8, 16 * 50 - 8};
			// int end =   {16 * 60 + 8, 16 * 50 + 8};

			// first copy source area to bottom-left
			// ivec2 center = ivec2{GLRenderer::width / 2, GLRenderer::height / 2};
			glBlitNamedFramebuffer(GLRenderer::view.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
				center.x - insetSize / 2, center.y - insetSize / 2, center.x + insetSize / 2, center.y + insetSize / 2,
				0, 0, insetSize, insetSize,
				GL_COLOR_BUFFER_BIT, GL_NEAREST
			);

			// then copy and rescale to target area
			glBlitNamedFramebuffer(GLRenderer::view.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
				0, 0, insetSize, insetSize,
				center.x - factor * insetSize / 2, 
				10, //center.y - factor * insetSize / 2, 
				center.x + factor * insetSize / 2, 
				10 + factor * insetSize, // center.y + factor * insetSize / 2,
				GL_COLOR_BUFFER_BIT, GL_NEAREST
			);

			// make border in source area
			// left
			glBlitNamedFramebuffer(GLRenderer::view.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
				200, 200, 1, 1,
				center.x - insetSize / 2, center.y - insetSize / 2, center.x - insetSize / 2 + 1, center.y + insetSize / 2,
				GL_COLOR_BUFFER_BIT, GL_NEAREST
			);
			// right
			glBlitNamedFramebuffer(GLRenderer::view.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
				200, 200, 1, 1,
				center.x + insetSize / 2 - 1, center.y - insetSize / 2, center.x + insetSize / 2, center.y + insetSize / 2,
				GL_COLOR_BUFFER_BIT, GL_NEAREST
			);
			// bottom
			glBlitNamedFramebuffer(GLRenderer::view.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
				200, 200, 1, 1,
				center.x - insetSize / 2, center.y - insetSize / 2, center.x + insetSize / 2, center.y - insetSize / 2 + 1,
				GL_COLOR_BUFFER_BIT, GL_NEAREST
			);
			// top
			glBlitNamedFramebuffer(GLRenderer::view.framebuffer->handle, GLRenderer::view.framebuffer->handle, 
				200, 200, 1, 1,
				center.x - insetSize / 2, center.y + insetSize / 2 - 1, center.x + insetSize / 2, center.y + insetSize / 2,
				GL_COLOR_BUFFER_BIT, GL_NEAREST
			);
		}
	}

	cuCtxSynchronize();
	CudaModularProgram::resolveTimings();
	
	// DRAW GUI
	if(!ovr->isActive()){

		ImGui::SetCurrentContext(imguicontext_desktop);

		drawGUI();

		// { // show vr gui in desktop mode
		// 	imguiStyleVR();
		// 	makePaintingVR(imn_painting->page);
		// 	ImGui::StyleColorsDark();
		// }

		Runtime::totalTileFragmentCount = 0;

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}else{

		{ // render desktop gui while in VR
			ImGui::SetCurrentContext(imguicontext_desktop);
			ImGuiIO& io = ImGui::GetIO();

			auto fbo = GLRenderer::view.framebuffer;
			glBindFramebuffer(GL_FRAMEBUFFER, fbo->handle);
			glViewport(0, 0, fbo->width, fbo->height);

			drawGUI();
			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		}

		Runtime::totalTileFragmentCount = 0;
	}

	cuStreamSynchronize(0);
	cuStreamSynchronize(mainstream);
	cuCtxSynchronize();
	
	mouse_prev = Runtime::mouseEvents;

	if(viewmode == VIEWMODE_IMMERSIVE_VR && ovr->isActive()){

		auto size = ovr->getRecommmendedRenderTargetSize();	
		int width = size[0];
		int height = size[1];	

		vr::VRTextureBounds_t bounds = {
			0.0f, 
			1.0f - float(height) / float(viewLeft.framebuffer->height),
			float(width) / float(viewLeft.framebuffer->width), 
			1.0f,
		};

		ovr->submit(viewLeft.framebuffer->colorAttachments[0]->handle, vr::EVREye::Eye_Left, bounds);
		ovr->submit(viewRight.framebuffer->colorAttachments[0]->handle, vr::EVREye::Eye_Right, bounds);

		ovr->postPresentHandoff();
	}

	cuMemcpyDtoHAsync(h_state_pinned, cptr_state, sizeof(DeviceState), mainstream);
	memcpy(&deviceState, h_state_pinned, sizeof(DeviceState));

	Runtime::numSelectedSplats = deviceState.numSelectedSplats;

	settings.shortcutsDisabledForXFrames = max(settings.shortcutsDisabledForXFrames - 1, 0);

	Runtime::mouseEvents.clear();

	if(Runtime::measureTimings){
		CudaModularProgram::clearTimings();
	}

	ImGui::SetCurrentContext(imguicontext_desktop);

	postRenderStuff();
}