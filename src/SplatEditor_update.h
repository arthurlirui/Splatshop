

void SplatEditor::update(){

	cuCtxSynchronize();

	{
		string strfps = format("Splat Editor | FPS: {}", int(GLRenderer::fps));
		glfwSetWindowTitle(GLRenderer::window, strfps.c_str());
	}

	TWEEN::update();

	// Motion control: advance the keyframe timeline. dt is measured from the
	// previous frame; the timeline pushes sampled transforms into the scene
	// graph via MotionController, which updateTransformations() propagates below.
	{
		static double lastMotionTime = now();
		double curMotionTime = now();
		double motionDt = curMotionTime - lastMotionTime;
		lastMotionTime = curMotionTime;
		timeline.update(scene, motionDt);
	}

	// 4DGS auto-playback: when no timeline tracks are loaded but the scene
	// contains SN4DGSSplats nodes, auto-advance the internal time so that
	// the deformation animates continuously (cycling 0.0 → 1.0 → 0.0...).
	// The deformation itself is dispatched in draw() just before staging.
	{
		static double last4DGSTime = now();
		double curTime = now();
		double dt = curTime - last4DGSTime;
		last4DGSTime = curTime;

		bool has4DGS = false;
		scene.forEach<SN4DGSSplats>([&](SN4DGSSplats*) { has4DGS = true; });

		if (has4DGS && timeline.tracks.empty() && timeline.playing) {
			// Auto-cycle: bounce 0→1→0, loop duration ~4 seconds
			constexpr double cycleDuration = 4.0;
			static double autoTime = 0.0;
			static int direction = 1;
			autoTime += direction * dt / cycleDuration;
			if (autoTime >= 1.0) { autoTime = 1.0; direction = -1; }
			if (autoTime <= 0.0) { autoTime = 0.0; direction = 1; }
			timeline.playhead = autoTime;
		}
	}

	// Motion control: accumulate rigged-splat joint poses into skinning matrices
	// and upload to device. The actual skinning kernel is dispatched in draw(),
	// right before the per-node render launch, so deformed splats are ready for
	// the staging kernel.
	rigController.update(scene);

	auto flip = glm::dmat4(
		1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, -1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	);

	// FIXME: In vr we reroute mouse inputs, and because of that we somehow need to deffer this to after the rerouting.
	// In Desktop mode, we begin here because we need to begin the frame before drawing the gizmo.
	// if(!ovr->isActive())
	//ImGui::SetCurrentContext(imguicontext_desktop);
	ImGui::NewFrame();
	ImGuizmo::BeginFrame();
	Runtime::timings.newFrame();

	scene.updateTransformations();
	Runtime::debugValues.clear();
	Runtime::debugValueList.clear();
	Runtime::numVisibleSplats = 0;
	Runtime::numVisibleFragments = 0;
	Runtime::numRenderedTriangles = 0;
	*h_numLines = 0;
	triangleQueue.clear();

	if(settings.showBoundingBoxes){
		scene.process<SNSplats>([&](SNSplats* node) {
			if(!node->visible) return;
			if(node->hidden) return;
			drawBox(node->aabb);
		});
	}

	if(settings.showAxes)
	{ // DRAW AXIS
		Line x = {
			.start = {0.0f, 0.0f, 0.0f}, 
			.end = {10.0f, 0.0f, 0.0f},
			.color = 0xff0000ff
		};
		Line y = {
			.start = {0.0f, 0.0f, 0.0f}, 
			.end = {0.0f, 10.0f, 0.0f},
			.color = 0xff00ff00
		};
		Line z = {
			.start = {0.0f, 0.0f, 0.0f}, 
			.end = {00.0f, 0.0f, 5.0f},
			.color = 0xffff0000
		};

		h_lines[*h_numLines + 0] = x;
		h_lines[*h_numLines + 1] = y;
		h_lines[*h_numLines + 2] = z;
		*h_numLines = *h_numLines + 3;
	}

	// {
	// 	Box3 aabb = getSelectionAABB();

	// 	drawBox(aabb, 0xff0000ff);
	// }

	if(settings.showGrid)
	{ // DRAW GRID
		for(float i = -10.0f; i <= 10.0f; i += 1.0f){
			Line lx = {
				.start = {i, -10.0f, 0.0f}, 
				.end = {i, 10.0f, 0.0f},
				.color = 0xff666666
			};
			Line ly = {
				.start = {-10.0f, i, 0.0f}, 
				.end = {10.0f, i, 0.0f},
				.color = 0xff666666
			};

			if(settings.showAxes)
			if(i == 0.0f)
			{
				lx.end.y = 0.0f;
				ly.end.x = 0.0f;
			}

			// queue_lines.push_back(lx);
			// queue_lines.push_back(ly);

			h_lines[*h_numLines + 0] = lx;
			h_lines[*h_numLines + 1] = ly;
			*h_numLines = *h_numLines + 2;
		}
	}


	if(GLRenderer::width * GLRenderer::height == 0){
		return;
	}

	CudaModularProgram::measureTimings = Runtime::measureTimings;

	launchArgs = getCommonLaunchArgs();

	static CUdeviceptr cptr_view = 0;
	if(cptr_view == 0){
		cptr_view = CURuntime::alloc("view", sizeof(mat4));
	}

	cuMemsetD32Async(cptr_state + offsetof(DeviceState, visibleSplats), 0, 1, mainstream);
	cuMemsetD32Async(cptr_state + offsetof(DeviceState, visibleSplatFragments), 0, 1, mainstream);
	// cuMemsetD32Async(cptr_state + offsetof(DeviceState, numSelectedSplats), 0, 1, mainstream);
	cuMemsetD32Async(cptr_state + offsetof(DeviceState, numSplatsInLargestTile), 0, 1, mainstream);
	cuMemsetD32Async(cptr_state + offsetof(DeviceState, dbg.numAccepted), 0, 1, mainstream);
	cuMemsetD32Async(cptr_state + offsetof(DeviceState, dbg.numRejected), 0, 1, mainstream);
	cuMemsetD32Async(cptr_numLines, 0, 1, mainstream);

	if(tempSplats)
	{ // UPDATE POSITION OF DRAGED FILE
		vec3 pos = deviceState.hovered_pos;
		if (pos.x == Infinity || pos.y == Infinity || pos.z == Infinity) {
			pos = { 0.0f, 0.0f, 0.0f };
		}
		if (pos.x == -Infinity || pos.y == -Infinity || pos.z == -Infinity) {
			pos = { 0.0f, 0.0f, 0.0f };
		}

		tempSplats->transform = glm::translate(pos);
	}

	// upload newly loaded splats
	scene.forEach<SNSplats>([&](SNSplats* node) {
		uploadSplats(node);
	});
	for(auto node : AssetLibrary::assets){
		onTypeMatch<SNSplats>(node, [&](shared_ptr<SNSplats> node){
			uploadSplats((SNSplats*)node.get());
		});
	}

	// upload newly loaded points
	scene.forEach<SNPoints>([&](SNPoints* node) {

		auto points = node->points;

		uint64_t numLoaded = points->numPointsLoaded;
		node->manager.commit(numLoaded);
		PointData& pd = node->manager.data;

		int numUploaded = pd.numUploaded;
		int numToUpload = numLoaded - numUploaded;

		if(numToUpload > 0){
			CURuntime::check(cuMemcpyHtoDAsync((CUdeviceptr)pd.position   + 12llu * numUploaded, points->position->ptr + 12llu * numUploaded , 12llu * numToUpload, mainstream));
			CURuntime::check(cuMemcpyHtoDAsync((CUdeviceptr)pd.color      +  4llu * numUploaded, points->color->ptr    +  4llu * numUploaded ,  4llu * numToUpload, mainstream));
			CURuntime::check(cuMemsetD32Async((CUdeviceptr)pd.flags + numUploaded, 0, numToUpload, mainstream));

			pd.count += numToUpload;
			pd.numUploaded += numToUpload;
		}

		// Once the cloud is fully loaded AND fully uploaded, build the Skye-style
		// progressive (shuffled) buffers and run the load-time permutation. After
		// this, a contiguous draw over the shuffled buffer uniformly samples the
		// whole cloud, which is what makes the per-frame fill budget bounded. We
		// only do this when the user has selected the progressive renderer; the
		// HQS path ignores the shuffled buffers entirely.
		bool fullyLoaded   = (int64_t)points->numPointsLoaded == points->numPoints && points->numPoints > 0;
		bool fullyUploaded = pd.numUploaded == (int)numLoaded && numLoaded > 0;
		if(settings.pointRenderer == POINTRENDERER_PROGRESSIVE
			&& fullyLoaded && fullyUploaded
			&& !node->progressiveInitialized
			&& prog_progressive_points != nullptr)
		{
			uint64_t numPoints = uint64_t(pd.count);
			node->progressive.init(numPoints);
			node->progressiveInitialized = true;

			// Scatter canonical {pos,color} into permuted slots. One thread per
			// point. Runs on mainstream so it's ordered after the uploads above.
			ProgressivePointData pc = node->progressive.data;
			prog_progressive_points->launch(
				"kernel_progressive_distribute",
				{&launchArgs, &pd, &pc},
				int(numPoints), mainstream);

			// Mark ready once the scatter is dispatched. drawpoints_progressive
			// checks pc.ready before touching the shuffled buffers, and the
			// kernels run on mainstream so the dispatch ordering is guaranteed.
			node->progressive.data.ready = true;

			println("Progressive: shuffled {:L} points (prime={}, {} buffers) for \"{}\"",
				numPoints, node->progressive.prime, node->progressive.numBuffers, node->name);
		}
	});

	

	launchArgs = getCommonLaunchArgs();

	glm::mat4 scale = glm::scale(glm::vec3(0.3f, 0.3f, 0.3f));
	glm::mat4 trans = glm::translate(glm::vec3(0.0f, 0.0f, 1.5f));
	viewmodeImmersiveVR.world_vr = trans * scale;

	if(ovr->isActive()){
		ovr->updatePose(viewmode == VIEWMODE_IMMERSIVE_VR);

		ovr->processEvents();

		int leftHandID = ovr->system->GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole::TrackedControllerRole_LeftHand);
		int rightHandID = ovr->system->GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole::TrackedControllerRole_RightHand);

		if (leftHandID >= 0) {
			auto [model, texture] = ovr->getRenderModel(leftHandID);

			if (model) {
				
				static shared_ptr<SNTriangles> node = nullptr;

				if(!node){
					node = ovrToNode("controller_left", model, texture);
					node->hidden = true;
					scene.vr->children.push_back(node);
				}else{
					Pose poseLeft = ovr->getLeftControllerPose();

					float aspect = float(GLRenderer::width) / float(GLRenderer::height);

					node->transform = flip * poseLeft.transform;

					auto state = ovr->getLeftControllerState();
					if(poseLeft.valid){
						bool menuButtonPressed = (state.ulButtonPressed & (1 << vr::EVRButtonId::k_EButton_ApplicationMenu));

						// if(menuButtonPressed)
						{
							auto page = vrPages[currentVrPage];

							float sx = float(page.width) / 1000.0f;
							float sy = float(page.height) / 1000.0f;

							mat4 mRot = glm::rotate(-3.1415f * 0.5f, vec3{ 1.0f, 0.0f, 0.0f });
							mat4 mScale = glm::scale(vec3{sx, sy, 1.0f} * 0.2f);

							// sn_vr_editing->transform = mat4(flip * poseLeft.transform) * mRot * mScale;
							sn_vr_editing->visible = true;
						}

					}

					// { // DEBUG
					// 	auto mesh = ovrToMesh(model, texture);

					// 	node->set(mesh.position, mesh.uv);
					// }
				}
			}
		}
		if (rightHandID >= 0) {
			auto [model, texture] = ovr->getRenderModel(rightHandID);

			if (model) {

				static shared_ptr<SNTriangles> node = nullptr;

				if(!node){
					node = ovrToNode("controller_right", model, texture);
					node->hidden = true;
					scene.vr->children.push_back(node);
				}else{
					Pose pose = ovr->getRightControllerPose();
					node->transform = flip * pose.transform;
				}
			}
		}
		

		if(viewmode == VIEWMODE_IMMERSIVE_VR){
			auto size = ovr->getRecommmendedRenderTargetSize();	
			int width = size[0];
			int height = size[1];

			bool needsResizeX = viewLeft.framebuffer->width != width;
			bool needsResizeY = viewLeft.framebuffer->height != height;
			bool needsResize = needsResizeX || needsResizeY;

			// viewLeft.framebuffer->setSize(width, height);
			// viewRight.framebuffer->setSize(width, height);

			glBindFramebuffer(GL_FRAMEBUFFER, viewLeft.framebuffer->handle);
			glViewport(0, 0, width, height);
			glClearColor(1.0, 0.0, 0.0, 1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			glBindFramebuffer(GL_FRAMEBUFFER, viewRight.framebuffer->handle);
			glViewport(0, 0, width, height);
			glClearColor(0.0, 1.0, 0.0, 1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			auto poseHMD = ovr->getHmdPose();
			auto poseLeft = ovr->getEyePose(vr::Hmd_Eye::Eye_Left);
			auto poseRight = ovr->getEyePose(vr::Hmd_Eye::Eye_Right);

			viewLeft.view = glm::inverse(flip * poseHMD * poseLeft);
			viewLeft.proj = ovr->getProjection(vr::Hmd_Eye::Eye_Left, 0.01, 10'000.0);
			viewLeft.VP = ovr->getProjectionVP(vr::Hmd_Eye::Eye_Left, 0.2, 1'000.0, width, height);

			viewRight.view = glm::inverse(flip * poseHMD * poseRight);
			viewRight.proj = ovr->getProjection(vr::Hmd_Eye::Eye_Right, 0.2, 1'000.0);
			viewRight.VP = ovr->getProjectionVP(vr::Hmd_Eye::Eye_Right, 0.2, 1'000.0, width, height);
		}

		auto state = ovr->getRightControllerState();
		// if(state.ulButtonPressed == 2){
		// 	Pose pose_left = ovr->getLeftControllerPose();
		// 	Pose pose_right = ovr->getRightControllerPose();
			
		// 	viewmodeDesktopVr.m_controller_neutral_left = pose_left.transform;
		// 	viewmodeDesktopVr.m_controller_neutral_right = pose_right.transform;

		// 	glm::mat4 avg;
		// 	float* f_avg = (float*)&avg;
		// 	double* f_left = (double*)&pose_left.transform;
		// 	double* f_right = (double*)&pose_right.transform;

		// 	for(int i = 0; i < 16; i++){
		// 		f_avg[i] = (f_left[i] + f_right[i]) / 2.0;
		// 	}

		// 	viewmodeDesktopVr.m_controller_neutral = avg;
		// }

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	scene.updateTransformations();

	inputHandling();

	scene.updateTransformations();

	scene.process<SNSplats>([&](SNSplats* node){
		if(!node->visible) return;
		
		GaussianData model = node->dmng.data;

		if(node->hovered){
			updateBoundingBox(node);
			drawBox(node->aabb, 0xff00ff00);
		}
	});

	if(ovr->isActive()){
		SceneNode* sn_vr_controller_right = scene.find("controller_right");

		// DRAW LINE FOR RIGHT CONTROLLER
		if(sn_vr_controller_right){
			if(state_vr.menu_intersects){
				mat4 mRot = glm::rotate(-140.0f * 3.1415f / 180.0f, vec3{ 1.0f, 0.0f, 0.0f });
				mat4 transform = sn_vr_controller_right->transform * mRot;
				vec3 origin = vec3(transform * vec4{0.0f, 0.0f, 0.0f, 1.0f});

				h_lines[*h_numLines] = Line{.start = origin, .end = state_vr.menu_intersection, .color = 0x0000ff00,};
				*h_numLines = *h_numLines + 1;
			}
		}
		}

	// start asynchronously sending host-queued lines to device
	virt_lines_host->commit(*h_numLines * sizeof(Line));
	cuMemcpyHtoDAsync(virt_lines_host->cptr, h_lines, *h_numLines * sizeof(Line), mainstream);
	cuMemsetD32Async(cptr_numLines_host, *h_numLines, 1, mainstream);
	
};