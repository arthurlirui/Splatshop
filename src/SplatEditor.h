#pragma once

#include <atomic>

#include "glm/gtc/matrix_access.hpp"

#ifdef NO_OPENVR
	#include "NOpenVRHelper.h"
#else
	#include "OpenVRHelper.h"
#endif
#include "CudaVirtualMemory.h"

#include "Mesh.h"
#include "./scene/SceneNode.h"
#include "./scene/Scene.h"
#include "./scene/SNSplats.h"
#include "./scene/SNRiggedSplats.h"
#include "./scene/SN4DGSSplats.h"
#include "./scene/ImguiNode.h"
#include "./scene/SNPoints.h"
#include "./scene/SNPointCloudBA.h"
#include "./scene/SNTriangles.h"
#include "./scene/SNOrbbec.h"
#include "./scene/SNK4a.h"
#ifdef SPLATSHOP_HAS_ORBBEC
#include "./camera/OrbbecCapture.h"
#endif
#ifdef SPLATSHOP_HAS_K4A
#include "./camera/K4aCapture.h"
#endif
#ifdef SPLATSHOP_HAS_OPENCV
#include "Calibration.h"
#include "./calibration/Calibrator.h"
#include "./calibration/ExtrinsicCalibrator.h"
#include "./calibration/CalibrationStore.h"
#endif

#include "cuda.h"
#include "cuda_runtime.h"
#include "CudaModularProgram.h"

#include "OrbitControls.h"
#include "DesktopVRControls.h"
#include "./actions/InputAction.h"

#include "common.h"
#include "AssetLibrary.h"

#include "loader/GSPlyLoader.h"
#include "loader/SplatsyFilesLoader.h"
#include "loader/SplatsyLoader.h"
#include "loader/PLYHeightmapLoader.h"
#include "loader/GLBLoader.h"
#include "loader/LASLoader.h"
#include "writer/GSPlyWriter.h"
#include "writer/SplatsyWriter.h"
#include "writer/SplatsyFilesWriter.h"
// #include "writer/SplatsyPlyWriter.h"
#include "Runtime.h"
#include "tween.h"

#include "common.h"
#include "utils.h"

#include "motion/MotionTypes.h"
#include "motion/MotionController.h"
#include "motion/Timeline.h"
#include "motion/RiggedHumanController.h"

struct Action;
struct InputAction;

using glm::transpose;
using glm::vec2;
using glm::quat;
using glm::vec3;

struct TriangleQueueItem{
	TriangleData geometry;
	TriangleMaterial material;
};

struct GuiVrPage{
	string label;
	int width;
	int height;
};

struct _DrawSphereArgs{
	vec3 pos   = {0.0f, 0.0f, 0.0f};
	vec3 scale = {1.0f, 1.0f, 1.0f};
	vec4 color = {0.0f, 1.0f, 0.0f, 1.0f};
};

struct CudaGlMappings{
	std::vector<CUgraphicsResource> resources;
	CUgraphicsResource resource;
	CUsurfObject surface;
	CUtexObject texture;
	
	
	bool isMapped = true;

	~CudaGlMappings(){
		if(isMapped){
			unmap();
		}
	}

	void unmap(){
		cuSurfObjectDestroy(surface);
		cuTexObjectDestroy (texture);
		cuGraphicsUnmapResources(resources.size(), resources.data(), ((CUstream)CU_STREAM_DEFAULT));
		// cuGraphicsUnregisterResource(resource);

		isMapped = false;
	};
};

struct SplatEditor{
	
	inline static SplatEditor* instance;

	Scene scene;
	shared_ptr<SNSplats> tempSplats = nullptr;
	shared_ptr<SNTriangles> sn_vr_editing = nullptr;
	shared_ptr<SNTriangles> sn_dbgsphere = nullptr;
	shared_ptr<SNSplats> sn_brushsphere = nullptr;

#ifdef SPLATSHOP_HAS_ORBBEC
	// Orbbec RGBD camera module. orbbecCapture owns the SDK pipeline and
	// polling thread; snOrbbec is the live point-cloud scene node fed from
	// it. Both are created lazily from the Orbbec GUI panel.
	shared_ptr<orbbec::OrbbecCapture> orbbecCapture = nullptr;
	shared_ptr<SNOrbbec> snOrbbec = nullptr;
	vector<orbbec::DeviceInfo> orbbecDevices;
	int orbbecSelectedDevice = 0;

	// --- Orbbec stream config / camera params (persisted as members so
	//     they survive device switches, unlike the old static locals) ---
	orbbec::StreamConfig orbbecCfgColor;
	orbbec::StreamConfig orbbecCfgDepth;
	orbbec::CameraParams orbbecParams;
	bool orbbecStreamCfgLoaded = false;
	bool orbbecParamsLoaded = false;
	vector<orbbec::StreamConfig> orbbecColorProfiles;
	vector<orbbec::StreamConfig> orbbecDepthProfiles;
	int orbbecColorProfileIdx = 0;
	int orbbecDepthProfileIdx = 0;
	// Last Save/Load Params result message shown in the Orbbec panel.
	string orbbecParamsSaveMsg;

	// --- Orbbec real-time preview (RGB + Depth textures in ImGui) ---
	GLuint orbbecTexColor = 0;
	GLuint orbbecTexDepth = 0;
	GLuint orbbecTexDepthRaw = 0;           // raw (pre-filter) depth texture
	int orbbecTexColorW = 0, orbbecTexColorH = 0, orbbecTexColorBpp = 0;
	int orbbecTexDepthW = 0, orbbecTexDepthH = 0;
	int orbbecTexDepthRawW = 0, orbbecTexDepthRawH = 0;
	vector<uint8_t> orbbecColorScratch;    // YUYV->RGB conversion buffer
	vector<uint8_t> orbbecDepthScratch;    // depth colormap output buffer
	vector<uint8_t> orbbecDepthScratchRaw; // raw depth colormap output (before/after compare)
	vector<uint8_t> orbbecDepthLUT;        // 256x3 colormap lookup table
	int orbbecDepthLUTType = -1;           // which colormap the LUT was built for
	shared_ptr<orbbec::RGBDFrame> orbbecPreviewHeldFrame = nullptr; // for pause
	uint64_t orbbecPreviewFrameIndex = 0;

	// --- Undistortion preview textures (compare mode side-by-side) ---
	// Separate GL textures + scratch buffers for the undistorted color/depth
	// so the raw and corrected images can coexist in compare mode.
	GLuint orbbecTexColorUndist = 0;
	int orbbecTexColorUndistW = 0, orbbecTexColorUndistH = 0, orbbecTexColorUndistBpp = 0;
	vector<uint8_t> orbbecColorScratchUndist;   // undistorted color upload buffer
	GLuint orbbecTexDepthUndist = 0;
	int orbbecTexDepthUndistW = 0, orbbecTexDepthUndistH = 0;
	vector<uint8_t> orbbecDepthScratchUndist;   // undistorted depth colormap buffer
	vector<uint16_t> orbbecDepthUndist16;       // undistorted uint16 depth (pre-colormap)

	// --- Orbbec live point-cloud display panel (runtime state) ---
	// The GL texture + dedicated CUDA buffers used to render the streaming
	// SNOrbbec cloud into its own small RenderTarget (see SplatEditor_render.h)
	// and blit it to an ImGui::Image. Camera state lives on the SNOrbbec node
	// itself; user-tunable toggles live in `settings`.
	GLuint orbbecTexPointCloud = 0;        // GL texture receiving the blitted cloud image
	int orbbecTexPointCloudW = 0, orbbecTexPointCloudH = 0;

	// --- Orbbec lens calibration (runtime state) ---
	// Calibrator engine + the active device calibration. Texture for the
	// live detection-overlay preview. All guarded by both ORBBEC and OPENCV
	// because the Calibrator depends on OpenCV.
#ifdef SPLATSHOP_HAS_OPENCV
	std::shared_ptr<orbbec::Calibrator> orbbecCalibrator;
	orbbec::DeviceCalibration orbbecActiveCalibration;
	GLuint orbbecCalibTexOverlay = 0;
	int orbbecCalibTexOverlayW = 0, orbbecCalibTexOverlayH = 0;
	std::vector<uint8_t> orbbecCalibOverlayScratch;  // BGR8 overlay buffer
	int orbbecCalibTargetStream = 0;  // 0=Color 1=IR 2=Depth
	std::string orbbecCalibSavePath;
	std::string orbbecCalibLoadPath;

	// --- Orbbec extrinsic (solvePnP) + depth correction (runtime state) ---
	// ExtrinsicCalibrator engine + overlay texture for the live detection
	// preview. The solved pose + depth-correction fit live on
	// orbbecActiveCalibration (ExtrinsicPose / DepthCorrection). Depends on
	// OpenCV, so guarded by SPLATSHOP_HAS_OPENCV.
	std::shared_ptr<orbbec::ExtrinsicCalibrator> orbbecExtCalibrator;
	GLuint orbbecPnPTexOverlay = 0;
	int    orbbecPnPTexOverlayW = 0, orbbecPnPTexOverlayH = 0;
	std::vector<uint8_t> orbbecPnPOverlayScratch;   // BGR8 overlay buffer
	std::vector<uint16_t> orbbecDepthCorrected16;   // corrected uint16 depth (pre-colormap)
	bool orbbecPnPUseCalibIntrinsics = true;        // use calibrated IR intrinsics for solvePnP
#endif
#endif

#ifdef SPLATSHOP_HAS_K4A
	// K4A Wrapper RGBD camera module. k4aCapture owns the K4A device and
	// polling thread; snK4a is the live point-cloud scene node fed from it.
	// Both are created lazily from the K4A GUI panel.
	shared_ptr<k4a::K4aCapture> k4aCapture = nullptr;
	shared_ptr<SNK4a> snK4a = nullptr;
	vector<k4a::K4aDeviceInfo> k4aDevices;
	int k4aSelectedDevice = 0;
	k4a::K4aStreamConfig k4aStreamCfg;

	// --- K4A real-time preview (Color + Depth + IR textures in ImGui) ---
	GLuint k4aTexColor = 0;
	GLuint k4aTexDepth = 0;
	GLuint k4aTexIr = 0;
	int k4aTexColorW = 0, k4aTexColorH = 0, k4aTexColorBpp = 0;
	int k4aTexDepthW = 0, k4aTexDepthH = 0, k4aTexDepthBpp = 0;
	int k4aTexIrW = 0, k4aTexIrH = 0, k4aTexIrBpp = 0;
	vector<uint8_t> k4aColorScratch;
	vector<uint8_t> k4aDepthScratch;
	vector<uint8_t> k4aIrScratch;
	vector<uint8_t> k4aDepthLUT;
	int k4aDepthLUTType = -1;
	shared_ptr<k4a::K4aFrame> k4aPreviewHeldFrame = nullptr;

	// --- K4A calibration display text ---
	std::string k4aCalibText;

	// --- K4A recording / playback state ---
	k4a::K4aRecordConfig k4aRecordCfg;
	std::string k4aRecordPath;
	std::string k4aPlaybackPath;
	bool k4aPlaybackPlaying = false;
	int64_t k4aPlaybackPos = 0;
	shared_ptr<k4a::K4aFrame> k4aPlaybackFrame = nullptr;

	// --- K4A transformation / point cloud state ---
	shared_ptr<Buffer> k4aTransformBuf = nullptr;   // transformed image buffer
	int k4aTransformW = 0, k4aTransformH = 0;
	int k4aTransformMode = 0;  // 0=depth-to-color, 1=color-to-depth
	shared_ptr<Buffer> k4aPointCloudBuf = nullptr;  // point cloud buffer
	int k4aPointCloudW = 0, k4aPointCloudH = 0;
	int64_t k4aPointCloudCount = 0;
	bool k4aPointCloudColorized = false;
	std::string k4aPlyPath;
#endif

	vector<SceneNode*> scheduledForRemoval;

	CudaModularProgram* prog_gaussians_rendering = nullptr;
	CudaModularProgram* prog_gaussians_editing = nullptr;
	CudaModularProgram* prog_points = nullptr;
	CudaModularProgram* prog_progressive_points = nullptr; // Skye-style progressive point-cloud renderer (port to CUDA)
	CudaModularProgram* prog_points_remesh = nullptr;     // Point-cloud density optimization (voxel-grid downsampling)
	CudaModularProgram* prog_triangles = nullptr;
	CudaModularProgram* prog_lines = nullptr;
	CudaModularProgram* prog_helpers = nullptr;
	CudaModularProgram* prog_skinning = nullptr; // motion: per-frame LBS skinning + blendshape kernel

	OpenVRHelper* ovr = nullptr;
	View viewLeft;
	View viewRight;

	// External (remote-HMD) stereo pose state. Written by the remote control
	// thread, read by the main render thread under `remoteStereoMutex`.
	// Drives VIEWMODE_REMOTE_STEREO in SplatEditor_update.h / SplatEditor_render.h.
	struct RemoteStereoState {
		// Head pose in tracking space (OPENVR) - combined with eye offsets below.
		// Ignored for WEBXR/RAW_VIEW, where the eye views are supplied directly.
		glm::dmat4 headPose     = glm::dmat4(1.0);
		glm::dmat4 eyeLeft      = glm::dmat4(1.0);   // eye-to-head offset (OPENVR)
		glm::dmat4 eyeRight     = glm::dmat4(1.0);
		// Direct view matrices (WEBXR / RAW_VIEW): world->eye, in GL convention.
		glm::dmat4 viewLeft     = glm::dmat4(1.0);
		glm::dmat4 viewRight    = glm::dmat4(1.0);
		// Per-eye projection + viewport-inclusive VP (already GL-space).
		glm::dmat4 projLeft     = glm::dmat4(1.0);
		glm::dmat4 projRight    = glm::dmat4(1.0);
		glm::dmat4 vpLeft       = glm::dmat4(1.0);
		glm::dmat4 vpRight      = glm::dmat4(1.0);
		int   width  = 2048;
		int   height = 2048;
		PoseSpace space = POSE_SPACE_WEBXR;
		bool  fresh = false;          // set true on each new pose packet
		std::atomic<bool> active{false};
		std::mutex mutex;
	};
	RemoteStereoState remoteStereo;

	shared_ptr<Framebuffer> fbGuiVr;
	shared_ptr<Framebuffer> fbGuiVr_assets;
	vec2 vrGuiResolution = {1024, 1024};
	MouseEvents mouse_prev;
	shared_ptr<ImguiNode> imn_brushes = nullptr;
	shared_ptr<ImguiNode> imn_assets = nullptr;
	shared_ptr<ImguiNode> imn_layers = nullptr;
	shared_ptr<ImguiNode> imn_painting = nullptr;


	vector<shared_ptr<Action>> history;
	int history_offset = 0;
	
	vector<GuiVrPage> vrPages = {
		{
			.label = "Brushes",
			.width = 440,
			.height = 700,
		},{
			.label = "Assets",
			.width = 440,
			.height = 700,
		},{
			.label = "Layers",
			.width = 440,
			.height = 700,
		}
	};
	int currentVrPage = 0;

	CUstream stream_upload;
	CUstream mainstream;
	CUstream sidestream;

	CUevent event_mainstream;
	CUevent event_edl_applied;
	CUevent ev_reset_stagecounters;

	// CUdeviceptr cptr_framebuffer;
	shared_ptr<CudaVirtualMemory> virt_framebuffer = CURuntime::allocVirtual("framebuffer");
	CUdeviceptr cptr_keys;
	CUdeviceptr cptr_lines = 0;
	CUdeviceptr cptr_numLines = 0;
	CUdeviceptr cptr_uniforms;

	DeviceState deviceState;
	void* h_state_pinned = nullptr;
	void* h_tilecounter = nullptr;
	CUdeviceptr cptr_state;

	Line* h_lines = nullptr;
	uint32_t* h_numLines = nullptr;
	shared_ptr<CudaVirtualMemory> virt_lines_host = CURuntime::allocVirtual("lines_host");
	CUdeviceptr cptr_numLines_host = 0;

	vector<TriangleQueueItem> triangleQueue;

	AssetLibrary assetLibrary;

	// Motion control module: keyframe timeline for rigid objects.
	// (MotionController is a static-utility class; RiggedHumanController is
	// added once skinned human nodes are introduced.)
	motion::Timeline timeline;
	motion::RiggedHumanController rigController;

	// This is prepared at the start of the frame and provides most properties for CUDA kernels
	CommonLaunchArgs launchArgs;

	shared_ptr<SNTriangles> sn_box = nullptr;

	ImGuiContext* imguicontext_desktop = nullptr;
	ImGuiContext* imguicontext_vr = nullptr;

	struct{
		float splatSize                  = 1.0f;
		int splatAppearance              = SPLATAPPEARANCE_GAUSSIAN;
		bool sort                        = true;
		bool disableCUDA                 = false;
		bool showSolid                   = false;
		bool showTiles                   = false;
		bool showRing                    = false;
		bool showHeatmap                 = false;
		bool showAxes                    = true;
		bool showGrid                    = true;
		bool makePoints                  = false;
		bool showBoundingBoxes           = false;
		bool frontToBack                 = true;
		bool enableOpenglRendering       = false;
		bool renderCooperative           = false;
		bool enableEDL                   = true;
		bool showSplatletBoxes           = false;
		float splatletBoxSizeThreshold   = 16.0f;
		bool showDirtySplatletBoxes      = false;
		int rendermode                   = RENDERMODE_COLOR;
		bool enableStereoFramebufferTest = false;
		bool enableSplatCulling          = false;
		bool disableFrustumCulling       = false;
		bool cullSmallSplats             = true;
		bool requestDebugDump            = false;
		bool enableOverlapped            = true;
	int splatRenderer                = SPLATRENDERER_3DGS;
	int intersectionMode             = INTERSECTION_APPROXIMATE;
	int brushColorMode               = BRUSHCOLORMODE_NORMAL;

	// Point-cloud renderer selection. HQS (the existing path in points.cu) draws
	// every point every frame via atomicMin-depth + atomicAdd-color; PROGRESSIVE
	// is the Skye-style port (progressive_points.cu) that scales to hundreds of
	// millions of points by reprojecting last frame + filling a budget of new
	// points per frame.
	int pointRenderer                = POINTRENDERER_HQS;
	uint32_t progressiveBudget       = 1'000'000;   // points filled per frame (fixed mode)
	float progressivePointSize       = 1.0f;
	bool progressiveAdaptiveBudget   = false;       // GPU-timer-driven adaptive fill
	float progressiveTargetFrameMs   = 16.0f;       // target frame time for adaptive mode
	bool progressiveResetRequested   = false;       // host: clear reproject buffer next frame

		Brush brush;
		RectSelect rectselect;
		ColorCorrection colorCorrection;
		bool hideGUI                     = false;
		float vr_brushSize               = 0.1f;

		// - We need to be able to disable shortcuts while typing, for example.
		// - Actions at the end of a frame (e.g. processing an opened context menu) may want to disable shortcuts for as long as context menu is open.
		// - Such actions simply disable shortcuts for "two" frames. If the action is processed close to the end of the frame, -1 is subtracted right away, but the other preserves during the next frame. 
		int shortcutsDisabledForXFrames = 0;

		bool renderWarpwise = false;

		bool showDevStuff = false;
		bool showEditing = true;
		bool showKernelInfos = false;
		bool showMemoryInfos = false;
		bool showTimingInfos = false;
		bool showStats = false;
		bool showFileSaveDialog = false;
		bool showGettingStarted = false;
		bool showColorCorrection = false;
	bool showToolbar = true;
	bool showMotion = false;
	bool showPointCloud = false;          // progressive point-cloud control panel
	bool showPointCloudLoadDialog = false;
	bool showRemesh = false;              // point-cloud density optimization (voxel downsample) panel
	float remeshVoxelSize = 0.05f;        // current voxel size h for downsampling (meters)
	bool showPointCloudBA = false;        // GPU bundle-adjustment-style point-cloud refinement panel
	bool show4DGSImportDialog = false;    // 4DGS bundle import (canonical.ply + deformation_model.pt)
#ifdef SPLATSHOP_HAS_ORBBEC
	bool showOrbbec = false;              // Orbbec RGBD camera control panel
	bool showOrbbecPreview = false;       // Orbbec RGBD live preview panel
	bool orbbecPreviewPaused = false;     // freeze the preview on the current frame
	bool orbbecPreviewAutofit = true;     // auto-fit images to window
	int  orbbecDepthColormap = 0;         // 0=Turbo 1=Jet 2=Gray 3=Inferno
	float orbbecDepthMaxMeters = 4.0f;    // max distance for depth normalization
	bool orbbecDenoiseCompare = false;    // show raw vs denoised depth side-by-side
	// Orbbec live point-cloud display panel (3D, dedicated window).
	bool showOrbbecPointCloud = false;    // panel toggle
	bool orbbecPCPaused = false;          // freeze the panel on the current frame
	bool orbbecPCAutoFit = true;          // re-frame the camera from the AABB each frame
	float orbbecPCPointSize = 1.0f;       // HQS splat radius (in pixels)
	// Orbbec lens calibration panel + software-side undistortion toggles.
	bool showOrbbecCalibration = false;   // calibration panel toggle
	// 0 = off (show raw), 1 = on (show undistorted), 2 = compare (side-by-side).
	int  orbbecUndistortMode = 0;
	bool orbbecUseCalibratedIntrinsics = false; // use calibrated intrinsics for point cloud
	// Orbbec extrinsic (solvePnP) + depth correction panel + preview toggle.
	bool showOrbbecPnP = false;          // extrinsic PnP + depth correction panel
	// 0 = off (show raw depth), 1 = on (apply a*depth+b in the preview).
	int  orbbecDepthCorrectMode = 0;
#endif
#ifdef SPLATSHOP_HAS_K4A
	bool showK4a = false;                  // K4A Wrapper camera control panel
	bool showK4aPreview = false;           // K4A live preview panel (color/depth/IR)
	bool showK4aRecord = false;            // K4A recording / playback panel
	bool showK4aTransform = false;         // K4A coordinate transformation / point cloud panel
	bool k4aPreviewPaused = false;         // freeze the preview on the current frame
	bool k4aPreviewAutofit = true;         // auto-fit images to window
	int  k4aDepthColormap = 0;             // 0=Turbo 1=Jet 2=Gray 3=Inferno
	float k4aDepthMaxMeters = 4.0f;        // max distance for depth normalization
	bool k4aShowIr = false;                // show IR stream in preview
#endif
	bool openContextMenu = false;

		bool showInset = false;
		float dbg_factor = 1.0f;
		bool renderSoA = false;
		bool renderBandwidth = false;
		bool renderFragIntersections = false;

	} settings;

	struct {
		int hoveredObjectIndex = -1;
		int doubleClickedObjectIndex = -1;
		// int action = ACTION_NONE;
		// shared_ptr<SceneNode> placingItem = nullptr;
		int dbg_method = 0;

		shared_ptr<InputAction> currentAction = nullptr;
	} state;

	struct {
		bool menu_intersects = false;
		vec3 menu_intersection = vec3{0.0f, 0.0f, 0.0f};
	} state_vr; 

	struct ViewmodeDesktop{

	} viewmodeDesktop;

	struct ViewmodeDesktopVR{
		// Controllers should be relative to desktop monitor.
		// This transforms controllers into desktop view space.
		glm::mat4 m_controllers;

		// The right controllers neutral pose.
		// Use to callibrate from physical space to monitor/view space.
		glm::mat4 m_controller_neutral_left = glm::mat4(1.0);
		glm::mat4 m_controller_neutral_right = glm::mat4(1.0);
		glm::mat4 m_controller_neutral = glm::mat4(1.0);
	} viewmodeDesktopVr;

	struct ViewmodeImmersiveVR{
		// - Special world matrix for immersive VR.
		// - This is because in VR, we may want to transform the object
		//   as if it was a miniature toy model.
		// - Composition is: proj * view * world_vr * world
		glm::mat4 world_vr; 
	} viewmodeImmersiveVR;

	ImFont* font_default = nullptr;
	ImFont* font_big = nullptr;
	ImFont* font_vr_title = nullptr;
	ImFont* font_vr_text = nullptr;
	ImFont* font_vr_smalltext = nullptr;

	static void setup();

	void imguiStyleVR();

	CommonLaunchArgs getCommonLaunchArgs();

	// Some of these functions mainly exist because their functionality is used twice (e.g. via GUI & shortcut),
	// and we want to ensure that both approaches end up doing exactly the same thing.
	Box3 getSelectionAABB();
	void updateBoundingBox(SNSplats* node, bool onlySelected = false);
	void updateBoundingBox(SNTriangles* node);
	void updateBoundingBox(PointData& model);
	void insertNodeToNode(shared_ptr<SceneNode> node, shared_ptr<SceneNode> layer, bool onlySelected = false);
	bool merge(shared_ptr<SceneNode> snsource, shared_ptr<SceneNode> sntarget); 
	void applyTransformation(GaussianData& model, mat4 transformation, bool onlySelected = false);
	void apply(GaussianData& model, ColorCorrection value);
	// void setSelected(GaussianData& model);
	void setSelectedNode(SceneNode* node);
	void transformAllSelected(mat4 transform);
	void selectAll();
	void deselectAll();
	void invertSelection();
	void deleteSelection();
	void createOrUpdateThumbnail(SceneNode* node);
	void uploadSplats(SNSplats* node);
	void drawGUI();
	void resetEditor();
	void unloadTempSplats();
	void loadTempSplats(string path);
	void inputHandling();
	void inputHandlingDesktop();
	void inputHandlingVR();
	void setDesktopMode();
	void setDesktopVrMode();
	void setImmersiveVrMode();
	void setRemoteStereoMode();
	shared_ptr<SNSplats> clone(SNSplats* source);
	void sortSplatsDevice(SNSplats* node, bool putDeletedLast = false);
	void drawSphere(_DrawSphereArgs args);
	void drawLine(vec3 start, vec3 end, uint32_t color = 0xffff00ff);
	void drawBox(Box3 box, uint32_t color = 0xffff00ff);
	Uniforms getUniforms();
	void initCudaProgram();
	shared_ptr<SNTriangles> ovrToNode(string name, RenderModel_t* model, RenderModel_TextureMap_t* texture);
	shared_ptr<SceneNode> getSelectedNode();
	void temporarilyDisableShortcuts(int numFrames = 2);
	bool areShortcutsDisabled();
	void filter(SNSplats* source, SNSplats* target, FilterRules rules);
	shared_ptr<SNSplats> filterToNewLayer(FilterRules rules);
	vector<shared_ptr<SceneNode>> getLayers();
	// Erases given node at the very end of the frame. Was needed because there were issues when
	// directly deleting a node while drawing that node's gui.
	void scheduleRemoval(SceneNode* node);
	void applyDeletion();
	void revertDeletion();
	int32_t getNumSelectedSplats();
	int32_t getNumDeletedSplats();

	struct LambdaAction{
		function<void()> undo;
		function<void()> redo;
	};

	void addAction(LambdaAction action);
	void addAction(shared_ptr<Action> action);
	void undo();
	void redo();
	void clearHistory();

	// undoable functions
	void selectAll_undoable();
	void selectAllInNode_undoable(shared_ptr<SNSplats> node);
	void deselectAll_undoable();
	void invertSelection_undoable();
	void deleteSelection_undoable();
	void deleteNode_undoable(shared_ptr<SceneNode> node);
	shared_ptr<SNSplats> filterToNewLayer_undoable(FilterRules rules);
	shared_ptr<SNSplats> duplicateLayer_undoable(shared_ptr<SNSplats> node);
	// shared_ptr<SNSplats> extractLayer_undoable(shared_ptr<SNSplats> node);
	void merge_undoable(shared_ptr<SceneNode> snsource, shared_ptr<SceneNode> sntarget); 

	// GUI
	void setAction(shared_ptr<InputAction> action);
	void makeSettings();
	void makePerf();
	void makeMenubar();
	void makeLayers();
	void makeStats();
	void makeTODOs();
	void makeToolbar();
	// void makeGuiVr();
	void makeEditingGUI();
	void makeDevGUI();
	void makeDebugInfo();
	void makeAssetGUI();
	void makeColorCorrectionGui();
	void makeSaveFileGUI();
	void makeGettingStarted();
	void makeMotionGUI();
	void makePointCloudGUI();
	void makeRemeshGUI();
	void makePointCloudBAGUI();
	void makeOrbbecGUI();
	void makeOrbbecPreviewGUI();
	void makeOrbbecPointCloudGUI();
	void makeOrbbecCalibrationGUI();
	void makeOrbbecPnPGUI();

	// K4A Wrapper panel methods (stubs when SPLATSHOP_HAS_K4A is undefined).
	void makeK4aGUI();
	void makeK4aPreviewGUI();
	void makeK4aRecordGUI();
	void makeK4aTransformGUI();

	// Point-cloud density optimization. Voxel-grid downsampling: collapses every
	// occupied voxel of edge length voxelSize to its centroid, producing a new
	// SNPoints node with uniform point spacing ~voxelSize. Non-destructive: the
	// source cloud is preserved. adaptiveFill is reserved for Phase-2 splitting
	// of sparse regions and is not yet implemented.
	shared_ptr<SNPoints> remeshPointCloud(SNPoints* src, float voxelSize, bool adaptiveFill = false);

	// MISC
	CudaGlMappings mapCudaGl(shared_ptr<GLTexture> source);
	
	// UPDATE & DRAW 
	void update();
	void render();
	// void draw(Scene* scene, View view, RenderTarget& target);
	void draw(Scene* scene, vector<RenderTarget> targets);
	void postRenderStuff();

};