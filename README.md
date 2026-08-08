# Splatshop 

Paper: [Splatshop: Efficiently Editing Large Gaussian Splat Models](https://diglib.eg.org/bitstream/handle/10.1111/cgf70214/cgf70214.pdf), Markus Schütz, Christoph Peters, Florian Hahlbohm, Elmar Eisemann, Marcus Magnor, Michael Wimmer, Computer Graphics Forum 2025

This editor aims to enable editing of [gaussian splat models](https://github.com/graphdeco-inria/gaussian-splatting), e.g., cleaning up reconstructed models, assembling cleaned-up assets into a library, composing scenes, and painting. It currently supports up to a hundred million splats on desktop and up to around ten million in VR on an RTX 4090. 

There is still a lot of work to do - we are releasing it now in order to gather feedback and figure out which features and fixes to prioritize. 

An example data set is available [here](http://users.cg.tuwien.ac.at/mschuetz/permanent/splatmodels.zip). Unpack it, then drag&drop it into the editor. The garden splat model is from [Inria 3DGS](https://github.com/graphdeco-inria/gaussian-splatting), and originates from [Mip-NeRF 360: Unbounded Anti-Aliased Neural Radiance Fields](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9878829). 

A precompiled windows binary is available [here](http://users.cg.tuwien.ac.at/mschuetz/permanent/Splatshop_2025.03.21_v0.01f.zip). However, it may still be necessary to install CUDA 12.4 or later since CUDA kernels are compiled at startup. 

<img src="docs/paint_vr.gif" width="45%"/> <img src="docs/transform_vr.gif" width="45%"/>

### Features

<ul>
	<li> Undo and Redo
	<li> Virtual Reality viewing and editing
	<li> Select via brush, sphere, rectangle; Delete with brush or del button
	<li> Translate, scale, rotate of selected nodes or splats.
	<li> Painting
	<li> No pip or conda issues! (no Python)
	<li> CUDA Driver API - Can edit and hot-reload CUDA code like shaders.
	<li> Includes support for <a href="https://github.com/nerficg-project/HTGS">perspective-correct gaussians</a>. (Splat models need to be trained with respective method)
		<li> Object motion control and skeletal skinning (see <code>src/motion/</code>).
		<li> HTTP-based remote control API: camera / mouse / keyboard / rigid-body motion / Gaussian splats creation, loading, deletion and coloring, drivable from local Python programs or remote WebRTC receivers (see <a href="#remote-control-api">Remote Control API</a>).
		<li> <b>4D Gaussian Splatting (4DGS) dynamic scene rendering</b> — load and play back dynamic 4DGS scenes trained with <a href="https://github.com/hustvl/4DGaussians">hustvl/4DGaussians</a>. Includes deformation via HexPlane + MLP, timeline controls, and real-time editing of canonical Gaussians (see <a href="#4dgs-dynamic-scenes">4DGS Dynamic Scenes</a>).
		<li> <b>Point cloud loading &amp; rendering</b> — load <code>.las</code> / <code>.bin</code> / point-cloud <code>.ply</code> files into dedicated <code>SNPoints</code> nodes, rendered either with the all-points HQS path or the Skye-style progressive renderer that scales to hundreds of millions of points via per-frame reprojection + budgeted hole-filling.
		<li> <b>Point cloud density optimization (remeshing)</b> — voxel-grid downsampling that collapses every occupied voxel of edge <code>h</code> to its centroid, producing a new uniformly-spaced cloud that reduces VRAM and improves render speed while preserving geometric coverage. Non-destructive: the source cloud is kept. Open the <em>Remesh</em> panel from the toolbar (see <code>src/remesh/points_remesh.cu</code>).
		<li> <b>VR remote viewing</b> - browse a 3D scene from a standalone/PC VR headset over the network: the headset's pose is streamed to the server, which renders stereo left/right eyes and streams them back (see <a href="#vr-remote-viewing">VR Remote Viewing</a>).
	</ul>

### Known Issues
<ul>
	<li> Academic Prototype - expect a fair amount of bugs and crashes -> Consider it an "alpha" version.
	<li> VR only on windows - No idea how to include OpenVR on linux. 
	<li> No Spherical Harmonics due to large mem req. with modest gains - we will check out more frugal SH options in the future. 
	<li> CUDA Driver API - Long startup because CUDA compiles at runtime. 
	<li> No precompiled binaries yet. 
	<li> Undo and redo may not cover all functionality yet. 
	<li> Undo of translate/scale/rotate is currently lossy. Especially scale is prone to loss of precision when scaling too far down. 
	<li> App freezing: Line rendering caused freezes on RTX 20xx series cards. That was fixed, but there is a chance that triangle models in VR mode may also freeze RTX 20xx series cards. 
</ul>

## Installing

<details>
<summary>Windows</summary>

Dependencies: 
* CUDA 12.4 or later (CUDA 13.x also verified)
* Visual Studio 2022 (version 17.10.3 or later)
* CMake 3.22 or later
* RTX 4070 or better recommended (RTX 5090(D) also verified).
* For 4DGS dynamic scenes: LibTorch 2.6+ (see <a href="#4dgs-dynamic-scenes">4DGS section</a>)

Generate the Visual Studio solution and build the Release binary from the command line:

```bash
# 1. Configure (generates build/SplatEditor.sln)
cmake -B build -S .

# 2. Build the Release target
cmake --build build --config Release --target SplatEditor

# 3. Run from the repository root so resources resolve correctly
./build/Release/SplatEditor.exe
```

Alternatively, open `build/SplatEditor.sln` in Visual Studio and compile in `Release` mode.

Running from the project directory is semi-important because the editor will look for resources such as ```./src/gaussians_rendering.cu``` and ```./resources/images/symbols_32_32.png``` relative to the project directory. Alternatively, you can copy the resources and src folders into the binary directory.

</details>

<details>
<summary>Linux</summary>

Dependencies:
* gcc 14, g++14
* CUDA 12.4 or higher
* NVIDIA driver version 550 or higher. (Lower may cause issues when compiling GPUSorting)

```bash
mkdir build
cd build
export CUDA_PATH=/usr/local/cuda-12.4/
cmake -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/libcudart.so ..
make
```

Potential Issues:
- May or may not require TBB. Uncomment the TBB part in CMakeLists, if it does.

```
// set CUDA_PATH variable to wherever your CUDA Toolkit is installed
export CUDA_PATH=/usr/local/cuda-12.4/

// May or may not require setting LD_LIBRARY_PATH to gcc 14, similar to this, or whereever gcc14 is installed
export LD_LIBRARY_PATH=~/gcc-14.1.0/x86_64-linux-gnu/libstdc++-v3/src/.libs:$LD_LIBRARY_PATH

// run from workspace root
./build/SplatEditor
```
	</details>

## 4DGS Dynamic Scenes

Splatshop supports loading and interactive playback of 4D Gaussian Splatting (4DGS) dynamic scenes trained by [hustvl/4DGaussians](https://github.com/hustvl/4DGaussians). The 4DGS approach represents a dynamic scene as:
- **Canonical Gaussians** (a standard `.ply` file, the rest-pose 3D Gaussians)
- **Deformation field** (a HexPlane + MLP that displaces canonical Gaussians per timestep)

### Dependencies

**LibTorch 2.0+ (C++ distribution)**

Download the LibTorch C++ distribution from [pytorch.org](https://pytorch.org/get-started/locally/). Select: **LibTorch**, **C++**, match your GPU.

| GPU | Recommended LibTorch |
|-----|---------------------|
| RTX 30/40 series | [LibTorch CUDA 12.4](https://download.pytorch.org/libtorch/cu124/libtorch-win-shared-with-deps-latest.zip) |
| **RTX 5090 (Blackwell)** | [LibTorch CUDA 12.8](https://download.pytorch.org/libtorch/cu128/libtorch-win-shared-with-deps-2.6.0%2Bcu128.zip) |

> **Note**: LibTorch bundles its own CUDA runtime libraries. Your system CUDA version (e.g. 13.2) only needs to be ≥ the LibTorch CUDA version at the driver level — full backward compatibility is guaranteed. Splatshop itself compiles against your system CUDA toolkit independently.

	Set the `Torch_DIR` CMake variable (or `LIBTORCH_PATH` environment variable):

	```bash
	# Windows
	cmake -B build -S . -DTorch_DIR="C:/libtorch/share/cmake/Torch"

	# Linux
	export LIBTORCH_PATH=/path/to/libtorch
	cmake -B build -S .
	```

	### Compatibility

	LibTorch bundles its own CUDA runtime libraries — it does **not** depend on your system CUDA Toolkit. It only requires the GPU driver to be ≥ the LibTorch CUDA version.

	| System CUDA | GPU | LibTorch to use |
	|---|---|---|
	| 12.4 — 12.7 | RTX 30/40 series | cu124 ✅ |
	| 12.8+ | RTX 50 series (Blackwell) | cu128 ✅ |
	| 13.x | RTX 5090(D) | cu128 ✅ (driver backward-compatible) |

	### Quick Setup (RTX 5090D + CUDA 13.2)

	```bash
	# Step 1: Download LibTorch cu128
	curl -L -o libtorch.zip ^
	  "https://download.pytorch.org/libtorch/cu128/libtorch-win-shared-with-deps-2.6.0%%2Bcu128.zip"

	# Step 2: Extract to libs/libtorch (project-relative, auto-detected by CMake)
	tar -xf libtorch.zip -C libs/

	# Step 3: Configure & build — CMake auto-detects libs/libtorch
	cmake -B build -S .
	cmake --build build --config Release
	```

If LibTorch is not found, 4DGS is silently disabled — the rest of Splatshop builds normally.

### Exporting a 4DGS Model

**Step 1**: Train a 4DGS model using the [hustvl/4DGaussians](https://github.com/hustvl/4DGaussians) codebase:

```bash
# In the 4DGaussians repo
python train.py -s data/dnerf/lego --port 6009 --expname "dnerf/lego" --configs arguments/dnerf/lego.py
```

**Step 2**: Convert the training checkpoint to a TorchScript bundle:

```bash
# In the Splatshop repo
python tools/export_4dgs_torchscript.py \
    --checkpoint <4DGaussians>/output/dnerf/lego/chkpnt30000.pth \
    --ply <4DGaussians>/output/dnerf/lego/point_cloud/iteration_30000/point_cloud.ply \
    --out lego_4dgs
```

This produces:
```
lego_4dgs/
  canonical.ply          # Canonical (rest-pose) 3D Gaussians
  deformation_model.pt   # TorchScript deformation module
  config.json            # Metadata
```

### Loading & Playback

1. Launch Splatshop
2. Drag `canonical.ply` into the editor window → loads the rest pose
3. In the **Motion** panel (Toolbar → Motion), go to **4DGS Dynamic Scene**
4. Click **Import model** and select `deformation_model.pt`
5. Use the play/pause button or scrub the timeline to animate

The timeline cycles from t=0.0 to t=1.0 and back (bounce loop). Adjust **speed** to control playback rate.

### Editing

- **Pause** at any frame to inspect the dynamic scene
- **Select/delete/paint** canonical Gaussians — edits apply to the rest pose
- All frames automatically reflect the edit after the next deformation pass
- Toggle **Deformation active** to temporarily view/compare the canonical pose

### Architecture Notes

The integration uses LibTorch's CUDA backend; deformed tensors share the same CUDA context as Splatshop's own CUDA buffers. The deformation is computed once per frame (cached when time hasn't changed), producing deformed positions/scales/rotations that are fed into the existing 3DGS tile-based rasterizer unchanged. No new CUDA kernels are needed for the deformation itself.

## Remote Control API

Splatshop ships with an optional out-of-process HTTP API that exposes the running editor to other local programs or remote clients (e.g. a WebRTC receiver). It is a two-layer bridge:

```
remote client (HTTP)  ──►  remote_api/ (FastAPI, Python)  ──►  TCP JSON-RPC  ──►  Splatshop C++ (RemoteControlServer)
```

* **C++ side** (`src/remote/RemoteControlServer.{h,cpp}`) is built into `SplatEditor.exe` and listens on TCP port `7654` by default. It dispatches commands to the main thread via the existing `schedule()` event queue, so it is safe with CUDA / OpenGL contexts.
* **Python side** (`remote_api/`) is a FastAPI server that translates HTTP requests into newline-delimited JSON-RPC calls against the C++ bridge. It exposes camera, mouse, keyboard, rigid-body motion and Gaussian splats creation/loading/deletion/coloring endpoints, plus a self-contained browser test page at `GET /test`.

Capabilities exposed over the API:

| Category | Examples |
|---|---|
| Camera | orbit / pan / zoom / set pose / focus on bounding box |
| Mouse | move, button press/release, scroll, raw events |
| Keyboard | key press/release, key sequences (GLFW key names) |
| Scene / rigid body | list nodes, get/set/translate/rotate/scale/anime transform |
| Splats | create sphere / box / points clouds, load `.ply` / `scene.json`, remove node, set color |

### Setup (Python frontend)

The Python frontend is independent of the C++ build and can run in a dedicated conda environment:

```bash
# Option A: conda (recommended, see environment.yml)
conda env create -f environment.yml
conda activate splat-remote

# Option B: plain pip
pip install -r remote_api/requirements.txt
```

### Running

```bash
# 1. Start Splatshop (the C++ editor) from the repo root
./build/Release/SplatEditor.exe

# 2. Start the Python HTTP API (default port 8080)
uvicorn remote_api.server:app --host 0.0.0.0 --port 8080

# 3. Open the browser test page
#    http://localhost:8080/test
```

Environment variables (all optional):

| Variable | Default | Description |
|---|---|---|
| `SPLAT_BRIDGE_HOST` | `127.0.0.1` | C++ bridge host |
| `SPLAT_BRIDGE_PORT` | `7654` | C++ bridge port |
| `SPLAT_API_TOKEN` | _(unset)_ | If set, clients must send `Authorization: Bearer <token>` |
| `SPLAT_API_PORT` | `8080` | Override the API listen port |

### Quick example

Create a sphere of splats from a remote client and move it:

```bash
# Create a sphere of 500 red splats, returns {"id": 15, ...}
curl -X POST http://localhost:8080/scene/splats/create \
  -H "Content-Type: application/json" \
  -d '{"type":"sphere","params":{"radius":1.5,"count":500,"color":[1,0,0,1]}}'

# Animate node 15 to translation [3,0,0] over 1.5s
curl -X POST http://localhost:8080/motion/node/15/animate \
  -H "Content-Type: application/json" \
  -d '{"target":{"translation":[3,0,0]},"duration_s":1.5,"ease":"in_out"}'
```

Full endpoint reference, schemas, WebRTC integration guide and troubleshooting live in [`docs/remote_api.md`](docs/remote_api.md).

## VR Remote Viewing

In addition to local OpenVR rendering, Splatshop can render for a **remote** VR headset (a standalone headset's browser via WebXR, or a PC-VR client) over the network. The headset's 6DOF pose is sent to the server, which renders stereo left/right eyes using the existing Gaussian-splatting rasterizer and streams the frames back to the headset.

```
VR headset ──pose (HTTP :8080)──► remote_api (FastAPI) ──► Splatshop C++
                                                                   │ stereo render (VIEWMODE_REMOTE_STEREO)
VR headset ◄──video (WebSocket :8081)── FrameStreamer ◄────────────┘
```

This adds the previously-missing **video return channel** alongside the existing control channel - the two are deliberately decoupled (control = small high-frequency pose packets; video = encoded frame stream).

**Using the WebXR client** (standalone headset, e.g. Quest):

1. Start Splatshop and the Python API as in the Remote Control API section.
2. Serve `remote_api/examples/vr_webxr_client.html` over **HTTPS or localhost** (WebXR requires a secure context) on a machine reachable from the headset's browser.
3. Open the page in the headset browser, ensure the host points to the Splatshop machine, and click **Enter VR**.

The client reads `XRViewerPose` per-eye view/projection matrices, POSTs them to `/vr/pose` with `pose_space: "webxr"`, and renders the returned side-by-side JPEG frames into the WebXR layer.

**Coordinate spaces** (the most error-prone part - see `docs/remote_api.md` for the full table): `pose_space` selects how the server interprets the matrices. `"webxr"` applies a basis change into the app's GL space; `"openvr"` reuses the local-VR `flip` convention; `"raw_view"` uses matrices already in app space.

**Encoding**: JPEG (default, no external dependency, suitable for LAN validation). H.264 hardware encoding via NVENC is a prepared path gated behind `SPLATSHOP_HAS_NVENC` (requires the NVIDIA Video Codec SDK, not bundled with the CUDA toolkit).

## Getting Started

### Key and Mouse (as hardcoded in ```inputHandlingDesktop.h```)

<table>
	<tr>
		<td>Right Click</td>
		<td>Cancel current action</td>
	</tr>
	<tr>
		<td>Double Click</td>
		<td>Move towards hovered splats</td>
	</tr>
	<tr>
		<td>Ctrl + z, Ctrl + y</td>
		<td>Undo, Redo</td>
	</tr>
	<tr>
		<td>1</td>
		<td>Brush-Selection Action</td>
	</tr>
	<tr>
		<td>2</td>
		<td>Brush-Deletion Action</td>
	</tr>
	<tr>
		<td>t, r, s</td>
		<td>Translate, Rotate, Scale</td>
	</tr>
	<tr>
		<td>b</td>
		<td>Painting</td>
	</tr>
	<tr>
		<td>Ctrl + D</td>
		<td>Duplicate selection to new layer</td>
	</tr>
	<tr>
		<td>Ctrl + E</td>
		<td>Merge selected layer into the one below</td>
	</tr>
</table>

<img src="docs/vr_interaction.jpg" />

### Delete and Duplicate

<table>
	<tr>
		<td><img src="./docs/gardening_0.jpg" />1</td>
		<td><img src="./docs/gardening_1.jpg" />2</td>
		<td><img src="./docs/gardening_2.jpg" />3</td>
		</tr><tr>
		<td><img src="./docs/gardening_3.jpg" />4</td>
		<td><img src="./docs/gardening_4.jpg" />5</td>
		<td><img src="./docs/gardening_5.jpg" />6</td>
	</tr>
</table>

1. Select splats with spherical selection brush.
2. Delete selection (key: del).
3. Select a patch of grass with brush selection and intersection set to "center". This avoids selecting nearby large splats.
4. Duplicate selection (key: ctrl + d), then move it with translation tool (key: t). 
5. Repeat with varying patches of grass until the hole is covered. Overlapping lighter and darker patches may result in near-seamless transitions. 

### Cleanup

<table>
	<tr>
		<td><img src="./docs/cleanup_0.jpg" />1</td>
		<td><img src="./docs/cleanup_1.jpg" />2</td>
		<td><img src="./docs/cleanup_2.jpg" />3</td>
		<td colspan="2" rowspan="2"><img src="./docs/cleanup_6.jpg" width="1500px"/></td>
		</tr><tr>
		<td><img src="./docs/cleanup_3.jpg" />4</td>
		<td><img src="./docs/cleanup_4.jpg" />5</td>
		<td><img src="./docs/cleanup_5.jpg" />6</td>
		<td></td>
	</tr>
</table>

1. Drag&Drop ply file
2. Select region of interest with spherical selection tool. 
3. Invert selection and delete (key: del).
4. Rotate until model aligns with ground plane (key: r) and translate to origin (key: t). If the ground is sufficiently densely reconstructed, you can also try using the 3-point-alignment tool, which aligns the model by specifying three points on the ground. 
5. Remove undesired splats with a combination of circular and spherical selection and deletion tools. To retain a nice circular ground, select splats with a spherical brush
6. Then select the remainder of the splats that should remain. Invert selection and delete.

### Add an asset to library

<table>
	<tr>
		<td><img src="./docs/newasset_0.jpg" /></td>
		<td><img src="./docs/newasset_1.jpg" /></td>
		<td><img src="./docs/newasset_2.jpg" /></td>
		<td><img src="./docs/newasset_3.jpg" /></td>
	</tr>
</table>

1. Select splats
2. Duplicate selection (key: ctrl + d)
3. In the layers menu, right click the duplicated layer and give it a new name.
4. Right click the layer again, and select "Create asset from Layer".

### Adding new actions to toolbar

- Choose a suitable existing action and copy it, e.g. [BrushSelectAction](./src/actions/BrushSelectAction.h).
- Rename accordingly and adapt the code to your needs. Perhaps remove the undoable parts for now.
- ```update()``` is called every frame and handles most things. 
- ```makeToolbarSettings()``` allows you do specify an imgui user interface that is shown below the toolbar while your action is active. 
- Add your action to [toolbar.h](src/gui/toolbar.h). Include it and add a Button or ImageButton, similar to other actions. When the button is clicked, create a shared_ptr of your action and call setAction(action) to activate it. It will automatically be deactivated on right-click. 
- If you need to add an icon, you can add one to [symbols.svg](resources/images/symbols.svg) and export it to symbols_32x32.png. 

## Acknowledgements

* Contributors: Markus Schütz, Christoph Peters, Florian Hahlbohm, Elmar Eisemann, Michael Wimmer
* Bernhard Kerbl, Georgios Kopanas, Thomas Leimkühler, George Drettakis for [3D Gaussian Splatting
for Real-Time Radiance Field Rendering](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/)
* Mark Kellog for his WebGL implementation https://github.com/mkkellogg/GaussianSplats3D which helped substantially with the 3DGS implementation.
* Thomas Smith for his [GPUSorting](https://github.com/b0nes164/GPUSorting) project that is used to sort the gaussians in this project. 
* Omar's [Dear ImGui](https://github.com/ocornut/imgui). 
* This particular garden model is a pretrained model from [Inria 3DGS](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/) and rotated to fit our editor. The garden data set originates from [Mip-NeRF 360: Unbounded Anti-Aliased Neural Radiance Fields](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9878829).
* [SuperSplat](https://superspl.at/) - A web-based gaussian splat editor, from which we learned the usefullness of the ring rendering mode for editing, especially finding and removing floaters. 

## Citation

<pre>
@article{10.1111:cgf.70214,
	journal = {Computer Graphics Forum},
	title = {{Splatshop: Efficiently Editing Large Gaussian Splat Models}},
	author = {Schütz, Markus and Peters, Christoph and Hahlbohm, Florian and Eisemann, Elmar and Magnor, Marcus and Wimmer, Michael},
	year = {2025},
	publisher = {The Eurographics Association and John Wiley & Sons Ltd.},
	ISSN = {1467-8659},
	DOI = {10.1111/cgf.70214}
}
</pre>