## 已确认的根因（经代码检查验证）

**问题1 - Color Correction 对 RGBD 流不生效：**
- `Apply` 按钮 (`src/gui/colorCorrection.h:41`) 使用 `onTypeMatch<SNSplats>`。`SNOrbbec` 继承自 `SNPoints`，**不是** `SNSplats`，所以对实时点云是空操作。
- 渲染时预览路径位于 `scene->process<SNSplats>(...)` 内 (`SplatEditor_draw.h:380-409`)。点云渲染分派 (`SplatEditor_draw.h:1688-1740`，`scene->process<SNPoints>`) 从不读取 `settings.colorCorrection`。
- 点云内核 `kernel_hqs_depth`/`kernel_hqs_color` (`src/render/points.cu:302,363`) **没有** `ColorCorrection` 参数。`points.cu` 已 `#include` `HostDeviceInterface.h`（第25行），所以 `applyColorCorrection()` 在 device 端可用。
- 2D RGB 预览 (`src/gui/orbbec_preview.h`) 完全绕过颜色校正。

**问题2 - RGB 显示成 BGR：**
- `src/gui/orbbec_preview.h:220-222`：`OB_FORMAT_BGRA` 分支用 `GL_RGBA` 上传原始字节。`GL_RGBA` 把内存解释为 R,G,B,A，但 BGRA 内存是 B,G,R,A → R 和 B 互换。这是确切的 bug。（`GL_BGRA = 0x80E1` 在 glew.h 中可用，已确认。）

**问题3 - 实时点云没有显示窗口：**
- 3D 点云只在主视口显示，且必须在 Orbbec 面板手动点击 "Show preview" 后才出现。点云的 transform 是单位矩阵，默认相机远离原点，所以经常在视野外。没有专门的实时点云显示窗口。

---

## 实现计划

### Part A - 修复 RGB/BGR 显示 bug（问题2）
**文件：`src/gui/orbbec_preview.h`**（220-222行）

将 `OB_FORMAT_BGRA || OB_FORMAT_RGBA` 分支拆为两个：
- `OB_FORMAT_BGRA`：用 `GL_BGRA`（源格式）上传到 `GL_RGBA8` 内部纹理 —— 无需字节交换。
- `OB_FORMAT_RGBA`：保持当前 `GL_RGBA` 直接上传。

两者都保持 `uploadBpp = 4`。为安全明确，将**内部格式**固定为 `GL_RGBA8`，只让**源格式**在 `GL_RGB`/`GL_RGBA`/`GL_BGRA` 间切换。这需要在 `glTexImage2D` 分配和 `glTexSubImage2D` 上传中将 `uploadGlFormat`（源）与新的 `internalFormat`（颜色固定为 `GL_RGBA8`）解耦。深度纹理路径不受影响。

### Part B - 将 Color Correction 接入 RGBD 流（问题1）
采用实时预览方式（非破坏性、逐帧），镜像现有 splat 预览语义。

**B1. 给点云渲染内核加 `ColorCorrection` 参数 - `src/render/points.cu`**
- 给 `kernel_hqs_color`（363行）加 `ColorCorrection colorCorrection` 参数，在 splat 之前对点的 RGB 应用校正：
  ```cpp
  vec4 c = vec4(float(rgba[0])/255.f, float(rgba[1])/255.f, float(rgba[2])/255.f, 1.f);
  c = applyColorCorrection(c, colorCorrection);
  rgba[0] = (uint8_t)clamp(c.r * 255.f, 0.f, 255.f);
  rgba[1] = (uint8_t)clamp(c.g * 255.f, 0.f, 255.f);
  rgba[2] = (uint8_t)clamp(c.b * 255.f, 0.f, 255.f);
  ```
  放在 `isSelected` 高亮块**之后**、splat 循环之前。
- `kernel_hqs_depth` 不需要该参数（只写深度）。
- `kernel_hqs_normalize` 不需要（它平均的是已校正的颜色）。
- progressive 路径 (`progressive_points.cu`) 不动 —— 它只用于静态、已上传完毕的点云，从不对 `SNOrbbec` 使用。

**B2. 从 host 传 `settings.colorCorrection` - `src/SplatEditor_draw.h`**（HQS color 循环，约1727行）
- 按节点构造 `ColorCorrection colorCorrection`，以 `node->selected` 为门控，与 splat 路径完全一致：
  ```cpp
  for(SNPoints* node : hqsNodes){
      ColorCorrection cc;
      if(node->selected) cc = settings.colorCorrection;
      prog_points->launch("kernel_hqs_color",
          {&launchArgs, &node->manager.data, &target,
           &virt_fb_depth->cptr, &virt_fb_color->cptr, &pointSize, &cc},
          node->manager.data.count, mainstream);
  }
  ```
- 内核签名变更需要使 `cubins/points.cu.cubin` 缓存失效（删除它），以便下次启动时通过 NVRTC 重新编译 `points.cu`。我会删除 `./cubins/points.cu.cubin`（如果存在）。

**B3. 保持 `Apply`（bake）按钮行为一致 - `src/gui/colorCorrection.h`**（38-46行）
- bake 路径只操作 `GaussianData`，对 `PointData` 没有等价的就地 bake。为避免选中 `SNOrbbec`/`SNPoints` 时令人困惑的空操作，给 Apply 加一个 `onTypeMatch<SNPoints>` 分支：对非 splat 选中节点，Apply 仅重置滑块（实时预览滑块已为选中节点驱动渲染，对瞬态流"bake"无意义），与 `Revert` 相同。这是最小、诚实的行为 —— 不做假 bake。

### Part C - 专用实时点云显示窗口（问题3）
在 Orbbec Preview 区域新增第三个 ImGui 面板（"Orbbec Point Cloud"），通过现有 HQS CUDA 路径把实时 `SNOrbbec` 点云渲染到自己的小 `RenderTarget`，blit 到 GL 纹理，用 `ImGui::Image` 显示。复用所有现有基础设施（无新 GLFW 窗口）。

**C1. `SplatEditor` 新状态**（`src/SplatEditor.h`，`#ifdef SPLATSHOP_HAS_ORBBEC` 块内，约139行）：
- `bool showOrbbecPointCloud = false;` —— 面板开关
- `GLuint orbbecTexPointCloud = 0;` + `int orbbecTexPointCloudW/H`
- `shared_ptr<CudaVirtualMemory> orbbecPCframebuffer` —— 小型专用 framebuffer（如 512×384），不覆盖主视口的 `virt_framebuffer`。
- 面板视图相机状态：`float orbbecPCyaw`、`orbbecPCpitch`、`orbbecPCradius`、`vec3 orbbecPCtarget`、`bool orbbecPCautoFit = true` —— 从点云包围盒 (`manager.data.min`/`max`) 计算的轨道相机。
- `shared_ptr<CudaVirtualMemory> orbbecPCfb_depth`、`orbbecPCfb_color` —— 面板的 HQS 暂存缓冲（主 `virt_fb_depth`/`virt_fb_color` 按主视口尺寸分配；面板用不同分辨率）。

**C2. `SplatEditor::render()` 中新增 render+blit 步骤**（`src/SplatEditor_render.h`，在主 `kernel_toOpenGL` blit 之后，由 `#ifdef SPLATSHOP_HAS_ORBBEC && settings.showOrbbecPointCloud` 守卫）：
- 若 `editor->snOrbbec` 存在且已上传点（`manager.data.count > 0`）：
  1. `orbbecPCframebuffer->commit(pcW*pcH*8)`。
  2. 构造 `RenderTarget pcTarget`：`framebuffer = orbbecPCframebuffer->cptr`，`width/height = pcW/pcH`，`view`/`proj` 来自由点云 AABB (`min`/`max`) 推导的轨道相机。`autoFit` 每帧从 AABB 重算 target/radius，让流式点云始终被框住。
  3. `cuMemsetD32Async` 清 `orbbecPCfb_depth`/`orbbecPCfb_color`（INF / 0）。
  4. 在 `prog_points` 上启动 `kernel_clearFramebuffer`，然后 `kernel_hqs_depth` + `kernel_hqs_color`（带 `ColorCorrection` 参数 —— 让面板也反映颜色校正）+ `kernel_hqs_normalize`，与主 HQS 块一致，但指向面板的 `pcTarget` 和暂存缓冲。
  5. `mapCudaGl(orbbecTexPointCloud)` → `kernel_toOpenGL`（blit `pcTarget` → GL 纹理）→ `unmap()`。
- 面板**不**在没有 `SNOrbbec` 时向场景添加节点；而是要求用户开启 "Generate RGB point cloud" 并惰性自动创建 `snOrbbec` 节点（镜像 "Show preview" 按钮），保证实时馈送。保持点云单一数据源。

**C3. 新 GUI 面板 - `src/gui/orbbec_preview.h`**（追加 `makeOrbbecPointCloudGUI()`，从 `SplatEditor.cpp` 在其它 Orbbec GUI 调用附近调用）：
- `ImGui::Begin("Orbbec Point Cloud", &settings.showOrbbecPointCloud)`
- 工具栏："Auto-fit" 复选框（从 AABB 重算相机）、点大小滑块、"Pause" 开关、FPS 文本。
- `ImGui::Image(orbbecTexPointCloud, ...)` 带宽高保持 fit（复用 RGB/Depth 图像相同的 autofit 数学）。
- 状态行：点数、AABB 范围、若 `snOrbbec` 为空则显示 "no cloud - enable RGB point cloud in Orbbec panel"。
- 轨道相机鼠标控制：拖拽旋转 yaw/pitch、滚轮缩放 radius（直接写入面板相机状态）。这是自包含轨道，不触碰 `GLRenderer::camera`（主视口相机）。

**C4. 菜单栏入口 - `src/gui/menubar.h`**（现有 "Orbbec Preview" 按钮旁，约120行）：
- 加 "PC View" 开关按钮设置 `settings.showOrbbecPointCloud`。

**C5. 声明并调用新 GUI 方法 - `src/SplatEditor.h`**（在 `makeOrbbecPreviewGUI` 附近，472行声明 `makeOrbbecPointCloudGUI()`）和 **`src/SplatEditor.cpp`**（约1731行调用点）。

### 改动文件
| 文件 | 改动 |
|---|---|
| `src/render/points.cu` | 给 `kernel_hqs_color` 加 `ColorCorrection` 参数 + `applyColorCorrection` 调用 |
| `src/SplatEditor_draw.h` | 向 `kernel_hqs_color` launch 传入 `cc`（selected 门控） |
| `src/gui/colorCorrection.h` | 给 Apply 加 `SNPoints` 分支（仅重置，不假 bake） |
| `src/gui/orbbec_preview.h` | 修复 BGRA 上传 (GL_BGRA)；加 `makeOrbbecPointCloudGUI()` 面板 |
| `src/SplatEditor_render.h` | 加专用点云 RenderTarget render+blit 块 |
| `src/SplatEditor.h` | 新面板状态字段 + `makeOrbbecPointCloudGUI()` 声明 |
| `src/SplatEditor.cpp` | 调用 `makeOrbbecPointCloudGUI()` |
| `src/gui/menubar.h` | "PC View" 开关按钮 |

### 验证
- 通过现有 CMake target 构建（我会运行构建命令）。
- 删除 `./cubins/points.cu.cubin` 以便内核签名变更重新编译。
- 手动检查（用户）：打开 Orbbec 面板，连接设备，开启 "Generate RGB point cloud"，打开新 "Orbbec Point Cloud" 面板 → 实时点云应可见且自动框定；拖拽轨道、滚轮缩放。
- 打开 Color Correction，选中实时点云节点 → 滑块应同时在面板和主视口明显影响点云；Revert 重置。
- 2D RGB 预览对 BGRA 设备现在应显示正确颜色（红色物体看起来是红色）。

### 非目标 / 范围外
- 不改硬件相机参数（独立的 `CameraParams` 面板已处理 exposure/gain/WB）。
- 不给 progressive 渲染器加颜色校正（实时流不用它）。
- 不为点云面板加独立 GLFW 窗口 / VR 集成。
- 不把面板相机状态持久化到磁盘。