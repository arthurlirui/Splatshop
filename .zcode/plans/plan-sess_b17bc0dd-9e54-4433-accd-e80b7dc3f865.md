## 深度图去噪前后对比模块

### 需求
- 一个开关切换"去噪前/去噪后"对比模式
- 对比模式下左右并排显示原始深度图和去噪后深度图
- 参数调整立即反映在深度图上（无需重启流，下一帧即生效）

### 设计

#### 核心思路
在 `captureLoop()` 中，**滤波器应用之前**快照原始深度帧到第二个 `RGBDFrame`，与滤波后的帧一起发布。预览面板增加对比模式，并排显示两个深度图。

#### "参数立即生效"的实现
当前滤波器在 `start()` 中创建，参数变化需要 `stop()/start()` 重启流。为实现参数即时生效，新增 `applyDepthFilterParams()` 方法：在线更新已有滤波器的参数（调用 `setFilterParams`/`setWeight`/`setDiffScale`），无需重启流。同时用开关控制每个滤波器的 `enable(bool)` 而非重建——这样参数变化下一帧即生效。

#### 改动详情

**1. `OrbbecCapture.h`** — 新增公开方法：
```cpp
std::shared_ptr<RGBDFrame> getLatestRawFrame();  // 原始（未滤波）帧
void applyDepthFilterParams(const CameraParams& p);  // 在线更新滤波器参数
```

**2. `OrbbecCapture.cpp`** — 三处改动：

*a) Impl 新增成员：*
```cpp
std::shared_ptr<RGBDFrame> latestRawFrame;  // 原始帧快照
```

*b) `captureLoop()` — 滤波前快照原始深度：*
在提取 `colorFrame`/`depthFrame` 之后、滤波链之前，将原始 depth 数据拷贝到 `rawFrame`：
```cpp
// Snapshot raw (pre-filter) depth for before/after comparison.
auto rawFrame = std::make_shared<RGBDFrame>();
if (depthFrame) {
    auto df = depthFrame->as<ob::DepthFrame>();
    uint32_t sz = df->getDataSize();
    auto buf = std::make_shared<Buffer>((int64_t)sz);
    std::memcpy(buf->data, df->getData(), sz);
    rawFrame->depthData   = buf;
    rawFrame->depthWidth  = (int)df->getWidth();
    rawFrame->depthHeight = (int)df->getHeight();
    rawFrame->depthFormat = formatToInt(df->getFormat());
    rawFrame->depthScale  = df->getValueScale();
    rawFrame->frameIndex  = df->getIndex();
}
```
然后在发布块中同时发布：
```cpp
impl->latestFrame = frame;
impl->latestRawFrame = rawFrame;
```

*c) `applyDepthFilterParams()` — 在线更新滤波器参数：*
```cpp
void OrbbecCapture::applyDepthFilterParams(const CameraParams& p) {
    impl->params = p;
    // Enable/disable existing filters without recreating them
    if (impl->hwNoiseRemoval) impl->hwNoiseRemoval->enable(p.hwNoiseRemovalEnabled);
    if (impl->noiseRemoval)   impl->noiseRemoval->enable(p.denoiseFilterEnabled);
    if (impl->spatialFilter)  impl->spatialFilter->enable(p.spatialFilterEnabled);
    if (impl->temporalFilter) impl->temporalFilter->enable(p.temporalFilterEnabled);
    // Update params on enabled filters
    if (impl->noiseRemoval && p.denoiseFilterEnabled) {
        OBNoiseRemovalFilterParams np{};
        np.max_size  = (uint16_t)(p.denoiseMaxSize  >= 0 ? p.denoiseMaxSize  : 80);
        np.disp_diff = (uint16_t)(p.denoiseDispDiff >= 0 ? p.denoiseDispDiff : 256);
        try { impl->noiseRemoval->setFilterParams(np); } catch (...) {}
    }
    if (impl->spatialFilter && p.spatialFilterEnabled) {
        OBSpatialAdvancedFilterParams sp{};
        sp.alpha = p.spatialAlpha >= 0.f ? p.spatialAlpha : 0.1f;
        sp.radius = (uint16_t)(p.spatialRadius >= 0 ? p.spatialRadius : 1);
        sp.magnitude = (uint8_t)(p.spatialMagnitude >= 0 ? p.spatialMagnitude : 2);
        sp.disp_diff = (uint16_t)(p.spatialDispDiff >= 0 ? p.spatialDispDiff : 160);
        try { impl->spatialFilter->setFilterParams(sp); } catch (...) {}
    }
    if (impl->temporalFilter && p.temporalFilterEnabled) {
        if (p.temporalWeight >= 0.f) try { impl->temporalFilter->setWeight(p.temporalWeight); } catch (...) {}
        if (p.temporalDiffScale >= 0.f) try { impl->temporalFilter->setDiffScale(p.temporalDiffScale); } catch (...) {}
    }
}
```

注意：`start()` 中所有四个滤波器都会被创建（无论 enable 与否），但初始 `enable` 状态由参数决定。这样 `applyDepthFilterParams` 只需切换 `enable` 而无需重建滤波器。修改 `start()` 中的创建逻辑：始终创建滤波器，但用 `enable(bool)` 控制开关。

**3. `SplatEditor.h`** — 新增预览成员：
```cpp
GLuint orbbecTexDepthRaw = 0;
int orbbecTexDepthRawW = 0, orbbecTexDepthRawH = 0;
vector<uint8_t> orbbecDepthScratchRaw;  // 原始深度图 colormap 输出
```
Settings 新增：
```cpp
bool orbbecDenoiseCompare = false;  // 去噪前后对比模式开关
```

**4. `src/gui/orbbec.h`** — Depth Denoising 区改进：
- 每个滤波器的 checkbox 改为**即时调用** `cap->applyDepthFilterParams(editor->orbbecParams)`
- 参数滑块/输入框变化时也即时调用
- 不再需要 "Apply (restart)" 按钮来使滤波器参数生效（流配置仍需要重启）
- 新增 "Compare Before/After" checkbox，绑定到 `settings.orbbecDenoiseCompare`

**5. `src/gui/orbbec_preview.h`** — 对比模式布局：
- 当 `orbbecDenoiseCompare` 为 true 时，Depth 区域分为左右两个子窗口：
  - 左："Depth (Raw)" — 使用 `getLatestRawFrame()` 的 depth 数据
  - 右："Depth (Denoised)" — 使用 `getLatestFrame()` 的 depth 数据
- 两个深度图使用相同的 colormap LUT 和 maxMm 参数，确保公平对比
- 需要第二个纹理 (`orbbecTexDepthRaw`) 和第二个 scratch buffer (`orbbecDepthScratchRaw`)
- 当 `orbbecDenoiseCompare` 为 false 时，保持现有的单深度图布局

### 涉及文件

| 文件 | 改动 |
|------|------|
| `src/camera/OrbbecCapture.h` | 新增 `getLatestRawFrame()` + `applyDepthFilterParams()` 声明 |
| `src/camera/OrbbecCapture.cpp` | Impl 新增 `latestRawFrame`；captureLoop 快照原始深度；`start()` 始终创建滤波器；新增 `applyDepthFilterParams()`；新增 `getLatestRawFrame()` |
| `src/SplatEditor.h` | 新增 `orbbecTexDepthRaw`/`orbbecDepthScratchRaw` + `orbbecDenoiseCompare` 设置 |
| `src/gui/orbbec.h` | Depth Denoising 区参数变化即时调用 `applyDepthFilterParams`；新增对比开关 |
| `src/gui/orbbec_preview.h` | 对比模式下并排显示原始/去噪深度图 |

### 验证
构建通过后运行 `SplatEditor.exe`，连接相机启动流，打开 Depth Denoising 面板，勾选 "Compare Before/After"，开启任一滤波器并调节参数，观察左右两个深度图的实时差异。