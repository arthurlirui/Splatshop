# Splatshop 近期工作技术报告

> **版本日期**: 2026-08-07  
> **基于论文**: *"Splatshop: Efficiently Editing Large Gaussian Splat Models"* — Schütz, Peters, Hahlbohm, Eisemann, Magnor, Wimmer, CGF 2025

---

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构总览](#2-系统架构总览)
3. [近期新增功能详解](#3-近期新增功能详解)
   - [3.1 渐进式点云渲染器](#31-渐进式点云渲染器)
   - [3.2 4DGS 动态场景支持](#32-4dgs-动态场景支持)
   - [3.3 远程控制 API](#33-远程控制-api)
   - [3.4 运动控制与骨骼蒙皮](#34-运动控制与骨骼蒙皮)
   - [3.5 VR 远程观看](#35-vr-远程观看)
   - [3.6 OrbbecSDK 集成](#36-orbbecsdk-集成)
   - [3.7 场景 Splats CRUD API](#37-场景-splats-crud-api)
4. [构建系统](#4-构建系统)
5. [已知限制与未来方向](#5-已知限制与未来方向)
6. [附录](#6-附录)

---

## 1. 项目概述

### 1.1 项目定位

**Splatshop** 是一个高效的大型 **3D Gaussian Splatting (3DGS)** 模型编辑器，旨在对高斯泼溅模型进行编辑、清理、组装和场景创作。该项目发表于 Computer Graphics Forum 2025，目前处于学术原型 (Alpha) 阶段。

### 1.2 核心能力指标

| 指标 | 数值 |
|------|------|
| 桌面端最多 splats | **1 亿** |
| VR 模式最多 splats (RTX 4090) | **~1000 万** |
| 运行时语言 | 纯 C++/CUDA (无 Python 运行依赖) |
| CUDA 编译方式 | Driver API (NVRTC, JIT 编译 + 热重载) |

### 1.3 技术栈

| 层级 | 技术选型 |
|------|----------|
| 语言 | C++23, CUDA (NVRTC JIT) |
| 图形 API | OpenGL 4.5 (桌面) + OpenVR (VR) |
| 深度学习 | **LibTorch 2.6+** (4DGS 变形网络) ⭐ |
| GUI | Dear ImGui + ImPlot + ImGuizmo |
| 数学库 | GLM + 自研四元数运算 |
| 构建系统 | CMake 3.22+ |
| 平台 | Windows (VS 2022) / Linux (GCC 14) |

### 1.4 已有功能概览

| 类别 | 功能 |
|------|------|
| 基础编辑 | 撤销/重做 (Ctrl+Z/Y) |
| VR | VR 查看和编辑 (OpenVR/SteamVR) |
| 选择 | 笔刷选择、球形选择、矩形选择 |
| 变换 | 平移 (t)、旋转 (r)、缩放 (s) |
| 绘制 | 笔刷上色 (b) |
| 图层 | 复制到新图层 (Ctrl+D)、合并图层 (Ctrl+E) |
| 对齐 | 3 点对齐工具 |
| 高级 | 透视校正高斯、渐进式 SH 评估 |
| 资产 | 资产库 (Asset Library)、图层管理 |

### 1.5 近期新增功能 ⭐

以下功能为 **2025 年底至 2026 年第三季度** 新增：

| 序号 | 功能 | 对应提交 |
|------|------|----------|
| 1 | 渐进式点云渲染器 (Skye 算法) | `3aea527` |
| 2 | 4DGS 动态场景加载与播放 | `d02db21` |
| 3 | 远程控制 HTTP API (FastAPI) | `48b3427` |
| 4 | 对象运动控制 + 骨骼蒙皮 | `4781987` |
| 5 | VR 远程观看 (WebXR 客户端) | plan `sess_659002cd` |
| 6 | OrbbecSDK RGBD 相机集成 | plan `sess_b17bc0dd` |
| 7 | 场景 Splats 创建/加载/删除/着色 | `9fe9bd8` |

---

## 2. 系统架构总览

### 2.1 SplatEditor 主类结构

`SplatEditor` 是应用程序的中央控制枢纽（struct, ~528 行），通过静态单例 `SplatEditor::instance` 访问。

```
SplatEditor
├── Scene scene                          — 场景图 (root → world / vr)
├── CudaModularProgram* (8 个)          — CUDA 可执行模块
│   ├── prog_gaussians_rendering        — 3DGS 泼溅渲染
│   ├── prog_gaussians_editing          — 笔刷编辑
│   ├── prog_points                     — HQS 点云渲染
│   ├── prog_progressive_points         — 渐进式点云 ⭐
│   ├── prog_triangles                  — 三角网格
│   ├── prog_lines                      — 线段渲染
│   ├── prog_helpers                    — 辅助内核
│   └── prog_skinning                   — 骨骼蒙皮 ⭐
├── OpenVRHelper* ovr                   — OpenVR/SteamVR 集成
├── RemoteStereoState remoteStereo      — 远程 VR 状态 ⭐
├── motion::Timeline timeline           — 关键帧时间线 ⭐
├── motion::RiggedHumanController       — 骨骼动画控制器 ⭐
├── vector<Action> history              — 撤销栈
├── AssetLibrary assetLibrary           — 资产库
├── Settings / State                    — 设置与运行时状态
└── CUDA 资源                           — 多流、事件、虚拟内存
```

### 2.2 渲染管线 (draw 流程)

每帧在 `SplatEditor::draw()` 中执行的步骤：

```
Step 0: 骨骼蒙皮调度
    rigController.dispatchSkinning(scene)  — SNRiggedSplats 节点

Step 1: 4DGS 变形 ⭐
    for each SN4DGSSplats:
        → node->deform(t, mainstream)       // LibTorch 正向传播
        → node->swapToDeformed()            // 交换指针到变形输出
        → 记录 deformEvent                  // 供后续流等待

Step 2: 收集可渲染节点
    scene.forEach<SNPoints>
    scene.forEach<SNTriangles>

Step 3: 逐目标并行渲染 (每个 RenderTarget 独立 CUDA 流)
    3a. cuStreamWaitEvent(deformEvent)      // 等待变形完成
    3b. 清空帧缓冲
    3c. 点云渲染:
        ├── HQS 路径: atomicMin 深度 + atomicAdd 颜色
        └── Progressive 路径 ⭐: 4 阶段流水线
    3d. 3DGS 瓦片渲染:
        预过滤 → 排序 → 瓦片分配 → 正向渲染 → EDL 后处理
    3e. 三角形渲染

Step 4: 恢复 4DGS 规范态
    node->swapToCanonical()  — 恢复 dmng.data 原始指针
```

**ConcurrentTarget** (每渲染目标缓存):

| 资源 | 说明 |
|------|------|
| `mainstream`, `sidestream` | 专用 CUDA 流 |
| `fb_depth`, `fb_color` | 深度/颜色缓冲 (虚拟内存) |
| `tiles`, `splatIndices` | 瓦片列表 + splat 索引 |
| `staging`, `ordering` | 暂存 + 排序缓冲 |
| `numVisibleSplats/Fragments` | 主机可读计数器 |

**ProgressiveTarget** (每渲染目标渐进式状态) ⭐:

| 资源 | 说明 |
|------|------|
| `indexImage` | 每像素 uint32 (0xFFFFFFFF = 空) |
| `reprojectBuffer` | ProgressiveVertex[] 局部空间可见点 |
| `indirect` | ProgressiveIndirectCommand 原子计数 |
| `hostIndirect` | 固定主机镜像 (零拷贝读取) |
| `evFillStart/End` | CUDA 事件 (自适应预算计时) |
| `pointsPerMs` | 吞吐量估算 |

### 2.3 更新管线 (update 流程)

```
① cuCtxSynchronize()
② 窗口标题 FPS 更新
③ TWEEN::update()                        — Tween 动画推进
④ timeline.update(scene, dt)             — 关键帧时间线 ⭐
⑤ 4DGS 自动回放 (无轨道时)              ⭐
⑥ rigController.update(scene)            — 骨骼姿态更新 ⭐
⑦ ImGui 新帧
⑧ Runtime::timings.newFrame()            — 滑动窗口计时重置
⑨ scene.updateTransformations()          — 变换传播
⑩ 重置每帧计数器
⑪ 构建 CommonLaunchArgs
⑫ 零初始化设备状态
⑬ uploadSplats() / uploadPoints()       — 异步加载上传
⑭ Orbbec 实时上传 ⭐                     — SNOrbbec 帧就绪
⑮ VR 姿态更新 (OpenVR)
⑯ RemoteStereo 视图设置 ⭐               — 3 种姿态空间
⑰ inputHandling()                        — 输入分发
⑱ 异步线段上传
```

### 2.4 CUDA 模块化程序设计

文件: `include/CudaModularProgram.h`

基于 **NVRTC** (编译) + **nvJitLink** (链接) 的 JIT 编译流水线。

#### CudaModule — 单个 .cu 源文件

| 字段 | 说明 |
|------|------|
| `path` / `name` | 源文件路径与名称 |
| `ltoirSize` / `ltoir` | LTO 中间表示 |

**编译选项**:
- `--gpu-architecture=compute_XY` (自动检测)
- `--use_fast_math`, `--extra-device-vectorization`
- `--std=c++20`, `--relocatable-device-code=true`
- `--dlink-time-opt`, `--split-compile=0` (全核并行)

#### CudaModularProgram — 完整链接程序

| 能力 | 说明 |
|------|------|
| 链接 | nvJitLink: `-dlto`, `-arch=sm_XY`, `-O3` |
| 内核枚举 | `cuModuleEnumerateFunctions` → `kernels` 映射 |
| 启动 | `launch()` / `launchCooperative()` / `fromCubin()` |
| 热重载 | `monitorFile` 监视源文件变化 → 自动重编译+重链接 |
| 计时 | `EventPool` 回收 CUevent, 异步解析 |

### 2.5 场景图节点层次

```
SceneNode (基类)
├── SNSplats                 — 3DGS 高斯泼溅 (核心图元)
│   ├── SNRiggedSplats      — 骨骼蒙皮高斯 ⭐
│   └── SN4DGSSplats        — 4DGS 动态高斯 ⭐
├── SNPoints                 — 点云
│   └── SNOrbbec             — Orbbec 实时 RGBD 点云 ⭐
├── SNTriangles              — 三角形网格
├── SNLines                  — 线段 (调试)
└── ImguiNode                — VR GUI 覆盖面板
```

**GaussianData** (SNSplats 包装):

| 缓冲区 | 类型 | 说明 |
|--------|------|------|
| `position[]` | vec3 | 3D 位置 |
| `scale[]` | vec3 | 各向异性缩放 |
| `quaternion[]` | vec4 | 旋转四元数 |
| `color[]` | uint16 | RGBA 打包 (含 opacity) |
| `SH[]` | float | 球谐系数 (当前未使用) |
| `flags[]` | uint8 | 每 splat 标志位 |

**PointData** (SNPoints 包装):

| 缓冲区 | 类型 | 说明 |
|--------|------|------|
| `position[]` | vec3 | 3D 位置 |
| `color[]` | uint32 | RGBA 打包 |

### 2.6 全局源文件发现

CMakeLists.txt 使用 `file(GLOB ... CONFIGURE_DEPENDS)` 自动发现 12 个子目录:

```
src/*.*
src/actions/*.*    src/GPUPrefixSums/*.*    src/GPUSorting/*.*
src/gui/*.*        src/motion/*.* ⭐         src/render/*.*
src/remote/*.* ⭐   src/scene/*.*            src/update/*.*
src/loader/*.* ⭐   src/writer/*.*           src/camera/*.*
include/*.*
src/win32/*.* 或 src/unix/*.*
```

> 添加或删除 `.cu/.cpp/.h` 文件**无需修改 CMakeLists.txt**，下次 cmake 配置自动生效。

### 2.7 FPS 与性能计时

| 组件 | 说明 |
|------|------|
| `GLRenderer::fps` | 主循环 FPS (300ms 平滑窗口) |
| `Runtime::timings` | 50 帧滑动窗口、按标签 (label) 聚合 |
| 主机计时 | `now()` + `cuCtxSynchronize()` 前后测量 |
| 设备计时 | CUDA 事件 `cuEventElapsedTime` |
| Progressive 自适应 | `evFillStart/End` 测量重投影+填充耗时 |
| `Runtime::measureTimings` | 全局开关 |

---

## 3. 近期新增功能详解

### 3.1 渐进式点云渲染器

> **提交**: `3aea527` — *feat: Add progressive point cloud renderer and point cloud loading support*  
> **影响文件**: 14 个 (新增 7 个源文件)  
> **计划文档**: `.zcode/plans/plan-sess_7da06295.md`

#### 3.1.1 理论基础

基于 **Schütz et al. (2019)**: *"Progressive Real-Time Rendering of One Billion Points Without Hierarchical Acceleration Structures"* (TU Wien)。参考实现: Skye 项目 (`E:\Code\Skye`)。

**核心技术: 素数同余置换 (Prime-Congruence Permutation)**

> 对于任意素数 `p ≡ 3 (mod 4)`，映射 `f(n) = n² mod p` 是 `[0, p)` 上的**二次双射 (bijection)**，且 `f(f(n))` 同样为双射。

对点云中每个点索引 `i`，计算目标槽位:

```
slot = permuteI(permuteI(i, prime), prime)
```

其中 `permuteI(n, p) = (n * n) % p`（使用 uint64 防溢出）。

**核心性质**: 在置换后的缓冲区中，从偏移 `fillOffset` 开始**连续顺序绘制**，相当于对整个点云进行**均匀随机采样**。无需层次加速结构，每帧开销仅受填充预算限制。

#### 3.1.2 数据结构

**`ProgressivePointData`** (设备端 POD):

```cpp
struct ProgressivePointData {
    uint32_t count = 0;                          // 总点数
    uint32_t numBuffers = 0;                     // 填充的块缓冲区数
    uint64_t maxPointsPerBuffer = 134'000'000;   // 单块最大点数
    vec3*     position[8] = {};                  // 每块位置指针
    uint32_t* color[8]    = {};                  // 每块颜色指针
    uint64_t prime = 0;                          // 置换素数
    uint32_t fillOffset = 0;                     // 实时填充游标
    bool ready = false;                          // distribute 完成标志
};
```

**`ProgressiveVertex`** (20 字节, 重投影缓冲条目):

```cpp
struct ProgressiveVertex {
    float    ux, uy, uz;     // 局部空间位置
    uint32_t color;          // RGBA
    uint32_t index;          // 全局点 ID
};
```

> 位置存储在**局部空间**：每帧重投影时重新应用节点当前世界变换，保证对象移动后重投影仍然正确。

**`ProgressiveIndirectCommand`** (间接绘制命令):

```cpp
struct ProgressiveIndirectCommand {
    uint32_t count;        // ★ 实际使用
    uint32_t primCount;    // 仅与 Skye 兼容
    uint32_t firstIndex;
    uint32_t baseVertex;
    uint32_t baseInstance;
};
```

**`ProgressivePointCloud`** (主机端):

```cpp
struct ProgressivePointCloud {
    shared_ptr<CudaVirtualMemory> vm_position[8];
    shared_ptr<CudaVirtualMemory> vm_color[8];
    uint32_t numBuffers;      // 填充块数
    uint64_t pointsPerBuffer;
    uint64_t count;           // 总点数
    uint64_t prime;           // 置换素数
    ProgressivePointData data{};

    void init(uint64_t numPoints);  // 计算素数 → 虚拟内存分配
    static uint64_t largestPrimeCongruent3mod4(uint64_t n);
};
```

> 限制: 最多 8 × 134M ≈ **10.7 亿点**

#### 3.1.3 五阶段流水线

所有内核位于 `src/render/progressive_points.cu`。

```
┌─────────────────────────────────────────────────────────────────┐
│  加载时 (一次性)                                                 │
│                                                                  │
│  Stage 0: kernel_progressive_distribute                         │
│    每个规范态点 → permuteI(permuteI(i), prime) → 写入分块缓冲    │
│    完成后 ProgressivePointData.ready = true                     │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  每帧                                                            │
│                                                                  │
│  Stage 1: kernel_progressive_clear_index                        │
│    每像素 indexImage = 0xFFFFFFFF ("无点")                       │
│                                                                  │
│  Stage 2: kernel_progressive_reproject                          │
│    对上一帧 reprojectBuffer 每个点:                              │
│    → 重新应用当前 MVP 矩阵                                      │
│    → atomicMin 写入 framebuffer ((depth<<32)|color)             │
│    → 写入 indexImage[pixelID] (最前点 ID)                       │
│                                                                  │
│  Stage 3: kernel_progressive_fill                               │
│    对 budget 个线程: 读取置换缓冲 (fillOffset+i)%count 位置      │
│    → 投影 → 写入 framebuffer + indexImage                       │
│    主机推进 fillOffset                                           │
│                                                                  │
│  Stage 4: kernel_progressive_reset_indirect (单线程)            │
│    清零 indirect->count                                          │
│                                                                  │
│  Stage 5: kernel_progressive_create_vbo                         │
│    对每像素: 读 indexImage 全局点 ID → atomicAdd 压缩槽位        │
│    → 查渐进式云取局部位置+颜色                                  │
│    → 写入 reprojectBuffer[slot]                                 │
│    最终计数 = 下一帧 indirect->count                            │
└─────────────────────────────────────────────────────────────────┘
```

**关键辅助函数**:

| 函数 | 说明 |
|------|------|
| `permuteI(n, prime)` | 二次双射 (uint64 防溢出) |
| `fetchProgressivePoint(pc, idx, outPos, outColor)` | 全局索引 → 块路由 + 读取 |
| `storeProgressivePoint(pc, idx, pos, color)` | 全局索引 → 块路由 + 写入 |
| `writeProgressivePoint(...)` | 共享帧缓冲 + 索引图写入逻辑 |

#### 3.1.4 自适应帧预算

**`kernel_progressive_compute_fill`** — Skye 的 `compute_fill_fixed.cs` 移植:

```
输入:
  • 重投影耗时 (ms, CUDA 事件)
  • 填充耗时 (ms)
  • 目标帧时间 (ms)
  • 吞吐量估算 (points/ms)

计算:
  remaining = (targetMs - elapsedMs) × pointsPerMs
  钳制到 [minFill, numPoints]

自调节: 重投影+固定填充便宜 → 本帧多花点；昂贵 → 少画点
```

#### 3.1.5 加载器

**`BINPointCloudLoader`** (`src/loader/BINPointCloudLoader.h`):

| 特性 | 说明 |
|------|------|
| 格式 | Skye 快速路径 `.bin` — 16 字节/点 `{x,y,z,r,g,b,a}` LE |
| 流式 | 分离 `jthread` 流加载，500K 点/批 |
| 转换 | 交叉格式 → Splatshop 的分离 position/color 布局 |

**`PointCloudPlyLoader`** (`src/loader/PointCloudPlyLoader.h`):

| 特性 | 说明 |
|------|------|
| 格式 | 通用点云 PLY (区别于 GSPlyLoader 的高斯泼溅 PLY) |
| 检测 | 通过 `scale_0`/`rot_0`/`f_dc_0` 属性拒绝高斯泼溅 PLY |
| 支持 | ASCII + binary_little_endian, uint8 + uint16 颜色 |
| 防御 | `try/catch` 跳过损坏行 (不调用 `std::terminate`) |

#### 3.1.6 GUI 面板

**文件**: `src/gui/pointcloud.h`

| 控件 | 说明 |
|------|------|
| 渲染器单选 | HQS ↔ Progressive |
| 填充预算 | 滑块 10K–30M |
| 点大小 | 可调 |
| 自适应开关 | + 目标帧时间 |
| 重置按钮 | 重置渐进式状态 |
| 统计 | 加载/上传点数、包围盒、VRAM、置换缓冲详情 |
| 加载对话框 | LAS / BIN / PLY 三格式 |

---

### 3.2 4DGS 动态场景支持

> **提交**: `d02db21` — *feat: Add 4DGS dynamic scene support*  
> **影响文件**: 10 个 (新增 3 个源文件 + CMake 模块)  
> **核心依赖**: LibTorch 2.6+

#### 3.2.1 理论基础

Splatshop 支持 [hustvl/4DGaussians](https://github.com/hustvl/4DGaussians) 训练的 **4D Gaussian Splatting** 动态场景。

**4DGS 将动态场景表示为两部分:**

```
① 规范态高斯 (Canonical Gaussians)
   标准 .ply 文件 — 静止姿态的 3DGS

② 变形场 (Deformation Field)
   HexPlane 六平面网格 + MLP 多层感知机
   对每个时间步 t ∈ [0, 1] 将规范态高斯位移到动态姿态
```

**变形网络 I/O:**

| 输入 | 输出 |
|------|------|
| 规范态 position | 变形后 position |
| 规范态 scale | 变形后 scale |
| 规范态 quaternion | 变形后 rotation |
| 中性 opacity | 变形后 [opacity] (Splatshop 不消费) |
| 时间 t | — |

#### 3.2.2 场景节点设计

**文件**: `src/scene/SN4DGSSplats.h` (179 行) / `.cpp`

```
SN4DGSSplats : 继承 SNSplats
│
├── 规范态高斯: dmng.data (继承, GSPlyLoader 加载)
│
├── Deform4DGSConfig               — 变形配置
│   ├── shDegree = 3               — SH 阶数
│   ├── nGaussians = 0             — 高斯总数
│   ├── formatVersion = "1.0"      — 格式版本
│   └── loadFromFile(jsonPath)     — 从 config.json 解析
│
├── Deform4DGSBuffer               — 设备端变形输出缓冲
│   ├── vm_deformedPosition/Scale/Rotation
│   ├── cptr_deformedPosition/Scale/Rotation
│   ├── allocDevice(numSplats)
│   └── freeDevice()
│
├── 变形模块 (TorchScript)
│   ├── #ifdef SPLATSHOP_HAS_LIBTORCH
│   │     unique_ptr<torch::jit::script::Module> deformModule
│   ├── #else
│   │     unique_ptr<void, void(*)(void*)> deformModule  // 桩
│   └── #endif
│
├── 运行时状态
│   ├── currentTime = 0.0f         — 归一化时间 [0,1]
│   ├── needsRecompute = true      — atomic<bool> 编辑后强制重算
│   ├── deformationEnabled = true  — 变形开关
│   ├── canonicalPosition          — 保存的规范态指针
│   ├── canonicalScale
│   └── canonicalQuaternion
│
└── 核心方法
    ├── deform(t, stream)          — 变形管线入口
    ├── swapToDeformed()           — 交换指针到变形输出
    ├── swapToCanonical()          — 恢复规范态指针
    ├── getDeformedPositions()
    ├── getDeformedScales()
    └── getDeformedRotations()
```

#### 3.2.3 LibTorch 集成架构

**依赖隔离策略:**

- 仅 **`SN4DGSSplats.cpp`** 包含 `<torch/script.h>` — 唯一引入 LibTorch 头文件的翻译单元
- 所有 4DGS 代码由 `#ifdef SPLATSHOP_HAS_LIBTORCH` 守卫
- 若无 LibTorch: 桩实现 (节点变直通，规范态 = 变形态，打印警告)

**正向传播管线 (`runDeformation`):**

```
① 从 dmng.data 获取规范态 CUDA 设备指针
② torch::from_blob(ptr, sizes, torch::kCUDA)  — 零拷贝包装
   (直接指向 Splatshop 自有 CUDA 缓冲，不复制)
③ 组装输入张量: positions, scales, quaternions, opacity, time
④ deformModule->forward(inputs).toTuple()  — TorchScript 执行
⑤ cuMemcpyDtoDAsync  — 输出复制到 Deform4DGSBuffer
   (所有操作在同一 CUDA 流上异步完成)
```

#### 3.2.4 指针交换与流同步

**swapToDeformed():**

```
保存: canonicalPosition = dmng.data.position
回填: dmng.data.position = deformedBuffer 的指针
→ 渲染内核现在读取变形数据 (无需修改内核代码)
```

**swapToCanonical():**

```
恢复: dmng.data.position = canonicalPosition
→ 渲染完成后数据回到规范态
```

**流同步:**

```
deform() 在 mainstream 上排队 cuMemcpyDtoDAsync
→ 记录 CUDA 事件 deformEvent
→ 每个渲染目标的流在执行 staging 前: cuStreamWaitEvent(deformEvent)
→ 保证变形完成后再读取
```

#### 3.2.5 增量计算优化

变形仅在以下情况重新计算:

| 条件 | 检测方式 |
|------|----------|
| 时间步进 | `|currentTime - prevTime| <= 1e-6` 则跳过 |
| 编辑规范态后 | `needsRecompute = true` |
| 首次加载 | 初始 `needsRecompute = true` |

`deform()` 入口检查: 若时间和 dirty 均未变化 → **直接返回 (无操作)**。

#### 3.2.6 编辑工作流

```
① 暂停时间线 → 变形网络停在当前帧
② 用现有工具选择/删除/绘制规范态高斯
③ 自动设置 needsRecompute = true
④ 恢复播放 → 下一帧重新变形，编辑反映到所有后续帧
⑤ 关闭 deformationEnabled → 临时查看/对比规范态
```

#### 3.2.7 导出工具

**`tools/export_4dgs_torchscript.py`**:

```
输入:
  • --checkpoint: hustvl/4DGaussians 训练检查点 (.pth)
  • --ply: 规范态 point_cloud.ply
  • --out: 输出目录

流程:
  ① 从检查点加载 state_dict
  ② 重建 _DeformationWrapper 模块 (HexPlane 网格平面 + MLP 头)
  ③ torch.jit.trace() → TorchScript
  ④ 输出: canonical.ply + deformation_model.pt + config.json
```

**使用:**

```bash
python tools/export_4dgs_torchscript.py \
    --checkpoint <4DGaussians>/output/dnerf/lego/chkpnt30000.pth \
    --ply <4DGaussians>/output/dnerf/lego/point_cloud/iteration_30000/point_cloud.ply \
    --out lego_4dgs
```

#### 3.2.8 使用流程

```
① 启动 Splatshop
② 拖放 canonical.ply → 加载规范态
③ 工具栏 → Motion 面板 → 4DGS Dynamic Scene
④ Import model → 选 deformation_model.pt
⑤ 播放/暂停/拖拽/调速
⑥ 时间线在 [0.0, 1.0] 间反弹循环
```

---

### 3.3 远程控制 API

> **提交**: `48b3427` → `8a56308` → `b27e963` (后续迭代)  
> **影响文件**: 15+ 个 (新增 `remote_api/` 目录 + C++ 桥)  
> **计划文档**: `.zcode/plans/plan-sess_659002cd.md` 等

#### 3.3.1 三层桥接架构

```
远程客户端 (HTTP)
      │ POST /camera/orbit, /mouse/move, ...
      ▼
FastAPI (Python, :8080)          ← remote_api/server.py (686 行)
      │ TCP JSON-RPC              ← remote_api/splat_client.py (92 行)
      ▼
RemoteControlServer (C++, :7654) ← src/remote/RemoteControlServer.cpp
      │ EventQueue (schedule())
      ▼
SplatEditor 主线程 (render loop)
```

> 控制通道和视频通道**刻意解耦**: 控制 = 小包高频姿态 + 低频命令; 视频 = 独立帧流。

#### 3.3.2 C++ 端设计

```
remote::RemoteControlServer
│
├── start(port = 7654)
│   ├── 绑定 127.0.0.1:port (仅 loopback, 安全)
│   ├── 生成分离的监听线程
│   └── 每个连接生成分离的处理线程 (支持并发客户端)
│
├── 协议: 换行分隔 JSON-RPC
│   ├── 请求: {"id": <int>, "cmd": "<name>", "args": {...}}
│   ├── 响应: {"id": <int>, "ok": true/false, "data": {...}}
│   └── 一行一个 JSON, UTF-8, \n 结束
│
└── 主线程调度 (核心模式)
    ├── std::promise<json> / future<json> 对
    ├── schedule(lambda) → EventQueue 入队
    ├── 渲染循环每帧 process() 排空
    ├── 套接字线程阻塞 fut.wait_for(10s)
    └── 响应在一个帧内返回 (~16ms)
```

**实现了 27 个命令处理器。**

#### 3.3.3 Python 端设计

| 文件 | 行数 | 角色 |
|------|------|------|
| `server.py` | 686 | FastAPI 应用, CORS 启用, 可选 Token 认证, `/test` 测试页 |
| `splat_client.py` | 92 | TCP JSON-RPC 客户端 (短生命周期, 线程安全) |
| `models.py` | 241 | Pydantic 请求/响应模型 (18 个模式) |
| `config.py` | 30 | 6 个环境变量配置 |
| `keymap.py` | 60 | GLFW 键名 ↔ 键码映射 |

#### 3.3.4 API 命令矩阵

| 类别 | 命令 | 说明 |
|------|------|------|
| **健康** | `health` | FPS, 帧计数, 窗口大小 |
| **相机** | `camera.orbit` | yaw/pitch/radius/target |
| | `camera.pan`, `camera.zoom` | 绝对 + 增量模式 |
| | `camera.pose.set/get` | 设置/获取姿态 |
| | `camera.focus` | 聚焦包围盒 |
| **VR** | `camera.vr.enter/exit` | 进出 VIEWMODE_REMOTE_STEREO |
| | `camera.vr.pose` | 高频姿态包 (3 种姿态空间) |
| **鼠标** | `mouse.move` | 绝对/相对移动 |
| | `mouse.button` | 按钮按下/释放 |
| | `mouse.scroll` | 滚轮 |
| | `mouse.event` | 复合事件 (单次往返拖拽) |
| **键盘** | `keyboard.key` | GLFW 键名注入 |
| | `keyboard.press` | 按下-保持-释放 |
| | `keyboard.sequence` | 打字序列 |
| **场景** | `scene.nodes` | 枚举场景树 |
| **运动** | `motion.node.{id}/transform` | 获取/设置变换 |
| | `motion.node.{id}/translate` | 平移 |
| | `motion.node.{id}/rotate` | 旋转 |
| | `motion.node.{id}/scale` | 缩放 |
| | `motion.node.{id}/animate` | Tween 动画 (缓动 + 时长) |
| **Splats** ⭐ | `scene.splats.create` | 创建原语 (球/盒/点集) |
| | `scene.splats.load` | 加载 .ply / scene.json |
| | `scene.node.{id}` DELETE | 删除节点 |
| | `scene.splats.{id}/color` | 设置均匀颜色 |

#### 3.3.5 快速示例

```bash
# 创建 500 个红色 splats 的球体 → {"id": 15, ...}
curl -X POST http://localhost:8080/scene/splats/create \
  -H "Content-Type: application/json" \
  -d '{"type":"sphere","params":{"radius":1.5,"count":500,"color":[1,0,0,1]}}'

# 动画节点 15 平移到 [3,0,0], 1.5 秒, 缓入缓出
curl -X POST http://localhost:8080/motion/node/15/animate \
  -H "Content-Type: application/json" \
  -d '{"target":{"translation":[3,0,0]},"duration_s":1.5,"ease":"in_out"}'
```

#### 3.3.6 启动命令

```bash
# 1. 从仓库根目录运行 C++ 编辑器
./build/Release/SplatEditor.exe

# 2. 启动 Python HTTP API (端口 8080)
uvicorn remote_api.server:app --host 0.0.0.0 --port 8080

# 3. 打开浏览器测试页
# http://localhost:8080/test
```

---

### 3.4 运动控制与骨骼蒙皮

> **提交**: `4781987` — *feat(motion): Add object motion control and skeletal skinning module*  
> **影响文件**: 10 个新文件 (`src/motion/` 目录)

#### 3.4.1 文件结构

| 文件 | 用途 |
|------|------|
| `MotionTypes.h` | 共享数据结构, 自研四元数数学, 缓动函数 |
| `MotionController.h` | 单节点刚体运动接口 |
| `Timeline.h/.cpp` | 关键帧驱动动画播放器 + JSON 加载 |
| `RiggedHumanController.h/.cpp` | 骨骼蒙皮 + 面部 blendshape 控制器 |
| `ProceduralRigSource.h/.cpp` | 程序化 5 关节测试骨架生成器 |
| `skinning.cu` | LBS + blendshape 变形 CUDA 内核 |

#### 3.4.2 MotionTypes.h — 共享类型

**`TransformSample`** — 分解 T·R·S 变换

```cpp
struct TransformSample {
    vec3 translation, scale;
    vec4 quaternion;  // [x,y,z,w]
    mat4 toMatrix();
    static TransformSample fromMatrix(mat4);
};
```

**`JointPose` / `SkeletonPose`**:

```cpp
struct JointPose { vec4 rotation; vec3 translation, scale; };
struct SkeletonPose { vector<JointPose> joints; };
```

**`FaceData`** — 52 元素 ARKit 兼容 blendshape 权重数组

**自研四元数数学库** (避免 GLM 的 `gtc/quaternion` 无法干净编译标量参数):

| 函数 | 说明 |
|------|------|
| `quatIdentity()` | 单位四元数 |
| `quatMul(a, b)` | 四元数乘法 |
| `slerpQuat(a, b, t)` | 球面线性插值 |
| `quatToMat4(q)` / `quatFromMat4(m)` | 四元数 ↔ 矩阵 |
| `quatToEulerXYZ(q)` | 四元数 → 欧拉角 |
| `mix(a, b, t)` | 标量/向量线性混合 |

**`EaseMode` 枚举** — `Linear`, `EaseIn`, `EaseOut`, `EaseInOut` + `applyEase()`

#### 3.4.3 MotionController — 刚体运动

静态方法类，通过 `node->transform` 操作节点局部变换:

| 类别 | 方法 |
|------|------|
| 绝对设置 | `setTransform`, `setTranslation`, `setRotation`, `setScale` |
| 相对变异 | `translate` (累加), `rotate` (预乘), `scaleBy` |
| 获取 | `getTransform` → `TransformSample` (分解) |
| 动画 | `setTransformAnimated` — Tween 系统 + 缓动 + 时长 |

`setTransformAnimated` 插值方案:

| 分量 | 插值器 |
|------|--------|
| 平移 | `mix()` 线性 |
| 旋转 | `slerpQuat()` 球面 |
| 缩放 | `mix()` 线性 |

#### 3.4.4 Timeline — 关键帧播放器

纯 CPU 驱动:

```
内部数据:
  vector<TransformTrack>        — 每条轨道绑定 NodeID
    vector<TransformKeyframe>   — 按时间排序的关键帧

插值模式: Step / Linear / Slerp

update(scene, dt):
  推进 playhead → 每条轨道采样
  → MotionController::setTransform 写入

loadTracksFromJSON(json):
  解析节点名 (scene.find()), 插值模式, 关键帧
  四元数 [x,y,z,w] 数组, 向量 [x,y,z] 数组

支持: loop, auto-stop, start/stop/reset/scrub
```

#### 3.4.5 RiggedHumanController — 骨骼动画

**每帧流程:**

```
① setPose/setFace/setJointPose/setBlendshape
   → 写入期望姿态 + face → poseDirty = true

② update(scene):
   对每个脏节点:
     累加关节局部姿态: M_global[i] = M_global[parent] × M_local[i]
     计算蒙皮矩阵: M_skin[i] = M_global[i] × IBP[i]
        (IBP = 逆绑定姿态 Inverse Bind Pose)
     cuMemcpyHtoD → 上传蒙皮矩阵

③ dispatchSkinning(scene):
   对每个脏 SNRiggedSplats: 启动 kernel_skin_splats
   → 写入 dmng.data 缓冲
```

#### 3.4.6 ProceduralRigSource — 测试骨架

在 splat 包围盒内沿 Y 轴堆叠 5 关节骨架:

```
root → spine → chest → neck → head
```

每个 splat 按 Y 高度绑定到最近关节 (单骨骼权重 = 1.0)。用于验证蒙皮管线无需真实绑定资产。

#### 3.4.7 skinning.cu — CUDA 蒙皮内核

**`kernel_skin_splats`** — 每 splat 线性混合蒙皮 (LBS):

| 步骤 | 操作 |
|------|------|
| ① | 读静止姿态 position/scale/quaternion, boneIndices, boneWeights |
| ② | 位置: 加权混合 `M_skin[i] × restPos` |
| ③ | 旋转/缩放: 加权混合 3×3 旋缩矩阵 → 四元数 → `qDeformed = qBlend × qRest` |
| ④ | 可选 blendshape: `faceWeights` 累加 delta (position/rotation/scale) |
| ⑤ | 输出: 直接写入 `dmng.data` 缓冲 (原地变形) |

**限制:**

| 参数 | 值 |
|------|-----|
| 每 splat 最大骨骼数 | 4 (`MAX_BONES_PER_SPLAT`) |
| 每 blendshape 浮点数 | 10 (`BS_FLOATS_PER_SPLAT`) |
| blendshape 通道数 | 52 (ARKit 标准) |

---

### 3.5 VR 远程观看

> **计划文档**: `.zcode/plans/plan-sess_659002cd.md` — *VR remote browsing*
> 5 阶段实现，新增 `VIEWMODE_REMOTE_STEREO`、远程 6DOF 姿态输入、
> NVENC 视频流、WebXR 客户端

#### 3.5.1 整体架构

```
VR 头显 ──姿态 (HTTP :8080)──► remote_api (FastAPI) ──► Splatshop C++
                                                                 │
                                                         VIEWMODE_REMOTE_STEREO
                                                         立体渲染 (左/右眼)
                                                                 │
VR 头显 ◄──视频 (WebSocket :8081)── FrameStreamer ◄───────────────┘
```

#### 3.5.2 远程立体视图模式

`VIEWMODE_REMOTE_STEREO` — SplatEditor 的第三显示模式:

| 模式 | 说明 |
|------|------|
| Desktop | 桌面单视 (默认) |
| VIEWMODE_IMMERSIVE_VR | 本地 OpenVR |
| **VIEWMODE_REMOTE_STEREO** ⭐ | 远程立体, 停止本地 OpenVR |

#### 3.5.3 三种姿态空间

| 空间 | 处理方式 | 用途 |
|------|----------|------|
| `POSE_SPACE_WEBXR` | 基变换矩阵 B: WebXR 设备空间 → 应用 GL 空间 | ★ 浏览器 WebXR 主路径 |
| `POSE_SPACE_OPENVR` | 应用 `flip` 变换 (跟踪 → GL), 取逆 | 复用本地 VR 约定 |
| `POSE_SPACE_RAW_VIEW` | 使用提供的视图矩阵原样 | 已在应用空间中 |

每种模式提供: 逐眼视图矩阵 (4×4), 投影矩阵 (4×4), 渲染分辨率。

#### 3.5.4 WebXR 客户端

**文件**: `remote_api/examples/vr_webxr_client.html` (307 行)

自包含 HTML/JS 页面，利用 WebXR Device API (`immersive-vr` 会话)。

**每 XR 帧:**

```
① frame.getViewerPose() → 每眼 view/proj 矩阵
② POST 即发即忘 fetch 到 /vr/pose (pose_space: "webxr")
   传递逐眼 view/proj/VP 的 flat 16 浮点
③ 通过 WebSocket :8081 接收 SBS (Side-By-Side) JPEG 帧
   二进制头: 20 字节 (magic 0x56524653 + 尺寸 + 编码 + 长度)
④ 全屏四边形着色器 (UV offset/scale) 分割左右半图
   → 渲染到 XRWebGLLayer
```

**要求**: HTTPS 或 localhost (WebXR 安全上下文)  
**参数**: `?host=` + `?token=`

#### 3.5.5 视频编码

| 编码 | 说明 |
|------|------|
| JPEG (默认) | 无外部依赖, 适合局域网验证 |
| H.264 NVENC (预备) | 需 NVIDIA Video Codec SDK, 由 `SPLATSHOP_HAS_NVENC` 守卫 |

---

### 3.6 OrbbecSDK 集成

> **计划文档**: `.zcode/plans/plan-sess_b17bc0dd.md`, `plan-sess_84e1bca6.md`

#### 3.6.1 集成目的

**OrbbecSDK** 是 Orbbec 公司的 RGBD 深度相机开发工具包。用于在 Splatshop 中**实时采集 3D 点云**。

#### 3.6.2 Vendored SDK 规模

| 组件 | 说明 |
|------|------|
| `OrbbecSDK.dll` | 主库 |
| `OrbbecSDKConfig.xml` | 设备配置 |
| `MultiDeviceSyncConfig.json` | 多设备同步 |
| `extensions/depthengine/` | 深度引擎插件 |
| `extensions/filters/` | 滤镜处理器 |
| `extensions/firmwareupdater/` | 固件更新器 |
| `extensions/frameprocessor/` | 帧处理器 |
| 20+ 官方示例 .exe | benchmark, depth, color, imu, record 等 |
| MSVC 运行时 DLL | msvcp140, concrt140 |

#### 3.6.3 构建集成

**`cmake/orbbec.cmake`** → `ADD_ORBBEC(TARGET_NAME)`:

- **可选依赖**: SDK 目录缺失 → 警告 + 静默跳过
- **条件编译**: `#ifdef SPLATSHOP_HAS_ORBBEC` 守卫
- **包配置**: 使用 SDK 自带的 `libs/OrbbecSDK/lib/OrbbecSDKConfig.cmake`
- **运行时复制**: Windows 上复制 DLL + extensions + XML → 输出目录

#### 3.6.4 SNOrbbec 场景节点

继承 `SNPoints`，增加:

```
frameReady 标志 — 每帧新数据到来时 true
→ Update: 检查 frameReady → 专用全量上传路径
  (固定大小, 跳过增量 + 渐进式)
→ Draw: 渲染为标准点云 (HQS 或 Progressive 模式)
```

---

### 3.7 场景 Splats CRUD API

> **提交**: `9fe9bd8` — *feat: New scene Splats creation, loading, deletion, and coloring API*  
> **影响文件**: 5 个

#### 3.7.1 端点规范

**A. `POST /scene/splats/create`** — 从原语创建

```json
// 球体
{"type": "sphere", "params": {"radius": 1.5, "count": 500, "color": [1,0,0,1]}}

// 立方体
{"type": "box", "params": {"size": [2.0, 1.5, 1.0], "count": 800, "color": [0,1,0,1]}}

// 点集
{"type": "points", "params": {"positions": [[0,0,0],[1,1,1]], "color": [0,0,1,1]}}
```

响应: `{"id": 15, "name": "Sphere_0", "type": "SNSplats"}`

**B. `POST /scene/splats/load`** — 加载文件

```json
{"path": "C:/models/bicycle.ply", "type": "ply"}
{"path": "scene.json", "type": "scene_json"}
```

**C. `DELETE /scene/node/{id}`** — 删除节点

响应: `{"removed": true, "was": {"id": 15, "name": "Sphere_0"}}`

**D. `POST /scene/splats/{id}/color`** — 设置颜色

```json
{"color": [1, 0, 1, 1]}
```

响应: `{"id": 15, "color_set": true, "n_colored": 500}`

#### 3.7.2 实现分布

| 层 | 文件 | 负责 |
|-----|------|------|
| C++ 桥 | `RemoteControlServer.cpp` | `scene.splats.create_*`, `load_file`, `node.remove`, `set_color` |
| Python API | `server.py` | HTTP 端点 + 响应格式化 |
| 数据模型 | `models.py` | `SphereParams`, `BoxParams`, `PointsParams` 等 |

---

## 4. 构建系统

### 4.1 CMake 辅助模块

| 文件 | 功能 |
|------|------|
| `common.cmake` | ImGui, ImPlot, ImGuizmo, GLM, CUDA(NVRTC), OpenGL, GLFW |
| `libtorch.cmake` ⭐ | LibTorch 三级解析 (find_package → libs/ → LIBTORCH_PATH) |
| `orbbec.cmake` ⭐ | OrbbecSDK 可选集成 |
| `glfw.cmake` | GLFW 3.3.2 FetchContent + CMake 4.x 兼容修复 |

### 4.2 LibTorch 自动检测

```
优先级:
  ① find_package(Torch QUIET)          — CMake 标准
  ② libs/libtorch/                      — 项目内回退
  ③ LIBTORCH_PATH 环境变量             — 用户配置

找到时:
  • 链接 torch, torch_cpu, torch_cuda, c10 等
  • SPLATSHOP_HAS_LIBTORCH=1
  • _GLIBCXX_USE_CXX11_ABI=1 (C++17 + C++11 ABI 兼容)
  • Windows: 复制 DLL → 输出目录

未找到: "4DGS support will be disabled"
```

### 4.3 LibTorch CUDA 兼容性

| 系统 CUDA | GPU | LibTorch |
|-----------|-----|----------|
| 12.4 – 12.7 | RTX 30/40 系列 | cu124 ✅ |
| 12.8+ | RTX 50 系列 (Blackwell) | cu128 ✅ |
| 13.x | RTX 5090(D) | cu128 ✅ (驱动后向兼容) |

> **关键**: LibTorch 捆绑自有 CUDA 运行时 — **不依赖系统 CUDA Toolkit**。  
> 系统 CUDA 版本仅需在驱动层面 ≥ LibTorch CUDA 版本。  
> Splatshop 独立于系统 CUDA Toolkit 编译。

### 4.4 快速构建 (Windows)

```bash
# 基础构建
cmake -B build -S .
cmake --build build --config Release --target SplatEditor
./build/Release/SplatEditor.exe
```

```bash
# RTX 5090D + 4DGS
curl -L -o libtorch.zip \
  "https://download.pytorch.org/libtorch/cu128/libtorch-win-shared-with-deps-2.6.0%%2Bcu128.zip"
tar -xf libtorch.zip -C libs/
cmake -B build -S .
cmake --build build --config Release
```

### 4.5 第三方依赖汇总

| 库 | 来源 |
|----|------|
| OpenVR 1.16 | `libs/openvr/` (vendored) |
| Dear ImGui / ImPlot / ImGuizmo | `libs/` (vendored 源码) |
| GLM | `libs/glm/` (header-only) |
| GLEW | 系统 |
| GLFW 3.3.2 | FetchContent |
| nlohmann JSON | `include/json/` (header-only) |
| GPUSorting | `src/GPUSorting/` (vendored) |
| OrbbecSDK ⭐ | `libs/OrbbecSDK/` (vendored SDK) |
| LibTorch ⭐ | 外部 (自动检测) |
| CUDA Toolkit 12.4+ | 系统 |

---

## 5. 已知限制与未来方向

### 5.1 当前限制

| 类别 | 限制 |
|------|------|
| 成熟度 | Alpha 学术原型 — 预计较多 bug 和崩溃 |
| VR 平台 | 仅 Windows (Linux OpenVR 未解决) |
| 球谐函数 | 未实现 (内存需求大、增益有限) |
| 启动速度 | CUDA Driver API 运行时编译 → 启动缓慢 |
| 撤销覆盖 | 不完整、变换撤销有损 (缩放精度丢失) |
| 兼容性 | RTX 20xx 系列已知冻结问题 |
| 预编译包 | 无正式 release 二进制包 |

### 5.2 未来方向

| 优先级 | 方向 |
|--------|------|
| 高 | 撤销/重做覆盖扩展到全部功能 |
| 高 | 无损精度的变换撤销重构 |
| 中 | Linux OpenVR 集成 |
| 中 | 经济型球谐函数编码 |
| 中 | NVENC H.264 视频流管道完善 |
| 低 | 多缓冲渐进式渲染 Phase 2/3 |
| 进行中 | 收集反馈以确定优先级 |

---

## 6. 附录

### A. 键盘快捷键

| 按键 | 功能 |
|------|------|
| `Right Click` | 取消当前操作 |
| `Double Click` | 移向悬停 splats |
| `Ctrl+Z` / `Ctrl+Y` | 撤销 / 重做 |
| `1` | 笔刷选择 |
| `2` | 笔刷删除 |
| `t` | 平移 |
| `r` | 旋转 |
| `s` | 缩放 |
| `b` | 上色 |
| `Ctrl+D` | 选择复制到新图层 |
| `Ctrl+E` | 合并选中图层 |
| `Del` | 删除选中 |

### B. 坐标系统约定

| 约定 | 说明 |
|------|------|
| 坐标系 | 右手坐标系 |
| 上方向 | **Z 轴向上** |
| 角度 | 弧度 (radians) |
| 四元数 | `[x, y, z, w]` 数组 |
| 矩阵 | 列主序 (column-major), flat 16 元素 |
| 鼠标坐标 | 浏览器约定 (左上角原点), 桥接自动翻转 Y |

### C. 数据格式

**BIN 格式 (Skye 快速路径):**

| 偏移 | 大小 | 说明 |
|------|------|------|
| 头 0 | 4B | magic `0x11223344` |
| 头 4 | 8B | numPoints |
| 头 12 | 8B | stride = 16 |
| 点 0 | 4B | float x |
| 点 4 | 4B | float y |
| 点 8 | 4B | float z |
| 点 12 | 1B | uint8 r |
| 点 13 | 1B | uint8 g |
| 点 14 | 1B | uint8 b |
| 点 15 | 1B | uint8 a |

**4DGS Bundle:**

```
bundle_dir/
├── canonical.ply           — 规范态 3DGS PLY
├── deformation_model.pt   — TorchScript 变形模块
└── config.json             — {"sh_degree":3, "n_gaussians":123456, "format_version":"1.0"}
```

### D. 关键源文件索引

**新增文件 (⭐):**

| 文件 | 说明 |
|------|------|
| `src/render/progressive_points.cu` | 渐进式点云渲染内核 |
| `src/scene/ProgressivePointData.h` | 渐进式数据 + 素数置换 |
| `src/scene/SNPoints.h` | 点云场景节点 (双渲染路径) |
| `src/scene/SN4DGSSplats.h/.cpp` | 4DGS 动态高斯节点 |
| `src/scene/SNOrbbec.h` | Orbbec 实时点云节点 |
| `src/loader/BINPointCloudLoader.h` | BIN 格式加载器 |
| `src/loader/PointCloudPlyLoader.h` | 点云 PLY 加载器 |
| `src/motion/MotionTypes.h` | 运动类型、四元数、缓动 |
| `src/motion/MotionController.h` | 刚体运动控制器 |
| `src/motion/Timeline.h/.cpp` | 关键帧动画播放器 |
| `src/motion/RiggedHumanController.h/.cpp` | 骨骼蒙皮 + blendshape |
| `src/motion/ProceduralRigSource.h/.cpp` | 测试骨架生成 |
| `src/motion/skinning.cu` | CUDA 蒙皮内核 |
| `src/remote/RemoteControlServer.h/.cpp` | JSON-RPC 控制桥 |
| `remote_api/server.py` | FastAPI HTTP 服务 |
| `remote_api/splat_client.py` | TCP 桥客户端 |
| `remote_api/models.py` | Pydantic 数据模型 |
| `remote_api/config.py` | 配置管理 |
| `remote_api/keymap.py` | GLFW 键名映射 |
| `remote_api/examples/vr_webxr_client.html` | WebXR VR 客户端 |
| `remote_api/examples/webrtc_receiver.py` | WebRTC 接收器示例 |
| `tools/export_4dgs_torchscript.py` | 4DGS 模型导出 |
| `cmake/libtorch.cmake` | LibTorch 集成 |
| `cmake/orbbec.cmake` | OrbbecSDK 集成 |
| `cmake/glfw.cmake` | GLFW 集成 (CMake 4.x 兼容) |
| `src/gui/pointcloud.h` | 点云 GUI 面板 |
| `src/gui/motion.h` | 运动/4DGS GUI 面板 |

**核心已有文件:**

| 文件 | 说明 |
|------|------|
| `src/SplatEditor.h` | 主编辑器声明 (528 行) |
| `src/SplatEditor_draw.h` | 渲染管线 |
| `src/SplatEditor_update.h` | 更新管线 |
| `include/CudaModularProgram.h` | CUDA 模块化编程 (810 行) |
| `include/HostDeviceInterface.h` | 设备数据接口 |
| `include/Points.h` | 点云数据结构 |
| `src/scene/SNSplats.h` | 高斯泼溅节点 |
| `CMakeLists.txt` | 主构建文件 |

### E. 设计计划文档

| 计划文件 | 主题 |
|----------|------|
| `plan-sess_7da06295` | Skye 渐进式点云集成 ⭐ |
| `plan-sess_659002cd` | VR 远程浏览 ⭐ |
| `plan-sess_0f0a15b9` | 运动控制模块 ⭐ |
| `plan-sess_0ede3fd6` | 场景 Splats API ⭐ |
| `plan-sess_84e1bca6` | 色彩校正 + Orbbec ⭐ |
| `plan-sess_b17bc0dd` | Orbbec GUI 改进 ⭐ |
| `plan-sess_f04152b0` | LibTorch 自动检测 ⭐ |
| `plan-sess_184389bd` | CMake 4.x + GLFW 兼容 ⭐ |
| `plan-sess_7f116a59` | MCP 服务器安装 |

### F. 示例数据集

> [splatmodels.zip](http://users.cg.tuwien.ac.at/mschuetz/permanent/splatmodels.zip) — 包含花园场景 (garden)，源自 Inria 3DGS 预训练模型，原始数据来自 Mip-NeRF 360。

---

*报告结束。如需某一子系统的更详细分析或架构图，请进一步告知。*
