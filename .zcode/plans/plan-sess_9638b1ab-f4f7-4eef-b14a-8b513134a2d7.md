# Splatshop GPU BA / 点云优化 — 调研 + 原型实现方案

## 一、调研结论

### 范式区分（关键框架）
两种"Bundle Adjustment"被混用，必须区分：

| | (A) 经典稀疏 BA | (B) 稠密/可微点云优化 |
|---|---|---|
| 优化变量 | 相机位姿 + 稀疏 3D 特征点 XYZ | 稠密图元的 **位置 + 颜色(+尺度/旋转/不透明度)** |
| 损失 | 特征重投影误差 | 光度/渲染损失 |
| 颜色是否为优化参数 | 否（颜色是观测值，不是自由参数） | **是** |
| 代表 | Ceres / g2o / GTSAM / PBA | 3DGS / GS-SLAM / SplaTAM / 2DGS-SLAM |
| GPU | Ceres 仅 dense 求解器 GPU，稀疏 BA 仍 CPU；PBA GPU 但 GPL | 原生 GPU，autograd |

**对本任务"优化点云的 3D 位置 AND 颜色"，只有范式 (B) 能原生同时优化两者。** 经典 BA 把颜色当观测值而非优化参数，无法满足需求。

### 候选库评估（按 Splatshop stack：libtorch 2.6 + CUDA 12.4 + OrbbecSDK + Windows + 现有可微 splatting 光栅器）

| 候选 | 优化颜色? | 优化位置? | GPU? | 许可 | Windows | 适配度 |
|---|---|---|---|---|---|---|
| **3DGS 可微光栅化(复用现有)** | ✅ SH | ✅ + 尺度/旋转/不透明度 | ✅ 原生 | 已在项目中 | ✅ | ★★★★★ |
| Ceres | ❌ | ✅(稀疏点XYZ) | ⚠️ 仅 dense 求解器 | BSD | ✅ vcpkg | ★★(只能做位姿预条件器) |
| PBA (ccwu) | ❌ | ✅ | ✅ CUDA | **GPL-3** | ⚠️ 休眠 | ★(许可风险) |
| g2o | ❌ | ✅ | ❌ 无 GPU | BSD | ⚠️ 实验 | ★ |
| GTSAM | ❌ | ✅ | ❌ 无 GPU | BSD | ⚠️ 重依赖 | ★ |
| fixstars/cuda-BA | ❌ | ✅ | ✅ | 宽松 | ❌ Linux | ★★(需移植) |
| ElasticFusion | 部分(融合设定) | ✅ surfel | ✅ | GPL | ❌ 遗留 | ★ |
| ICP 族(PCL/Open3D/fast_gicp) | ❌ | ❌ 只优化位姿 | 部分 | BSD/MIT | ⚠️ | ★(位姿用) |
| PyTorch3D PointsRenderer | ✅ | ✅ | ✅ | MIT | ⚠️ Python 优先 | ★★(重复光栅器) |

**推荐：范式 (B) — 用 Splatshop 已有的可微高斯光栅化作为"BA"步骤。** 这是 2023-2025 文献主流（MonoGS/SplaTAM/GS-SLAM/Photo-SLAM 全部如此），且零新增依赖、复用现有 CUDA + libtorch autograd。经典 BA 仅在需要全局位姿一致性时作为可选位姿预条件器。

### Splatshop 现状（源码核实）
- **libtorch 零拷贝模式已验证**：`SN4DGSSplats.cpp:151-187` 用 `torch::from_blob(data.position, {N,3}, strides, torch::kCUDA)` 直接包裹 Splatshop 的 `CUdeviceptr`，同 CUDA 上下文 forward。→ BA 优化器可把 splat 缓冲区包成 `requires_grad` 张量，autograd 更新直接落回渲染缓冲区，**无拷贝**。
- **点云数据模型** (`Points.h`)：SoA `position(vec3,12B)` + `color(uint32 RGBA8,4B)`，无尺度/旋转/SH/法线。BA 直接把位置+颜色当优化变量。
- **现有光栅器是 forward-only** (`gaussians_rendering.cu:1590` `kernel_render_gaussians_perspectivecorrect`)：标准 3DGS 算法（cov3D=R·S·S·Rᵀ at `:588`，SH 颜色，tile-based alpha blend），但**无 backward/梯度 kernel**，framebuffer 原地写，不保留每像素透射率/每 splat 贡献。**反向通路缺失，是原型的主要工作量。**
- **无任何 optimizer**：grep `backward/grad/dL_d/autograd/torch::optim/loss` 全无命中。BA 是 greenfield。
- **Orbbec 只给帧不给位姿** (`OrbbecTypes.h`)：`RGBDFrame` 有 depth+color+intrinsics+depthScale，**无 extrinsics/位姿字段**；`PointCloudFilter` 单帧相机系内反投影，无多帧融合/位姿估计。

## 二、原型设计（用户选定）

- **范式**：可微渲染优化（3DGS 式）
- **前向模型**：高斯 Splatting 光栅器
- **位姿**：假设静止（位姿设为恒等/相机内参单帧）
- **反向通路**：vendor 原版 3DGS `forward.cu` + `backward.cu` kernel，用 C++ `torch::autograd::Function` 包裹（与现有 perspective-correct 光栅器并行，优化用 vendor 的，显示用现有的）
- **许可**：可接受原版 3DGS 非商业许可（研究/原型）

### 模块划分

```
src/optim/
  BAState.h              // 优化状态：位置/尺度/旋转/不透明度/SH 的 requires_grad 张量 + AdamW 优化器
  PointCloudBA.h/.cpp    // BA 优化器：init from Points/Splats → 可微光栅化 forward → 光度损失 → backward → AdamW step → 写回
  (整个模块 #ifdef SPLATSHOP_HAS_LIBTORCH，仿 SN4DGSSplats 隔离依赖)

src/optim/cuda/
  rasterize_fwd.cu       // vendor 自 graphdeco-inria/gaussian-splatting (forward.cu)，标准 EWA 投影
  rasterize_bwd.cu       // vendor 自 backward.cu，链式导数到 position/scale/rot/opacity/SH
  RasterizeFunction.h/.cpp  // torch::autograd::Function<RasterizeFunction> 包裹 fwd/bwd，注册进 autograd 图

src/scene/SNPointCloudBA.h  // 场景节点：持有 Points/Splats + BAState，提供 startOptimize()/step() 接口
```

### 数据流（原型，单帧静止相机）

1. **采集目标帧**：从 Orbbec `RGBDFrame` 取一帧 color 图作为优化目标 `target_image`（H×W×3 张量，kCUDA），相机位姿=恒等，内参来自 `Intrinsics{fx,fy,cx,cy}`。
2. **初始化 Gaussians**：从当前点云 `PointDataManager` 的 position + color 实例化高斯——位置=点位置，颜色=点颜色→SH 0 阶，尺度=点间距/深度噪声估，不透明度=1，旋转=恒等。
3. **优化循环**（每 step）：
   - 把 splat 缓冲区用 `torch::from_blob(..., kCUDA)` 包成 `requires_grad` 叶子张量（位置/尺度/旋转/不透明度/SH）——复用 `SN4DGSSplats.cpp` 模式
   - `RasterizeFunction::apply(...)` 跑 vendor forward 光栅化 → 渲染图 `pred_image`
   - 损失 `L = λ1·L1(pred, target) + λ2·DSSIM(pred, target)`（可选加 depth 一致性项，从点深度 vs `RGBDFrame.depthData`）
   - `loss.backward()` → autograd 调 vendor backward kernel，梯度写回叶子张量的 `.grad()`
   - `torch::optim::AdamW.step()` 更新参数
   - 把更新后的位置/颜色 `cuMemcpyDtoD` 写回 `PointDataManager` 渲染缓冲区（显示实时刷新）
4. **循环 N 步或收敛**，结果点云落回 `SNPoints` 节点显示。

### 关键实现点

- **CudaModularProgram vs libtorch**：vendor 的 3DGS kernel 用 **NVRTC 运行时编译**（`CudaModularProgram`，仿 `prog_gaussians_rendering`）还是 **静态 .cu + CMake CUDA**？推荐用 CMake 静态编译（vendor 3DGS kernel 是 .cu 源文件，随项目编译更稳；`CudaModularProgram` 适合热重载迭代但 autograd Function 跨边界更复杂）。→ 落地时再定，倾向于 CMake 静态编译 vendor kernel + `torch::autograd::Function` 桥接。
- **autograd Function 桥**：`RasterizeFunction::forward` 调 vendor fwd kernel 并保存中间状态（每像素透射率、每 splat 贡献）到 `ctx`；`backward` 接 `dL_dout_color` 调 vendor bwd kernel 输出 `dL_dposition/dL_dscale/...`。这是原版 3DGS Python `_RasterizeGaussiansCUDA` 的 C++ 等价。
- **CUDA 上下文共享**：`SN4DGSSplats.cpp:156` 注释已确认 libtorch kCUDA 默认设备 0 与 Splatshop 主上下文一致；BA 沿用同模式。
- **主线程约束**：CUDA/GL 上下文绑定主线程；BA 优化 step 可在主线程 `SplatEditor_update.h` 中每帧跑 K 步（仿 4DGS `deform()` 在 draw 路径），或用 `EventQueue::schedule()` 从后台调度。原型先在主线程每帧跑若干步。
- **点云→高斯映射**：点云无尺度/旋转/SH，需补齐。尺度用近邻间距或固定小球（如 0.5mm），SH 用颜色 DC 系数，旋转=恒等，不透明度=0.9。

### Remote API 扩展（可选，原型后）
在 `RemoteControlServer.cpp` 命令表（~:830）+ `remote_api/server.py` 新增：
- `scene.points.optimize` — 对指定 SNPoints 节点启动 BA（参数：步数、学习率、目标帧来源）
- `scene.points.optimize_step` — 单步推进（便于观察收敛）
- `scene.points.optimize_status` — 查询损失/步数

### 验证标准
- 对 Orbbec 单帧 RGB-D 点云，跑 N 步 BA 后渲染图与目标 color 图的 L1 误差单调下降
- 优化后的点云位置/颜色在 Splatshop 视口肉眼可见地变清晰（颜色更准、边缘更锐利）
- 无新增运行时依赖崩溃；libtorch 缺失时（`!SPLATSHOP_HAS_LIBTORCH`）BA 模块编译为 stub，不影响现有功能

### 风险与缓解
- **vendor 3DGS kernel 的 C++/libtorch autograd 桥接**是新写代码，原版是 Python `torch::autograd::Function`。需参考 libtorch C++ autograd 文档实现 `forward/backward`。→ 先用最小版本（只优化 position + SH color，尺度/旋转/不透明度冻结）跑通，再逐步放开。
- **许可**：原版 3DGS 非商业，仅研究/原型可接受；若未来需商业，再切 gsplat 或自推 backward。
- **静止相机假设**：原型只验证单帧精修；多帧需后续加位姿估计（ICP/里程计），本次不做。

## 三、原型实现顺序（落地步骤）

1. vendor `forward.cu`/`backward.cu` 到 `src/optim/cuda/`，CMake 静态编译（先确认能 build）
2. 写 `RasterizeFunction`（`torch::autograd::Function`）桥接 fwd/bwd，单测：给定 splat + 相机，forward 能渲染，backward 能回传梯度（用 `torch::autograd::gradcheck` 验证）
3. 写 `BAState`：把 `PointDataManager` position/color 包成 requires_grad 张量 + AdamW
4. 写 `PointCloudBA::step()`：fwd → L1 loss → backward → AdamW step → 写回渲染缓冲区
5. 接入 `SNPointCloudBA` 节点 + `SplatEditor_update.h` 每帧跑 K 步
6. GUI/Remote 触发 + 损失曲线显示
7. 对真实 Orbbec 帧验证收敛与视觉改善