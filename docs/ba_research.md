# GPU Bundle Adjustment / 点云优化调研报告

调研目标：在点云生成后，通过 GPU CUDA 支持的 Bundle Adjustment（BA）或其他点云优化算法，优化点云的 **3D 位置** 和 **颜色**。

调研时间：2026-08-07。宿主项目 Splatshop 技术栈：C++23 + libtorch 2.6 (cu124) + CUDA 12.4 + OrbbecSDK v2.9.3，Windows x64 / MSVC，含 3DGS/4DGS 渲染（forward-only，NVRTC 运行时编译 `.cu`）。

---

## 一、范式区分（最关键的框架）

"Bundle Adjustment" 被混用于两种本质不同的优化，必须区分：

| | (A) 经典稀疏 BA | (B) 稠密 / 可微点云优化 |
|---|---|---|
| 优化变量 | 相机位姿 + 稀疏 3D 特征点 XYZ | 稠密图元的 **位置 + 颜色（+尺度/旋转/不透明度）** |
| 损失函数 | 特征重投影误差 | 光度 / 渲染损失（photometric） |
| 颜色是否为优化参数 | **否**（颜色是观测值，驱动重投影残差，不是自由参数） | **是** |
| 代表实现 | Ceres / g2o / GTSAM / PBA | 3DGS / GS-SLAM / SplaTAM / 2DGS-SLAM |
| GPU 加速 | Ceres 仅 dense 求解器 GPU，稀疏 BA 仍 CPU；PBA GPU 但 GPL | 原生 GPU，autograd |
| 输出 | 精修的相机位姿 + 稀疏 3D 点 | 精修的稠密 surfel/Gaussian 的位置 + 颜色 + 形状 |

**对本任务"优化点云的 3D 位置 AND 颜色"，只有范式 (B) 能原生同时优化两者。** 经典 BA 把颜色当观测值而非优化参数，无法满足"优化颜色"这一需求——它能精修稀疏点 XYZ 和位姿，但稠密点云的颜色仍是输入测量值。

---

## 二、候选库评估

按 Splatshop stack（libtorch + CUDA 12.4 + OrbbecSDK + Windows + 现有可微 splatting 光栅器）评估：

| 候选 | 优化颜色? | 优化位置? | GPU? | 许可 | Windows | 适配度 |
|---|---|---|---|---|---|---|
| **3DGS 可微光栅化（复用现有）** | ✅ SH | ✅ + 尺度/旋转/不透明度 | ✅ 原生 | 已在项目中 | ✅ | ★★★★★ |
| Ceres | ❌ | ✅（稀疏点 XYZ） | ⚠️ 仅 dense 求解器 | BSD | ✅ vcpkg | ★★（只能做位姿预条件器） |
| PBA (Changchang Wu) | ❌ | ✅ | ✅ CUDA | **GPL-3** | ⚠️ 上游休眠 | ★（许可风险） |
| g2o | ❌ | ✅ | ❌ 无 GPU | BSD | ⚠️ 实验 | ★ |
| GTSAM | ❌ | ✅ | ❌ 无 GPU | BSD | ⚠️ 重依赖 | ★ |
| fixstars/cuda-BA | ❌ | ✅ | ✅ | 宽松 | ❌ Linux 导向 | ★★（需移植） |
| ElasticFusion | 部分（融合设定） | ✅ surfel | ✅ | GPL | ❌ 遗留 2015 | ★ |
| BundleFusion | 部分（融合设定） | ✅ TSDF | ✅ | 非商业 | ❌ 遗留 | ★ |
| ICP 族（PCL GPU / Open3D / fast_gicp） | ❌ | ❌ 只优化位姿 | 部分 | BSD/MIT | ⚠️ | ★（位姿用） |
| PyTorch3D PointsRenderer | ✅ | ✅ | ✅ | MIT | ⚠️ Python 优先 | ★★（重复光栅器） |
| Kaolin DIB-R | ✅ | ✅ | ✅ | MIT | ⚠️ PyTorch 优先 | ★★ |

### 各候选细节

**Ceres Solver（BSD）**：通用非线性最小二乘；BA = 位姿 + 3D 点 XYZ 的重投影误差。2024-2025 的真实 CUDA 支持仅限 dense 求解器（`DENSE_QR` / `DENSE_NORMAL_CHOLESKY` / `DENSE_SCHUR` + 混合精度，`-DUSE_CUDA=ON`）。稀疏 BA 求解器（CG / SPARSE_SCHUR）仍 CPU——这就是 COLMAP/GLOMAP 用户遇到"compiled without cuDSS, falling back to CPU"的原因，实验性 cuDSS/cuSOLVER GPU 稀疏后端不在发布版。Windows/vcpkg/CMake 优秀。**但不能优化点颜色**——最适合做位姿预条件器，不能做颜色精修。

**PBA (Changchang Wu)**：纯 GPU 稀疏 BA（位姿 + 点，inexact Newton + PCG），最强经典 GPU BA（COLMAP 历史使用）。**GPL-3**（Debian colmap 版权文件确认），上游休眠。点数据是 `float XYZ`，无颜色字段。许可与维护性都不适合非 GPL 的 Splatshop。

**g2o**：上游无 GPU（CPU/Eigen）；Windows "experimental"；人们说的"CUDA g2o"其实是 fixstars/cuda-bundle-adjustment，是独立项目。不优化颜色。

**GTSAM（BSD-3）**：无 first-class GPU 支持（SuiteSparse/iSAM2 均 CPU）；增量 SLAM 平滑对一次性精修是 overkill；Windows 需 Boost/SuiteSparse。不优化颜色。

**fixstars/cuda-bundle-adjustment**：g2o `BlockSolver_6_3` + LM 的 CUDA 重实现，GTX1080 上相对 CPU g2o ~8x 加速，CUDA >= 6.0，宽松许可，但 Linux 导向、需 Windows 移植。位姿 + 点，无颜色。

**3DGS / 4DGS（范式 B）**：原生优化 per-primitive 的 position + scale/rotation + opacity + **color（球谐 SH）**，通过可微光栅化器 + autograd 在 GPU 上做梯度下降。RGB-D 变体 GS-SLAM (CVPR24)、SplaTAM (CVPR24)、2DGS-SLAM (2025)、SGAD-SLAM 加入深度监督损失和联合位姿优化。原版 3DGS 代码非商业/研究许可，但宽松再实现（gsplat 等）存在，且 Splatshop 已携带 splatting。**这是宿主项目已具备的、唯一原生同时优化位置和颜色的 GPU 范式。**

**PyTorch3D PointsRenderer (MIT)**：可微、优化点位置 + 颜色、CUDA 光栅化器——但 Python 优先，作为 C++/libtorch CMake 依赖很别扭，且与 Splatshop 自有光栅器重复。

**ElasticFusion**：GPU 上精修 surfel 位置 + 颜色（CUDA），但 2015-16 遗留 Linux 代码库（OpenGL + OpenNI2 + Pangolin + SuiteSparse），不维护，Windows 移植繁重。GPL。

**ICP 族**（PCL GPU ICP BSD、Open3D MIT、fast_gicp MIT）：全部只优化 **相机位姿**，不优化 per-point 颜色或 per-point 位置。Colored-ICP 把颜色作为代价函数中的测量值，不是自由参数。仅适合作为位姿预条件器。

---

## 三、推荐方案

**范式 (B) — 用 Splatshop 已有的可微高斯光栅化作为"BA"步骤。**

理由：
1. **范式正确**：是 2023-2025 文献主流（MonoGS / SplaTAM / GS-SLAM / Photo-SLAM 全部如此），3DGS 优化本身就是"对图元做稠密 BA，同时优化位置 + 颜色"。
2. **栈契合**：Splatshop 已是 C++ + libtorch + CUDA + 高斯光栅器。优化器已在二进制中——只需 (a) 把点云包成可优化图元，(b) 对目标帧跑光度损失 + AdamW，零新增依赖。
3. **无 GPU BA 替代品**：COLMAP/Ceres BA 是 CPU 且不优化颜色；PBA 是 GPU 但只位姿；hloc 不加 GPU；surfel 库优化位姿/TSDF 而非颜色。范式 (A) 要么 CPU Ceres（慢、无颜色），要么自写 CUDA BA——工作量严格大于复用已有光栅器。
4. **许可**：Photo-SLAM (C++ 候选) 是 GPLv3（ORB-SLAM3 血统），与 Splatshop 可能不兼容；ElasticFusion 是 GPL。复用自有优化器避免引入 GPL。
5. **质量**：3DGS 额外优化形状（scale/rotation）和 opacity，比刚性点云 BA 更好。

**何时仍需经典 BA（A）**：需要全局位姿一致性（回环、漂移修正）而非信任深度相机轨迹时，加位姿图/Ceres 步骤只优化位姿，仍让 3DGS 优化图元颜色 + 位置。

---

## 四、Splatshop 现状（源码核实）

- **libtorch 零拷贝模式已验证**：`src/scene/SN4DGSSplats.cpp:151-187` 用 `torch::from_blob(data.position, {N,3}, strides, torch::TensorOptions().device(torch::kCUDA))` 直接包裹 Splatshop 的 `CUdeviceptr`，同 CUDA 上下文 forward。→ BA 优化器可把 splat 缓冲区包成 `requires_grad` 张量，autograd 更新直接落回渲染缓冲区，无拷贝。
- **点云数据模型** (`include/Points.h`)：SoA `position (vec3, 12B/pt)` + `color (uint32 RGBA8, 4B/pt)`，无 scale/rotation/SH/normal。BA 把位置 + 颜色当优化变量。
- **现有光栅器 forward-only** (`src/gaussians_rendering.cu:1590` `kernel_render_gaussians_perspectivecorrect`)：标准 3DGS 算法（cov3D=R·S·S·Rᵀ at `:588`，SH 颜色，tile-based alpha blend），但无 backward/梯度 kernel，framebuffer 原地写，不保留每像素透射率/每 splat 贡献。**反向通路缺失，是原型主要工作量。**
- **无任何 optimizer**：repo 内 grep `backward/grad/dL_d/autograd/torch::optim/loss` 全无命中。BA 是 greenfield。
- **NVRTC 编译模型**：`CMakeLists.txt:63` `#enable_language(CUDA)` 被注释，所有 `.cu` 是纯文本资源，运行时由 `CudaModularProgram`（`include/CudaModularProgram.h`）用 NVRTC 编译。新增 CUDA kernel 应走此路径或用 libtorch tensor op。
- **Orbbec 只给帧不给位姿** (`src/camera/OrbbecTypes.h`)：`RGBDFrame` 有 depth + color + intrinsics + depthScale，**无 extrinsics/位姿字段**；`PointCloudFilter` 单帧相机系内反投影，无多帧融合/位姿估计。

---

## 五、原型设计

- **范式**：可微渲染优化（3DGS 式），范式 (B)。
- **前向模型**：高斯 Splatting 光栅化（EWA 高斯混合）。
- **位姿**：假设静止（位姿恒等，单帧目标）。
- **反向通路**：用 libtorch autograd（tensorized EWA 高斯前向，autograd 自动提供反向），原型阶段避免写自定义 CUDA backward kernel。后续可升级为 tile-based 可微光栅化器。
- **许可**：可接受原版 3DGS 非商业许可（研究/原型）。本原型不 vendor 原版 kernel，自实现 tensorized 前向，无许可负担。

### 数据流（单帧静止相机）
1. 采集目标帧：从 Orbbec `RGBDFrame` 取 color 图作为 `target_image`（H×W×3 张量，kCUDA），位姿恒等，内参 `Intrinsics{fx,fy,cx,cy}`。
2. 初始化 Gaussians：点云 `PointDataManager` 的 position + color → 高斯（位置=点，颜色=点颜色，尺度=固定小球/点间距，不透明度=1）。
3. 优化循环（每 step）：包成 `requires_grad` 张量 → 可微前向渲染 `pred_image` → 损失 `L = λ1·L1 + λ2·DSSIM`（可选深度一致性）→ `loss.backward()` → `torch::optim::AdamW.step()` → `cuMemcpyDtoD` 写回渲染缓冲区。
4. 循环 N 步或收敛，结果点云落回 `SNPoints` 显示。

### 验证标准
- 对 Orbbec 单帧 RGB-D 点云，N 步 BA 后渲染图与目标 color 图 L1 误差单调下降。
- 优化后点云位置/颜色在视口肉眼可见变清晰。
- libtorch 缺失时（`!SPLATSHOP_HAS_LIBTORCH`）BA 模块编译为 stub，不影响现有功能。
