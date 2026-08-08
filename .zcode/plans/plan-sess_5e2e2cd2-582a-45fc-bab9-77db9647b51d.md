# 点云 Remeshing（密度优化）实施方案

## 目标
为 Splatshop 新增"点云重采样/密度优化"功能：通过体素网格下采样把整个点云降到目标间距 h，减少 VRAM 占用、提升渲染流畅度，同时保留几何覆盖。采用非破坏式工作流——生成新的 SNPoints 节点，原点云保留可对比/回退。

## 算法选型
**体素网格下采样（Voxel Grid Downsampling）**，CUDA 实现：
1. 对每个点计算其所在体素的 Morton/线性索引 `idx = floor(p/h)` 三轴编码。
2. 按体素索引对点排序（复用现有 `GPUSorting::sort_32bit_keyvalue`，必要时升到 64bit key 或分块）。
3. 对排序后数组做**去重 + 质心聚合**：相邻同 key 的点用 reduce 求和 position、加权平均 color，写出代表点。
- 时间复杂度 O(n log n)（排序主导），空间 O(n)，GPU 友好，热重载友好。
- 目标间距 h 由 GUI 滑块给出（按点云包围盒对角线自动给默认值，如对角线/512）。

这是现有桩 `src/gaussians_editing.cu:1996 kernel_downsample` 注释中 256³ 体素方案的工程化精简版，但落到点云而非 splat，且用排序去重替代栅格哈希以支持任意大包围盒。

## 复用的现有基础设施
- `include/CudaModularProgram.h` —— 新建 `prog_points_remesh = new CudaModularProgram({"./src/remesh/points_remesh.cu"})`，运行时 NVRTC 编译 + 热重载。
- `src/GPUSorting/GPUSorting::sort_32bit_keyvalue` —— 体素索引排序。
- `src/HostDeviceInterface.h:PointData` —— 输入/输出设备数据结构。
- `src/PointsManagement.h:PointDataManager` —— 输出点云的设备缓冲（commit 增长式分配）。
- `src/scene/SNPoints.h` —— 新建结果节点。
- `src/SplatEditor.h` 程序注册表 + `SplatEditor.cpp` 构造处（仿 `prog_points` 注册）。

## 新增/修改文件

### 1. 新增 `src/remesh/points_remesh.cu`（核心 kernel）
三个 `extern "C" __global__`：
- `kernel_compute_voxelKeys(PointData in, uint64_t* keys, uint32_t* vals, vec3 invH, ivec3 dims, vec3 min)` —— 每点算 `key=pack(floor((p-min)/h))`，`val=点索引`。若 dims 超出 32^3 范围用 64bit key（三轴各 21bit）。
- `kernel_compact_centroids(PointData in, uint64_t* sortedKeys, uint32_t* sortedVals, uint32_t count, PointData out, uint32_t* outCount, vec3 min, vec3 invH, ivec3 dims)` —— 单 pass：同 key 段求 position 和、color 和、计数，段末写出质心点（position/计数、color/计数）。利用相邻比较 + shared memory reduce；输出点数写回 `*outCount`。
- `kernel_boundingbox_remesh(PointData out)` —— 复用 `points.cu` 同名思路重算结果包围盒。

### 2. 修改 `src/SplatEditor.h`
- 声明 `CudaModularProgram* prog_points_remesh = nullptr;`
- 新增方法 `void remeshPointCloud(SNPoints* src, float voxelSize, bool adaptiveFill);`（`adaptiveFill` 预留，首版不实现逻辑，仅留接口与 GUI 勾选项灰显）。

### 3. 修改 `src/SplatEditor.cpp`
- 在程序注册区（~`:1554`）注册 `prog_points_remesh`，路径 `{"./src/remesh/points_remesh.cu"}`。
- 实现 `remeshPointCloud`：分配 keys/vals 虚拟内存 → launch `kernel_compute_voxelKeys` → `GPUSorting::sort_32bit_keyvalue` → 分配输出缓冲（按上界估算 count，或两阶段先计数后写入）→ launch `kernel_compact_centroids` → launch `kernel_boundingbox_remesh` → 回读 outCount → 构造新 `SNPoints` 节点（复用 host `Points` 结构填写 min/max/world，device 由新 `PointDataManager` 接管，跳过 host→device 拷贝，直接把 kernel 输出缓冲嫁接）→ 加入 scene 树。
- 输出缓冲上界：用 `min(count, dims.x*dims.y*dims.z)` 与去重计数二者取小；为稳妥先做一次计数 pass（`kernel_compact_centroids` 内 atomic 计数 + 写入），再 compact。可接受两阶段。

### 4. 新增 `src/gui/remesh.h`（GUI 面板）
- 在点云 GUI（`src/gui/pointcloud.h` 旁）加 "Remesh / 密度优化" 折叠组：
  - 体素间距 h 滑块（按选中点云包围盒给默认 + 数值输入）。
  - "预估输出点数" 实时显示（`ceil(vol/h^3)` 上界，简化提示）。
  - "生成新节点"（默认勾上）/ "原地替换" 单选。
  - "执行 Remesh" 按钮 → 调 `editor.remeshPointCloud(...)`。
  - 进度/耗时显示（用 `EventPool` timing）。
- 接入主菜单/属性面板渲染处（仿 `pointcloud.h` 接入点）。

### 5. 修改 `src/HostDeviceInterface.h`
- 若需要 64bit voxel key 辅助结构（`VoxelKey`/pack/unpack `__device__` 内联函数），在 `PointData` 附近补充；或直接放 `points_remesh.cu` 内部以减少对共享头文件的侵入。倾向后者。

### 6. 文档
- 更新 `README.md`：在点云功能段加"密度优化 / 体素下采样重采样"条目。
- 更新 `docs/remote_api.md`：若后续接入 remote API（可选，首版不做）。

## 关键实现细节与取舍
- **64bit key**：点云包围盒可能很大（如户外扫描），21bit/轴 → 单轴 2M 体素，对 h=对角线/2048 足够。复用 `GPUSorting` 时需确认其是否支持 64bit key；若仅 32bit，则改为分块（按最高位分批）或直接在 `points_remesh.cu` 内加一个轻量 64bit radix sort pass。**实施第一步需先读 `src/GPUSorting/GPUSorting.h` 确认 key 位宽**，这是方案的最大不确定点。
- **质心 vs 代表点**：首版用质心（position 平均、color 平均），去噪效果更好；若要求保特征，后续可加"最接近体素中心的代表点"开关。
- **嫁接缓冲而非拷贝**：新节点的 `PointDataManager` 直接持有 kernel 输出的 `CudaVirtualMemory`，避免一次 D2D 拷贝；需确保 `PointDataManager` 支持外部接管缓冲（若不支持则加一个 `adopt()` 方法或走一次拷贝，二者皆可，倾向加 `adopt`）。
- **自适应分裂**：首版仅留 GUI 勾选项与函数参数，不实现，作为 Phase 2。文档注明。

## 验证
- 构建项目（CMake + MSVC，CUDA 由 NVRTC 运行时编译，无需 nvcc）。
- 加载一个大点云（用现有 LAS/BIN/PLY loader），执行 remesh，确认：
  - 输出点数 ≈ `vol/h^3` 上界以下且点云视觉覆盖保留。
  - 新节点出现在场景树，原点云仍在。
  - VRAM 占用下降（看 `gui/pointcloud.h` 的 VRAM 统计）。
  - 渲染帧率提升（HQS 与 progressive 两条路径都验证）。
- 热重载 `points_remesh.cu` 修改后能即时重编译（验证 `CudaModularProgram` 监听生效）。

## 阶段划分
- **Phase 1（本次实现）**：体素网格下采样全流程（kernel + 排序 + compact + GUI + 新节点），全局整点云，生成新节点。这是核心交付。
- **Phase 2（后续）**：密度自适应分裂（过疏区沿切向插入新点）、Poisson disk 高质量模式、局部画刷交互、原地替换 + undo。
