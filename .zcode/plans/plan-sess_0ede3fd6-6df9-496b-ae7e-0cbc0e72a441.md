## 扩展计划：场景 Splats 创建与修改 API

### 目标
在已有 HTTP API（相机/鼠标/键盘/刚体运动）基础上，新增高斯点云的**创建、加载、删除、合并**能力，实现跨应用端/网络端的完整场景控制。

### 新增 C++ 桥接命令（src/remote/RemoteControlServer.cpp）

数据流：JSON 命令 → `schedule()` 主线程 → 构建 `Splats` host-side → 创建 `SNSplats` → `scene.world->children.push_back` → 下帧 `uploadSplats` 自动上传 GPU。

| 命令 | 参数 | 实现 |
|---|---|---|
| `scene.splats.create_sphere` | `{center:[x,y,z], radius, count?, density?, color:[r,g,b,a]?}` | 仿 `Splats::createSphere()`，填充 Buffer，返回新节点 `{id, name}` |
| `scene.splats.create_points` | `{positions:[[x,y,z],...], scale?/scales:[...]?, color?/colors:[[r,g,b,a],...]?}` | 按给定位置创建独立 splats（最小 scale=0.02 if 未指定），可单独设色 |
| `scene.splats.create_box` | `{min:[x,y,z], max:[x,y,z], count?, color?}` | 在 AABB 内均匀撒点，scale 自适应密度 |
| `scene.splats.load_file` | `{path:"string"}` | 调用 `GSPlyLoader::load` 或 `SplatsyFilesLoader::load` 加载服务端文件 |
| `scene.node.remove` | `{id}` | `scene.erase(node)` + `scheduleRemoval` 或在 world 子节点中移除；如果只有 world 直接移除，其他父节点用 scene.erase |
| `scene.splats.set_color` | `{id, color:[r,g,b,a]}` | 遍历 `dmng.data.color` 写 uint16 值（只改已有 splats） |

> 所有创建命令在 Buffer 中填充后同时设置 `node->transform`（如 sphere 将 center 设为 transform translation），这样 MotionController 后续可整体移动。

### 新增 HTTP 端点（remote_api/server.py）

| 方法 | 路径 | 请求体 | 说明 |
|---|---|---|---|
| POST | `/scene/splats/create` | `{type:"sphere"\|"box"\|"points", params:{...}}` | 统一创建端点 |
| POST | `/scene/splats/load` | `{path}` | 从服务端磁盘加载 |
| DELETE | `/scene/node/{id}` | — | 删除节点 |
| POST | `/scene/splats/{id}/color` | `{color:[r,g,b,a]}` | 设颜色 |

**models.py 新增：**
```python
class SplatsCreateRequest(BaseModel):
    type: Literal["sphere","box","points"]
    params: Dict[str, Any]

class SphereParams(BaseModel):
    center: List[float] = [0,0,0]
    radius: float = 1.0
    count: Optional[int] = None
    density: Optional[float] = None
    color: Optional[List[float]] = [1,0,0,1]

class BoxParams(BaseModel):
    min: List[float]; max: List[float]
    count: Optional[int] = None
    color: Optional[List[float]] = [1,0,0,1]

class PointsParams(BaseModel):
    positions: List[List[float]]
    scale: Optional[float] = 0.02
    color: Optional[List[float]] = [1,0,0,1]
```

### 测试页更新（GET /test 内嵌 HTML）

新增「场景操作」面板：
- 下拉选创建类型（球/方块/散点）→ 填参数（位置/半径/颜色/数量）→ 「创建」按钮
- 「加载文件」输入框 + 按钮
- 节点表格每行增加 「删除」「设色」按钮
- 创建后自动刷新节点列表，选中可拖拽移动（复用已有 MotionController 端点）

### 编译

扩展仅修改 `RemoteControlServer.cpp` 和 Python 端，需重新编译：

```bash
cd build && cmake --build . --config Release --target SplatEditor
```

### 文档更新

`docs/remote_api.md` 新增「场景 Splats 操作」章节，含每个端点的 schema / curl 示例 / 注意事项。

### 实现顺序
1. C++ `RemoteControlServer.cpp` 新增 6 个命令处理器
2. `models.py` 新增创建请求模型
3. `server.py` 新增 4 个端点
4. 测试页 HTML 新增场景操作面板
5. 编译验证
6. `docs/remote_api.md` 追加文档

### 注意事项
- 创建 splats 时 color 值为 float [0,1]，桥接转换为 uint16 [0,65535]
- scale 值为 world 单位（非 log-scale），默认 sphere 的 scale ≈ radius/count^(1/3)
- 加载文件时路径相对于 Splatshop 工作目录（通常为仓库根目录，如 `./splatmodels/scene.json`）
- 所有创建操作在主线程执行，大数据量（>100K splats）会阻塞一帧 ~ 几百 ms；可在后台线程先建 Buffer，主线程仅 attach 和设置 upload（快速）