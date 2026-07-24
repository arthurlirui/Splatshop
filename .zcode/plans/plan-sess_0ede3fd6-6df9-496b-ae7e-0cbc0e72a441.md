## 远程控制 API 设计方案

### 目标
为 Splatshop 设计一套基于 Python(FastAPI) 的 HTTP API，让远程 WebRTC 接收端能控制渲染窗口的鼠标/键盘/相机，实现远程视角与相机运动；并附带刚体运动控制。产出 = 可运行的 Python API + C++ 桥接 + 调用文档。

### 架构（进程外 + 本地 TCP Socket）

```
[WebRTC 接收端 / 浏览器] --HTTP--> [Python FastAPI :8080] --TCP JSON-RPC(换行分隔) localhost:7654--> [C++ SplatEditor 后台线程 socket 服务] --EventQueue::schedule--> [主线程 GLFW 渲染循环]
```

- **桥接依据**：渲染循环与 CUDA/GL 上下文绑定主线程，任何外部命令必须转回主线程执行。代码库已有线程安全 `EventQueue`/`schedule()`（`include/unsuck.hpp:734-761`），每帧在 `GLRenderer::loop()` 中 `process()`，是最干净的注入点。
- **同步语义**：socket 线程用 `std::promise<json>`/`future`，调度一个 lambda 到主线程执行动作并回填结果，socket 线程 `future.get()` 阻塞直到主线程执行完 → 干净的同步请求/响应。
- **协议**：换行分隔 JSON。请求 `{"id":int,"cmd":"string","args":{...}}`，响应 `{"id":int,"ok":bool,"data":{...},"error":"..."}`。
- **Python 端**：FastAPI 每个请求构造一条 socket 请求并等待响应。可复用单条长连接 + 互斥锁，或每请求短连接（首版短连接更简单稳健）。

### 命令到代码库的映射

| 命令组 | 操作对象（代码库） |
|---|---|
| 相机轨道/平移/缩放/姿态 | `Runtime::controls`（`OrbitControls`：yaw/pitch/radius/target/`onMouseMove`/`onMouseScroll`/`update()`，`include/OrbitControls.h`） |
| 鼠标移动/按键/滚轮 | `Runtime::mousePosition`、`Runtime::mouseButtons`、`Runtime::mouseEvents`、`Runtime::controls->onMouseButton/onMouseMove/onMouseScroll`（`include/Runtime.h:75-88`、`MouseEvents.h`、`OrbitControls.h:84-135`） |
| 键盘按键 | `Runtime::keyStates[key]`、`Runtime::frame_keys/scancodes/actions/mods`、`Runtime::mods`（`Runtime.h:75-80`） |
| 刚体运动 | `motion::MotionController::setTransform/setTranslation/setRotation/setScale/translate/rotate/scaleBy/getTransform/setTransformAnimated`（`src/motion/MotionController.h`），操作对象 = `SplatEditor::instance->scene`，目标按 `NodeID`(int64) 解析 |
| 节点列表/状态 | `Scene::root->traverse`（`src/scene/Scene.h`）枚举 id/name/type |

### HTTP 端点设计

约定：
- 四元数一律 `[x,y,z,w]`（与 motion 模块 Timeline JSON 一致，`Timeline.cpp:99`）。
- 角度单位弧度。鼠标坐标 = 窗口像素坐标，原点左上、Y 向下；桥接层负责翻转为 app 内部坐标（app 内 cursor 回调已做 `height - ypos`）。
- 按钮名：`left`/`middle`/`right`；动作：`press`/`release`。
- 键 key：GLFW key 常量名（如 `"W"`、`"SPACE"`、`"LEFT_SHIFT"`）或其数值；桥接层维护名称→GLFW 码映射。
- 所有 `POST` 返回 `{"ok":true,"data":{...}}`；失败返回 HTTP 4xx + `{"ok":false,"error":"..."}`。

**系统**
- `GET /` → API 信息
- `GET /health` → ping C++ 桥接，返回 `{bridge:"up", fps:...}`

**相机（`Runtime::controls`）**
- `POST /camera/orbit` `{yaw?, pitch?, d_yaw?, d_pitch?}` → 绝对或增量设置 yaw/pitch
- `POST /camera/pan` `{dx, dy}` → 本地坐标系平移 target（复用 `translate_local` 语义）
- `POST /camera/zoom` `{radius?, factor?}` → 绝对或乘性缩放
- `POST /camera/pose` `{yaw, pitch, radius, target:[x,y,z]}` → 整体设置
- `GET /camera/pose` → 读回 `{yaw,pitch,radius,target,position}`（WebRTC 端同步视角用）
- `POST /camera/focus` `{node_id? | min:[..], max:[..], factor?}` → 调用 `OrbitControls::focus`

**鼠标**
- `POST /mouse/move` `{x, y}` → 设 `Runtime::mousePosition` 并喂 `controls->onMouseMove`（带翻转）
- `POST /mouse/button` `{button, action, mods?}` → 更新 `Runtime::mouseButtons` 位掩码 + `controls->onMouseButton` + `mouseEvents.onMouseButton`
- `POST /mouse/scroll` `{dx, dy}` → `controls->onMouseScroll` + `mouseEvents.onMouseScroll`
- `POST /mouse/event` `{x, y, button?, action?, mods?, scroll_dx?, scroll_dy?}` → 复合事件，便于一次往返完成"按下移动释放"

**键盘**
- `POST /keyboard/key` `{key, action, mods?}` → 写 `keyStates` + 压入 `frame_*` 向量（使 `Runtime::getKeyAction` 生效，与现有快捷键系统兼容）
- `POST /keyboard/press` `{key, duration_ms?, mods?}` → 便捷：press→等待→release
- `POST /keyboard/sequence` `{text}` → 逐字符 press/release（仅可打印字符）

**刚体运动（`motion::MotionController`，目标 = `SplatEditor::instance->scene`）**
- `GET /scene/nodes` → `[{id, name, type}]`（枚举场景树，用于查 node_id）
- `GET /motion/node/{id}/transform` → `{translation:[x,y,z], rotation:[x,y,z,w], scale:[x,y,z]}`
- `POST /motion/node/{id}/transform` `{translation, rotation, scale}` → `setTransform`
- `POST /motion/node/{id}/translate` `{delta:[x,y,z]}`
- `POST /motion/node/{id}/rotate` `{delta:[x,y,z,w]}` → `rotate`（本地原点预乘）
- `POST /motion/node/{id}/scale` `{factor:[x,y,z]}`
- `POST /motion/node/{id}/animate` `{target:{translation,rotation,scale}, duration_s, ease:"linear|in|out|in_out"}` → `setTransformAnimated`（异步，TWEEN 驱动）

### 要新增/修改的文件

**C++ 侧（新增）**
- `src/remote/RemoteControlServer.h` / `RemoteControlServer.cpp`
  - 后台 TCP 服务线程（Win32/POSIX 兼容 socket，尽量用条件编译或最小依赖；优先原生 socket 避免引入新库）
  - nlohmann/json（已 vendored 于 `libs/json`）解析请求
  - 命令分发表：`cmd` → handler，handler 内 `schedule(lambda)` 到主线程执行并 `promise.set_value(response)`
  - key 名→GLFW 码映射表
- `CMakeLists.txt`：将 `src/remote/*.*` 加入 source glob（当前 glob 模式需确认是否覆盖新目录；若不覆盖则显式添加）

**C++ 侧（修改）**
- `src/main.cpp`：在 `SplatEditor::setup()` 之后调用 `RemoteControlServer::start(7654)`；确保 `EventQueue::instance` 已初始化（loop 中已 drain）
- （可选）`include/unsuck.hpp`：若 `EventQueue::instance` 尚未在某处构造，需在 main 早期 `EventQueue::instance = new EventQueue();`（按现状 loop 已 process，应已存在；实现时确认）

**Python 侧（新增，目录 `remote_api/`）**
- `remote_api/splat_client.py` — TCP JSON-RPC 客户端（短连接版），含 `request(cmd, args)->dict`、自动 id 递增、超时、错误抛出
- `remote_api/models.py` — pydantic 请求模型（CameraPose、MouseEvent、KeyEvent、Transform、AnimateRequest 等）
- `remote_api/server.py` — FastAPI app，所有端点实现，启动入口 `uvicorn`
- `remote_api/keymap.py` — GLFW key 名→码（与 C++ 端一致）
- `remote_api/config.py` — 桥接端口/地址、HTTP 端口
- `remote_api/requirements.txt` — `fastapi`、`uvicorn`、`pydantic`
- `remote_api/examples/webrtc_receiver.py` — 浏览器/接收端调用示例（含信令外的控制调用时序）

**文档（新增）**
- `docs/remote_api.md` — 完整 API 调用文档：
  - 架构图与数据流
  - 协议约定（坐标系、单位、四元数格式、错误格式）
  - 每个端点：方法、路径、请求体 schema、响应、示例 curl
  - WebRTC 接收端集成说明（视频流由外部 pipeline 提供，本 API 仅负责输入控制；给出接收端在拿到视频流后如何按帧/按事件调用相机与鼠标端点的时序建议、节流/去抖策略、与本地预测的配合）
  - 启动顺序（先启 SplatEditor，再启 Python server，接收端连 HTTP）
  - 故障排查（桥接未连、主线程阻塞、坐标系翻转）

### WebRTC 说明（仅文档）
- 不在本期实现信令/视频流端点。文档将说明：视频流假定由现有 pipeline 采集渲染窗口（如 OBS/采集卡/独立 WebRTC 发送端）推送至浏览器；浏览器端在显示画面的同时调用本 HTTP API 回传输入。
- 文档给出推荐时序：低频相机姿态用 `POST /camera/pose` 或 `GET /camera/pose` 同步；高频鼠标移动用 `POST /mouse/move` 节流到 30–60Hz 并在客户端做惯性平滑；按键事件用 edge-trigger（press/release 各一次）。

### 实现顺序
1. C++ `RemoteControlServer`（socket 线程 + EventQueue promise/future + nlohmann json + 命令表），先实现 `/health` 与 `camera/pose` 两个端到端打通。
2. `main.cpp` 接线。
3. Python `splat_client` + `server.py` + `models.py`，实现全部端点。
4. `docs/remote_api.md` + 示例。
5. 本地联调（启 C++ 程序 → 启 uvicorn → curl/示例脚本验证相机与鼠标）。

### 约束/风险
- 现有 `OrbitControls::onMouseMove` 依赖 `mousePos` 差分，首次注入需先设 `mousePos`，桥接层会先初始化该字段避免跳变。
- `Runtime::getKeyAction(char)` 用 `glfwGetKeyName`，桥接注入键盘事件时需同时填 `frame_scancodes`（可填 0，名称匹配路径仍可用）。
- Windows 下 socket 需 `WSAStartup`；用条件编译处理。
- 不触碰 CUDA/GL 上下文：所有动作经 `schedule` 在主线程跑，避免跨线程上下文问题。