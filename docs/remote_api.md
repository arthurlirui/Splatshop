# Splatshop 远程控制 API

一套基于 Python (FastAPI) 的 HTTP API，将 Splatshop 渲染窗口的鼠标、键盘、相机、刚体运动与高斯点云创建/修改控制暴露给远程的 WebRTC 接收端，实现远程视角与相机运动及场景编辑。

> 目录
> - [概述](#概述)
> - [架构与数据流](#架构与数据流)
> - [启动顺序](#启动顺序)
> - [协议约定](#协议约定)
> - [认证](#认证)
> - [错误处理](#错误处理)
> - [API 端点参考](#api-端点参考)
>   - [系统](#系统)
>   - [相机](#相机)
>   - [鼠标](#鼠标)
>   - [键盘](#键盘)
>   - [场景与刚体运动](#场景与刚体运动)
>   - [场景 Splats 操作](#场景-splats-操作)
> - [WebRTC 接收端集成指南](#webrtc-接收端集成指南)
> - [环境变量配置](#环境变量配置)
> - [故障排查](#故障排查)

---

## 概述

Splatshop 是一个 C++/CUDA 的 3D 高斯泼溅（Gaussian Splatting）编辑器，渲染循环与 CUDA/GL 上下文绑定在主线程。本 API 在渲染进程之外运行一个 Python HTTP 服务，远程客户端（如浏览器中的 WebRTC 接收端）通过 HTTP 调用本 API；Python 服务再把命令通过本地 TCP 转发给 C++ 渲染进程，并在主线程上执行。

```
[WebRTC 接收端 / 浏览器] --HTTP--> [Python FastAPI :8080] --TCP JSON-RPC 127.0.0.1:7654--> [C++ SplatEditor 后台线程] --EventQueue--> [主线程渲染循环]
```

控制通道（本 API）与视频通道是分离的：
- **视频流**由独立的采集/推流管线提供（例如 OBS、采集卡、或单独的 WebRTC 发送端把 Splatshop 窗口画面推送到浏览器）。
- **本 API 仅负责输入回传**：浏览器在显示画面的同时，把用户的鼠标/键盘/相机操作通过 HTTP 发回，驱动远端 Splatshop 的视角。

---

## 架构与数据流

```
┌─────────────────────┐     HTTP/JSON      ┌──────────────────────┐    TCP 换行分隔 JSON    ┌──────────────────────────┐
│  WebRTC 接收端       │ ─────────────────▶ │  Python FastAPI       │ ─────────────────────▶ │  C++ RemoteControlServer │
│  (浏览器 JS / Python)│ ◀───────────────── │  remote_api/server.py │ ◀───────────────────── │  (后台 socket 线程)        │
└─────────────────────┘    请求/响应        └──────────────────────┘     请求/响应            └────────────┬─────────────┘
                                                                                                    │ schedule(lambda)
                                                                                                    ▼
                                                                                          ┌─────────────────────┐
                                                                                          │  主线程 GLFW 渲染循环  │
                                                                                          │  Runtime::controls   │
                                                                                          │  Runtime::keyStates  │
                                                                                          │  MotionController    │
                                                                                          └─────────────────────┘
```

**为什么需要转发到主线程**：CUDA 上下文与 GL 上下文绑定在创建它们的线程（主线程）。代码库已内置线程安全的 `EventQueue` / `schedule()`（`include/unsuck.hpp`），每帧在 `GLRenderer::loop()` 中 `process()` 排空。C++ 桥接用 `std::promise<json>`/`future` 实现同步请求/响应：socket 线程把命令 `schedule` 到主线程，阻塞等待主线程执行完回填结果，再写回响应。

**桥接协议**（换行分隔 JSON，UTF-8）：
```jsonc
// 请求
{"id": 1, "cmd": "camera.orbit", "args": {"yaw": -1.3, "pitch": -0.3}}
// 成功响应
{"id": 1, "ok": true, "data": {"yaw": -1.3, "pitch": -0.3}}
// 失败响应
{"id": 1, "ok": false, "error": "node not found: 42"}
```

---

## 启动顺序

1. **编译并启动 Splatshop**：渲染窗口打开后，`main()` 会自动在 `127.0.0.1:7654` 启动 C++ 桥接（见 `src/main.cpp` 中 `remote::RemoteControlServer::start(7654)`）。控制台会打印 `RemoteControlServer: listening on 127.0.0.1:7654`。
2. **安装并启动 Python 服务**：
   ```bash
   cd Splatshop
   pip install -r remote_api/requirements.txt
   uvicorn remote_api.server:app --host 0.0.0.0 --port 8080
   # 或：python -m remote_api.server
   ```
3. **验证连通**：`curl http://<server>:8080/health` 应返回 `{"bridge":"up","fps":...,"frame":...,"width":...,"height":...}`。
4. **WebRTC 接收端连接**：浏览器同时拉取视频流并调用本 API 回传输入（见 [WebRTC 接收端集成指南](#webrtc-接收端集成指南)）。

---

## 协议约定

| 约定 | 说明 |
|---|---|
| **坐标系** | 3D 世界坐标 = Splatshop 内部坐标（右手系，Z 向上，见 `OrbitControls`）。 |
| **鼠标坐标** | 窗口像素坐标，原点**左上**，Y 向下（浏览器/DOM 标准）。桥接层自动翻转为 app 内部坐标（`y' = height - y`）。 |
| **角度** | 一律**弧度**。相机 `yaw` 绕 Z 轴，`pitch` 绕 X 轴。 |
| **四元数** | 数组 `[x, y, z, w]`（与 motion Timeline JSON 一致，`src/motion/Timeline.cpp`）。 |
| **向量** | 3 元素数组 `[x, y, z]`。 |
| **鼠标按钮** | 字符串 `"left"` / `"right"` / `"middle"`。 |
| **动作** | 字符串 `"press"` / `"release"` / `"repeat"`。 |
| **键盘 key** | GLFW key 名（如 `"W"`、`"SPACE"`、`"LEFT_SHIFT"`，大小写不敏感），或直接传 GLFW 数值码。见 `remote_api/keymap.py` 与 C++ 端 `resolveKeyCode`。 |
| **mods 位掩码** | shift=1, ctrl=2, alt=4, super=8（GLFW 约定）。 |
| **响应体** | 成功 `200 {"ok":true,"data":{...}}`；失败见 [错误处理](#错误处理)。 |

---

## 认证

默认无认证，仅适用于本地/可信网络。设置环境变量 `SPLAT_API_TOKEN=<secret>` 启用共享密钥：所有请求须带 `X-Splat-Token: <secret>` 头，否则返回 `401`。详见 [环境变量配置](#环境变量配置)。

---

## 错误处理

| HTTP 状态 | 含义 | 触发条件 |
|---|---|---|
| `200` | 成功 | 正常返回 `{"ok":true,"data":...}` |
| `400` | 请求体非法 | pydantic 校验失败（字段缺失/类型错误） |
| `401` | 未认证 | token 不匹配 |
| `404` | 资源不存在 | 路径错误（FastAPI 默认） |
| `422` | 请求体可解析但校验失败 | pydantic 默认 |
| `502` | 桥接返回错误 | C++ 桥接执行命令失败（如 node 不存在、未知命令）。`{"detail":"node not found: 42"}` |
| `503` | 桥接不可达 | C++ 桥接未启动/连接失败。`{"detail":"cannot reach Splatshop bridge at 127.0.0.1:7654: ..."}` |

桥接内部对每个命令有 10 秒主线程看门狗：若主线程超过 10s 未处理（例如渲染卡死），返回 `{"ok":false,"error":"main thread did not respond within 10s"}`，Python 端转成 `502`。

---

## API 端点参考

所有端点路径见下表。`POST` 请求体为 JSON，响应为 `{"ok":true,"data":{...}}`（下文仅展示 `data` 内容以节省篇幅）。

### 系统

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/` | API 元信息 |
| GET | `/health` | ping C++ 桥接，返回 `{"bridge","fps","frame","width","height"}` |
| GET | `/test` | 本地浏览器测试控制页（HTML，见[本地浏览器测试页](#本地浏览器测试页get-test)） |

**`GET /health`** → `data`
```jsonc
{"bridge":"up","fps":60.0,"frame":12345,"width":1920,"height":1080}
```
```bash
curl http://localhost:8080/health
```

---

### 相机

相机模型为轨道相机（`OrbitControls`）：`yaw`/`pitch` 围绕 `target` 旋转，`radius` 控制距离。

| 方法 | 路径 | 请求体 | 说明 |
|---|---|---|---|
| POST | `/camera/orbit` | `{yaw?, pitch?, d_yaw?, d_pitch?}` | 绝对或增量设置旋转角（弧度） |
| POST | `/camera/pan` | `{dx, dy}` | 相机本地坐标系平移 target |
| POST | `/camera/zoom` | `{radius?}` 或 `{factor?}` | 绝对或乘性缩放 |
| POST | `/camera/pose` | `{yaw?, pitch?, radius?, target?}` | 整体设置姿态 |
| GET | `/camera/pose` | — | 读回当前姿态 + 位置 |
| POST | `/camera/focus` | `{node_id?}` 或 `{min,max,factor?}` | 聚焦到节点包围盒或显式 AABB |

**`POST /camera/orbit`** — 绝对设置或增量。绝对字段优先，`d_*` 累加。
```jsonc
// 请求
{"yaw": -1.325, "pitch": -0.330}
// 或增量
{"d_yaw": 0.05, "d_pitch": -0.02}
// data
{"yaw": -1.325, "pitch": -0.330}
```
```bash
curl -X POST http://localhost:8080/camera/orbit -H "Content-Type: application/json" -d '{"d_yaw":0.1}'
```

**`POST /camera/pan`** — 在相机本地坐标系平移 `target`（左/上为正），单位随 `radius` 缩放。
```jsonc
{"dx": -0.3, "dy": 0.2}
// data
{"target": [0.0, 0.0, 2.0]}
```

**`POST /camera/zoom`** — 二选一：`radius` 绝对值，或 `factor` 乘数（`factor<1` 拉近，`>1` 推远）。
```jsonc
{"factor": 0.9}
// data
{"radius": 4.22}
```

**`POST /camera/pose`** — 任意子集字段均可（缺省字段不变）。
```jsonc
{"yaw":-1.325,"pitch":-0.330,"radius":4.691,"target":[-0.028,-0.100,2.301]}
// data
{"yaw":-1.325,"pitch":-0.330,"radius":4.691,"target":[-0.028,-0.100,2.301]}
```

**`GET /camera/pose`** — 供 WebRTC 接收端同步视角。
```jsonc
{"yaw":-1.325,"pitch":-0.330,"radius":4.691,
 "target":[-0.028,-0.100,2.301],
 "position":[-3.879,12.585,-8.257]}
```
```bash
curl http://localhost:8080/camera/pose
```

**`POST /camera/focus`** — 聚焦到某节点（用 `/scene/nodes` 查 `node_id`）或显式 AABB。
```jsonc
{"node_id": 5, "factor": 1.2}
// 或
{"min":[-1,-1,-1],"max":[1,1,1],"factor":1.0}
// data
{"target":[0.0,0.0,0.0],"radius":3.46}
```

---

### VR 远程立体浏览

本组端点把一个**外部位姿源**（独立 VR 头显的浏览器 WebXR，或远端 SteamVR 客户端）接入渲染器的 `VIEWMODE_REMOTE_STEREO` 路径。头部位姿上行经控制通道（HTTP），渲染出的左右眼画面下行经**独立的视频通道**（WebSocket 帧服务器，默认 `:8081`），二者解耦。这是对“控制/视频分离”既有设计的扩展：远程浏览时首次内置了视频回传。

**坐标系约定**（关键，最易出错）。`pose_space` 字段决定服务端如何处理位姿：

| `pose_space` | 含义 | 服务端处理 |
|---|---|---|
| `"webxr"`（默认） | WebXR 视图矩阵（world→eye，+Y 上 / -Z 前，列主序） | 做基底变换 `B·view·B⁻¹` 转到应用 GL 空间（+Z 上 / +Y 前） |
| `"openvr"` | OpenVR 跟踪空间位姿（+Y 上 / +Z 前） | 套用与本地沉浸式 VR 相同的 `flip` 并求逆：`view = inverse(flip·head·eye)` |
| `"raw_view"` | 调用方已在应用 GL 空间（+Z 上 / +Y 前）提供最终视图矩阵 | 直接使用，不做任何变换 |

所有矩阵均为 16 元素扁平数组，**列主序**（与 `glm::value_ptr` / `make_mat4` 一致，WebXR `viewMatrix`/`projectionMatrix` 天然满足）。

| 方法 | 路径 | 请求体 | 说明 |
|---|---|---|---|
| POST | `/vr/enter` | - | 切换到 `VIEWMODE_REMOTE_STEREO`，停止本地 OpenVR（跳过 HMD 提交块） |
| POST | `/vr/exit` | - | 返回桌面模式，停止接收远程位姿 |
| POST | `/vr/pose` | 见下 | 高频位姿包（每帧一次），驱动左右眼视图/投影 |

**`POST /vr/enter`** - 进入远程立体模式。
```jsonc
// data
{"mode": "remote_stereo", "active": true}
```

**`POST /vr/pose`** - 高频位姿包。`webxr` 空间下提供 `view_left/right` + `proj_left/right`；`openvr` 空间下提供 `head_pose` + `eye_left/right` + 投影。`width/height` 设置每眼渲染目标分辨率。
```jsonc
// webxr 示例（省略 16 元素数组的具体数值）
{
  "pose_space": "webxr",
  "view_left":  [...16...], "view_right": [...16...],
  "proj_left":  [...16...], "proj_right": [...16...],
  "width": 2048, "height": 2048
}
// data
{"mode": "remote_stereo", "active": true, "width": 2048, "height": 2048}
```

**视频回传通道（WebSocket `:8081`）** - 浏览器连接 `ws://<host>:8081` 后，服务端按帧推送二进制消息，格式（小端序）：
```
u32 magic=0x56524653 | u16 sbsW | u16 sbsH | u16 eyeW | u16 eyeH | u16 codec | u32 len | u8 payload[len]
```
- `sbsW/sbsH`：左右并排合成图分辨率（`sbsW = 2·eyeW`）
- `codec`：`0`=JPEG（默认，LAN 验证用，无需外部依赖）；`1`=H.264（需 NVENC，见下）
- 客户端解码后，将左半图绘到左眼视口、右半图绘到右眼视口

**编码后端**：默认 JPEG（`stb_image_write`，无外部依赖，适合 LAN 跑通端到端链路）。H.264 硬编（NVENC）为预留路径，需 NVIDIA Video Codec SDK 头文件 `nvEncodeAPI.h` + `nvencode.lib`（不随 CUDA 工具包分发），通过编译宏 `SPLATSHOP_HAS_NVENC` 启用，未提供时静默回退到 JPEG。

**浏览器客户端**：`remote_api/examples/vr_webxr_client.html` 是开箱即用的 WebXR 客户端：读取 `getViewerPose().views` 的逐眼 `viewMatrix`/`projectionMatrix` 上行至 `/vr/pose`，同时接收 `:8081` 的 JPEG 帧并绘到 WebXR 层。需 HTTPS 或 localhost 提供（WebXR 安全上下文要求）。

---

### 鼠标

注入鼠标事件，驱动 `Runtime::controls` 与 `Runtime::mouseEvents`。坐标原点左上、Y 向下，桥接自动翻转。

| 方法 | 路径 | 请求体 | 说明 |
|---|---|---|---|
| POST | `/mouse/move` | `{x, y}` | 移动光标到 (x,y) 并喂给轨道相机 |
| POST | `/mouse/button` | `{button, action, mods?}` | 按下/释放按钮 |
| POST | `/mouse/scroll` | `{dx, dy}` | 滚轮（`dy>0` 拉近） |
| POST | `/mouse/event` | `{x?, y?, button?, action?, mods?, scroll_dx?, scroll_dy?}` | 复合事件，一次往返完成多步 |

**`POST /mouse/move`**
```jsonc
{"x": 960, "y": 540}
// data
{"x": 960, "y": 540}
```

**`POST /mouse/button`**
```jsonc
{"button":"left","action":"press","mods":0}
// data
{"button":0,"action":1,"mouseButtons":1}
```

**`POST /mouse/scroll`**
```jsonc
{"dx":0.0,"dy":1.0}
// data
{"dx":0.0,"dy":1.0,"radius":4.26}
```

**`POST /mouse/event`** — 高频拖拽推荐：一次往返里既移动又按下/释放，减少 RTT。
```jsonc
{"x":200,"y":300,"button":"left","action":"press"}
```

> 轨道相机依赖 `mousePos` 差分：首次 `/mouse/move` 应先发一次当前坐标建立基准，再发增量移动，避免首帧跳变。

---

### 键盘

注入键盘事件到 `Runtime::keyStates` 与 `Runtime::frame_keys/...`，使编辑器的快捷键系统（`Runtime::getKeyAction`）正常工作。

| 方法 | 路径 | 请求体 | 说明 |
|---|---|---|---|
| POST | `/keyboard/key` | `{key, action, mods?}` | 单个按键事件 |
| POST | `/keyboard/press` | `{key, duration_ms?, mods?}` | 便捷：按下→等待→释放 |
| POST | `/keyboard/sequence` | `{text}` | 逐字符 press/release（仅可打印字符） |

**`POST /keyboard/key`**
```jsonc
{"key":"W","action":"press","mods":0}
// data
{"key":87,"action":1}
```
```bash
curl -X POST http://localhost:8080/keyboard/key -H "Content-Type: application/json" -d '{"key":"SPACE","action":"press"}'
```

**`POST /keyboard/press`** — 适合触发一次快捷键。
```jsonc
{"key":"t","duration_ms":50}
```

**`POST /keyboard/sequence`** — 逐字符打入。
```jsonc
{"text":"hello"}
// data
{"results":[{"char":"h","ok":true},...]}
```

> 键名表见 `remote_api/keymap.py`（与 C++ `resolveKeyCode` 一致）。单字符 A–Z / 0–9 直接传字符；功能键用名称如 `"ENTER"`、`"ESC"`/`"ESCAPE"`、`"F1"`、`"LEFT_SHIFT"`、`"KP_0"`。

---

### 场景与刚体运动

刚体运动基于 `motion::MotionController`（`src/motion/MotionController.h`），目标是节点的局部 transform，下一帧由 `Scene::updateTransformations()` 传播。节点按 `SceneNode::ID`（int64）寻址，先用 `/scene/nodes` 查 id。

| 方法 | 路径 | 请求体 | 说明 |
|---|---|---|---|
| GET | `/scene/nodes` | — | 枚举场景树 `[{id,name,type}]` |
| GET | `/motion/node/{id}/transform` | — | 读 transform |
| POST | `/motion/node/{id}/transform` | `Transform{translation?, rotation?, scale?}` | 整体设置 |
| POST | `/motion/node/{id}/translate` | `{delta:[x,y,z]}` | 相对平移 |
| POST | `/motion/node/{id}/rotate` | `{delta:[x,y,z,w]}` | 本地原点预乘旋转 |
| POST | `/motion/node/{id}/scale` | `{factor:[x,y,z]}` | 乘性缩放 |
| POST | `/motion/node/{id}/animate` | `{target:Transform, duration_s?, ease?}` | 平滑过渡（TWEEN 驱动，非阻塞） |

**`GET /scene/nodes`**
```jsonc
{"nodes":[{"id":0,"name":"root","type":"SceneNode"},
          {"id":1,"name":"world","type":"SceneNode"},
          {"id":5,"name":"garden","type":"SNSplats"}]}
```

**`GET /motion/node/5/transform`**
```jsonc
{"translation":[0.0,0.0,0.0],
 "rotation":[0.0,0.0,0.0,1.0],
 "scale":[1.0,1.0,1.0]}
```

**`POST /motion/node/5/transform`** — 缺省字段保留原值。
```jsonc
{"translation":[10.0,0.0,0.0],"rotation":[0.0,1.0,0.0,0.0],"scale":[1.0,1.0,1.0]}
```

**`POST /motion/node/5/translate`**
```jsonc
{"delta":[1.0,2.0,3.0]}
```

**`POST /motion/node/5/rotate`** — 四元数 `[x,y,z,w]`，本地原点预乘。
```jsonc
{"delta":[0.0,1.0,0.0,0.0]}
```

**`POST /motion/node/5/scale`**
```jsonc
{"factor":[2.0,2.0,2.0]}
```

**`POST /motion/node/5/animate`** — 异步过渡，立即返回；过渡由 TWEEN 系统在主线程逐帧推进，无需客户端持续调用。
```jsonc
{"target":{"translation":[10,0,0],"rotation":[0,0,0,1],"scale":[1,1,1]},
 "duration_s":2.0,
 "ease":"in_out"}
// data
{"duration_s":2.0,"ease":"in_out"}
```
`ease` 取值：`"linear"` / `"in"` / `"out"` / `"in_out"`（默认 `in_out`）。

```bash
curl -X POST http://localhost:8080/motion/node/5/animate \
  -H "Content-Type: application/json" \
  -d '{"target":{"translation":[10,0,0]},"duration_s":1.5,"ease":"out"}'
```

---

### 场景 Splats 操作

在场景中**创建、加载、删除、修改**高斯点云。创建走 `Splats` host-side 构建（position/scale/rotation/color Buffer），包装成 `SNSplats` 节点附加到 `scene.world`，下一帧由 `uploadSplats()` 自动上传 GPU。删除移除节点及其子节点。设色直接写 GPU color buffer。

| 方法 | 路径 | 请求体 | 说明 |
|---|---|---|---|
| POST | `/scene/splats/create` | `{type, params}` | 从几何原语创建 splats |
| POST | `/scene/splats/load` | `{path}` | 从服务端磁盘加载 .ply 或 scene.json |
| DELETE | `/scene/node/{id}` | — | 删除节点 |
| POST | `/scene/splats/{id}/color` | `{color:[r,g,b,a]}` | 设所有 splats 颜色 |

**`POST /scene/splats/create`** — `type` 取值：

| type | params | 说明 |
|---|---|---|
| `"sphere"` | `{center?, radius?, count?, color?}` | 球面均匀分布（Fibonacci sphere）。默认 center=[0,0,0], radius=1, count=576 |
| `"box"` | `{min, max, count?, color?}` | 在 AABB 内随机均匀撒点。scale 自适应密度（cbrt(spacing)）|
| `"points"` | `{positions:[[x,y,z],...], scale?, color?}` | 逐点指定位置。scale 默认 0.02 |

```jsonc
// sphere
{"type":"sphere","params":{"center":[0,0,0],"radius":2,"count":500,"color":[0.2,0.6,1.0,1.0]}}
// data
{"id": 15, "name": "remote_sphere", "count": 500}

// box
{"type":"box","params":{"min":[0,0,0],"max":[3,3,3],"count":2000,"color":[1,0.5,0,1]}}

// points
{"type":"points","params":{"positions":[[0,0,0],[1,0,0],[2,0,0]],"scale":0.03,"color":[0,1,0,1]}}
```
```bash
curl -X POST http://localhost:8080/scene/splats/create \
  -H "Content-Type: application/json" \
  -d '{"type":"sphere","params":{"radius":1.5,"count":300,"color":[0.2,0.8,1,1]}}'
```

> color 值为 float [0,1]，桥接转为 uint16 [0,65535]。scale 为 world 单位（非 log-scale）。

**`POST /scene/splats/load`** — 路径相对于 Splatshop 工作目录：
```jsonc
{"path": "./splatmodels/scene.json"}
// 或
{"path": "E:/data/my_model.ply"}
// data
{"id": 16, "name": "my_model.ply", "count": 1234567}
```
```bash
curl -X POST http://localhost:8080/scene/splats/load \
  -H "Content-Type: application/json" -d '{"path":"./splatmodels/scene.json"}'
```

**`DELETE /scene/node/{id}`** — 删除节点及其所有子节点：
```bash
curl -X DELETE http://localhost:8080/scene/node/15
# data
{"id": 15}
```

**`POST /scene/splats/{id}/color`** — 把节点内所有 splats 颜色覆写为同一色（直接 cuMemcpyHtoD 到 GPU color buffer）：
```jsonc
{"color":[0.2,0.8,0.3,1.0]}
// data
{"id": 15, "count": 500, "color": [0.2,0.8,0.3,1.0]}
```
```bash
curl -X POST http://localhost:8080/scene/splats/15/color \
  -H "Content-Type: application/json" -d '{"color":[0.2,0.8,0.3,1]}'
```

> 设色仅修改 GPU 端的 `dmng.data.color`，不回溯 host 端 `Splats`（加载后 host 端已释放），且不影响 SHs。

---

### 点云 Bundle Adjustment 优化

通过可微高斯光栅化在 GPU 上精修点云的 **3D 位置 + 颜色**（范式 B，详见 `docs/ba_research.md`）。流程：把已加载的点云节点转换为 BA 节点 → 捕获当前视图作为光度目标 → 启动逐帧 AdamW 优化 → 优化后的位置/颜色实时写回渲染缓冲区。

底层 C++ 桥接命令（点号命名，经 Python `server.py` 暴露为 REST 端点需自行补充路由）：

| C++ 命令 | 作用 |
|---|---|
| `scene.points.ba.convert` | `{id}`：把一个 `SNPoints` 节点原地替换为 `SNPointCloudBA` 节点（复用 host `Points` 与 device 指针），返回新节点 id。 |
| `scene.points.ba.capture` | `{id}`：把当前渲染帧缓冲（`virt_framebuffer`）回读为 HxWx3 RGB，连同桌面相机内参（由 `fovy`+`aspect`+视口推导）与 `view` 矩阵设为优化目标。 |
| `scene.points.ba.start` | `{id, lr_position?, lr_color?, init_scale?, steps_per_frame?, max_steps?, optimize_position?, optimize_color?}`：按目标帧初始化优化器并开启逐帧优化。 |
| `scene.points.ba.stop` | `{id}`：停止优化（已精修的点云保留）。 |
| `scene.points.ba.reset` | `{id}`：丢弃优化状态。 |
| `scene.points.ba.status` | `{id}`：返回 `{initialized, running, step, loss, loss_l1, loss_ssim, point_count, target_w, target_h, max_steps}`。 |

调用示例（直接发 C++ 桥接 JSON）：

```bash
# 1. 转换点云节点 12 为 BA 节点
printf '{"id":1,"cmd":"scene.points.ba.convert","args":{"id":12}}\n' | nc 127.0.0.1 7654

# 2. 捕获当前视图为目标
printf '{"id":2,"cmd":"scene.points.ba.capture","args":{"id":13}}\n' | nc 127.0.0.1 7654

# 3. 启动优化（每帧 8 步，最多 2000 步）
printf '{"id":3,"cmd":"scene.points.ba.start","args":{"id":13,"steps_per_frame":8,"max_steps":2000}}\n' | nc 127.0.0.1 7654

# 4. 查询状态
printf '{"id":4,"cmd":"scene.points.ba.status","args":{"id":13}}\n' | nc 127.0.0.1 7654
```

> 优化在主线程逐帧推进（`SplatEditor_update.h` 的 `forEach<SNPointCloudBA>`），与渲染共用 CUDA 主上下文；libtorch 缺失时（`!SPLATSHOP_HAS_LIBTORCH`）相关命令返回错误且模块编译为 stub。

---

## 本地浏览器测试页（GET /test）

服务内置一个自包含的测试控制页，用于在**本地浏览器**中验证鼠标/键盘/相机/刚体控制链路，无需任何 WebRTC 视频流。页面是纯 HTML/CSS/JS（无外部依赖），从同 origin 调用 API，因此没有 CORS 问题。

### 启动

```bash
# 1. 启动 Splatshop（自动在 127.0.0.1:7654 启动 C++ 桥接）
# 2. 启动 Python API（同机）
conda activate splat-remote            # 或你的 venv
uvicorn remote_api.server:app --host 0.0.0.0 --port 8080
# 3. 浏览器打开
http://localhost:8080/test
```

> 端口 8080 必须放行；若设置了 `SPLAT_API_TOKEN`，在页面顶部 Token 框填入即可（页面 fetch 自动带 `X-Splat-Token`）。

### 页面功能

| 区域 | 功能 | 调用端点 |
|---|---|---|
| 状态条 | 每 2s 检测桥接，红/绿指示灯 + FPS + 窗口尺寸 | `GET /health` |
| 鼠标/相机拖拽 | 在画布区拖拽：左键=轨道旋转、右键=平移、滚轮=缩放。按下时先发一次 move 建基准避免首帧跳变，移动时节流 60Hz | `POST /mouse/event`、`/mouse/scroll` |
| 相机姿态面板 | `yaw/pitch/radius/target` 输入 + 应用/读取 + 轨道微调按钮 + 缩放按钮 | `POST /camera/pose`、`GET /camera/pose`、`/camera/orbit`、`/camera/zoom` |
| 键盘事件 | 页面任意处 `keydown/keyup` 边沿触发（去重防自动重复），失焦自动释放。另有「单次按键」输入框 | `POST /keyboard/key` |
| 场景节点 | 「刷新节点」列表，点击选中；选中后填 translation/duration/ease 发动画 | `GET /scene/nodes`、`POST /motion/node/{id}/animate` |
| 日志 | 实时显示请求与响应（含错误） | — |

### 使用要点

- **鼠标坐标映射**：画布坐标按比例缩放到 1920×1080 窗口像素（页面假设 Splatshop 窗口为该分辨率；若不同，拖拽仍有效但缩放比会偏）。
- **键盘焦点**：按键需焦点在页面（非输入框）。在输入框内打字不会触发控制，避免误操作；空格/方向键已阻止浏览器默认行为。
- **边沿触发**：键盘只发 press/release 各一次；鼠标按下建基准→移动发增量→松开发 release，符合 [WebRTC 集成指南](#webrtc-接收端集成指南) 的推荐时序。
- **节点动画**：先点「刷新节点」选中目标，再填 translation 点「动画移动」；若节点不存在会返回 502 `node not found`（场景重载后 id 会变，需重新刷新）。

### 不依赖视频流

此页用于**验证控制链路**，不显示 Splatshop 画面。要边看画面边控制，请直接观察本机 Splatshop 窗口，或参照 [WebRTC 接收端集成指南](#webrtc-接收端集成指南) 接入视频流。

---

## WebRTC 接收端集成指南

本 API 只负责输入控制，不提供视频流。WebRTC 接收端需自行获取视频流（窗口采集 + WebRTC 推送），并在显示画面的同时把用户输入回传到本 API。

### 拓扑

```
[Splatshop 窗口] --采集(OBS/采集卡/WebRTC sender)--> [WebRTC] --> [浏览器接收端]
                                                                        │  用户输入(鼠标/键盘/手势)
                                                                        ▼ HTTP
                                                              [Python FastAPI :8080]
                                                                        │
                                                                        ▼
                                                              [Splatshop 桥接/主线程]
```

### 推荐调用时序

| 输入类型 | 端点 | 频率/策略 |
|---|---|---|
| 相机姿态同步（一次性/低频） | `GET /camera/pose` | 连接时拉一次建立基准；切换场景后重拉 |
| 相机绝对定位（用户切换预设视角） | `POST /camera/pose` | 按需（事件触发） |
| 轨道旋转（拖拽） | `POST /mouse/event` 复合：press→move…→release | 节流 30–60Hz，客户端做惯性平滑 |
| 平移（右键拖拽） | `/mouse/event` 配合 `button:"right"` | 同上 |
| 缩放（滚轮/捏合） | `/mouse/scroll` | 边沿触发，按事件发送 |
| 单次快捷键 | `/keyboard/press` | 边沿触发，press+release 各一次 |
| 持续移动键（WASD） | `/keyboard/key` press / release | 仅在按下与松开时各发一次（边沿触发，**不要**轮询） |

### 性能与平滑建议

1. **节流高频输入**：浏览器 `mousemove` 可达 200+Hz。用一个 ~16ms（60Hz）的节流/合并循环，把累积的最后一次坐标用 `/mouse/event` 发出，避免淹没桥接。
2. **客户端预测**：视频流有 50–200ms 端到端延迟。为减少拖拽"滞后感"，可在客户端用最近一次 `GET /camera/pose` + 本地增量先做预测，待视频帧到达再校正；低频（如 2Hz）用 `GET /camera/pose` 校正漂移。
3. **边沿触发优先**：按键与按钮只在状态变化时发送（press/release），不要按帧轮询，否则会重复触发编辑器快捷键。
4. **连接复用**：浏览器侧用 `fetch` + `keepalive` 或 HTTP/2 连接复用，减少握手开销。Python 端对每个 HTTP 请求开一条到 C++ 的新 TCP 短连接（C++ 端每连接一个线程，开销低）。
5. **退避**：若 `/health` 返回 503 或超时，以指数退避重连，避免在桥接未就绪时刷请求。

### 浏览器侧调用示例（fetch）

```js
const API = "http://splatshop-host:8080";

async function orbit(dyaw, dpitch) {
  await fetch(`${API}/camera/orbit`, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({d_yaw: dyaw, d_pitch: dpitch}),
  });
}

async function drag(x, y, action) {
  // action: "press" | null | "release"
  const body = {x, y};
  if (action === "press")   { body.button = "left"; body.action = "press"; }
  if (action === "release") { body.button = "left"; body.action = "release"; }
  await fetch(`${API}/mouse/event`, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body),
  });
}
```

完整 Python 示例见 `remote_api/examples/webrtc_receiver.py`。

---

## 环境变量配置

所有配置可用环境变量覆盖（前缀 `SPLAT_`），定义在 `remote_api/config.py`。

| 变量 | 默认 | 说明 |
|---|---|---|
| `SPLAT_BRIDGE_HOST` | `127.0.0.1` | C++ 桥接地址 |
| `SPLAT_BRIDGE_PORT` | `7654` | C++ 桥接端口 |
| `SPLAT_BRIDGE_TIMEOUT` | `11.0` | 单请求 socket 超时（秒） |
| `SPLAT_HTTP_HOST` | `0.0.0.0` | HTTP 服务监听地址 |
| `SPLAT_HTTP_PORT` | `8080` | HTTP 服务端口 |
| `SPLAT_API_TOKEN` | `""` | 共享密钥（空=不鉴权） |

```bash
SPLAT_HTTP_PORT=9000 SPLAT_API_TOKEN=secret uvicorn remote_api.server:app --host 0.0.0.0
# 之后所有请求须带：-H "X-Splat-Token: secret"
```

---

## 故障排查

| 现象 | 可能原因与处理 |
|---|---|
| `/health` 返回 503 `cannot reach Splatshop bridge` | Splatshop 未启动，或桥接端口被占用。检查控制台是否有 `RemoteControlServer: listening on 127.0.0.1:7654`；确认 `SPLAT_BRIDGE_PORT` 一致。 |
| `/health` 503 但程序在运行 | 防火墙/端口转发问题。桥接仅监听 `127.0.0.1`，Python 必须与 Splatshop 同机，除非手动转发端口。 |
| 502 `node not found: N` | node_id 失效（场景重载后 id 会变）。重新 `GET /scene/nodes` 取最新 id。 |
| 502 `main thread did not respond within 10s` | 主线程卡死（如加载大场景）。等待恢复或重启。 |
| 鼠标拖拽首帧视角跳变 | 轨道相机用 `mousePos` 差分。先发一次 `/mouse/move` 当前坐标建立基准，再发增量。 |
| 旋转方向/Y 轴反 | 浏览器坐标 Y 向下、app 内部 Y 向上，桥接已做 `height - y` 翻转；若仍异常，确认传的是窗口内像素坐标而非屏幕坐标。 |
| 快捷键重复触发 | 改为边沿触发：只在 keydown/keyup 调用 `/keyboard/key`，不要轮询。 |
| 远程浏览器无法访问 | HTTP 默认 `0.0.0.0` 监听，检查防火墙放行 8080；生产环境务必设置 `SPLAT_API_TOKEN`。 |

---

## 相关文件

- C++ 桥接：`src/remote/RemoteControlServer.h` / `.cpp`，`src/main.cpp`
- Python API：`remote_api/`（`server.py`、`splat_client.py`、`models.py`、`keymap.py`、`config.py`）
- 示例：`remote_api/examples/webrtc_receiver.py`
- VR 远程浏览客户端：`remote_api/examples/vr_webxr_client.html`
- VR 视频回传：`src/remote/FrameStreamer.h`、`src/remote/FrameStreamer.cpp`（WebSocket 帧服务器 + JPEG 编码）
- VR 外部位姿路径：`src/common.h`（`VIEWMODE_REMOTE_STEREO`、`PoseSpace`）、`src/SplatEditor.h`（`RemoteStereoState`）、`src/SplatEditor_update.h`（位姿装配）、`src/SplatEditor_render.h`（双目渲染 + 帧捕获）
- 底层：`include/OrbitControls.h`、`include/Runtime.h`、`include/MouseEvents.h`、`src/motion/MotionController.h`、`include/unsuck.hpp`（`EventQueue`）
- Splats 创建：`include/Splats.h`、`src/scene/SNSplats.h`、`src/scene/SceneNode.h`、`src/loader/GSPlyLoader.h`、`src/SplatsManagement.h`（`GaussianDataManager`）
- 点云 BA 优化：`src/optim/PointCloudBA.h`/`.cpp`（libtorch 可微高斯光栅化 + AdamW）、`src/scene/SNPointCloudBA.h`（场景节点）、`src/gui/pointcloud_ba.h`（GUI 面板）、`docs/ba_research.md`（算法调研）
