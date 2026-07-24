## 实施方案：本地浏览器测试控制页

在 `remote_api/server.py` 中内嵌一个自包含 HTML 测试页，新增 `GET /test` 路由返回。覆盖全部四类控制：

### 新增内容
1. **`GET /test` 路由** — 返回 `HTMLResponse`，HTML 作为字符串常量内嵌在 server.py
2. **HTML 页面功能**（原生 HTML/CSS/JS，无外部依赖）：
   - **状态条**：定时 `GET /health`（2s）显示 bridge 连通状态 + FPS + 窗口尺寸，红/绿指示灯
   - **鼠标 + 相机区**：占位画布捕获 `mousedown/move/up/wheel`，映射到 `/mouse/event` 与 `/mouse/scroll`；首次按下建 `mousePos` 基准避免跳变；移动时节流 60Hz；左键=轨道旋转、右键=平移
   - **键盘事件**：页面 `keydown/keyup` 边沿触发 → `/keyboard/key`；显示最近按键日志；阻止浏览器默认行为（避免空格滚动等）
   - **相机姿态面板**：`yaw/pitch/radius/target` 数字输入 + 「应用」(POST /camera/pose) + 「读取」(GET /camera/pose) + 轨道微调按钮（d_yaw/d_pitch 步进）
   - **场景节点**：点「刷新节点」拉 `/scene/nodes` 列表，每行显示 id/name/type；选中节点后可用「动画移动」表单（translation + duration + ease）发 `/motion/.../animate`
   - 顶部：API base（默认同 origin）+ 可选 token 输入，所有 fetch 自动带 `X-Splat-Token`

### 使用方式
```bash
# 1. 启 Splatshop（自动监听 7654）
# 2. 启 API：uvicorn remote_api.server:app --host 0.0.0.0 --port 8080
# 3. 浏览器打开 http://localhost:8080/test
```

### 验证
实施后用 mock bridge 跑一遍 `/test` 返回页 + 关键 API 端点的 HTTP 集成测试，确认页面可加载、各按钮的请求路径正确。

### 不改动的部分
- 不动 C++ 桥接（已验证可用）
- 不动其它端点
- 文档 `docs/remote_api.md` 末尾追加 `/test` 测试页说明与启动步骤