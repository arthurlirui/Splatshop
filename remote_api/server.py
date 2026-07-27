"""Splatshop remote control HTTP API (FastAPI).

Exposes mouse / keyboard / camera / rigid-motion control of a running
Splatshop instance to remote WebRTC receiving clients. This server talks to
the C++ SplatEditor process over a local TCP JSON-RPC bridge
(remote_api/splat_client.py -> src/remote/RemoteControlServer.cpp).

Run with:
    uvicorn remote_api.server:app --host 0.0.0.0 --port 8080

See docs/remote_api.md for the full endpoint reference and the WebRTC receiver
integration guide.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from fastapi import Depends, FastAPI, Header, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse

from . import config, splat_client
from . import models as M

# --------------------------------------------------------------------------- #
# App setup
# --------------------------------------------------------------------------- #
app = FastAPI(
    title="Splatshop Remote Control API",
    description=(
        "Control a running Splatshop Gaussian-splatting viewer's mouse, "
        "keyboard, camera and rigid-object motion over HTTP. Intended for "
        "remote WebRTC receiving clients to drive the viewpoint."
    ),
    version="0.1.0",
)

# Allow browser-based WebRTC receivers to call us cross-origin.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# --------------------------------------------------------------------------- #
# Auth dependency (optional shared-secret token)
# --------------------------------------------------------------------------- #
def _check_token(x_splat_token: Optional[str] = Header(default=None)) -> None:
    if not config.API_TOKEN:
        return
    if x_splat_token != config.API_TOKEN:
        raise HTTPException(status.HTTP_401_UNAUTHORIZED, "invalid or missing token")


# Bridge errors -> HTTP 502 (bad gateway); unavailable -> 503.
def _bridge_error_to_http(e: Exception) -> HTTPException:
    if isinstance(e, splat_client.BridgeUnavailable):
        return HTTPException(status.HTTP_503_SERVICE_UNAVAILABLE, str(e))
    return HTTPException(status.HTTP_502_BAD_GATEWAY, str(e))


def _call(cmd: str, args: Optional[Dict[str, Any]] = None,
          token_ok: None = None) -> Dict[str, Any]:
    try:
        return splat_client.request(cmd, args)
    except (splat_client.BridgeError, splat_client.BridgeUnavailable) as e:
        raise _bridge_error_to_http(e) from e


# --------------------------------------------------------------------------- #
# System
# --------------------------------------------------------------------------- #
@app.get("/", dependencies=[Depends(_check_token)])
def api_info():
    return {
        "name": "Splatshop Remote Control API",
        "version": app.version,
        "bridge": {"host": config.BRIDGE_HOST, "port": config.BRIDGE_PORT},
        "docs": "/docs",
    }


@app.get("/health", dependencies=[Depends(_check_token)])
def health():
    """Ping the C++ bridge; returns fps / frame / window size when up."""
    return _call("health", token_ok=None)


# --------------------------------------------------------------------------- #
# Browser test page (GET /test)
# --------------------------------------------------------------------------- #
# A self-contained HTML/CSS/JS page for testing mouse / keyboard / camera /
# motion control from a local browser. No external dependencies. The page calls
# the same-origin API, so there are no CORS issues. Open http://localhost:8080/test
# once the server (and a running Splatshop with the C++ bridge) is up.
_TEST_PAGE = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Splatshop 远程控制测试台</title>
<style>
  :root { --bg:#1e1e2e; --panel:#2a2a3c; --fg:#e0e0ef; --muted:#9a9ab0;
          --accent:#7aa2f7; --ok:#9ece6a; --err:#f7768e; --border:#3b3b52; }
  * { box-sizing:border-box; }
  body { margin:0; font-family:system-ui,Segoe UI,sans-serif; background:var(--bg);
         color:var(--fg); font-size:14px; }
  header { padding:12px 16px; background:var(--panel); border-bottom:1px solid var(--border);
           display:flex; gap:16px; align-items:center; flex-wrap:wrap; }
  header h1 { margin:0; font-size:16px; font-weight:600; }
  .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
  label { color:var(--muted); }
  input[type=text], input[type=number] { background:#15151f; border:1px solid var(--border);
         color:var(--fg); padding:4px 6px; border-radius:4px; width:90px; font-size:13px; }
  input.wide { width:200px; }
  button { background:var(--accent); color:#0d0d16; border:0; padding:5px 10px; border-radius:4px;
           cursor:pointer; font-size:13px; font-weight:600; }
  button:hover { filter:brightness(1.1); }
  button.ghost { background:transparent; color:var(--accent); border:1px solid var(--accent); }
  .grid { display:grid; grid-template-columns:1.1fr 1fr; gap:12px; padding:12px; }
  @media (max-width:900px){ .grid{ grid-template-columns:1fr; } }
  .panel { background:var(--panel); border:1px solid var(--border); border-radius:8px; padding:12px; }
  .panel h2 { margin:0 0 8px; font-size:14px; color:var(--accent); }
  .status { display:flex; gap:10px; align-items:center; font-size:13px; }
  .dot { width:10px; height:10px; border-radius:50%; background:var(--err); }
  .dot.ok { background:var(--ok); }
  #mouseArea { width:100%; height:300px; background:#0d0d16; border:1px dashed var(--border);
               border-radius:6px; position:relative; cursor:crosshair; user-select:none;
               touch-action:none; }
  #mouseArea .hint { position:absolute; top:50%; left:50%; transform:translate(-50%,-50%);
               color:var(--muted); pointer-events:none; font-size:13px; text-align:center; }
  .log { background:#0d0d16; border:1px solid var(--border); border-radius:6px; padding:6px;
         height:120px; overflow:auto; font-family:ui-monospace,Consolas,monospace; font-size:12px; }
  .log .line { white-space:pre-wrap; }
  .log .err { color:var(--err); }
  .log .ok { color:var(--ok); }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  th,td { text-align:left; padding:3px 6px; border-bottom:1px solid var(--border); }
  th { color:var(--muted); font-weight:500; }
  tr.sel { background:#2f2f4a; cursor:pointer; }
  tr:hover { background:#26264a; cursor:pointer; }
  .field { margin-bottom:6px; }
  .small { font-size:12px; color:var(--muted); }
</style>
</head>
<body>
<header>
  <h1>Splatshop 远程控制测试台</h1>
  <div class="row">
    <label>API base</label>
    <input id="base" class="wide" type="text" value="">
    <label>Token</label>
    <input id="token" type="text" placeholder="(无)">
  </div>
  <div class="status">
    <span id="dot" class="dot"></span>
    <span id="stat">连接中…</span>
  </div>
</header>

<div class="grid">
  <!-- 左列：鼠标 + 相机 -->
  <div>
    <div class="panel">
      <h2>鼠标 / 相机拖拽</h2>
      <div id="mouseArea"><div class="hint">在此区域拖拽：左键=轨道旋转<br>右键=平移　滚轮=缩放</div></div>
      <div class="small">坐标为窗口像素，左上原点。已自动 Y 翻转并发到 /mouse/event。</div>
    </div>

    <div class="panel" style="margin-top:12px">
      <h2>相机姿态</h2>
      <div class="row">
        <div class="field"><label>yaw</label><input id="yaw" type="number" step="0.05"></div>
        <div class="field"><label>pitch</label><input id="pitch" type="number" step="0.05"></div>
        <div class="field"><label>radius</label><input id="radius" type="number" step="0.1"></div>
      </div>
      <div class="field"><label>target [x,y,z]</label>
        <input id="tx" type="number" step="0.1"> <input id="ty" type="number" step="0.1"> <input id="tz" type="number" step="0.1">
      </div>
      <div class="row">
        <button onclick="applyPose()">应用</button>
        <button class="ghost" onclick="readPose()">读取当前</button>
        <button onclick="orbit(-0.1,0)">←转</button>
        <button onclick="orbit(0.1,0)">转→</button>
        <button onclick="orbit(0,-0.1)">↑仰</button>
        <button onclick="orbit(0,0.1)">↓俯</button>
        <button class="ghost" onclick="zoom(0.9)">拉近</button>
        <button class="ghost" onclick="zoom(1.1)">推远</button>
      </div>
    </div>
  </div>

  <!-- 右列：键盘 + 节点 + 日志 -->
  <div>
    <div class="panel">
      <h2>键盘事件</h2>
      <div class="small">在页面任意处按键即可下发（边沿触发：press / release 各一次）。焦点需在此页。</div>
      <div class="row" style="margin-top:6px">
        <label>单次按键</label>
        <input id="singleKey" type="text" placeholder="如 W 或 SPACE" style="width:120px">
        <button onclick="pressOnce()">下发</button>
      </div>
	    </div>

	    <div class="panel" style="margin-top:12px">
	      <h2>场景 Splats 创建</h2>
	      <div class="row" style="margin-top:6px">
	        <label>类型</label>
	        <select id="createType" style="background:#15151f;color:var(--fg);border:1px solid var(--border);padding:4px;border-radius:4px">
	          <option>sphere</option><option>box</option><option>points</option>
	        </select>
	        <label>数量</label><input id="createCount" type="number" value="576" style="width:70px">
	        <label>颜色</label><input id="createColor" type="text" value="1,0,0,1" style="width:100px" placeholder="r,g,b,a">
	        <button onclick="createSplats()">创建</button>
	      </div>
	      <div id="sphereOpts" class="row" style="margin-top:4px">
	        <label>中心</label><input id="scx" type="number" step="0.5" value="0"> <input id="scy" type="number" step="0.5" value="0"> <input id="scz" type="number" step="0.5" value="0">
	        <label>半径</label><input id="sradius" type="number" step="0.1" value="1">
	      </div>
	      <div id="boxOpts" class="row" style="margin-top:4px; display:none">
	        <label>min</label><input id="bx0" type="number" step="0.5" value="0"><input id="by0" type="number" step="0.5" value="0"><input id="bz0" type="number" step="0.5" value="0">
	        <label>max</label><input id="bx1" type="number" step="0.5" value="2"><input id="by1" type="number" step="0.5" value="2"><input id="bz1" type="number" step="0.5" value="2">
	      </div>
	      <div id="pointsOpts" class="row" style="margin-top:4px; display:none">
	        <label>scale</label><input id="ptScale" type="number" step="0.01" value="0.02">
	      </div>
	      <div class="row" style="margin-top:6px">
	        <label>或加载文件</label>
	        <input id="loadPath" type="text" placeholder="./splatmodels/scene.json" style="width:200px">
	        <button onclick="loadFile()">加载</button>
	      </div>
	    </div>

	    <div class="panel" style="margin-top:12px">
	      <h2>场景节点 / 刚体动画</h2>
      <div class="row">
        <button onclick="refreshNodes()">刷新节点</button>
        <span id="selInfo" class="small">未选中</span>
      </div>
      <div style="max-height:160px; overflow:auto; margin-top:6px">
	        <table id="nodesTable"><thead><tr><th>id</th><th>name</th><th>type</th><th>操作</th></tr></thead><tbody></tbody></table>
      </div>
      <div class="field" style="margin-top:8px"><label>动画目标 translation [x,y,z]</label>
        <input id="ax" type="number" step="0.5" value="1"> <input id="ay" type="number" step="0.5" value="0"> <input id="az" type="number" step="0.5" value="0">
      </div>
      <div class="field"><label>duration_s</label><input id="dur" type="number" step="0.1" value="1.5" style="width:70px">
        <label>ease</label>
        <select id="ease" style="background:#15151f;color:var(--fg);border:1px solid var(--border);padding:4px;border-radius:4px">
          <option>in_out</option><option>linear</option><option>in</option><option>out</option>
        </select>
        <button onclick="animateNode()">动画移动</button>
      </div>
    </div>

    <div class="panel" style="margin-top:12px">
      <h2>日志</h2>
      <div id="log" class="log"></div>
    </div>
  </div>
</div>

<script>
// ---- 配置 ----
const baseEl = document.getElementById('base');
baseEl.value = location.origin;            // 默认同 origin
function api(p){ return baseEl.value.replace(/\/$/,'') + p; }
function headers(){ const h={'Content-Type':'application/json'}; const t=document.getElementById('token').value.trim();
  if(t) h['X-Splat-Token']=t; return h; }

// ---- 日志 ----
const logEl = document.getElementById('log');
function log(msg, kind){ const d=document.createElement('div'); d.className='line'+(kind?' '+kind:'');
  const ts=new Date().toLocaleTimeString(); d.textContent=`[${ts}] ${msg}`; logEl.appendChild(d);
  logEl.scrollTop=logEl.scrollHeight; if(logEl.children.length>200) logEl.removeChild(logEl.firstChild); }

// ---- 通用请求 ----
async function call(method, path, body){
  try{
    const opt={method, headers:headers()};
    if(body) opt.body=JSON.stringify(body);
    const r=await fetch(api(path), opt);
    const j=await r.json();
    if(!r.ok){ log(`${method} ${path} -> ${r.status} ${JSON.stringify(j)}`, 'err'); return null; }
    return j;
  }catch(e){ log(`${method} ${path} 异常: ${e}`, 'err'); return null; }
}
async function post(path, body){ return call('POST', path, body); }
async function get(path){ return call('GET', path); }

// ---- 健康检查 ----
const dot=document.getElementById('dot'), stat=document.getElementById('stat');
async function checkHealth(){
  const j=await get('/health');
  if(j && j.bridge==='up'){ dot.classList.add('ok');
    stat.textContent=`bridge up · fps ${j.fps.toFixed?.(1) ?? j.fps} · ${j.width}x${j.height}`; }
  else { dot.classList.remove('ok'); stat.textContent='bridge 不可达'; }
}
checkHealth(); setInterval(checkHealth, 2000);

// ---- 鼠标区 ----
const area=document.getElementById('mouseArea');
let dragging=false, btnName=null, primed=false, lastSent=0;
const BTN={0:'left', 1:'middle', 2:'right'};
function relPos(e){ const r=area.getBoundingClientRect();
  // 画布坐标 → 窗口像素坐标。简单按比例映射到常见 1920x1080。
  const sx=Math.round((e.clientX-r.left)/r.width*1920);
  const sy=Math.round((e.clientY-r.top)/r.height*1080); return {x:sx, y:sy}; }

area.addEventListener('contextmenu', e=>e.preventDefault());
area.addEventListener('pointerdown', e=>{
  e.preventDefault(); area.setPointerCapture(e.pointerId);
  dragging=true; btnName=BTN[e.button]||'left'; primed=false;
  const p=relPos(e);
  // 先发一次 move 建基准，再发 press，避免 OrbitControls 首帧跳变
  post('/mouse/move', p);
  post('/mouse/event', {x:p.x, y:p.y, button:btnName, action:'press'});
  log(`mouse press ${btnName} @ ${p.x},${p.y}`);
});
area.addEventListener('pointermove', e=>{
  if(!dragging) return;
  const now=performance.now();
  if(now-lastSent < 1000/60) return;   // 60Hz 节流
  lastSent=now; const p=relPos(e);
  post('/mouse/event', {x:p.x, y:p.y});
});
area.addEventListener('pointerup', e=>{
  if(!dragging) return; dragging=false;
  const p=relPos(e);
  post('/mouse/event', {x:p.x, y:p.y, button:btnName, action:'release'});
  log(`mouse release ${btnName} @ ${p.x},${p.y}`);
});
area.addEventListener('pointerleave', ()=>{
  if(dragging){ post('/mouse/event', {button:btnName, action:'release'}); dragging=false; }
});
area.addEventListener('wheel', e=>{
  e.preventDefault();
  post('/mouse/scroll', {dx:0, dy: e.deltaY<0?1:-1});
}, {passive:false});

// ---- 键盘 ----
const GLFW_NAMES={' ':'SPACE','Escape':'ESCAPE','Enter':'ENTER','Tab':'TAB',
  'Backspace':'BACKSPACE','Insert':'INSERT','Delete':'DELETE',
  'ArrowLeft':'LEFT','ArrowRight':'RIGHT','ArrowUp':'UP','ArrowDown':'DOWN',
  'PageUp':'PAGE_UP','PageDown':'PAGE_DOWN','Home':'HOME','End':'END',
  'Shift':'LEFT_SHIFT','Control':'LEFT_CONTROL','Alt':'LEFT_ALT','Meta':'LEFT_SUPER'};
const sentKeys=new Set();
function keyName(e){
  if(e.key.length===1) return e.key.toUpperCase();   // A-Z 0-9
  return GLFW_NAMES[e.key] || e.key;
}
window.addEventListener('keydown', e=>{
  // 忽略在输入框内按键，避免干扰打字
  const t=e.target.tagName; if(t==='INPUT'||t==='SELECT'||t==='TEXTAREA') return;
  if([' ','ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Tab'].includes(e.key)) e.preventDefault();
  const name=keyName(e);
  if(sentKeys.has(name)) return;            // 边沿去重，避免 keydown 自动重复
  sentKeys.add(name);
  post('/keyboard/key', {key:name, action:'press', mods:0});
  log(`key press ${name}`);
});
window.addEventListener('keyup', e=>{
  const t=e.target.tagName; if(t==='INPUT'||t==='SELECT'||t==='TEXTAREA') return;
  const name=keyName(e); sentKeys.delete(name);
  post('/keyboard/key', {key:name, action:'release', mods:0});
  log(`key release ${name}`);
});
window.addEventListener('blur', ()=>{ // 失焦时释放所有按下的键
  sentKeys.forEach(name=>post('/keyboard/key',{key:name,action:'release',mods:0}));
  sentKeys.clear();
});

async function pressOnce(){
  const k=document.getElementById('singleKey').value.trim(); if(!k) return;
  await post('/keyboard/key', {key:k, action:'press', mods:0});
  await new Promise(r=>setTimeout(r,60));
  await post('/keyboard/key', {key:k, action:'release', mods:0});
  log(`press once ${k}`);
}

// ---- 相机姿态 ----
async function applyPose(){
  const body={};
  const f=id=>{ const v=document.getElementById(id).value; if(v!=='') body[id]=parseFloat(v); };
  f('yaw'); f('pitch'); f('radius');
  if(document.getElementById('tx').value!==''){ body.target=[+tx.value,+ty.value,+tz.value]; }
  const j=await post('/camera/pose', body); if(j) log('pose applied '+JSON.stringify(body));
}
async function readPose(){
  const j=await get('/camera/pose'); if(!j) return;
  yaw.value=j.yaw.toFixed(3); pitch.value=j.pitch.toFixed(3); radius.value=j.radius.toFixed(3);
  tx.value=j.target[0].toFixed(3); ty.value=j.target[1].toFixed(3); tz.value=j.target[2].toFixed(3);
  log('pose read '+JSON.stringify(j));
}
async function orbit(dy,dp){ const j=await post('/camera/orbit',{d_yaw:dy,d_pitch:dp}); if(j) log(`orbit d_yaw=${dy} d_pitch=${dp}`); }
async function zoom(f){ const j=await post('/camera/zoom',{factor:f}); if(j) log(`zoom factor=${f}`); }

// ---- 节点 ----
let selectedId=null;
async function refreshNodes(){
  const j=await get('/scene/nodes'); if(!j) return;
  const tb=document.querySelector('#nodesTable tbody'); tb.innerHTML='';
  j.nodes.forEach(n=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${n.id}</td><td>${n.name}</td><td>${n.type}</td>
      <td><button style="font-size:11px;padding:2px 5px;margin-right:2px" onclick="event.stopPropagation();removeNode(${n.id})">删除</button>
      <button style="font-size:11px;padding:2px 5px" onclick="event.stopPropagation();setNodeColor(${n.id})">设色</button></td>`;
    tr.onclick=()=>{ selectedId=n.id;
      document.querySelectorAll('#nodesTable tr').forEach(r=>r.classList.remove('sel'));
      tr.classList.add('sel'); document.getElementById('selInfo').textContent=`选中 #${n.id} ${n.name}`; };
    tb.appendChild(tr);
  });
  log(`nodes: ${j.nodes.length}`);
}
async function animateNode(){
  if(selectedId==null){ log('请先选中节点','err'); return; }
  const body={target:{translation:[+ax.value,+ay.value,+az.value]},
              duration_s:+dur.value, ease:ease.value};
  const j=await post(`/motion/node/${selectedId}/animate`, body);
  if(j) log(`animate #${selectedId} -> ${JSON.stringify(body.target)}`);
}

// ---- 场景 Splats 创建 / 加载 / 删除 / 设色 ----
document.getElementById('createType').onchange=function(){
  const t=this.value; document.getElementById('sphereOpts').style.display=t==='sphere'?'flex':'none';
  document.getElementById('boxOpts').style.display=t==='box'?'flex':'none';
  document.getElementById('pointsOpts').style.display=t==='points'?'flex':'none';
};
function parseColorStr(s){ return s.split(',').map(parseFloat); }
async function createSplats(){
  const type=createType.value; let params={}; const cStr=document.getElementById('createColor').value;
  params.color=parseColorStr(cStr); const count=+createCount.value;
  if(type==='sphere'){ params.center=[+scx.value,+scy.value,+scz.value]; params.radius=+sradius.value; params.count=count; }
  else if(type==='box'){ params.min=[+bx0.value,+by0.value,+bz0.value]; params.max=[+bx1.value,+by1.value,+bz1.value]; params.count=count; }
  else if(type==='points'){ params.scale=+ptScale.value; params.count=count; params.positions=[]; for(let i=0;i<count;i++){ params.positions.push([Math.random()*4-2,Math.random()*4-2,Math.random()*4-2]); } }
  const j=await post('/scene/splats/create', {type, params});
  if(j){ log(`created ${type} node #${j.id} "${j.name}" x${j.count}`); refreshNodes(); }
}
async function loadFile(){
  const p=loadPath.value.trim(); if(!p) return;
  const j=await post('/scene/splats/load', {path:p}); if(j) { log('loaded '+p); refreshNodes(); }
}
async function removeNode(id){
  if(!confirm('删除节点 #'+id+'？')) return;
  const j=await call('DELETE','/scene/node/'+id);
  if(j){ log('removed #'+id); refreshNodes(); }
}
async function setNodeColor(id){
  const c=prompt('新颜色 r,g,b,a (0~1)','1,0,0,1'); if(!c) return;
  const rgba=parseColorStr(c); const j=await post(`/scene/splats/${id}/color`, {color:rgba});
  if(j) log(`set color #${id} to ${JSON.stringify(rgba)}`);
}

log('测试台就绪。先确认 Splatshop 与 C++ 桥接已启动。');
</script>
</body>
</html>
"""


@app.get("/test", response_class=HTMLResponse)
def test_page():
    """Self-contained browser test page for mouse/keyboard/camera/motion control.

    No auth required so it works out-of-the-box in a local browser; the page's
    own fetch calls honor the optional token input. Open http://localhost:8080/test.
    """
    return _TEST_PAGE


# --------------------------------------------------------------------------- #
# Camera
# --------------------------------------------------------------------------- #
@app.post("/camera/orbit", dependencies=[Depends(_check_token)])
def camera_orbit(req: M.OrbitRequest):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("camera.orbit", args)


@app.post("/camera/pan", dependencies=[Depends(_check_token)])
def camera_pan(req: M.PanRequest):
    return _call("camera.pan", req.model_dump())


@app.post("/camera/zoom", dependencies=[Depends(_check_token)])
def camera_zoom(req: M.ZoomRequest):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("camera.zoom", args)


@app.post("/camera/pose", dependencies=[Depends(_check_token)])
def camera_pose_set(req: M.CameraPose):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("camera.pose.set", args)


@app.get("/camera/pose", dependencies=[Depends(_check_token)])
def camera_pose_get():
    return _call("camera.pose.get")


@app.post("/camera/focus", dependencies=[Depends(_check_token)])
def camera_focus(req: M.FocusRequest):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("camera.focus", args)


# --------------------------------------------------------------------------- #
# Mouse
# --------------------------------------------------------------------------- #
@app.post("/mouse/move", dependencies=[Depends(_check_token)])
def mouse_move(req: M.MouseMoveRequest):
    return _call("mouse.move", req.model_dump())


@app.post("/mouse/button", dependencies=[Depends(_check_token)])
def mouse_button(req: M.MouseButtonRequest):
    return _call("mouse.button", req.model_dump())


@app.post("/mouse/scroll", dependencies=[Depends(_check_token)])
def mouse_scroll(req: M.MouseScrollRequest):
    return _call("mouse.scroll", req.model_dump())


@app.post("/mouse/event", dependencies=[Depends(_check_token)])
def mouse_event(req: M.MouseEventRequest):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    return _call("mouse.event", args)


# --------------------------------------------------------------------------- #
# Keyboard
# --------------------------------------------------------------------------- #
@app.post("/keyboard/key", dependencies=[Depends(_check_token)])
def keyboard_key(req: M.KeyRequest):
    return _call("keyboard.key", req.model_dump())


@app.post("/keyboard/press", dependencies=[Depends(_check_token)])
def keyboard_press(req: M.KeyPressRequest):
    """Press then release a key, sleeping between the two events."""
    splat_client.request("keyboard.key",
                         {"key": req.key, "action": "press", "mods": req.mods or 0})
    time.sleep(max(0.0, (req.duration_ms or 0) / 1000.0))
    try:
        return splat_client.request("keyboard.key",
                                    {"key": req.key, "action": "release", "mods": req.mods or 0})
    except (splat_client.BridgeError, splat_client.BridgeUnavailable) as e:
        raise _bridge_error_to_http(e) from e


@app.post("/keyboard/sequence", dependencies=[Depends(_check_token)])
def keyboard_sequence(req: M.KeySequenceRequest):
    results: List[Dict[str, Any]] = []
    for ch in req.text:
        try:
            splat_client.request("keyboard.key", {"key": ch, "action": "press", "mods": 0})
            splat_client.request("keyboard.key", {"key": ch, "action": "release", "mods": 0})
            results.append({"char": ch, "ok": True})
        except (splat_client.BridgeError, splat_client.BridgeUnavailable) as e:
            results.append({"char": ch, "ok": False, "error": str(e)})
    return {"results": results}


# --------------------------------------------------------------------------- #
# Scene / Motion (rigid body)
# --------------------------------------------------------------------------- #
@app.get("/scene/nodes", dependencies=[Depends(_check_token)])
def scene_nodes():
    return _call("scene.nodes")


@app.post("/scene/splats/create", dependencies=[Depends(_check_token)])
def scene_splats_create(req: M.SplatsCreateRequest):
    """Create a new Gaussian splats cloud from geometry primitives.

    type: "sphere" | "box" | "points". params depend on type (see models.py).
    Returns the new node's id, name, and splat count.
    """
    cmd_map = {"sphere": "scene.splats.create_sphere",
               "box":    "scene.splats.create_box",
               "points": "scene.splats.create_points"}
    cmd = cmd_map.get(req.type)
    if not cmd:
        raise HTTPException(status.HTTP_400_BAD_REQUEST,
                            f"unknown type '{req.type}'; use sphere, box, or points")
    return _call(cmd, req.params)


@app.post("/scene/splats/load", dependencies=[Depends(_check_token)])
def scene_splats_load(req: M.LoadFileRequest):
    """Load splats from a .ply or scene.json file on the server's disk."""
    return _call("scene.splats.load_file", {"path": req.path})


@app.delete("/scene/node/{node_id}", dependencies=[Depends(_check_token)])
def scene_node_remove(node_id: int):
    """Remove a scene node by ID."""
    return _call("scene.node.remove", {"id": node_id})


@app.post("/scene/splats/{node_id}/color", dependencies=[Depends(_check_token)])
def scene_splats_set_color(node_id: int, req: M.SetColorRequest):
    """Overwrite the color of every splat in the given node."""
    return _call("scene.splats.set_color", {"id": node_id, "color": req.color})


@app.get("/motion/node/{node_id}/transform", dependencies=[Depends(_check_token)])
def motion_get(node_id: int):
    return _call("motion.get", {"id": node_id})


@app.post("/motion/node/{node_id}/transform", dependencies=[Depends(_check_token)])
def motion_set_transform(node_id: int, req: M.Transform):
    args = {k: v for k, v in req.model_dump().items() if v is not None}
    args["id"] = node_id
    return _call("motion.set_transform", args)


@app.post("/motion/node/{node_id}/translate", dependencies=[Depends(_check_token)])
def motion_translate(node_id: int, req: M.TranslateRequest):
    return _call("motion.translate", {"id": node_id, "delta": req.delta})


@app.post("/motion/node/{node_id}/rotate", dependencies=[Depends(_check_token)])
def motion_rotate(node_id: int, req: M.RotateRequest):
    return _call("motion.rotate", {"id": node_id, "delta": req.delta})


@app.post("/motion/node/{node_id}/scale", dependencies=[Depends(_check_token)])
def motion_scale(node_id: int, req: M.ScaleRequest):
    return _call("motion.scale", {"id": node_id, "factor": req.factor})


@app.post("/motion/node/{node_id}/animate", dependencies=[Depends(_check_token)])
def motion_animate(node_id: int, req: M.AnimateRequest):
    target = {k: v for k, v in req.target.model_dump().items() if v is not None}
    args = {"id": node_id, "target": target,
            "duration_s": req.duration_s, "ease": req.ease}
    return _call("motion.animate", args)


# --------------------------------------------------------------------------- #
# Entrypoint
# --------------------------------------------------------------------------- #
def main():
    import uvicorn
    uvicorn.run(
        "remote_api.server:app",
        host=config.HTTP_HOST,
        port=config.HTTP_PORT,
        reload=False,
    )


if __name__ == "__main__":
    main()
