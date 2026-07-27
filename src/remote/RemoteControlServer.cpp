#include "RemoteControlServer.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Splatshop headers (GLFW / windows.h pulled in transitively here).
// IMPORTANT: include these BEFORE winsock2.h. The Windows SDK defines
// `near` and `far` macros in windef.h (used by GLRenderer.h for camera
// members), so winsock must come last; we silence the resulting APIENTRY
// redefinition warning by defining it before winsock2 is pulled in.
#include "GLRenderer.h"          // GLRenderer::window/width/height/fps, GLFW
#include "Runtime.h"             // Runtime::controls, keyStates, mouse*, mouseEvents
#include "MouseEvents.h"
#include "OrbitControls.h"
#include "unsuck.hpp"            // EventQueue / schedule()
#include "json/json.hpp"

#include "../SplatEditor.h"      // SplatEditor::instance->scene
#include "../scene/Scene.h"
#include "../scene/SceneNode.h"
#include "../motion/MotionController.h"
#include "../motion/MotionTypes.h"
#include "Splats.h"              // Splats struct + createSphere
#include "../loader/GSPlyLoader.h"
#include "../loader/SplatsyFilesLoader.h"

// Sockets
#ifdef _WIN32
	#ifndef NOMINMAX
		#define NOMINMAX
	#endif
	#define WIN32_LEAN_AND_MEAN
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#pragma comment(lib, "ws2_32.lib")
	using socklen_t = int;
	using ssize_t   = int;
	#define RCT_CLOSE_SOCKET closesocket
	#define RCT_SOCKET_INVALID (INVALID_SOCKET)
	// WSAStartup guard
	namespace {
	struct WinsockInit {
		WinsockInit()  { WSADATA d; WSAStartup(MAKEWORD(2, 2), &d); }
		~WinsockInit() { WSACleanup(); }
	};
	WinsockInit g_winsockInit;
	}
#else
	#include <arpa/inet.h>
	#include <netinet/in.h>
	#include <sys/socket.h>
	#include <unistd.h>
	#define RCT_CLOSE_SOCKET ::close
	#define RCT_SOCKET_INVALID (-1)
#endif

using json = nlohmann::json;
using namespace std;

namespace remote {

std::atomic<bool> RemoteControlServer::s_running{false};

// ---------------------------------------------------------------------------
// GLFW key name -> GLFW key code map. Keep in sync with remote_api/keymap.py.
// Keys are matched case-insensitively on the uppercased name. A numeric key
// code may also be passed directly (as int) by the client.
// ---------------------------------------------------------------------------
static int resolveKeyCode(const json& j) {
	if(j.is_number_integer()) return j.get<int>();
	if(!j.is_string()) return -1;
	string name = j.get<string>();
	// Uppercase single printable chars map to their ASCII (matches GLFW for A-Z, 0-9).
	if(name.size() == 1) {
		char c = name[0];
		if(c >= 'a' && c <= 'z') c -= 32;
		if((c >= 'A' && c <= 'Z')) return (int)c; // GLFW_KEY_A == 'A'
		if(c >= '0' && c <= '9') return (int)c;  // GLFW_KEY_0 == '0'
	}
	static const unordered_map<string, int> table = {
		{"SPACE", 32}, {"APOSTROPHE", 39}, {"COMMA", 44}, {"MINUS", 45},
		{"PERIOD", 46}, {"SLASH", 47}, {"SEMICOLON", 59}, {"EQUAL", 61},
		{"LEFT_BRACKET", 91}, {"BACKSLASH", 92}, {"RIGHT_BRACKET", 93},
		{"GRAVE_ACCENT", 96},
		{"WORLD_1", 161}, {"WORLD_2", 162},
		{"ESCAPE", 256}, {"ENTER", 257}, {"TAB", 258}, {"BACKSPACE", 259},
		{"INSERT", 260}, {"DELETE", 261}, {"RIGHT", 262}, {"LEFT", 263},
		{"DOWN", 264}, {"UP", 265}, {"PAGE_UP", 266}, {"PAGE_DOWN", 267},
		{"HOME", 268}, {"END", 269}, {"CAPS_LOCK", 280}, {"SCROLL_LOCK", 281},
		{"NUM_LOCK", 282}, {"PRINT_SCREEN", 283}, {"PAUSE", 284},
		{"F1", 290}, {"F2", 291}, {"F3", 292}, {"F4", 293}, {"F5", 294},
		{"F6", 295}, {"F7", 296}, {"F8", 297}, {"F9", 298}, {"F10", 299},
		{"F11", 300}, {"F12", 301}, {"F13", 302}, {"F14", 303}, {"F15", 304},
		{"F16", 305}, {"F17", 306}, {"F18", 307}, {"F19", 308}, {"F20", 309},
		{"F21", 310}, {"F22", 311}, {"F23", 312}, {"F24", 313}, {"F25", 314},
		{"KP_0", 320}, {"KP_1", 321}, {"KP_2", 322}, {"KP_3", 323},
		{"KP_4", 324}, {"KP_5", 325}, {"KP_6", 326}, {"KP_7", 327},
		{"KP_8", 328}, {"KP_9", 329}, {"KP_DECIMAL", 330}, {"KP_DIVIDE", 331},
		{"KP_MULTIPLY", 332}, {"KP_SUBTRACT", 333}, {"KP_ADD", 334},
		{"KP_ENTER", 335}, {"KP_EQUAL", 336},
		{"LEFT_SHIFT", 340}, {"LEFT_CONTROL", 341}, {"LEFT_ALT", 342},
		{"LEFT_SUPER", 343}, {"RIGHT_SHIFT", 344}, {"RIGHT_CONTROL", 345},
		{"RIGHT_ALT", 346}, {"RIGHT_SUPER", 347}, {"MENU", 348},
	};
	// Uppercase the name.
	string up = name;
	for(auto& c : up) if(c >= 'a' && c <= 'z') c -= 32;
	auto it = table.find(up);
	return it == table.end() ? -1 : it->second;
}

static int resolveButton(const string& s) {
	if(s == "left") return 0;
	if(s == "right") return 1;
	if(s == "middle") return 2;
	return -1;
}

static int resolveAction(const string& s) {
	if(s == "press" || s == "1") return 1;
	if(s == "release" || s == "0") return 0;
	if(s == "repeat" || s == "2") return 2;
	return -1;
}

// ---------------------------------------------------------------------------
// Helpers for reading optional/required args without exceptions bubbling up.
// ---------------------------------------------------------------------------
static bool getVec3(const json& a, const char* key, glm::dvec3& out, bool required) {
	if(!a.contains(key) || a[key].is_null()) return !required;
	const json& v = a[key];
	if(!v.is_array() || v.size() < 3) return false;
	out = {v[0].get<double>(), v[1].get<double>(), v[2].get<double>()};
	return true;
}
static bool getVec3f(const json& a, const char* key, glm::vec3& out, bool required) {
	glm::dvec3 d;
	if(!getVec3(a, key, d, required)) return false;
	out = {d.x, d.y, d.z};
	return true;
}
// Read a [x,y,z,w] quaternion (Timeline JSON convention) -> glm::quat (w,x,y,z).
static bool getQuat(const json& a, const char* key, glm::quat& out, bool required) {
	if(!a.contains(key) || a[key].is_null()) return !required;
	const json& v = a[key];
	if(!v.is_array() || v.size() < 4) return false;
	float x = v[0].get<float>(), y = v[1].get<float>(), z = v[2].get<float>(), w = v[3].get<float>();
	out = glm::quat(w, x, y, z);
	return true;
}

static json transformToJson(const motion::TransformSample& s) {
	return json{
		{"translation", {s.translation.x, s.translation.y, s.translation.z}},
		// glm::quat storage is (w,x,y,z); wire format is [x,y,z,w].
		{"rotation", {s.rotation.x, s.rotation.y, s.rotation.z, s.rotation.w}},
		{"scale", {s.scale.x, s.scale.y, s.scale.z}},
	};
}

static motion::EaseMode parseEase(const string& s) {
	if(s == "linear") return motion::EaseMode::Linear;
	if(s == "in")     return motion::EaseMode::EaseIn;
	if(s == "out")    return motion::EaseMode::EaseOut;
	return motion::EaseMode::EaseInOut;
}

// ---------------------------------------------------------------------------
// Command handlers. Each runs ON THE MAIN THREAD (inside schedule()), receives
// the parsed `args` object, and returns a json `data` payload. On failure it
// throws std::runtime_error; the dispatcher converts the message to an error
// response.
// ---------------------------------------------------------------------------
static json cmd_health(const json& /*args*/) {
	return json{
		{"bridge", "up"},
		{"fps", GLRenderer::fps},
		{"frame", GLRenderer::frameCount},
		{"width", GLRenderer::width},
		{"height", GLRenderer::height},
	};
}

// --- camera -----------------------------------------------------------------
static json cmd_camera_orbit(const json& a) {
	auto* c = Runtime::controls;
	if(a.contains("yaw")    && !a["yaw"].is_null())    c->yaw   = a["yaw"].get<double>();
	if(a.contains("pitch")  && !a["pitch"].is_null())  c->pitch = a["pitch"].get<double>();
	if(a.contains("d_yaw")  && !a["d_yaw"].is_null())  c->yaw   += a["d_yaw"].get<double>();
	if(a.contains("d_pitch")&& !a["d_pitch"].is_null())c->pitch += a["d_pitch"].get<double>();
	c->update();
	return json{{"yaw", c->yaw}, {"pitch", c->pitch}};
}

static json cmd_camera_pan(const json& a) {
	auto* c = Runtime::controls;
	if(!a.contains("dx") || !a.contains("dy")) throw runtime_error("pan requires dx, dy");
	c->translate_local(a["dx"].get<double>(), a["dy"].get<double>(), 0.0);
	c->update();
	return json{{"target", {c->target.x, c->target.y, c->target.z}}};
}

static json cmd_camera_zoom(const json& a) {
	auto* c = Runtime::controls;
	if(a.contains("radius") && !a["radius"].is_null()) {
		c->radius = a["radius"].get<double>();
	} else if(a.contains("factor") && !a["factor"].is_null()) {
		c->radius *= a["factor"].get<double>();
	}
	c->update();
	return json{{"radius", c->radius}};
}

static json cmd_camera_pose_set(const json& a) {
	auto* c = Runtime::controls;
	if(a.contains("yaw")    && !a["yaw"].is_null())    c->yaw   = a["yaw"].get<double>();
	if(a.contains("pitch")  && !a["pitch"].is_null())  c->pitch = a["pitch"].get<double>();
	if(a.contains("radius") && !a["radius"].is_null()) c->radius = a["radius"].get<double>();
	glm::dvec3 t = c->target;
	getVec3(a, "target", t, false);
	c->target = t;
	c->update();
	return json{{"yaw", c->yaw}, {"pitch", c->pitch}, {"radius", c->radius},
	            {"target", {c->target.x, c->target.y, c->target.z}}};
}

static json cmd_camera_pose_get(const json& /*a*/) {
	auto* c = Runtime::controls;
	c->update();
	auto pos = c->getPosition();
	return json{
		{"yaw", c->yaw}, {"pitch", c->pitch}, {"radius", c->radius},
		{"target", {c->target.x, c->target.y, c->target.z}},
		{"position", {pos.x, pos.y, pos.z}},
	};
}

static json cmd_camera_focus(const json& a) {
	auto* c = Runtime::controls;
	glm::vec3 mn, mx;
	float factor = 1.0f;
	if(a.contains("factor") && !a["factor"].is_null()) factor = a["factor"].get<float>();
	if(a.contains("node_id") && !a["node_id"].is_null()) {
		auto id = a["node_id"].get<int64_t>();
		SceneNode* node = nullptr;
		if(SplatEditor::instance) {
			SplatEditor::instance->scene.root->traverse([&](SceneNode* n){
				if(!node && n->ID == id) node = n;
			});
		}
		if(!node) throw runtime_error("node not found: " + to_string(id));
		auto box = node->getBoundingBox();
		mn = box.min; mx = box.max;
	} else {
		if(!getVec3f(a, "min", mn, true) || !getVec3f(a, "max", mx, true))
			throw runtime_error("focus requires node_id or min+max");
	}
	c->focus(mn, mx, factor);
	c->update();
	return json{{"target", {c->target.x, c->target.y, c->target.z}},
	            {"radius", c->radius}};
}

// --- mouse ------------------------------------------------------------------
// Inject a cursor move. Input x,y are window pixel coords with origin top-left,
// Y down (standard browser/client convention). Internally the app uses
// y' = height - y (see cursor_position_callback). We mirror that flip and feed
// both Runtime::mousePosition and Runtime::controls->onMouseMove.
static json cmd_mouse_move(const json& a) {
	if(!a.contains("x") || !a.contains("y")) throw runtime_error("move requires x, y");
	double x = a["x"].get<double>();
	double y = a["y"].get<double>();
	int h = GLRenderer::height > 0 ? GLRenderer::height : 1080;
	double yflip = h - y;
	Runtime::mousePosition = {x, y};
	Runtime::mouseEvents.onMouseMove(x, yflip);
	Runtime::controls->onMouseMove(x, yflip);
	Runtime::controls->update();
	return json{{"x", x}, {"y", y}};
}

static json cmd_mouse_button(const json& a) {
	if(!a.contains("button") || !a.contains("action"))
		throw runtime_error("button requires button, action");
	int btn = resolveButton(a["button"].get<string>());
	int act = resolveAction(a["action"].get<string>());
	if(btn < 0 || act < 0) throw runtime_error("invalid button/action");
	int mods = a.value("mods", 0);
	if(act == 1)      Runtime::mouseButtons |= (1 << btn);
	else if(act == 0) Runtime::mouseButtons &= ~(1 << btn);
	Runtime::controls->onMouseButton(btn, act, mods);
	Runtime::mouseEvents.onMouseButton(btn, act, mods);
	return json{{"button", btn}, {"action", act}, {"mouseButtons", Runtime::mouseButtons}};
}

static json cmd_mouse_scroll(const json& a) {
	if(!a.contains("dx") || !a.contains("dy")) throw runtime_error("scroll requires dx, dy");
	double dx = a["dx"].get<double>();
	double dy = a["dy"].get<double>();
	Runtime::mouseEvents.onMouseScroll(dx, dy);
	Runtime::controls->onMouseScroll(dx, dy);
	Runtime::controls->update();
	return json{{"dx", dx}, {"dy", dy}, {"radius", Runtime::controls->radius}};
}

// Composite event: optionally move, then optionally button, then optionally
// scroll, in one round-trip. Fields are all optional except x,y.
static json cmd_mouse_event(const json& a) {
	if(a.contains("x") && a.contains("y")) {
		double x = a["x"].get<double>();
		double y = a["y"].get<double>();
		int h = GLRenderer::height > 0 ? GLRenderer::height : 1080;
		double yflip = h - y;
		Runtime::mousePosition = {x, y};
		Runtime::mouseEvents.onMouseMove(x, yflip);
		Runtime::controls->onMouseMove(x, yflip);
	}
	if(a.contains("button") && a.contains("action")) {
		int btn = resolveButton(a["button"].get<string>());
		int act = resolveAction(a["action"].get<string>());
		if(btn < 0 || act < 0) throw runtime_error("invalid button/action");
		int mods = a.value("mods", 0);
		if(act == 1)      Runtime::mouseButtons |= (1 << btn);
		else if(act == 0) Runtime::mouseButtons &= ~(1 << btn);
		Runtime::controls->onMouseButton(btn, act, mods);
		Runtime::mouseEvents.onMouseButton(btn, act, mods);
	}
	if(a.contains("scroll_dx") || a.contains("scroll_dy")) {
		double dx = a.value("scroll_dx", 0.0);
		double dy = a.value("scroll_dy", 0.0);
		Runtime::mouseEvents.onMouseScroll(dx, dy);
		Runtime::controls->onMouseScroll(dx, dy);
	}
	Runtime::controls->update();
	return json{{"ok", true}};
}

// --- keyboard ---------------------------------------------------------------
// Inject a key event into the same per-frame vectors the GLFW key_callback
// writes, so Runtime::getKeyAction(int/char) and the editor's input handlers
// behave as if the user pressed the key locally this frame.
static json cmd_keyboard_key(const json& a) {
	if(!a.contains("key") || !a.contains("action"))
		throw runtime_error("key requires key, action");
	int key = resolveKeyCode(a["key"]);
	if(key < 0) throw runtime_error("unknown key");
	int act = resolveAction(a["action"].get<string>());
	if(act < 0) throw runtime_error("invalid action");
	int mods = a.value("mods", 0);
	if(key >= 0 && key < (int)Runtime::keyStates.size()) Runtime::keyStates[key] = act;
	Runtime::mods = mods;
	Runtime::frame_keys.push_back(key);
	Runtime::frame_scancodes.push_back(0);
	Runtime::frame_actions.push_back(act);
	Runtime::frame_mods.push_back(mods);
	return json{{"key", key}, {"action", act}};
}

// --- scene / motion ---------------------------------------------------------
static json cmd_scene_nodes(const json& /*a*/) {
	json nodes = json::array();
	if(!SplatEditor::instance) throw runtime_error("editor not ready");
	SplatEditor::instance->scene.root->traverse([&](SceneNode* n){
		nodes.push_back({
			{"id", n->ID},
			{"name", n->name},
			{"type", n->toString()},
		});
	});
	return json{{"nodes", nodes}};
}

static Scene& editorScene() {
	if(!SplatEditor::instance) throw runtime_error("editor not ready");
	return SplatEditor::instance->scene;
}

static json cmd_motion_get(const json& a) {
	auto id = a.at("id").get<int64_t>();
	motion::TransformSample s;
	if(!motion::MotionController::getTransform(editorScene(), id, s))
		throw runtime_error("node not found: " + to_string(id));
	return transformToJson(s);
}

static json cmd_motion_set_transform(const json& a) {
	auto id = a.at("id").get<int64_t>();
	motion::TransformSample s = motion::TransformSample::identity();
	getVec3f(a, "translation", s.translation, false);
	getQuat  (a, "rotation",   s.rotation,    false);
	getVec3f(a, "scale",       s.scale,       false);
	if(!motion::MotionController::setTransform(editorScene(), id, s))
		throw runtime_error("node not found: " + to_string(id));
	return transformToJson(s);
}

static json cmd_motion_translate(const json& a) {
	auto id = a.at("id").get<int64_t>();
	glm::vec3 d;
	if(!getVec3f(a, "delta", d, true)) throw runtime_error("delta required");
	if(!motion::MotionController::translate(editorScene(), id, d))
		throw runtime_error("node not found: " + to_string(id));
	return json{{"delta", {d.x, d.y, d.z}}};
}

static json cmd_motion_rotate(const json& a) {
	auto id = a.at("id").get<int64_t>();
	glm::quat q;
	if(!getQuat(a, "delta", q, true)) throw runtime_error("delta required");
	if(!motion::MotionController::rotate(editorScene(), id, q))
		throw runtime_error("node not found: " + to_string(id));
	motion::TransformSample s;
	motion::MotionController::getTransform(editorScene(), id, s);
	return transformToJson(s);
}

static json cmd_motion_scale(const json& a) {
	auto id = a.at("id").get<int64_t>();
	glm::vec3 f;
	if(!getVec3f(a, "factor", f, true)) throw runtime_error("factor required");
	if(!motion::MotionController::scaleBy(editorScene(), id, f))
		throw runtime_error("node not found: " + to_string(id));
	return json{{"factor", {f.x, f.y, f.z}}};
}

static json cmd_motion_animate(const json& a) {
	auto id = a.at("id").get<int64_t>();
	if(!a.contains("target")) throw runtime_error("target required");
	const json& t = a["target"];
	motion::TransformSample tgt = motion::TransformSample::identity();
	getVec3f(t, "translation", tgt.translation, false);
	getQuat  (t, "rotation",   tgt.rotation,    false);
	getVec3f(t, "scale",       tgt.scale,       false);
	double dur = a.value("duration_s", 1.0);
	string easeS = a.value("ease", "in_out");
	auto ease = parseEase(easeS);
	bool ok = motion::MotionController::setTransformAnimated(editorScene(), id, tgt, dur, ease);
	if(!ok) throw runtime_error("node not found or invalid: " + to_string(id));
	return json{{"duration_s", dur}, {"ease", easeS}};
}

// --- scene splats create / modify ------------------------------------------

// Helper: attach a freshly-built Splats+SNSplats node to the scene.
// Returns the new node's ID and name.
static json attachSplatsNode(shared_ptr<Splats> splats, const glm::vec3& colorHint) {
	(void)colorHint; // reserved for future per-node metadata
	shared_ptr<SNSplats> node = make_shared<SNSplats>(splats->name, splats);
	if(!SplatEditor::instance) throw runtime_error("editor not ready");
	SplatEditor::instance->scene.world->children.push_back(node);
	return json{{"id", node->ID}, {"name", node->name},
	            {"count", splats->numSplats}};
}

// Convert float [0,1] RGBA to uint16 [0,65535].
static void fillColor(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& a,
                      const vector<float>& rgba) {
	auto clamp = [](float v){ return v<0.0f?0.0f:(v>1.0f?1.0f:v); };
	r = (uint16_t)(clamp(rgba[0]) * 65535.0f);
	g = (uint16_t)(clamp(rgba[1]) * 65535.0f);
	b = (uint16_t)(clamp(rgba[2]) * 65535.0f);
	a = (uint16_t)(clamp(rgba.size()>3?rgba[3]:1.0f) * 65535.0f);
}

// Parse optional [r,g,b,a] or [r,g,b] from args.
static bool parseColor(const json& args, const char* key, uint16_t& r, uint16_t& g,
                       uint16_t& b, uint16_t& alpha, bool hasDefault) {
	if(!args.contains(key) || args[key].is_null()) {
		if(!hasDefault) return false;
		r=g=b=0; alpha=65535; return true;
	}
	const json& v = args[key];
	if(!v.is_array() || v.size()<3) return false;
	float a4 = v.size() >= 4 ? v[3].get<float>() : 1.0f;
	vector<float> rgba = {v[0].get<float>(), v[1].get<float>(), v[2].get<float>(), a4};
	fillColor(r,g,b,alpha,rgba);
	return true;
}

static json cmd_scene_splats_create_sphere(const json& a) {
	// Required: radius. Optional: center (default origin), count (default 576), color.
	float radius  = a.value("radius", 1.0f);
	int   count   = a.value("count", 576);
	float cx = 0, cy = 0, cz = 0;
	if(a.contains("center")) {
		const json& c = a["center"];
		if(c.is_array() && c.size()>=3) { cx=c[0].get<float>(); cy=c[1].get<float>(); cz=c[2].get<float>(); }
	}
	uint16_t cr,cg,cb,ca;
	parseColor(a, "color", cr,cg,cb,ca, true); // defaults to red if absent

	shared_ptr<Splats> splats = make_shared<Splats>();
	splats->name = "remote_sphere";
	splats->numSplats = count;
	splats->numSplatsLoaded = count;
	splats->numSHCoefficients = 0;
	splats->shDegree = 0;

	splats->position = make_shared<Buffer>(12 * count);
	splats->scale    = make_shared<Buffer>(12 * count);
	splats->rotation = make_shared<Buffer>(16 * count);
	splats->color    = make_shared<Buffer>(8  * count);

	// Generate points on unit sphere then scale by radius + translate
	for(int i=0; i<count; i++){
		// Fibonacci sphere distribution for uniform coverage
		float phi = acosf(1.0f - 2.0f*(i+0.5f)/count);
		float theta = 3.14159265359f * (1.0f + sqrtf(5.0f)) * i;
		float x = sinf(phi)*cosf(theta);
		float y = sinf(phi)*sinf(theta);
		float z = cosf(phi);
		splats->position->set<float>(cx + x*radius, 12*i+0);
		splats->position->set<float>(cy + y*radius, 12*i+4);
		splats->position->set<float>(cz + z*radius, 12*i+8);
		// Scale proportional to spacing
		float s = radius * 3.0f / sqrtf((float)count);
		splats->scale->set<float>(s, 12*i+0);
		splats->scale->set<float>(s, 12*i+4);
		splats->scale->set<float>(s, 12*i+8);
		splats->rotation->set<float>(1.0f, 16*i+0); // w
		splats->rotation->set<float>(0.0f, 16*i+4); // x
		splats->rotation->set<float>(0.0f, 16*i+8); // y
		splats->rotation->set<float>(0.0f, 16*i+12);// z
		splats->color->set<uint16_t>(cr, 8*i+0);
		splats->color->set<uint16_t>(cg, 8*i+2);
		splats->color->set<uint16_t>(cb, 8*i+4);
		splats->color->set<uint16_t>(ca, 8*i+6);
	}
	return attachSplatsNode(splats, glm::vec3(cr/65535.f,cg/65535.f,cb/65535.f));
}

static json cmd_scene_splats_create_points(const json& a) {
	if(!a.contains("positions") || !a["positions"].is_array())
		throw runtime_error("positions required (array of [x,y,z])");
	const json& pts = a["positions"];
	int count = (int)pts.size();
	if(count == 0) throw runtime_error("positions is empty");

	float defaultScale = a.value("scale", 0.02f);
	uint16_t cr,cg,cb,ca;
	parseColor(a, "color", cr,cg,cb,ca, true);

	shared_ptr<Splats> splats = make_shared<Splats>();
	splats->name = "remote_points";
	splats->numSplats = count;
	splats->numSplatsLoaded = count;
	splats->numSHCoefficients = 0;
	splats->shDegree = 0;

	splats->position = make_shared<Buffer>(12 * count);
	splats->scale    = make_shared<Buffer>(12 * count);
	splats->rotation = make_shared<Buffer>(16 * count);
	splats->color    = make_shared<Buffer>(8  * count);

	for(int i=0; i<count; i++){
		const json& p = pts[i];
		if(!p.is_array() || p.size()<3) throw runtime_error("positions["+to_string(i)+"] invalid");
		splats->position->set<float>(p[0].get<float>(), 12*i+0);
		splats->position->set<float>(p[1].get<float>(), 12*i+4);
		splats->position->set<float>(p[2].get<float>(), 12*i+8);
		splats->scale->set<float>(defaultScale, 12*i+0);
		splats->scale->set<float>(defaultScale, 12*i+4);
		splats->scale->set<float>(defaultScale, 12*i+8);
		splats->rotation->set<float>(1.0f, 16*i+0); // w
		splats->rotation->set<float>(0.0f, 16*i+4);
		splats->rotation->set<float>(0.0f, 16*i+8);
		splats->rotation->set<float>(0.0f, 16*i+12);
		splats->color->set<uint16_t>(cr, 8*i+0);
		splats->color->set<uint16_t>(cg, 8*i+2);
		splats->color->set<uint16_t>(cb, 8*i+4);
		splats->color->set<uint16_t>(ca, 8*i+6);
	}
	return attachSplatsNode(splats, glm::vec3(cr/65535.f,cg/65535.f,cb/65535.f));
}

static json cmd_scene_splats_create_box(const json& a) {
	if(!a.contains("min") || !a.contains("max"))
		throw runtime_error("box requires min and max [x,y,z] arrays");
	glm::vec3 mn, mx;
	if(!getVec3f(a, "min", mn, true) || !getVec3f(a, "max", mx, true))
		throw runtime_error("invalid min/max");
	int count = a.value("count", 1000);
	uint16_t cr,cg,cb,ca;
	parseColor(a, "color", cr,cg,cb,ca, true);

	float sx = mx.x-mn.x, sy = mx.y-mn.y, sz = mx.z-mn.z;
	float volume = sx*sy*sz;
	float spacing = cbrtf(volume / count);

	shared_ptr<Splats> splats = make_shared<Splats>();
	splats->name = "remote_box";
	splats->numSplats = count;
	splats->numSplatsLoaded = count;
	splats->numSHCoefficients = 0;
	splats->shDegree = 0;

	splats->position = make_shared<Buffer>(12 * count);
	splats->scale    = make_shared<Buffer>(12 * count);
	splats->rotation = make_shared<Buffer>(16 * count);
	splats->color    = make_shared<Buffer>(8  * count);

	for(int i=0; i<count; i++){
		// Random-uniform distribution in AABB
		float rx = (float)rand()/RAND_MAX, ry = (float)rand()/RAND_MAX, rz = (float)rand()/RAND_MAX;
		splats->position->set<float>(mn.x + rx*sx, 12*i+0);
		splats->position->set<float>(mn.y + ry*sy, 12*i+4);
		splats->position->set<float>(mn.z + rz*sz, 12*i+8);
		splats->scale->set<float>(spacing, 12*i+0);
		splats->scale->set<float>(spacing, 12*i+4);
		splats->scale->set<float>(spacing, 12*i+8);
		splats->rotation->set<float>(1.0f, 16*i+0);
		splats->rotation->set<float>(0.0f, 16*i+4);
		splats->rotation->set<float>(0.0f, 16*i+8);
		splats->rotation->set<float>(0.0f, 16*i+12);
		splats->color->set<uint16_t>(cr, 8*i+0);
		splats->color->set<uint16_t>(cg, 8*i+2);
		splats->color->set<uint16_t>(cb, 8*i+4);
		splats->color->set<uint16_t>(ca, 8*i+6);
	}
	return attachSplatsNode(splats, glm::vec3(cr/65535.f,cg/65535.f,cb/65535.f));
}

static json cmd_scene_splats_load_file(const json& a) {
	if(!a.contains("path") || !a["path"].is_string())
		throw runtime_error("path required (string)");
	string path = a["path"].get<string>();
	if(!SplatEditor::instance) throw runtime_error("editor not ready");

	if(!fs::exists(path)) throw runtime_error("file not found: " + path);

	if(iEndsWith(path, ".ply")){
		auto splats = GSPlyLoader::load(path);
		return attachSplatsNode(splats, glm::vec3(1,0,0));
	} else if(iEndsWith(path, ".json")){
		SplatsyFilesLoader::load(path, SplatEditor::instance->scene, *Runtime::controls);
		return json{{"msg", "scene loaded from " + path}};
	}
	throw runtime_error("unsupported file format (use .ply or .json): " + path);
}

static json cmd_scene_node_remove(const json& a) {
	auto id = a.at("id").get<int64_t>();
	if(!SplatEditor::instance) throw runtime_error("editor not ready");
	Scene& scene = SplatEditor::instance->scene;

	// Try world.children first (the common case)
	auto& worldKids = scene.world->children;
	for(size_t i=0; i<worldKids.size(); i++){
		if(worldKids[i]->ID == id){ worldKids.erase(worldKids.begin()+i); return json{{"id",id}}; }
	}
	// General traversal: find parent, erase by raw pointer ID
	bool removed = false;
	scene.root->traverse([&](SceneNode* parent){
		if(removed) return;
		for(size_t i=0; i<parent->children.size(); i++){
			if(parent->children[i]->ID == id){
				parent->children.erase(parent->children.begin()+i);
				removed = true; return;
			}
		}
	});
	if(!removed) throw runtime_error("node not found: " + to_string(id));
	return json{{"id", id}};
}

static json cmd_scene_splats_set_color(const json& a) {
	auto id = a.at("id").get<int64_t>();
	if(!a.contains("color") || !a["color"].is_array() || a["color"].size()<3)
		throw runtime_error("color required [r,g,b,a]");
	const json& c = a["color"];
	float a4 = c.size() >= 4 ? c[3].get<float>() : 1.0f;
	vector<float> rgba = {c[0].get<float>(),c[1].get<float>(),c[2].get<float>(),a4};
	uint16_t cr,cg,cb,ca; fillColor(cr,cg,cb,ca,rgba);

	if(!SplatEditor::instance) throw runtime_error("editor not ready");
	SceneNode* found = nullptr;
	SplatEditor::instance->scene.root->traverse([&](SceneNode* n){ if(!found && n->ID==id) found=n; });
	if(!found) throw runtime_error("node not found: " + to_string(id));
	SNSplats* sn = dynamic_cast<SNSplats*>(found);
	if(!sn) throw runtime_error("node is not a splats node: " + to_string(id));

	uint32_t count = sn->dmng.data.count;
	if(count == 0) throw runtime_error("splats node has no uploaded data");
	// Write to GPU color buffer directly (cuMemcpyHtoD)
	vector<uint16_t> colorBuf(4*count);
	for(uint32_t i=0; i<count; i++){
		colorBuf[4*i+0] = cr; colorBuf[4*i+1] = cg;
		colorBuf[4*i+2] = cb; colorBuf[4*i+3] = ca;
	}
	cuMemcpyHtoD((CUdeviceptr)sn->dmng.data.color, colorBuf.data(), 4*count*sizeof(uint16_t));
	return json{{"id", id}, {"count", count}, {"color", rgba}};
}


// ---------------------------------------------------------------------------
// Command dispatch table. The dispatcher runs on the main thread.
// ---------------------------------------------------------------------------
using Handler = json(*)(const json&);

static const unordered_map<string, Handler>& handlers() {
	static const unordered_map<string, Handler> table = {
		{"health",                cmd_health},
		{"camera.orbit",          cmd_camera_orbit},
		{"camera.pan",            cmd_camera_pan},
		{"camera.zoom",           cmd_camera_zoom},
		{"camera.pose.set",       cmd_camera_pose_set},
		{"camera.pose.get",       cmd_camera_pose_get},
		{"camera.focus",          cmd_camera_focus},
		{"mouse.move",            cmd_mouse_move},
		{"mouse.button",          cmd_mouse_button},
		{"mouse.scroll",          cmd_mouse_scroll},
		{"mouse.event",           cmd_mouse_event},
		{"keyboard.key",          cmd_keyboard_key},
		{"scene.nodes",           cmd_scene_nodes},
		{"scene.splats.create_sphere", cmd_scene_splats_create_sphere},
		{"scene.splats.create_points", cmd_scene_splats_create_points},
		{"scene.splats.create_box",    cmd_scene_splats_create_box},
		{"scene.splats.load_file",     cmd_scene_splats_load_file},
		{"scene.node.remove",     cmd_scene_node_remove},
		{"scene.splats.set_color",cmd_scene_splats_set_color},
		{"motion.get",            cmd_motion_get},
		{"motion.set_transform",  cmd_motion_set_transform},
		{"motion.translate",      cmd_motion_translate},
		{"motion.rotate",         cmd_motion_rotate},
		{"motion.scale",          cmd_motion_scale},
		{"motion.animate",        cmd_motion_animate},
	};
	return table;
}

// Run a command on the main thread, blocking until it completes. Returns the
// full response object (with id filled in by the caller).
static json runOnMainThread(int /*id*/, const string& cmd, const json& args) {
	const auto& table = handlers();
	auto it = table.find(cmd);
	if(it == table.end()) {
		return json{{"ok", false}, {"error", "unknown command: " + cmd}};
	}
	Handler h = it->second;

	promise<json> prom;
	future<json>  fut = prom.get_future();
	schedule([&prom, h, args]() {
		try {
			json data = h(args);
			prom.set_value(json{{"ok", true}, {"data", std::move(data)}});
		} catch(const std::exception& e) {
			prom.set_value(json{{"ok", false}, {"error", e.what()}});
		} catch(...) {
			prom.set_value(json{{"ok", false}, {"error", "unknown exception"}});
		}
	});

	// Block the socket thread until the main thread executes the lambda.
	// The render loop drains the EventQueue every frame, so this completes
	// within one frame (~16ms) under normal conditions.
	auto status = fut.wait_for(chrono::seconds(10));
	if(status == future_status::timeout) {
		return json{{"ok", false}, {"error", "main thread did not respond within 10s"}};
	}
	return fut.get();
}

// ---------------------------------------------------------------------------
// Socket I/O helpers (newline-delimited JSON).
// ---------------------------------------------------------------------------
static bool sendAll(int sock, const string& buf) {
	const char* p = buf.data();
	size_t remaining = buf.size();
	while(remaining > 0) {
#ifdef _WIN32
		int n = ::send(sock, p, (int)remaining, 0);
#else
		ssize_t n = ::send(sock, p, remaining, 0);
#endif
		if(n <= 0) return false;
		p += n;
		remaining -= (size_t)n;
	}
	return true;
}

static bool recvLine(int sock, string& line) {
	line.clear();
	char c;
	while(true) {
#ifdef _WIN32
		int n = ::recv(sock, &c, 1, 0);
#else
		ssize_t n = ::recv(sock, &c, 1, 0);
#endif
		if(n <= 0) return false;
		if(c == '\n') return true;
		line.push_back(c);
	}
}

bool RemoteControlServer::handleConnection(int clientSock) {
	// Per-connection read loop. Each request gets one response line.
	string line;
	while(recvLine(clientSock, line)) {
		if(line.empty()) continue; // tolerate blank keepalive lines
		json req;
		try {
			req = json::parse(line);
		} catch(const std::exception& e) {
			json err = {{"id", nullptr}, {"ok", false}, {"error", string("JSON parse: ") + e.what()}};
			sendAll(clientSock, err.dump() + "\n");
			continue;
		}

		int id = req.value("id", 0);
		string cmd = req.value("cmd", "");
		json args = req.value("args", json::object());

		json resp = runOnMainThread(id, cmd, args);
		resp["id"] = id;
		if(!sendAll(clientSock, resp.dump() + "\n")) return false;
	}
	return true;
}

void RemoteControlServer::listenerMain(uint16_t port) {
#ifdef _WIN32
	SOCKET listenSock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(listenSock == INVALID_SOCKET) return;
#else
	int listenSock = ::socket(AF_INET, SOCK_STREAM, 0);
	if(listenSock < 0) return;
#endif
	int yes = 1;
	setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1 only
	addr.sin_port = htons(port);
	if(::bind(listenSock, (sockaddr*)&addr, sizeof(addr)) < 0) {
		RCT_CLOSE_SOCKET(listenSock);
		std::println("RemoteControlServer: bind failed on port {}", port);
		return;
	}
	if(::listen(listenSock, 4) < 0) {
		RCT_CLOSE_SOCKET(listenSock);
		std::println("RemoteControlServer: listen failed on port {}", port);
		return;
	}

	s_running.store(true);
	std::println("RemoteControlServer: listening on 127.0.0.1:{}", port);

	while(true) {
		sockaddr_in cli{};
		socklen_t cliLen = sizeof(cli);
		int clientSock = (int)::accept(listenSock, (sockaddr*)&cli, &cliLen);
		if(clientSock == RCT_SOCKET_INVALID) break;
		// Handle each connection on its own thread so multiple clients (and
		// short-lived Python request sockets) can be served concurrently.
		std::thread([clientSock]() {
			handleConnection(clientSock);
			RCT_CLOSE_SOCKET(clientSock);
		}).detach();
	}

	RCT_CLOSE_SOCKET(listenSock);
	s_running.store(false);
}

bool RemoteControlServer::start(uint16_t port) {
	if(s_running.load()) return true;
	std::thread(listenerMain, port).detach();
	// Give the listener a moment to bind so callers can report failure early.
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	return s_running.load();
}

} // namespace remote
