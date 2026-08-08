#include "FrameStreamer.h"

#include <array>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <algorithm>

// GL readback (GLEW is linked by the project; GLRenderer.h pulls GL headers).
#include "GLRenderer.h"
#include <GL/glew.h>

// stb_image_write is vendored at libs/stb/. `./libs` is on the include path.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#include "stb/stb_image_write.h"

// Sockets (kept consistent with RemoteControlServer.cpp's setup).
#ifdef _WIN32
	#ifndef NOMINMAX
		#define NOMINMAX
	#endif
	#define WIN32_LEAN_AND_MEAN
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#pragma comment(lib, "ws2_32.lib")
	using rct_socklen_t = int;
	#define FS_CLOSE_SOCKET closesocket
	#define FS_SOCKET_INVALID (INVALID_SOCKET)
#else
	#include <arpa/inet.h>
	#include <netinet/in.h>
	#include <sys/socket.h>
	#include <unistd.h>
	#define FS_CLOSE_SOCKET ::close
	#define FS_SOCKET_INVALID (-1)
	#define SOCKET int
	#define INVALID_SOCKET (-1)
#endif

namespace remote {

// WSL Winsock init guard (mirrors RemoteControlServer).
namespace {
#ifdef _WIN32
struct WinsockInit {
	WinsockInit()  { WSADATA d; WSAStartup(MAKEWORD(2, 2), &d); }
	~WinsockInit() { WSACleanup(); }
};
WinsockInit g_winsockInit;
#endif
}

FrameStreamer& FrameStreamer::instance() {
	static FrameStreamer s;
	return s;
}

FrameStreamer::FrameStreamer() = default;
FrameStreamer::~FrameStreamer() { stop(); }

void FrameStreamer::start(uint16_t port) {
	if(running_.load()) return;
	running_.store(true);
	serverThread_ = std::thread(&FrameStreamer::serverLoop, this, port);
}

void FrameStreamer::stop() {
	if(!running_.exchange(false)) return;
	// Closing the listen socket unblocks the accept() in serverLoop so the
	// detached thread can exit promptly instead of lingering bound to the
	// port until the next connection attempt or process exit.
	if(listenSock_ != FS_SOCKET_INVALID) {
		FS_CLOSE_SOCKET(listenSock_);
		listenSock_ = FS_SOCKET_INVALID;
	}
	if(serverThread_.joinable()) serverThread_.detach();
}

// ---------------------------------------------------------------------------
// WebSocket framing helpers (minimal RFC 6455, server side).
// ---------------------------------------------------------------------------
namespace {

// Compute the 4-byte masking key XOR. Server->client frames are NOT masked,
// so this is only needed when reading client frames (we mostly ignore them).
void wsUnmask(uint8_t* data, size_t len, const uint8_t mask[4]) {
	for(size_t i = 0; i < len; ++i) data[i] ^= mask[i % 4];
}

// Send one WebSocket binary frame (server->client, unmasked). Returns false on
// socket failure.
bool wsSendBinary(SOCKET sock, const uint8_t* data, size_t len) {
	std::vector<uint8_t> hdr;
	hdr.push_back(0x82); // FIN + binary opcode
	if(len <= 125) {
		hdr.push_back(static_cast<uint8_t>(len));
	} else if(len <= 65535) {
		hdr.push_back(126);
		hdr.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
		hdr.push_back(static_cast<uint8_t>(len & 0xFF));
	} else {
		hdr.push_back(127);
		for(int i = 7; i >= 0; --i)
			hdr.push_back(static_cast<uint8_t>((len >> (8 * i)) & 0xFF));
	}
	if(::send(sock, reinterpret_cast<const char*>(hdr.data()),
	          static_cast<int>(hdr.size()), 0) != static_cast<int>(hdr.size()))
		return false;
	size_t sent = 0;
	while(sent < len) {
		int n = ::send(sock, reinterpret_cast<const char*>(data + sent),
		               static_cast<int>(len - sent), 0);
		if(n <= 0) return false;
		sent += static_cast<size_t>(n);
	}
	return true;
}

// Send a small close frame and close the socket.
void wsClose(SOCKET sock) {
	uint8_t close[2] = {0x88, 0x00};
	::send(sock, reinterpret_cast<const char*>(close), 2, 0);
	FS_CLOSE_SOCKET(sock);
}

// Perform the WebSocket handshake on a freshly-accepted socket. Returns true
// on success (socket is now a WS connection). We only need to find the
// Sec-WebSocket-Key header and echo back the Sec-WebSocket-Accept value.
bool wsHandshake(SOCKET sock) {
	// Minimal SHA-1 + Base64. Implement inline to avoid extra deps.
	std::vector<char> buf(4096);
	int total = 0;
	// Read until we find "\r\n\r\n".
	std::string request;
	request.reserve(2048);
	while(request.find("\r\n\r\n") == std::string::npos) {
		int n = ::recv(sock, buf.data() + (total > 0 ? 0 : 0),
		               static_cast<int>(buf.size()), 0);
		if(n <= 0) return false;
		request.append(buf.data(), static_cast<size_t>(n));
		if(request.size() > 8192) return false; // header too large
	}

	// Extract Sec-WebSocket-Key.
	auto keyPos = request.find("Sec-WebSocket-Key: ");
	if(keyPos == std::string::npos) keyPos = request.find("Sec-WebSocket-Key:");
	if(keyPos == std::string::npos) return false;
	auto valStart = request.find_first_not_of(' ', keyPos + strlen("Sec-WebSocket-Key:"));
	auto valEnd = request.find("\r\n", valStart);
	if(valEnd == std::string::npos) return false;
	std::string key = request.substr(valStart, valEnd - valStart);

	// SHA-1(key + magic) then base64.
	static const char* magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
	std::string combined = key + magic;

	// --- inline SHA-1 ---
	struct SHA1 {
		static std::array<uint8_t,20> hash(const uint8_t* d, size_t n) {
			uint32_t h0=0x67452301,h1=0xEFCDAB89,h2=0x98BADCFE,h3=0x10325476,h4=0xC3D2E1F0;
			uint64_t bits = uint64_t(n)*8;
			std::vector<uint8_t> msg(d, d+n);
			msg.push_back(0x80);
			while(msg.size()%64 != 56) msg.push_back(0);
			for(int i=7;i>=0;--i) msg.push_back(uint8_t((bits>>(8*i))&0xFF));
			auto rotl=[](uint32_t x,int c){return (x<<c)|(x>>(32-c));};
			for(size_t off=0; off<msg.size(); off+=64){
				uint32_t w[80];
				for(int i=0;i<16;i++){
					w[i]=(uint32_t(msg[off+i*4])<<24)|(uint32_t(msg[off+i*4+1])<<16)
					    |(uint32_t(msg[off+i*4+2])<<8)|uint32_t(msg[off+i*4+3]);
				}
				for(int i=16;i<80;i++)
					w[i]=rotl(w[i-3]^w[i-8]^w[i-14]^w[i-16],1);
				uint32_t a=h0,b=h1,c=h2,dd=h3,e=h4;
				for(int i=0;i<80;i++){
					uint32_t f,k;
					if(i<20){f=(b&c)|((~b)&dd);k=0x5A827999;}
					else if(i<40){f=b^c^dd;k=0x6ED9EBA1;}
					else if(i<60){f=(b&c)|(b&dd)|(c&dd);k=0x8F1BBCDC;}
					else{f=b^c^dd;k=0xCA62C1D6;}
					uint32_t t=rotl(a,5)+f+e+k+w[i];
					e=dd;dd=c;c=rotl(b,30);b=a;a=t;
				}
				h0+=a;h1+=b;h2+=c;h3+=dd;h4+=e;
			}
			return {{uint8_t(h0>>24),uint8_t(h0>>16),uint8_t(h0>>8),uint8_t(h0),
			         uint8_t(h1>>24),uint8_t(h1>>16),uint8_t(h1>>8),uint8_t(h1),
			         uint8_t(h2>>24),uint8_t(h2>>16),uint8_t(h2>>8),uint8_t(h2),
			         uint8_t(h3>>24),uint8_t(h3>>16),uint8_t(h3>>8),uint8_t(h3),
			         uint8_t(h4>>24),uint8_t(h4>>16),uint8_t(h4>>8),uint8_t(h4)}};
		}
	};
	auto digest = SHA1::hash(reinterpret_cast<const uint8_t*>(combined.data()),
	                         combined.size());

	// --- inline Base64 ---
	static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	std::string accept;
	for(size_t i=0;i<digest.size();i+=3){
		uint32_t v = uint32_t(digest[i])<<16;
		int pad = 0;
		if(i+1<digest.size()){v|=uint32_t(digest[i+1])<<8;pad=1;}
		if(i+2<digest.size()){v|=uint32_t(digest[i+2]);pad=0;}
		accept += b64[(v>>18)&0x3F];
		accept += b64[(v>>12)&0x3F];
		accept += (i+1<digest.size()) ? b64[(v>>6)&0x3F] : '=';
		accept += (i+2<digest.size()) ? b64[v&0x3F] : '=';
		(void)pad;
	}

	std::string resp =
		"HTTP/1.1 101 Switching Protocols\r\n"
		"Upgrade: websocket\r\n"
		"Connection: Upgrade\r\n"
		"Sec-WebSocket-Accept: " + accept + "\r\n"
		"\r\n";
	return ::send(sock, resp.data(), static_cast<int>(resp.size()), 0)
	       == static_cast<int>(resp.size());
}

} // namespace

// ---------------------------------------------------------------------------
// Server / client loops.
// ---------------------------------------------------------------------------
void FrameStreamer::serverLoop(uint16_t port) {
	SOCKET listenSock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(listenSock == INVALID_SOCKET) {
		std::printf("FrameStreamer: socket() failed\n");
		return;
	}
	listenSock_.store(static_cast<intptr_t>(listenSock));
	int yes = 1;
	setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR,
	           reinterpret_cast<const char*>(&yes), sizeof(yes));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY); // reachable on LAN, like the HTTP API
	addr.sin_port = htons(port);
	if(::bind(listenSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
		std::printf("FrameStreamer: bind failed on port %u\n", port);
		FS_CLOSE_SOCKET(listenSock);
		listenSock_.store(-1);
		return;
	}
	if(::listen(listenSock, 4) < 0) {
		std::printf("FrameStreamer: listen failed\n");
		FS_CLOSE_SOCKET(listenSock);
		listenSock_.store(-1);
		return;
	}
	std::printf("FrameStreamer: WebSocket frame server listening on 0.0.0.0:%u\n", port);

	while(running_.load()) {
		sockaddr_in cli{};
		rct_socklen_t clilen = sizeof(cli);
		SOCKET c = ::accept(listenSock, reinterpret_cast<sockaddr*>(&cli), &clilen);
		if(c == INVALID_SOCKET) {
			// stop() closes the listen socket -> accept() fails -> exit loop.
			if(!running_.load()) break;
			continue;
		}
		if(!wsHandshake(c)) { FS_CLOSE_SOCKET(c); continue; }
		clients.fetch_add(1);
		std::thread(&FrameStreamer::clientLoop, this, static_cast<int>(c)).detach();
	}
	// Clean up if we reached here via running_ going false without stop()
	// having closed the socket already.
	intptr_t old = listenSock_.exchange(-1);
	if(old != -1) FS_CLOSE_SOCKET(static_cast<SOCKET>(old));
}

void FrameStreamer::clientLoop(int sockInt) {
	SOCKET sock = static_cast<SOCKET>(sockInt);
	uint32_t lastSent = 0;
	// Drain any client traffic (ping/close) loosely; we mainly send.
	uint8_t rbuf[256];
	while(running_.load()) {
		uint32_t cur;
		Frame frame;
		{
			std::lock_guard<std::mutex> lk(latestMutex_);
			cur = latest_.seq;
			if(cur != lastSent && !latest_.bytes.empty())
				frame = latest_; // copy out under lock
		}
		if(cur != lastSent && !frame.bytes.empty()) {
			if(!wsSendBinary(sock, frame.bytes.data(), frame.bytes.size())) break;
			lastSent = cur;
		} else {
			// Idle: briefly poll for a close frame so we notice disconnects.
#ifdef _WIN32
			DWORD to = 200;
			::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
			             reinterpret_cast<const char*>(&to), sizeof(to));
#else
			struct timeval tv{0, 200000};
			::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
			             reinterpret_cast<const char*>(&tv), sizeof(tv));
#endif
			int n = ::recv(sock, reinterpret_cast<char*>(rbuf), sizeof(rbuf), 0);
			if(n == 0) break; // orderly close
			if(n < 0) {
#ifdef _WIN32
				if(WSAGetLastError() != WSAETIMEDOUT) break;
#else
				if(errno != EAGAIN && errno != EWOULDBLOCK) break;
#endif
			} else if(n > 0 && (rbuf[0] & 0x0F) == 0x8) {
				break; // close frame
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(4));
		}
	}
	clients.fetch_sub(1);
	wsClose(sock);
}

// ---------------------------------------------------------------------------
// Frame capture (render thread).
// ---------------------------------------------------------------------------
// stb_image_write callback: appends encoded bytes to a std::vector.
static void jpgWriteCb(void* ctx, void* data, int size) {
	auto* out = static_cast<std::vector<uint8_t>*>(ctx);
	auto* p = static_cast<const uint8_t*>(data);
	out->insert(out->end(), p, p + size);
}

void FrameStreamer::captureFrame(unsigned int glLeft, unsigned int glRight,
                                 int eyeWidth, int eyeHeight, FrameCodec codec) {
	// Skip the (expensive) readback/encode when nobody is watching.
	if(clients.load() <= 0) return;
	if(eyeWidth <= 0 || eyeHeight <= 0) return;

	// The eye textures are backing a fixed-size GL framebuffer (4096x4096 in
	// SplatEditor.cpp) but the render only fills an eyeWidth x eyeHeight
	// viewport. glGetTexImage reads the *entire* texture, so the readback
	// buffer must be sized to the actual texture dimensions - not the
	// viewport - or glGetTexImage overruns it. Query the real texture size.
	GLint texW = 0, texH = 0;
	GLint prevTex = 0;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &prevTex);

	glBindTexture(GL_TEXTURE_2D, glLeft);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &texW);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &texH);
	if(texW <= 0 || texH <= 0) {
		glBindTexture(GL_TEXTURE_2D, prevTex);
		return;
	}

	// Read back the full texture (glGetTexImage always returns the whole
	// image) into a buffer sized for texW*texH, then copy only the rendered
	// eyeWidth x eyeHeight viewport out of it.
	const size_t texPixels = static_cast<size_t>(texW) * texH;
	rgbaLeft_.assign(texPixels * 4, 0);
	rgbaRight_.assign(texPixels * 4, 0);

	glBindTexture(GL_TEXTURE_2D, glLeft);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbaLeft_.data());
	glBindTexture(GL_TEXTURE_2D, glRight);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbaRight_.data());

	glBindTexture(GL_TEXTURE_2D, prevTex);

	// Clamp the extracted viewport to the texture size (defensive: the
	// client may request a resolution larger than the framebuffer).
	int vw = std::min(eyeWidth, texW);
	int vh = std::min(eyeHeight, texH);

	// Composite side-by-side: [left | right], using only the rendered
	// viewport (top-left vw x vh region) of each eye texture.
	int sbsW = vw * 2;
	int sbsH = vh;
	rgbaSBS_.assign(static_cast<size_t>(sbsW) * sbsH * 4, 0);
	for(int y = 0; y < sbsH; ++y) {
		memcpy(&rgbaSBS_[static_cast<size_t>(y) * sbsW * 4],
		       &rgbaLeft_[static_cast<size_t>(y) * texW * 4],
		       static_cast<size_t>(vw) * 4);
		memcpy(&rgbaSBS_[(static_cast<size_t>(y) * sbsW + vw) * 4],
		       &rgbaRight_[static_cast<size_t>(y) * texW * 4],
		       static_cast<size_t>(vw) * 4);
	}

	// Encode.
	std::vector<uint8_t> payload;
	uint16_t codecId = 0;
	if(codec == FrameCodec::JPEG) {
		codecId = 0;
		jpegBuf_.clear();
		stbi_write_jpg_to_func(jpgWriteCb, &jpegBuf_,
		                       sbsW, sbsH, 4, rgbaSBS_.data(), 80);
		payload = std::move(jpegBuf_);
	}
#ifdef SPLATSHOP_HAS_NVENC
	else if(codec == FrameCodec::H264) {
		codecId = 1;
		// TODO: NVENC H.264 encode of rgbaSBS_. Requires Video Codec SDK.
		payload.clear();
	}
#endif
	else {
		return; // unsupported codec
	}

	// Build the framed message: header + payload.
	Frame out;
	out.seq = seq_.fetch_add(1) + 1;
	auto& b = out.bytes;
	b.reserve(20 + payload.size());
	auto put16 = [&](uint16_t v){
		b.push_back(uint8_t(v & 0xFF));
		b.push_back(uint8_t((v >> 8) & 0xFF));
	};
	auto put32 = [&](uint32_t v){
		for(int i = 0; i < 4; ++i) b.push_back(uint8_t((v >> (8 * i)) & 0xFF));
	};
	put32(0x56524653);            // magic "VRFS"
	put16(static_cast<uint16_t>(sbsW));
	put16(static_cast<uint16_t>(sbsH));
	put16(static_cast<uint16_t>(vw));
	put16(static_cast<uint16_t>(vh));
	put16(codecId);
	put32(static_cast<uint32_t>(payload.size()));
	b.insert(b.end(), payload.begin(), payload.end());

	{
		std::lock_guard<std::mutex> lk(latestMutex_);
		latest_ = std::move(out);
	}
}

} // namespace remote
