#pragma once

// FrameStreamer
// -----------------------------------------------------------------------------
// Video return channel for VR remote stereo. The existing remote control API
// (RemoteControlServer) is control-only; this is the missing video plane.
//
// Architecture:
//   - The main render thread calls captureFrame() once per frame (after the
//     stereo eyes have been blitted into viewLeft/viewRight.framebuffer). It
//     reads back the two eye images, composites them side-by-side, optionally
//     JPEG-encodes (via stb_image_write), and stores the latest encoded frame
//     in a single-slot buffer under a mutex.
//   - A server thread listens on a TCP port and speaks a minimal WebSocket
//     (RFC 6455) handshake + binary frames. Each connected client is sent the
//     latest frame as it arrives, throttled to the client's read speed.
//
// Encoding backends:
//   - JPEG (default): no external dependency, uses stb_image_write. Suitable
//     for LAN pipeline validation (latency/quality not VR-grade, but proves
//     the end-to-end path).
//   - NVENC (SPLATSHOP_HAS_NVENC): H.264 hardware encode. Requires the NVIDIA
//     Video Codec SDK headers (nvEncodeAPI.h) + nvencode.lib, which are NOT
//     shipped with the CUDA toolkit. Gated behind the macro so the build
//     succeeds without them; plug in later.
//
// Frame layout on the wire:
//   A small binary header (little-endian) followed by the encoded payload:
//     uint32 magic      = 0x56524653 ("SFRV"/"VRFS")
//     uint16 width      // SBS composite width  (left.w + right.w)
//     uint16 height     // max(left.h, right.h)
//     uint16 left_w     // left eye width
//     uint16 left_h
//     uint16 codec      // 0 = JPEG, 1 = H.264
//     uint32 payload_len
//     uint8  payload[payload_len]
//
// The client decodes per `codec` and renders left/right halves to its WebXR
// layer.

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace remote {

enum class FrameCodec : uint16_t {
    JPEG = 0,
    H264 = 1,
};

class FrameStreamer {
public:
    static FrameStreamer& instance();

    // Start the WebSocket server on the given port. Idempotent.
    void start(uint16_t port);
    void stop();

    // Called from the main render thread after the stereo eyes are drawn.
    // Reads back both eye framebuffers (GL texture objects) and publishes a
    // new frame. `glLeft`/`glRight` are the color attachment GL texture names
    // (Framebuffer::colorAttachments[0]->handle). Both eyes are expected to be
    // the same width/height. Cheap no-op when there are no connected clients.
    void captureFrame(unsigned int glLeft, unsigned int glRight,
                      int eyeWidth, int eyeHeight,
                      FrameCodec codec = FrameCodec::JPEG);

    // Number of clients currently subscribed (for diagnostics / deciding
    // whether to spend work on readback).
    int clientCount() const { return clients.load(); }

private:
    FrameStreamer();
    ~FrameStreamer();
    FrameStreamer(const FrameStreamer&) = delete;
    FrameStreamer& operator=(const FrameStreamer&) = delete;

    void serverLoop(uint16_t port);
    void clientLoop(int sock);

    // Single-slot latest-frame buffer. captureFrame writes, clientLoop reads.
    struct Frame {
        std::vector<uint8_t> bytes;   // header + payload
        uint32_t seq = 0;
    };
    Frame latest_;
    std::mutex latestMutex_;
    std::atomic<uint32_t> seq_{0};
    std::atomic<int> clients{0};

    std::atomic<bool> running_{false};
    std::thread serverThread_;
    // Listen socket handle (INVALID_SOCKET / -1 when not listening). Stored so
    // stop() can close it to unblock accept() in serverLoop.
    std::atomic<intptr_t> listenSock_{-1};

    // Scratch buffers reused across captureFrame calls (render-thread only).
    std::vector<uint8_t> rgbaLeft_;
    std::vector<uint8_t> rgbaRight_;
    std::vector<uint8_t> rgbaSBS_;
    std::vector<uint8_t> jpegBuf_;
};

} // namespace remote
