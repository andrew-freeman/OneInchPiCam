// minimal_raw_grab.cpp
// Build (example):
//   g++ -std=c++17 minimal_raw_grab.cpp -o minimal_raw_grab $(pkg-config --cflags --libs libcamera) -lpthread
//
// Usage:
//   ./minimal_raw_grab [camera_index] [output.bin]
//
// This grabs exactly ONE RAW frame (plane 0) and writes the bytes to output.bin.

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

static bool write_file(const std::string &path, const uint8_t *data, size_t size) {
    int fd = ::open(path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0644);
    if (fd < 0) {
        std::perror("open");
        return false;
    }
    ssize_t w = ::write(fd, data, size);
    ::close(fd);
    if (w < 0 || static_cast<size_t>(w) != size) {
        std::perror("write");
        return false;
    }
    return true;
}

struct MappedPlane {
    void *addr = nullptr;
    size_t len = 0;
};

int main(int argc, char **argv) {
    int camera_index = 0;
    std::string out_path = "raw_plane0.bin";
    if (argc >= 2) camera_index = std::stoi(argv[1]);
    if (argc >= 3) out_path = argv[2];

    libcamera::CameraManager cm;
    if (cm.start()) {
        std::cerr << "Failed to start CameraManager\n";
        return 1;
    }

    const auto &cams = cm.cameras();
    if (cams.empty()) {
        std::cerr << "No cameras found\n";
        cm.stop();
        return 1;
    }
    if (camera_index < 0 || static_cast<size_t>(camera_index) >= cams.size()) {
        std::cerr << "Invalid camera index " << camera_index
                  << " (have " << cams.size() << " cameras)\n";
        cm.stop();
        return 1;
    }

    std::shared_ptr<libcamera::Camera> cam = cams[camera_index];
    std::cerr << "Using camera[" << camera_index << "]: " << cam->id() << "\n";

    if (cam->acquire()) {
        std::cerr << "Failed to acquire camera\n";
        cm.stop();
        return 1;
    }

    // Configure a RAW stream
    std::unique_ptr<libcamera::CameraConfiguration> cfg =
        cam->generateConfiguration({libcamera::StreamRole::Raw});
    if (!cfg || cfg->empty()) {
        std::cerr << "Failed to generate RAW configuration\n";
        cam->release();
        cm.stop();
        return 1;
    }

    libcamera::StreamConfiguration &sc = cfg->at(0);

    // Optional: set a target size (comment out to let libcamera pick default)
    // sc.size.width = 2784;
    // sc.size.height = 1828;

    auto st = cfg->validate();
    std::cerr << "validate() status = " << static_cast<int>(st) << "\n";

    std::cerr << "Requested/validated RAW stream:\n";
    std::cerr << "  size: " << sc.size.width << "x" << sc.size.height << "\n";
    std::cerr << "  pixelFormat: " << sc.pixelFormat.toString() << "\n";
    std::cerr << "  stride: " << sc.stride << " bytes/row (0 means unknown)\n";
    std::cerr << "  bufferCount: " << sc.bufferCount << "\n";

    if (cam->configure(cfg.get())) {
        std::cerr << "Failed to configure camera\n";
        cam->release();
        cm.stop();
        return 1;
    }

    libcamera::Stream *stream = sc.stream();

    // Allocate buffers
    libcamera::FrameBufferAllocator alloc(cam);
    if (alloc.allocate(stream) < 0) {
        std::cerr << "Failed to allocate buffers\n";
        cam->release();
        cm.stop();
        return 1;
    }

    const auto &buffers = alloc.buffers(stream);
    if (buffers.empty()) {
        std::cerr << "No buffers allocated\n";
        cam->release();
        cm.stop();
        return 1;
    }
    std::cerr << "Allocated " << buffers.size() << " buffers\n";

    // Map plane0 for each buffer (minimal)
    std::map<libcamera::FrameBuffer *, MappedPlane> mapped;
    for (const auto &buf_up : buffers) {
        libcamera::FrameBuffer *buf = buf_up.get();
        if (buf->planes().empty()) {
            std::cerr << "Buffer has no planes\n";
            continue;
        }
        const auto &p0 = buf->planes()[0];
        void *addr = ::mmap(nullptr, p0.length, PROT_READ, MAP_SHARED, p0.fd.get(), p0.offset);
        if (addr == MAP_FAILED) {
            std::perror("mmap");
            continue;
        }
        mapped[buf] = {addr, p0.length};
    }

    // Create one request per buffer and queue them
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    requests.reserve(buffers.size());
    for (const auto &buf_up : buffers) {
        libcamera::FrameBuffer *buf = buf_up.get();
        std::unique_ptr<libcamera::Request> req = cam->createRequest();
        if (!req) {
            std::cerr << "Failed to create request\n";
            continue;
        }
        if (req->addBuffer(stream, buf) != 0) {
            std::cerr << "Failed to add buffer to request\n";
            continue;
        }
        requests.push_back(std::move(req));
    }

    if (requests.empty()) {
        std::cerr << "No requests created\n";
        // cleanup
        for (auto &kv : mapped) ::munmap(kv.second.addr, kv.second.len);
        cam->release();
        cm.stop();
        return 1;
    }

    // Start camera
    if (cam->start() < 0) {
        std::cerr << "Failed to start camera\n";
        for (auto &kv : mapped) ::munmap(kv.second.addr, kv.second.len);
        cam->release();
        cm.stop();
        return 1;
    }

    // Queue all requests
    for (auto &r : requests) cam->queueRequest(r.get());

    // Minimal completion loop: poll until any request completes successfully.
    // NOTE: This is the least code. A "proper" app uses an event loop + signals.
    // Here we do a crude busy-wait on request status by sleeping a bit.
    libcamera::Request *completed = nullptr;

    for (int tries = 0; tries < 500; ++tries) { // ~5s if 10ms sleep
        for (auto &r : requests) {
            if (r->status() == libcamera::Request::RequestComplete) {
                completed = r.get();
                break;
            }
        }
        if (completed) break;
        ::usleep(10 * 1000);
    }

    if (!completed) {
        std::cerr << "Timed out waiting for a completed request\n";
        cam->stop();
        for (auto &kv : mapped) ::munmap(kv.second.addr, kv.second.len);
        cam->release();
        cm.stop();
        return 1;
    }

    // Grab plane0 bytes
    auto it = completed->buffers().find(stream);
    if (it == completed->buffers().end()) {
        std::cerr << "Completed request has no buffer for the RAW stream\n";
        cam->stop();
        for (auto &kv : mapped) ::munmap(kv.second.addr, kv.second.len);
        cam->release();
        cm.stop();
        return 1;
    }

    libcamera::FrameBuffer *buf = it->second;
    auto mit = mapped.find(buf);
    if (mit == mapped.end() || !mit->second.addr) {
        std::cerr << "Buffer not mapped\n";
        cam->stop();
        for (auto &kv : mapped) ::munmap(kv.second.addr, kv.second.len);
        cam->release();
        cm.stop();
        return 1;
    }

    std::cerr << "Got one RAW frame.\n";
    std::cerr << "  pixelFormat: " << sc.pixelFormat.toString() << "\n";
    std::cerr << "  size: " << sc.size.width << "x" << sc.size.height << "\n";
    std::cerr << "  stride: " << sc.stride << "\n";
    std::cerr << "  plane0 length: " << mit->second.len << " bytes\n";

    const uint8_t *bytes = static_cast<const uint8_t *>(mit->second.addr);
    if (!write_file(out_path, bytes, mit->second.len)) {
        std::cerr << "Failed writing " << out_path << "\n";
    } else {
        std::cerr << "Wrote " << mit->second.len << " bytes to " << out_path << "\n";
    }

    // Shutdown
    cam->stop();
    for (auto &kv : mapped) ::munmap(kv.second.addr, kv.second.len);
    cam->release();
    cm.stop();
    return 0;
}

