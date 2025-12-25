#include "libcamera_source.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>

#ifdef RAWUDP_HAVE_LIBCAMERA

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/pixel_format.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <sys/mman.h>

#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rawudp {

namespace {

uint64_t monotonic_timestamp_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

PixelFormat parse_bayer_format(const std::string& fmt) {
    if (fmt.find("RGGB") != std::string::npos) return PixelFormat::RGGB;
    if (fmt.find("BGGR") != std::string::npos) return PixelFormat::BGGR;
    if (fmt.find("GRBG") != std::string::npos) return PixelFormat::GRBG;
    if (fmt.find("GBRG") != std::string::npos) return PixelFormat::GBRG;
    return PixelFormat::MONO;
}

bool is_packed_raw(const std::string& fmt) {
    return fmt.find("CSI2P") != std::string::npos || fmt.find("P") == fmt.size() - 1;
}

int packed_bits(const std::string& fmt) {
    if (fmt.find("10") != std::string::npos) return 10;
    if (fmt.find("12") != std::string::npos) return 12;
    return 0;
}

void write_le16(std::vector<uint8_t>& dst, std::size_t idx, uint16_t value) {
    dst[idx * 2 + 0] = static_cast<uint8_t>(value & 0xff);
    dst[idx * 2 + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

bool unpack12(const uint8_t* src, std::size_t src_len, std::size_t pixel_count,
              std::vector<uint8_t>& out) {
    out.assign(pixel_count * 2, 0);
    std::size_t si = 0;
    std::size_t di = 0;
    while (di + 1 < pixel_count * 2 && si + 2 < src_len) {
        const uint8_t b0 = src[si + 0];
        const uint8_t b1 = src[si + 1];
        const uint8_t b2 = src[si + 2];
        uint16_t p0 = static_cast<uint16_t>(b0 | ((b1 & 0x0f) << 8));
        uint16_t p1 = static_cast<uint16_t>((b1 >> 4) | (b2 << 4));
        write_le16(out, di / 2, p0);
        write_le16(out, di / 2 + 1, p1);
        si += 3;
        di += 4;
    }
    return true;
}

bool unpack10(const uint8_t* src, std::size_t src_len, std::size_t pixel_count,
              std::vector<uint8_t>& out) {
    out.assign(pixel_count * 2, 0);
    std::size_t si = 0;
    std::size_t di = 0;
    while (di + 1 < pixel_count * 2 && si + 4 < src_len) {
        const uint8_t b0 = src[si + 0];
        const uint8_t b1 = src[si + 1];
        const uint8_t b2 = src[si + 2];
        const uint8_t b3 = src[si + 3];
        const uint8_t b4 = src[si + 4];
        uint16_t p0 = static_cast<uint16_t>(b0 | ((b1 & 0x03) << 8));
        uint16_t p1 = static_cast<uint16_t>((b1 >> 2) | ((b2 & 0x0f) << 6));
        uint16_t p2 = static_cast<uint16_t>((b2 >> 4) | ((b3 & 0x3f) << 4));
        uint16_t p3 = static_cast<uint16_t>((b3 >> 6) | (b4 << 2));
        write_le16(out, di / 2 + 0, p0);
        if (di / 2 + 1 < pixel_count) write_le16(out, di / 2 + 1, p1);
        if (di / 2 + 2 < pixel_count) write_le16(out, di / 2 + 2, p2);
        if (di / 2 + 3 < pixel_count) write_le16(out, di / 2 + 3, p3);
        si += 5;
        di += 8;
    }
    return true;
}

uint16_t detect_container_bits(const std::string& fmt, uint16_t cli_bits) {
    if (fmt.find("16") != std::string::npos) return 16;
    if (fmt.find("14") != std::string::npos) return 14;
    if (fmt.find("12") != std::string::npos) return 16; // converted to 16-bit container
    if (fmt.find("10") != std::string::npos) return 16; // converted to 16-bit container
    return cli_bits;
}

class LibcameraSource : public FrameSource {
public:
    explicit LibcameraSource(const LibcameraConfig& cfg);
    ~LibcameraSource();

    bool start();
    bool get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                   FrameMeta& meta) override;

private:
    struct BufferMap {
        std::vector<void*> planes;
        std::vector<size_t> lengths;
    };

    struct CapturedFrame {
        std::vector<uint8_t> data;
        uint64_t timestamp_us = 0;
    };

    void handle_request(libcamera::Request* request);
    bool map_buffer(libcamera::FrameBuffer* buffer);
    void unmap_all();
    bool copy_buffer(libcamera::FrameBuffer* buffer, std::vector<uint8_t>& out);
    void choose_format(libcamera::StreamConfiguration& cfg);

    LibcameraConfig cfg_;
    std::unique_ptr<libcamera::CameraManager> manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    libcamera::Stream* stream_ = nullptr;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::map<libcamera::FrameBuffer*, BufferMap> mapped_buffers_;

    FrameMeta meta_;
    std::deque<CapturedFrame> ready_frames_;
    std::vector<uint8_t> current_frame_;

    std::mutex mutex_;
    std::condition_variable cv_;
    bool started_ = false;
    bool valid_ = false;
};

LibcameraSource::LibcameraSource(const LibcameraConfig& cfg) : cfg_(cfg) {}

LibcameraSource::~LibcameraSource() {
    if (camera_ && started_) {
        camera_->stop();
    }
    unmap_all();
    requests_.clear();
    allocator_.reset();
    if (camera_) {
        camera_->release();
    }
    if (manager_) {
        manager_->stop();
    }
}

void LibcameraSource::unmap_all() {
    for (auto& kv : mapped_buffers_) {
        for (std::size_t i = 0; i < kv.second.planes.size(); ++i) {
            if (kv.second.planes[i]) {
                munmap(kv.second.planes[i], kv.second.lengths[i]);
            }
        }
    }
    mapped_buffers_.clear();
}

void LibcameraSource::choose_format(libcamera::StreamConfiguration& cfg) {
    const libcamera::StreamFormats& formats = cfg.formats();
    for (const auto& fmt : formats.pixelformats()) {
        const std::string name = fmt.toString();
        if (name.find("16") != std::string::npos) {
            cfg.pixelFormat = fmt;
            return;
        }
    }
    // Fall back to default provided by libcamera.
}

bool LibcameraSource::start() {
    manager_ = std::make_unique<libcamera::CameraManager>();
    if (manager_->start()) {
        std::cerr << "Failed to start CameraManager\n";
        return false;
    }

    const auto& cams = manager_->cameras();
    if (cfg_.camera_index < 0 || static_cast<std::size_t>(cfg_.camera_index) >= cams.size()) {
        std::cerr << "Invalid camera index " << cfg_.camera_index << "\n";
        return false;
    }

    camera_ = cams[cfg_.camera_index];
    if (!camera_) {
        std::cerr << "Camera not available\n";
        return false;
    }
    if (camera_->acquire()) {
        std::cerr << "Failed to acquire camera\n";
        return false;
    }

    // Request a RAW stream role to pull Bayer data directly.
    config_ = camera_->generateConfiguration({libcamera::StreamRole::Raw});
    if (!config_ || config_->empty()) {
        std::cerr << "Failed to generate configuration\n";
        return false;
    }
    libcamera::StreamConfiguration& stream_cfg = config_->at(0);
    if (cfg_.width > 0 && cfg_.height > 0) {
        stream_cfg.size.width = cfg_.width;
        stream_cfg.size.height = cfg_.height;
    }
    choose_format(stream_cfg); // Prefer 16-bit container when offered.
    if (config_->validate() == libcamera::CameraConfiguration::Status::Invalid) {
        std::cerr << "Camera configuration invalid\n";
        return false;
    }
    if (camera_->configure(config_.get())) {
        std::cerr << "Failed to configure camera\n";
        return false;
    }

    stream_ = stream_cfg.stream();
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_) < 0) {
        std::cerr << "Failed to allocate buffers\n";
        return false;
    }

    for (const std::unique_ptr<libcamera::FrameBuffer>& buffer : allocator_->buffers(stream_)) {
        if (!map_buffer(buffer.get())) {
            std::cerr << "Failed to mmap buffer\n";
            return false;
        }
        std::unique_ptr<libcamera::Request> req = camera_->createRequest();
        if (!req) {
            std::cerr << "Failed to create request\n";
            return false;
        }
        if (req->addBuffer(stream_, buffer.get()) != 0) {
            std::cerr << "Failed to add buffer to request\n";
            return false;
        }
        requests_.push_back(std::move(req));
    }

    meta_.width = stream_cfg.size.width;
    meta_.height = stream_cfg.size.height;
    meta_.pixel_format = parse_bayer_format(stream_cfg.pixelFormat.toString());
    meta_.container_bits = detect_container_bits(stream_cfg.pixelFormat.toString(), cfg_.container_bits);
    meta_.effective_bits = cfg_.effective_bits;

    // Use the requestCompleted signal to recycle the small buffer pool continuously.
    camera_->requestCompleted.connect(this, &LibcameraSource::handle_request);
    if (camera_->start() < 0) {
        std::cerr << "Failed to start camera\n";
        return false;
    }
    started_ = true;

    for (auto& req : requests_) {
        camera_->queueRequest(req.get());
    }

    valid_ = true;
    return true;
}

bool LibcameraSource::map_buffer(libcamera::FrameBuffer* buffer) {
    BufferMap map;
    for (const libcamera::FrameBuffer::Plane& plane : buffer->planes()) {
        void* addr =
            mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), plane.offset);
        if (addr == MAP_FAILED) {
            return false;
        }
        map.planes.push_back(addr);
        map.lengths.push_back(plane.length);
    }
    mapped_buffers_[buffer] = map;
    return true;
}

bool LibcameraSource::copy_buffer(libcamera::FrameBuffer* buffer, std::vector<uint8_t>& out) {
    auto it = mapped_buffers_.find(buffer);
    if (it == mapped_buffers_.end() || it->second.planes.empty()) {
        return false;
    }
    const BufferMap& map = it->second;
    const uint8_t* src = static_cast<uint8_t*>(map.planes[0]);
    const std::size_t src_len = map.lengths[0];
    const std::size_t pixel_count = static_cast<std::size_t>(meta_.width) *
                                    static_cast<std::size_t>(meta_.height);

    const std::string fmt = config_->at(0).pixelFormat.toString();
    if (is_packed_raw(fmt)) {
        const int bits = packed_bits(fmt);
        if (bits == 12) {
            return unpack12(src, src_len, pixel_count, out);
        } else if (bits == 10) {
            return unpack10(src, src_len, pixel_count, out);
        }
    }

    const std::size_t bytes_needed = pixel_count * 2;
    out.resize(bytes_needed);
    const std::size_t to_copy = std::min(bytes_needed, src_len);
    std::memcpy(out.data(), src, to_copy);
    if (to_copy < bytes_needed) {
        std::fill(out.begin() + to_copy, out.end(), 0);
    }
    return true;
}

void LibcameraSource::handle_request(libcamera::Request* request) {
    if (request->status() == libcamera::Request::RequestCancelled) {
        return;
    }
    auto it = request->buffers().begin();
    if (it == request->buffers().end()) {
        return;
    }
    libcamera::FrameBuffer* buffer = it->second;

    CapturedFrame frame;
    frame.timestamp_us = monotonic_timestamp_us();
    if (!copy_buffer(buffer, frame.data)) {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_frames_.push_back(std::move(frame));
    }
    cv_.notify_one();

    // Re-queue immediately to keep a small live buffer pool cycling.
    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
}

bool LibcameraSource::get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                                FrameMeta& meta) {
    if (!valid_) {
        return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return !ready_frames_.empty(); });
    CapturedFrame frame = std::move(ready_frames_.front());
    ready_frames_.pop_front();
    lock.unlock();

    current_frame_ = std::move(frame.data);
    data = current_frame_.data();
    size_bytes = current_frame_.size();
    timestamp_us = frame.timestamp_us;
    meta = meta_;
    return true;
}

bool list_cameras() {
    libcamera::CameraManager mgr;
    if (mgr.start()) {
        std::cerr << "Failed to start CameraManager for listing\n";
        return false;
    }
    const auto& cams = mgr.cameras();
    if (cams.empty()) {
        std::cout << "No cameras found\n";
    }
    for (std::size_t i = 0; i < cams.size(); ++i) {
        std::cout << "[" << i << "] " << cams[i]->id() << "\n";
    }
    mgr.stop();
    return true;
}

std::unique_ptr<FrameSource> make_libcamera_source(const LibcameraConfig& cfg) {
    std::unique_ptr<LibcameraSource> src = std::make_unique<LibcameraSource>(cfg);
    if (!src->start()) {
        return nullptr;
    }
    return src;
}

} // namespace rawudp

#else // RAWUDP_HAVE_LIBCAMERA

namespace rawudp {

bool list_cameras() {
    std::cerr << "libcamera support not built; rebuild with RAWUDP_HAVE_LIBCAMERA\n";
    return false;
}

std::unique_ptr<FrameSource> make_libcamera_source(const LibcameraConfig&) {
    std::cerr << "libcamera support not built; rebuild with RAWUDP_HAVE_LIBCAMERA\n";
    return nullptr;
}

} // namespace rawudp

#endif
