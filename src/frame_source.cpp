#include "frame_source.h"

#include <chrono>
#include <iostream>

namespace rawudp {

namespace {

uint64_t monotonic_timestamp_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

void u16_to_bytes(const std::vector<uint16_t>& src, std::vector<uint8_t>& dst) {
    dst.resize(src.size() * 2);
    for (std::size_t i = 0; i < src.size(); ++i) {
        const uint16_t v = src[i];
        dst[i * 2 + 0] = static_cast<uint8_t>(v & 0xff);
        dst[i * 2 + 1] = static_cast<uint8_t>((v >> 8) & 0xff);
    }
}

} // namespace

RawFileSource::RawFileSource(const std::string& path, int width, int height, PixelFormat fmt,
                             uint16_t container_bits, uint16_t effective_bits)
    : width_(width),
      height_(height),
      pixel_format_(fmt),
      container_bits_(container_bits),
      effective_bits_(effective_bits) {
    std::vector<uint16_t> frame;
    if (!load_raw_file(path, width_, height_, frame)) {
        std::cerr << "Failed to load raw file for replay\n";
        return;
    }
    u16_to_bytes(frame, frame_bytes_);
    valid_ = true;
}

bool RawFileSource::get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                              FrameMeta& meta) {
    if (!valid_) {
        return false;
    }
    data = frame_bytes_.data();
    size_bytes = frame_bytes_.size();
    timestamp_us = monotonic_timestamp_us();
    meta.width = width_;
    meta.height = height_;
    meta.pixel_format = pixel_format_;
    meta.container_bits = container_bits_;
    meta.effective_bits = effective_bits_;
    return true;
}

SyntheticSource::SyntheticSource(Pattern pattern, int width, int height, PixelFormat fmt,
                                 uint16_t container_bits, uint16_t effective_bits,
                                 const std::vector<uint16_t>* replay_frame)
    : pattern_(pattern),
      width_(width),
      height_(height),
      pixel_format_(fmt),
      container_bits_(container_bits),
      effective_bits_(effective_bits),
      replay_frame_(replay_frame) {}

bool SyntheticSource::get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                                FrameMeta& meta) {
    const std::vector<uint16_t> frame =
        generate_frame(pattern_, width_, height_, effective_bits_, pixel_format_, frame_id_++,
                       replay_frame_);
    u16_to_bytes(frame, frame_bytes_);

    data = frame_bytes_.data();
    size_bytes = frame_bytes_.size();
    timestamp_us = monotonic_timestamp_us();

    meta.width = width_;
    meta.height = height_;
    meta.pixel_format = pixel_format_;
    meta.container_bits = container_bits_;
    meta.effective_bits = effective_bits_;
    return true;
}

} // namespace rawudp
