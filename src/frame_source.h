#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "generator.h"
#include "protocol.h"

namespace rawudp {

struct FrameMeta {
    int width = 0;
    int height = 0;
    PixelFormat pixel_format = PixelFormat::RGGB;
    RawEncoding raw_encoding = RawEncoding::RawUncompressed;
    uint16_t container_bits = 16;
    uint16_t effective_bits = 12;
};

struct FrameSource {
    virtual ~FrameSource() = default;
    // Returns true if a frame is produced, false on end/error.
    virtual bool get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                           FrameMeta& meta) = 0;
};

// Replays a single RAW file as an infinite stream of frames.
class RawFileSource : public FrameSource {
public:
    RawFileSource(const std::string& path, int width, int height, PixelFormat fmt,
                  uint16_t container_bits, uint16_t effective_bits);

    bool valid() const { return valid_; }

    bool get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                   FrameMeta& meta) override;

private:
    bool valid_ = false;
    int width_ = 0;
    int height_ = 0;
    PixelFormat pixel_format_ = PixelFormat::RGGB;
    uint16_t container_bits_ = 16;
    uint16_t effective_bits_ = 12;
    std::vector<uint8_t> frame_bytes_;
};

// Generates synthetic frames (or file replay) using existing generator helpers.
class SyntheticSource : public FrameSource {
public:
    SyntheticSource(Pattern pattern, int width, int height, PixelFormat fmt,
                    uint16_t container_bits, uint16_t effective_bits,
                    const std::vector<uint16_t>* replay_frame = nullptr);

    bool get_frame(uint8_t*& data, size_t& size_bytes, uint64_t& timestamp_us,
                   FrameMeta& meta) override;

private:
    Pattern pattern_;
    int width_;
    int height_;
    PixelFormat pixel_format_;
    uint16_t container_bits_;
    uint16_t effective_bits_;
    uint32_t frame_id_ = 0;
    const std::vector<uint16_t>* replay_frame_;
    std::vector<uint8_t> frame_bytes_;
};

} // namespace rawudp
