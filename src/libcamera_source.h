#pragma once

#include <memory>

#include "frame_source.h"

namespace rawudp {

struct LibcameraConfig {
    int camera_index = 0;
    int width = 0;
    int height = 0;
    uint16_t container_bits = 16;
    uint16_t effective_bits = 12;
};

// Enumerate attached cameras. Returns true on success.
bool list_cameras();

// Creates a libcamera-backed frame source. Returns nullptr if unavailable.
std::unique_ptr<FrameSource> make_libcamera_source(const LibcameraConfig& cfg);

} // namespace rawudp
