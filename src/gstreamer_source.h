#pragma once

#include <memory>
#include <string>

#include "frame_source.h"

namespace rawudp {

struct GStreamerConfig {
    int camera_index = 0;          // libcamerasrc camera-id
    int width = 0;                // 0 = sensor default
    int height = 0;               // 0 = sensor default
    int fps = 0;                  // 0 = libcamera default
    std::string format = "rggb16"; // video/x-bayer format string
    uint16_t container_bits = 16;
    uint16_t effective_bits = 12;
};


// Creates a GStreamer-based frame source using libcamerasrc -> appsink.
// Returns nullptr if GStreamer is unavailable or initialization fails.
std::unique_ptr<FrameSource> make_gstreamer_source(const GStreamerConfig& cfg);

} // namespace rawudp

