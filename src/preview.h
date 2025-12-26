#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "frame_decoder.h"

namespace rawudp {

enum class ViewMode {
    Mono,
    GreenOnly,
    HalfRes,
    Bilinear,
};

struct PreviewConfig {
    ViewMode mode = ViewMode::Mono;
    uint16_t black_level = 0;
    float wb_r = 1.0f;
    float wb_g = 1.0f;
    float wb_b = 1.0f;
    float gamma = 1.0f;
    bool headless = false;
};

// Converts a raw frame (little-endian UNPACKED_16) into an 8-bit BGR preview buffer.
// Returns false if preview cannot be generated (e.g., missing OpenCV when display requested).
bool render_preview(const DecodedFrame& frame, const PreviewConfig& cfg,
                    std::vector<uint8_t>& bgr, int& out_width, int& out_height);

bool display_preview(const std::vector<uint8_t>& bgr, int width, int height,
                     const std::string& window_name, bool headless);

} // namespace rawudp
