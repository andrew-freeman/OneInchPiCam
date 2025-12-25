#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "protocol.h"

namespace rawudp {

enum class Pattern {
    MonoRamp,
    ColorBars,
    Checkerboard,
    MovingBox,
    FileReplay,
};

Pattern parse_pattern(const std::string& name);

bool load_raw_file(const std::string& path, int width, int height, std::vector<uint16_t>& buffer);

std::vector<uint16_t> generate_frame(Pattern pattern, int width, int height, uint16_t bit_depth,
                                     PixelFormat format, uint32_t frame_id,
                                     const std::vector<uint16_t>* file_frame = nullptr);

} // namespace rawudp
