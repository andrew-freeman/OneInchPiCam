#pragma once

#include <cstdint>
#include <vector>

#include "protocol.h"
#include "reassembler.h"

namespace rawudp {

struct DecodedFrame {
    uint16_t width = 0;
    uint16_t height = 0;
    uint16_t stride_pixels = 0;
    uint8_t bit_depth = 0;
    uint8_t pixel_format = 0;
    std::vector<uint16_t> pixels;
};

// Decode a completed RAW frame into an unpacked 16-bit buffer.
// Returns false on decode or validation failure.
bool decode_frame(const CompletedFrame& in, DecodedFrame& out);

} // namespace rawudp

