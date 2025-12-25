#include "generator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace rawudp {

namespace {

uint16_t max_value(uint16_t bit_depth) {
    if (bit_depth == 0 || bit_depth > 16) {
        return 0xffffu;
    }
    return static_cast<uint16_t>((1u << bit_depth) - 1u);
}

uint16_t clamp16(int value) {
    if (value < 0) return 0;
    if (value > 65535) return 65535;
    return static_cast<uint16_t>(value);
}

} // namespace

Pattern parse_pattern(const std::string& name) {
    if (name == "mono" || name == "mono_ramp" || name == "ramp") return Pattern::MonoRamp;
    if (name == "bars" || name == "color_bars") return Pattern::ColorBars;
    if (name == "checker" || name == "checkerboard") return Pattern::Checkerboard;
    if (name == "box" || name == "moving_box") return Pattern::MovingBox;
    if (name == "file" || name == "replay") return Pattern::FileReplay;
    throw std::invalid_argument("Unknown pattern: " + name);
}

bool load_raw_file(const std::string& path, int width, int height, std::vector<uint16_t>& buffer) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open raw file: " << path << "\n";
        return false;
    }

    const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height) * 2;
    buffer.resize(expected / 2);
    file.read(reinterpret_cast<char*>(buffer.data()), expected);
    if (!file) {
        std::cerr << "Failed to read expected bytes from raw file\n";
        return false;
    }
    return true;
}

static std::vector<uint16_t> generate_mono_ramp(int width, int height, uint16_t bit_depth) {
    std::vector<uint16_t> frame(static_cast<size_t>(width) * static_cast<size_t>(height));
    const uint16_t maxv = max_value(bit_depth);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint16_t value = static_cast<uint16_t>((static_cast<uint32_t>(x) * maxv) / std::max(1, width - 1));
            frame[static_cast<size_t>(y) * width + x] = value;
        }
    }
    return frame;
}

static std::vector<uint16_t> generate_checkerboard(int width, int height, uint16_t bit_depth) {
    std::vector<uint16_t> frame(static_cast<size_t>(width) * static_cast<size_t>(height));
    const uint16_t maxv = max_value(bit_depth);
    const int block = std::max(4, width / 16);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            bool high = ((x / block) + (y / block)) % 2 == 0;
            frame[static_cast<size_t>(y) * width + x] = high ? maxv : maxv / 8;
        }
    }
    return frame;
}

static std::vector<uint16_t> generate_color_bars(int width, int height, uint16_t bit_depth, PixelFormat format) {
    std::vector<uint16_t> frame(static_cast<size_t>(width) * static_cast<size_t>(height));
    const uint16_t maxv = max_value(bit_depth);
    const int bars = 8;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int bar = (x * bars) / std::max(1, width);
            uint16_t r = 0, g = 0, b = 0;
            switch (bar) {
            case 0: r = maxv; g = maxv; b = maxv; break; // white
            case 1: r = maxv; g = maxv; b = 0; break;     // yellow
            case 2: r = 0; g = maxv; b = maxv; break;     // cyan
            case 3: r = 0; g = maxv; b = 0; break;        // green
            case 4: r = maxv; g = 0; b = maxv; break;     // magenta
            case 5: r = maxv; g = 0; b = 0; break;        // red
            case 6: r = 0; g = 0; b = maxv; break;        // blue
            default: r = maxv / 4; g = maxv / 4; b = maxv / 4; break;
            }

            // Write into Bayer plane
            bool even_row = (y % 2) == 0;
            bool even_col = (x % 2) == 0;
            uint16_t value = 0;
            switch (format) {
            case PixelFormat::RGGB:
                value = even_row ? (even_col ? r : g) : (even_col ? g : b);
                break;
            case PixelFormat::BGGR:
                value = even_row ? (even_col ? b : g) : (even_col ? g : r);
                break;
            case PixelFormat::GRBG:
                value = even_row ? (even_col ? g : r) : (even_col ? b : g);
                break;
            case PixelFormat::GBRG:
                value = even_row ? (even_col ? g : b) : (even_col ? r : g);
                break;
            case PixelFormat::MONO:
                value = static_cast<uint16_t>((r + g + b) / 3);
                break;
            }
            frame[static_cast<size_t>(y) * width + x] = value;
        }
    }
    return frame;
}

static std::vector<uint16_t> generate_moving_box(int width, int height, uint16_t bit_depth, uint32_t frame_id) {
    std::vector<uint16_t> frame(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
    const uint16_t maxv = max_value(bit_depth);
    const int box_size = std::max(8, std::min(width, height) / 8);
    const int path = std::max(1, width - box_size);
    const int y_span = std::max(1, height - box_size);
    const int x0 = static_cast<int>(frame_id % static_cast<uint32_t>(path));
    const int y0 =
        static_cast<int>((frame_id / static_cast<uint32_t>(path)) % static_cast<uint32_t>(y_span));
    for (int y = y0; y < y0 + box_size && y < height; ++y) {
        for (int x = x0; x < x0 + box_size && x < width; ++x) {
            frame[static_cast<size_t>(y) * width + x] = maxv;
        }
    }
    return frame;
}

std::vector<uint16_t> generate_frame(Pattern pattern, int width, int height, uint16_t bit_depth,
                                     PixelFormat format, uint32_t frame_id,
                                     const std::vector<uint16_t>* file_frame) {
    switch (pattern) {
    case Pattern::MonoRamp:
        return generate_mono_ramp(width, height, bit_depth);
    case Pattern::ColorBars:
        return generate_color_bars(width, height, bit_depth, format);
    case Pattern::Checkerboard:
        return generate_checkerboard(width, height, bit_depth);
    case Pattern::MovingBox:
        return generate_moving_box(width, height, bit_depth, frame_id);
    case Pattern::FileReplay:
        if (file_frame) {
            return *file_frame;
        }
        throw std::runtime_error("File replay selected but no file data loaded");
    }
    throw std::runtime_error("Unknown pattern");
}

} // namespace rawudp
