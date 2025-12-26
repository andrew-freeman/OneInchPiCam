#include "preview.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#ifdef RAWUDP_HAVE_OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace rawudp {

namespace {

enum class Channel { R, G, B };

double max_value_for_bits(uint8_t bits) {
    if (bits >= 16) {
        return 65535.0;
    }
    return static_cast<double>((1u << bits) - 1u);
}

Channel channel_at(int x, int y, PixelFormat fmt) {
    const bool even_row = (y % 2) == 0;
    const bool even_col = (x % 2) == 0;
    switch (fmt) {
    case PixelFormat::RGGB:
        return even_row ? (even_col ? Channel::R : Channel::G) : (even_col ? Channel::G : Channel::B);
    case PixelFormat::BGGR:
        return even_row ? (even_col ? Channel::B : Channel::G) : (even_col ? Channel::G : Channel::R);
    case PixelFormat::GRBG:
        return even_row ? (even_col ? Channel::G : Channel::R) : (even_col ? Channel::B : Channel::G);
    case PixelFormat::GBRG:
        return even_row ? (even_col ? Channel::G : Channel::B) : (even_col ? Channel::R : Channel::G);
    case PixelFormat::MONO:
        return Channel::G;
    }
    return Channel::G;
}

uint8_t tone_map(double value, double max_value, const PreviewConfig& cfg, float gain) {
    const double adjusted = std::max(0.0, value) * gain;
    const double clamped = std::min(adjusted, max_value);
    double normalized = clamped / std::max(1.0, max_value);
    if (cfg.gamma > 1.001f || cfg.gamma < 0.999f) {
        normalized = std::pow(normalized, 1.0 / cfg.gamma);
    }
    int scaled = static_cast<int>(std::round(normalized * 255.0));
    return static_cast<uint8_t>(std::clamp(scaled, 0, 255));
}

void mono_view(const DecodedFrame& frame, const PreviewConfig& cfg, std::vector<uint8_t>& bgr,
               double maxv) {
    const std::size_t pixel_count = static_cast<std::size_t>(frame.width) * frame.height;
    bgr.resize(pixel_count * 3);
    for (int y = 0; y < frame.height; ++y) {
        for (int x = 0; x < frame.width; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y) * frame.stride_pixels + x;
            const uint16_t raw = frame.pixels[idx];
            const double value = std::max<int>(0, static_cast<int>(raw) - cfg.black_level);
            uint8_t v = tone_map(value, maxv, cfg, cfg.wb_g);
            const std::size_t out_idx = static_cast<std::size_t>(y) * frame.width + x;
            bgr[out_idx * 3 + 0] = v;
            bgr[out_idx * 3 + 1] = v;
            bgr[out_idx * 3 + 2] = v;
        }
    }
}

void green_only_view(const DecodedFrame& frame, const PreviewConfig& cfg,
                     std::vector<uint8_t>& bgr, double maxv) {
    const std::size_t pixel_count = static_cast<std::size_t>(frame.width) * frame.height;
    bgr.assign(pixel_count * 3, 0);
    for (int y = 0; y < frame.height; ++y) {
        for (int x = 0; x < frame.width; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y) * frame.stride_pixels + x;
            Channel c = channel_at(x, y, static_cast<PixelFormat>(frame.pixel_format));
            if (c != Channel::G) {
                continue;
            }
            const uint16_t raw = frame.pixels[idx];
            const double value = std::max<int>(0, static_cast<int>(raw) - cfg.black_level);
            uint8_t v = tone_map(value, maxv, cfg, cfg.wb_g);
            const std::size_t out_idx = static_cast<std::size_t>(y) * frame.width + x;
            bgr[out_idx * 3 + 0] = v;
            bgr[out_idx * 3 + 1] = v;
            bgr[out_idx * 3 + 2] = v;
        }
    }
}

void half_res_view(const DecodedFrame& frame, const PreviewConfig& cfg, std::vector<uint8_t>& bgr,
                   int& out_w, int& out_h, double maxv) {
    out_w = frame.width / 2;
    out_h = frame.height / 2;
    bgr.assign(static_cast<std::size_t>(out_w) * out_h * 3, 0);
    for (int y = 0; y + 1 < frame.height; y += 2) {
        for (int x = 0; x + 1 < frame.width; x += 2) {
            const std::size_t idx00 = static_cast<std::size_t>(y) * frame.stride_pixels + x;
            const std::size_t idx01 = idx00 + 1;
            const std::size_t idx10 = idx00 + frame.stride_pixels;
            const std::size_t idx11 = idx10 + 1;
            const uint16_t p00 = frame.pixels[idx00];
            const uint16_t p01 = frame.pixels[idx01];
            const uint16_t p10 = frame.pixels[idx10];
            const uint16_t p11 = frame.pixels[idx11];

            double r = 0, g = 0, b = 0;
            const PixelFormat fmt = static_cast<PixelFormat>(frame.pixel_format);
            Channel c00 = channel_at(x, y, fmt);
            Channel c01 = channel_at(x + 1, y, fmt);
            Channel c10 = channel_at(x, y + 1, fmt);
            Channel c11 = channel_at(x + 1, y + 1, fmt);

            auto accumulate = [&](Channel c, uint16_t v) {
                switch (c) {
                case Channel::R: r += v; break;
                case Channel::G: g += v; break;
                case Channel::B: b += v; break;
                }
            };
            accumulate(c00, p00);
            accumulate(c01, p01);
            accumulate(c10, p10);
            accumulate(c11, p11);

            r = tone_map(r / 1.0, maxv, cfg, cfg.wb_r);
            g = tone_map(g / 2.0, maxv, cfg, cfg.wb_g); // two greens per 2x2
            b = tone_map(b / 1.0, maxv, cfg, cfg.wb_b);

            const std::size_t out_idx = static_cast<std::size_t>(y / 2) * out_w + (x / 2);
            bgr[out_idx * 3 + 0] = static_cast<uint8_t>(b);
            bgr[out_idx * 3 + 1] = static_cast<uint8_t>(g);
            bgr[out_idx * 3 + 2] = static_cast<uint8_t>(r);
        }
    }
}

void bilinear_view(const DecodedFrame& frame, const PreviewConfig& cfg,
                   std::vector<uint8_t>& bgr, double maxv) {
    const int w = frame.width;
    const int h = frame.height;
    bgr.assign(static_cast<std::size_t>(w) * h * 3, 0);
    const PixelFormat fmt = static_cast<PixelFormat>(frame.pixel_format);

    auto sample = [&](int x, int y) -> uint16_t {
        x = std::clamp(x, 0, w - 1);
        y = std::clamp(y, 0, h - 1);
        return frame.pixels[static_cast<std::size_t>(y) * frame.stride_pixels + x];
    };

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            Channel c = channel_at(x, y, fmt);
            double r = 0, g = 0, b = 0;
            uint16_t center = sample(x, y);
            switch (c) {
            case Channel::R:
                r = center;
                g = (sample(x - 1, y) + sample(x + 1, y) + sample(x, y - 1) + sample(x, y + 1)) / 4.0;
                b = (sample(x - 1, y - 1) + sample(x + 1, y - 1) +
                     sample(x - 1, y + 1) + sample(x + 1, y + 1)) / 4.0;
                break;
            case Channel::B:
                b = center;
                g = (sample(x - 1, y) + sample(x + 1, y) + sample(x, y - 1) + sample(x, y + 1)) / 4.0;
                r = (sample(x - 1, y - 1) + sample(x + 1, y - 1) +
                     sample(x - 1, y + 1) + sample(x + 1, y + 1)) / 4.0;
                break;
            case Channel::G:
                g = center;
                if (((y % 2) == 0 && (fmt == PixelFormat::RGGB || fmt == PixelFormat::GRBG)) ||
                    ((y % 2) == 1 && (fmt == PixelFormat::BGGR || fmt == PixelFormat::GBRG))) {
                    r = (sample(x - 1, y) + sample(x + 1, y)) / 2.0;
                    b = (sample(x, y - 1) + sample(x, y + 1)) / 2.0;
                } else {
                    r = (sample(x, y - 1) + sample(x, y + 1)) / 2.0;
                    b = (sample(x - 1, y) + sample(x + 1, y)) / 2.0;
                }
                break;
            }

            const std::size_t idx = static_cast<std::size_t>(y) * w + x;
            bgr[idx * 3 + 0] = tone_map(b, maxv, cfg, cfg.wb_b);
            bgr[idx * 3 + 1] = tone_map(g, maxv, cfg, cfg.wb_g);
            bgr[idx * 3 + 2] = tone_map(r, maxv, cfg, cfg.wb_r);
        }
    }
}

} // namespace

bool render_preview(const DecodedFrame& frame, const PreviewConfig& cfg,
                    std::vector<uint8_t>& bgr, int& out_width, int& out_height) {
    if (frame.bit_depth == 0 || frame.bit_depth > 16) {
        std::cerr << "Unsupported bit depth: effective=" << static_cast<int>(frame.bit_depth) << "\n";
        return false;
    }
    if (frame.pixels.size() < static_cast<std::size_t>(frame.stride_pixels) * frame.height) {
        std::cerr << "Frame buffer too small for preview\n";
        return false;
    }

    out_width = frame.width;
    out_height = frame.height;
    const double maxv = max_value_for_bits(frame.bit_depth);

    switch (cfg.mode) {
    case ViewMode::Mono:
        mono_view(frame, cfg, bgr, maxv);
        break;
    case ViewMode::GreenOnly:
        green_only_view(frame, cfg, bgr, maxv);
        break;
    case ViewMode::HalfRes:
        half_res_view(frame, cfg, bgr, out_width, out_height, maxv);
        break;
    case ViewMode::Bilinear:
        bilinear_view(frame, cfg, bgr, maxv);
        break;
    }
    return true;
}

bool display_preview(const std::vector<uint8_t>& bgr, int width, int height,
                     const std::string& window_name, bool headless) {
    if (headless) {
        return true;
    }
#ifdef RAWUDP_HAVE_OPENCV
    cv::Mat mat(height, width, CV_8UC3, const_cast<uint8_t*>(bgr.data()));
    cv::imshow(window_name, mat);
    cv::waitKey(1);
    return true;
#else
    (void)bgr;
    (void)width;
    (void)height;
    (void)window_name;
    std::cerr << "OpenCV not available; running headless\n";
    return false;
#endif
}

} // namespace rawudp

