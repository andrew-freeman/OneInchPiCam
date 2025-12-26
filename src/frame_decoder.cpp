#include "frame_decoder.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace rawudp {
namespace {

constexpr uint16_t kCompressOffset = 2048;
constexpr int kCompressMode = 1;

uint8_t effective_bits(const RawUdpHeader& header) {
    uint8_t bits = header.reserved != 0 ? header.reserved : header.bit_depth;
    return std::clamp<uint8_t>(bits, 1, 16);
}

RawEncoding parse_raw_encoding(uint8_t value) {
    switch (static_cast<RawEncoding>(value)) {
    case RawEncoding::RawUncompressed:
    case RawEncoding::RawPiSpComp1:
        return static_cast<RawEncoding>(value);
    default:
        return RawEncoding::RawUncompressed;
    }
}

uint16_t postprocess(uint16_t a) {
    if (kCompressMode & 2) {
        if (kCompressMode == 3 && a < 0x4000)
            a = a >> 2;
        else if (a < 0x1000)
            a = a >> 4;
        else if (a < 0x1800)
            a = (a - 0x800) >> 3;
        else if (a < 0x3000)
            a = (a - 0x1000) >> 2;
        else if (a < 0x6000)
            a = (a - 0x2000) >> 1;
        else if (a < 0xC000)
            a = (a - 0x4000);
        else
            a = 2 * (a - 0x8000);
    }

    return std::min<uint16_t>(0xFFFF, static_cast<uint16_t>(a + kCompressOffset));
}

uint16_t dequantize(uint16_t q, int qmode) {
    switch (qmode) {
    case 0:
        return (q < 320) ? static_cast<uint16_t>(16 * q) : static_cast<uint16_t>(32 * (q - 160));

    case 1:
        return static_cast<uint16_t>(64 * q);

    case 2:
        return static_cast<uint16_t>(128 * q);

    default:
        return (q < 94) ? static_cast<uint16_t>(256 * q)
                        : std::min<uint16_t>(0xFFFF, static_cast<uint16_t>(512 * (q - 47)));
    }
}

void sub_block_function(uint16_t* d, uint32_t w) {
    int q[4];

    int qmode = (w & 3);
    if (qmode < 3) {
        int field0 = (w >> 2) & 511;
        int field1 = (w >> 11) & 127;
        int field2 = (w >> 18) & 127;
        int field3 = (w >> 25) & 127;
        if (qmode == 2 && field0 >= 384) {
            q[1] = field0;
            q[2] = field1 + 384;
        } else {
            q[1] = (field1 >= 64) ? field0 : field0 + 64 - field1;
            q[2] = (field1 >= 64) ? field0 + field1 - 64 : field0;
        }
        int p1 = std::max(0, q[1] - 64);
        if (qmode == 2)
            p1 = std::min(384, p1);
        int p2 = std::max(0, q[2] - 64);
        if (qmode == 2)
            p2 = std::min(384, p2);
        q[0] = p1 + field2;
        q[3] = p2 + field3;
    } else {
        int pack0 = (w >> 2) & 32767;
        int pack1 = (w >> 17) & 32767;
        q[0] = (pack0 & 15) + 16 * ((pack0 >> 8) / 11);
        q[1] = (pack0 >> 4) % 176;
        q[2] = (pack1 & 15) + 16 * ((pack1 >> 8) / 11);
        q[3] = (pack1 >> 4) % 176;
    }

    d[0] = dequantize(static_cast<uint16_t>(q[0]), qmode);
    d[2] = dequantize(static_cast<uint16_t>(q[1]), qmode);
    d[4] = dequantize(static_cast<uint16_t>(q[2]), qmode);
    d[6] = dequantize(static_cast<uint16_t>(q[3]), qmode);
}

bool decode_uncompressed(const CompletedFrame& in, DecodedFrame& out) {
    if (static_cast<Packing>(in.header.packing) != Packing::UNPACKED_16) {
        std::cerr << "Unsupported packing for uncompressed RAW\n";
        return false;
    }

    const std::size_t expected =
        static_cast<std::size_t>(in.header.width) * in.header.height * 2;
    if (in.data.size() < expected) {
        std::cerr << "Frame data too small for uncompressed decode\n";
        return false;
    }

    out.width = in.header.width;
    out.height = in.header.height;
    out.stride_pixels = in.header.width;
    out.bit_depth = effective_bits(in.header);
    out.pixel_format = in.header.pixel_format;
    out.pixels.resize(static_cast<std::size_t>(out.stride_pixels) * out.height);
    for (std::size_t i = 0; i < static_cast<std::size_t>(out.width) * out.height; ++i) {
        out.pixels[i] =
            static_cast<uint16_t>(in.data[i * 2] | (static_cast<uint16_t>(in.data[i * 2 + 1]) << 8));
    }
    return true;
}

bool decode_pisp_comp1(const CompletedFrame& in, DecodedFrame& out) {
    if (in.header.width == 0 || in.header.height == 0) {
        std::cerr << "Invalid PiSP frame dimensions\n";
        return false;
    }
    if (in.data.empty()) {
        std::cerr << "Empty PiSP frame data\n";
        return false;
    }

    const std::size_t stride_bytes = in.data.size() / in.header.height;
    if (stride_bytes * in.header.height != in.data.size()) {
        std::cerr << "PiSP frame data not evenly divisible by height\n";
        return false;
    }
    const std::size_t min_stride =
        ((static_cast<std::size_t>(in.header.width) + 7) / 8) * 8;
    if (stride_bytes < min_stride) {
        std::cerr << "PiSP stride smaller than minimum expected\n";
        return false;
    }

    const std::size_t buf_stride_pixels = (static_cast<std::size_t>(in.header.width) + 7) & ~7u;
    out.width = in.header.width;
    out.height = in.header.height;
    out.stride_pixels = static_cast<uint16_t>(buf_stride_pixels);
    out.bit_depth = effective_bits(in.header);
    out.pixel_format = in.header.pixel_format;
    out.pixels.assign(buf_stride_pixels * out.height, 0);

    for (uint16_t y = 0; y < out.height; ++y) {
        uint16_t* dp = out.pixels.data() + static_cast<std::size_t>(y) * buf_stride_pixels;
        const uint8_t* sp = in.data.data() + static_cast<std::size_t>(y) * stride_bytes;
        std::size_t consumed = 0;

        for (uint16_t x = 0; x < out.width; x += 8) {
            if (consumed + 8 > stride_bytes) {
                std::cerr << "PiSP frame truncated while decoding\n";
                return false;
            }
            uint32_t w0 = 0, w1 = 0;
            for (int b = 0; b < 4; ++b) {
                w0 |= static_cast<uint32_t>(sp[consumed + b]) << (b * 8);
                w1 |= static_cast<uint32_t>(sp[consumed + 4 + b]) << (b * 8);
            }
            consumed += 8;
            sub_block_function(dp, w0);
            sub_block_function(dp + 1, w1);
            for (int i = 0; i < 8; ++i, ++dp) {
                *dp = postprocess(*dp);
            }
        }
    }

    return true;
}

} // namespace

bool decode_frame(const CompletedFrame& in, DecodedFrame& out) {
    const RawEncoding encoding = parse_raw_encoding(in.header.raw_encoding);
    switch (encoding) {
    case RawEncoding::RawUncompressed:
        return decode_uncompressed(in, out);
    case RawEncoding::RawPiSpComp1:
        return decode_pisp_comp1(in, out);
    default:
        std::cerr << "Unknown raw encoding\n";
        return false;
    }
}

} // namespace rawudp

