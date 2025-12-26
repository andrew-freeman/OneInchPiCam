#pragma once

#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace rawudp {

constexpr uint32_t kMagic = 0x52415731; // 'RAW1'
constexpr uint16_t kVersion = 1;

enum class PixelFormat : uint8_t {
    MONO = 0,
    RGGB = 1,
    BGGR = 2,
    GRBG = 3,
    GBRG = 4,
};

enum class Packing : uint8_t {
    UNPACKED_16 = 0,
    PACKED_12 = 1,
};

enum class RawEncoding : uint8_t {
    RawUncompressed = 0,
    RawPiSpComp1 = 1,
};

#pragma pack(push, 1)
struct RawUdpHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;

    uint32_t flow_id;
    uint32_t frame_id;

    uint32_t fragment_id;
    uint32_t fragment_count;

    uint16_t width;
    uint16_t height;

    uint8_t bit_depth;
    uint8_t raw_encoding;
    uint8_t pixel_format;
    uint8_t packing;
    uint8_t reserved;

    uint64_t timestamp_us;

    uint32_t payload_offset;
    uint32_t payload_size;

    uint32_t header_crc32;
    uint32_t payload_crc32;
};
#pragma pack(pop)

inline constexpr uint16_t rawudp_header_size() {
    return static_cast<uint16_t>(sizeof(RawUdpHeader));
}

inline std::string pixel_format_to_string(PixelFormat fmt) {
    switch (fmt) {
    case PixelFormat::MONO: return "MONO";
    case PixelFormat::RGGB: return "RGGB";
    case PixelFormat::BGGR: return "BGGR";
    case PixelFormat::GRBG: return "GRBG";
    case PixelFormat::GBRG: return "GBRG";
    }
    return "UNKNOWN";
}

inline PixelFormat parse_pixel_format(const std::string& value) {
    if (value == "mono" || value == "MONO") return PixelFormat::MONO;
    if (value == "rggb" || value == "RGGB") return PixelFormat::RGGB;
    if (value == "bggr" || value == "BGGR") return PixelFormat::BGGR;
    if (value == "grbg" || value == "GRBG") return PixelFormat::GRBG;
    if (value == "gbrg" || value == "GBRG") return PixelFormat::GBRG;
    throw std::invalid_argument("Unknown pixel format: " + value);
}

inline uint16_t host_to_net16(uint16_t v) { return htons(v); }
inline uint32_t host_to_net32(uint32_t v) { return htonl(v); }
inline uint64_t host_to_net64(uint64_t v) {
    // htonll is not portable; compose manually.
    uint32_t hi = htonl(static_cast<uint32_t>(v >> 32));
    uint32_t lo = htonl(static_cast<uint32_t>(v & 0xffffffffULL));
    return (static_cast<uint64_t>(lo) << 32) | hi;
}

inline uint16_t net_to_host16(uint16_t v) { return ntohs(v); }
inline uint32_t net_to_host32(uint32_t v) { return ntohl(v); }
inline uint64_t net_to_host64(uint64_t v) {
    uint32_t hi = ntohl(static_cast<uint32_t>(v >> 32));
    uint32_t lo = ntohl(static_cast<uint32_t>(v & 0xffffffffULL));
    return (static_cast<uint64_t>(hi) << 32) | lo;
}

inline void zero_header_crc(RawUdpHeader& hdr) {
    hdr.header_crc32 = 0;
}

} // namespace rawudp
