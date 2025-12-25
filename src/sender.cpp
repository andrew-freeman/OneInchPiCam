#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "crc32.h"
#include "generator.h"
#include "protocol.h"

using namespace std::chrono;

namespace rawudp {

struct SenderOptions {
    std::string dest_ip = "127.0.0.1";
    uint16_t dest_port = 9000;
    int width = 640;
    int height = 480;
    uint16_t bit_depth = 12;
    PixelFormat pixel_format = PixelFormat::RGGB;
    Packing packing = Packing::UNPACKED_16;
    Pattern pattern = Pattern::MonoRamp;
    uint32_t flow_id = 1;
    uint32_t fps = 30;
    uint32_t payload_size = 1200;
    uint32_t frame_count = 0; // 0 = infinite
    std::string raw_file;
};

void usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "  --dest <ip>            Destination IP (default 127.0.0.1)\n"
              << "  --port <port>          Destination UDP port (default 9000)\n"
              << "  --width <px>           Frame width (default 640)\n"
              << "  --height <px>          Frame height (default 480)\n"
              << "  --bit-depth <bits>     Bit depth (8-16, default 12)\n"
              << "  --pixel-format <fmt>   mono,rggb,bggr,grbg,gbrg (default rggb)\n"
              << "  --pattern <name>       mono, bars, checker, box, file (default mono)\n"
              << "  --payload-size <bytes> Fragment payload size (default 1200)\n"
              << "  --fps <value>          Frames per second (default 30)\n"
              << "  --frames <n>           Number of frames to send (0 = infinite)\n"
              << "  --flow-id <id>         Flow identifier (default 1)\n"
              << "  --raw-file <path>      Replay raw file instead of synthetic pattern\n";
}

bool parse_args(int argc, char** argv, SenderOptions& opts) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + name);
            return argv[++i];
        };

        if (arg == "--dest") {
            opts.dest_ip = require_value(arg);
        } else if (arg == "--port") {
            opts.dest_port = static_cast<uint16_t>(std::stoi(require_value(arg)));
        } else if (arg == "--width") {
            opts.width = std::stoi(require_value(arg));
        } else if (arg == "--height") {
            opts.height = std::stoi(require_value(arg));
        } else if (arg == "--bit-depth") {
            opts.bit_depth = static_cast<uint16_t>(std::stoi(require_value(arg)));
        } else if (arg == "--pixel-format") {
            opts.pixel_format = parse_pixel_format(require_value(arg));
        } else if (arg == "--pattern") {
            opts.pattern = parse_pattern(require_value(arg));
        } else if (arg == "--payload-size") {
            opts.payload_size = static_cast<uint32_t>(std::stoi(require_value(arg)));
        } else if (arg == "--fps") {
            opts.fps = static_cast<uint32_t>(std::stoi(require_value(arg)));
        } else if (arg == "--frames") {
            opts.frame_count = static_cast<uint32_t>(std::stoi(require_value(arg)));
        } else if (arg == "--flow-id") {
            opts.flow_id = static_cast<uint32_t>(std::stoul(require_value(arg)));
        } else if (arg == "--raw-file") {
            opts.raw_file = require_value(arg);
            opts.pattern = Pattern::FileReplay;
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return false;
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return true;
}

bool validate_options(const SenderOptions& opts) {
    if (opts.width <= 0 || opts.height <= 0) {
        std::cerr << "Width and height must be positive\n";
        return false;
    }
    if (opts.payload_size == 0) {
        std::cerr << "Payload size must be greater than zero\n";
        return false;
    }
    if (opts.bit_depth == 0 || opts.bit_depth > 16) {
        std::cerr << "Bit depth must be between 1 and 16\n";
        return false;
    }
    return true;
}

void build_header(RawUdpHeader& hdr, const SenderOptions& opts, uint32_t frame_id,
                  uint32_t fragment_id, uint32_t fragment_count, uint64_t timestamp_us,
                  uint32_t payload_offset, uint32_t payload_size) {
    hdr.magic = host_to_net32(kMagic);
    hdr.version = host_to_net16(kVersion);
    hdr.header_size = host_to_net16(rawudp_header_size());
    hdr.flow_id = host_to_net32(opts.flow_id);
    hdr.frame_id = host_to_net32(frame_id);
    hdr.fragment_id = host_to_net32(fragment_id);
    hdr.fragment_count = host_to_net32(fragment_count);
    hdr.width = host_to_net16(static_cast<uint16_t>(opts.width));
    hdr.height = host_to_net16(static_cast<uint16_t>(opts.height));
    hdr.bit_depth = static_cast<uint8_t>(opts.bit_depth);
    hdr.pixel_format = static_cast<uint8_t>(opts.pixel_format);
    hdr.packing = static_cast<uint8_t>(opts.packing);
    hdr.reserved = 0;
    hdr.timestamp_us = host_to_net64(timestamp_us);
    hdr.payload_offset = host_to_net32(payload_offset);
    hdr.payload_size = host_to_net32(payload_size);
    hdr.header_crc32 = 0;
    hdr.payload_crc32 = 0;
}

} // namespace rawudp

int main(int argc, char** argv) {
    using namespace rawudp;
    SenderOptions opts;
    try {
        if (!parse_args(argc, argv, opts)) {
            return 0;
        }
    } catch (const std::exception& e) {
        std::cerr << "Argument error: " << e.what() << "\n";
        usage(argv[0]);
        return 1;
    }
    if (!validate_options(opts)) {
        usage(argv[0]);
        return 1;
    }

    std::vector<uint16_t> replay_frame;
    if (!opts.raw_file.empty()) {
        if (!load_raw_file(opts.raw_file, opts.width, opts.height, replay_frame)) {
            return 1;
        }
    }

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("socket");
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(opts.dest_port);
    if (::inet_pton(AF_INET, opts.dest_ip.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "Invalid destination IP\n";
        return 1;
    }

    const std::size_t frame_bytes = static_cast<std::size_t>(opts.width) * opts.height * 2;
    std::vector<uint8_t> payload(frame_bytes);
    const Crc32 crc;
    const auto frame_interval = duration<double>(1.0 / std::max<uint32_t>(1, opts.fps));
    auto next_time = steady_clock::now();

    uint32_t frames_sent = 0;
    while (opts.frame_count == 0 || frames_sent < opts.frame_count) {
        const uint32_t frame_id = frames_sent;
        const std::vector<uint16_t> frame = generate_frame(
            opts.pattern, opts.width, opts.height, opts.bit_depth, opts.pixel_format, frame_id,
            replay_frame.empty() ? nullptr : &replay_frame);

        for (std::size_t i = 0; i < frame.size(); ++i) {
            uint16_t v = frame[i];
            payload[i * 2] = static_cast<uint8_t>(v & 0xff);
            payload[i * 2 + 1] = static_cast<uint8_t>((v >> 8) & 0xff);
        }

        const uint32_t fragments =
            static_cast<uint32_t>((frame_bytes + opts.payload_size - 1) / opts.payload_size);

        uint64_t timestamp_us =
            duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();

        for (uint32_t frag = 0; frag < fragments; ++frag) {
            const uint32_t offset = frag * opts.payload_size;
            const uint32_t remaining = static_cast<uint32_t>(frame_bytes - offset);
            const uint32_t chunk = std::min(opts.payload_size, remaining);

            RawUdpHeader hdr{};
            build_header(hdr, opts, frame_id, frag, fragments, timestamp_us, offset, chunk);

            uint32_t payload_crc = crc.compute(payload.data() + offset, chunk);
            hdr.payload_crc32 = host_to_net32(payload_crc);
            hdr.header_crc32 = 0;
            uint32_t header_crc =
                crc.compute(reinterpret_cast<const uint8_t*>(&hdr), sizeof(RawUdpHeader));
            hdr.header_crc32 = host_to_net32(header_crc);

            std::vector<uint8_t> packet(sizeof(RawUdpHeader) + chunk);
            std::memcpy(packet.data(), &hdr, sizeof(RawUdpHeader));
            std::memcpy(packet.data() + sizeof(RawUdpHeader), payload.data() + offset, chunk);

            ssize_t sent = ::sendto(sock, packet.data(), packet.size(), 0,
                                    reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
            if (sent < 0) {
                std::perror("sendto");
                return 1;
            }
        }

        frames_sent++;
        next_time += duration_cast<steady_clock::duration>(frame_interval);
        std::this_thread::sleep_until(next_time);
    }

    ::close(sock);
    return 0;
}
