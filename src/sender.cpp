#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "crc32.h"
#include "frame_source.h"
#include "generator.h"
#include "gstreamer_source.h"
#include "libcamera_source.h"
#include "protocol.h"

using namespace std::chrono;

namespace rawudp {

enum class SourceType { File, Generator, Libcamera, GStreamer };

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
    SourceType source = SourceType::File;
    bool list_cameras = false;
    int camera_index = 0;
    uint16_t effective_bits = 12;
    uint16_t container_bits = 16;
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
              << "  --raw-file <path>      Replay raw file instead of synthetic pattern\n"
              << "  --source <src>         file|generator|libcamera|gstreamer (default file)\n"
              << "  --camera <index>       Camera index for libcamera (default 0)\n"
              << "  --list-cameras         List available cameras and exit\n"
              << "  --effective-bits <N>   Sensor effective bits (default 12)\n"
              << "  --container-bits <N>   Container bit depth (default 16)\n";
}

SourceType parse_source(const std::string& name) {
    if (name == "file") return SourceType::File;
    if (name == "generator") return SourceType::Generator;
    if (name == "libcamera") return SourceType::Libcamera;
    if (name == "gstreamer") return SourceType::GStreamer;
    throw std::invalid_argument("Unknown source: " + name);
}

bool parse_args(int argc, char** argv, SenderOptions& opts) {
    bool source_set = false;
    bool container_set = false;
    bool effective_set = false;
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
            if (!effective_set) opts.effective_bits = opts.bit_depth;
            if (!container_set) opts.container_bits = opts.bit_depth;
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
            if (!source_set) opts.source = SourceType::File;
        } else if (arg == "--source") {
            opts.source = parse_source(require_value(arg));
            source_set = true;
        } else if (arg == "--camera") {
            opts.camera_index = std::stoi(require_value(arg));
        } else if (arg == "--list-cameras") {
            opts.list_cameras = true;
        } else if (arg == "--effective-bits") {
            opts.effective_bits = static_cast<uint16_t>(std::stoi(require_value(arg)));
            effective_set = true;
        } else if (arg == "--container-bits") {
            opts.container_bits = static_cast<uint16_t>(std::stoi(require_value(arg)));
            container_set = true;
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return false;
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    if (!source_set && opts.raw_file.empty()) {
        // Preserve legacy default behavior if no raw file is provided.
        opts.source = SourceType::Generator;
    }
    return true;
}

bool validate_options(const SenderOptions& opts) {
    if (opts.width <= 0 || opts.height <= 0) {
        if (opts.source != SourceType::Libcamera) {
            std::cerr << "Width and height must be positive\n";
            return false;
        }
    }
    if (opts.payload_size == 0) {
        std::cerr << "Payload size must be greater than zero\n";
        return false;
    }
    if (opts.bit_depth == 0 || opts.bit_depth > 16) {
        std::cerr << "Bit depth must be between 1 and 16\n";
        return false;
    }
    if (opts.effective_bits == 0 || opts.effective_bits > 16 ||
        opts.container_bits == 0 || opts.container_bits > 16 ||
        opts.effective_bits > opts.container_bits) {
        std::cerr << "Effective/container bits must be within 1-16 and effective <= container\n";
        return false;
    }
    if (opts.source == SourceType::File && opts.raw_file.empty()) {
        std::cerr << "File source selected but no raw file provided\n";
        return false;
    }
    return true;
}

void build_header(RawUdpHeader& hdr, const SenderOptions& opts, const FrameMeta& meta,
                  uint32_t frame_id, uint32_t fragment_id, uint32_t fragment_count,
                  uint64_t timestamp_us, uint32_t payload_offset, uint32_t payload_size) {
    hdr.magic = host_to_net32(kMagic);
    hdr.version = host_to_net16(kVersion);
    hdr.header_size = host_to_net16(rawudp_header_size());
    hdr.flow_id = host_to_net32(opts.flow_id);
    hdr.frame_id = host_to_net32(frame_id);
    hdr.fragment_id = host_to_net32(fragment_id);
    hdr.fragment_count = host_to_net32(fragment_count);
    hdr.width = host_to_net16(static_cast<uint16_t>(meta.width));
    hdr.height = host_to_net16(static_cast<uint16_t>(meta.height));
    hdr.bit_depth = static_cast<uint8_t>(meta.container_bits);
    hdr.raw_encoding = static_cast<uint8_t>(meta.raw_encoding);
    hdr.pixel_format = static_cast<uint8_t>(meta.pixel_format);
    hdr.packing = static_cast<uint8_t>(opts.packing);
    hdr.reserved = static_cast<uint8_t>(std::min<uint16_t>(meta.effective_bits, 0xff));
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

    if (opts.list_cameras) {
        list_cameras();
        return 0;
    }

    if (!validate_options(opts)) {
        usage(argv[0]);
        return 1;
    }

    std::unique_ptr<FrameSource> source;
    std::vector<uint16_t> replay_frame;
    if (opts.source == SourceType::File) {
        auto file_source = std::make_unique<RawFileSource>(opts.raw_file, opts.width, opts.height,
                                                           opts.pixel_format, opts.container_bits,
                                                           opts.effective_bits);
        if (!file_source->valid()) {
            return 1;
        }
        source = std::move(file_source);
    } else if (opts.source == SourceType::Generator) {
        if (opts.pattern == Pattern::FileReplay || !opts.raw_file.empty()) {
            if (!load_raw_file(opts.raw_file, opts.width, opts.height, replay_frame)) {
                return 1;
            }
        }
        source = std::make_unique<SyntheticSource>(
            opts.pattern, opts.width, opts.height, opts.pixel_format, opts.container_bits,
            opts.effective_bits, replay_frame.empty() ? nullptr : &replay_frame);
    } else if (opts.source == SourceType::Libcamera) {
        LibcameraConfig cfg;
        cfg.camera_index = opts.camera_index;
        cfg.width = opts.width;
        cfg.height = opts.height;
        cfg.container_bits = opts.container_bits;
        cfg.effective_bits = opts.effective_bits;
        source = make_libcamera_source(cfg);
        if (!source) {
            std::cerr << "Failed to initialize libcamera source\n";
            return 1;
        }
    } else if (opts.source == SourceType::GStreamer) {
        GStreamerConfig cfg;
        cfg.width = opts.width;
        cfg.height = opts.height;
        cfg.container_bits = opts.container_bits;
        cfg.effective_bits = opts.effective_bits;
        source = make_gstreamer_source(cfg);
        if (!source) {
            std::cerr << "Failed to initialize GStreamer source\n";
            return 1;
        }
    }

    if (!source) {
        std::cerr << "No frame source configured\n";
        return 1;
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

    const Crc32 crc;
    const bool throttle = opts.source != SourceType::Libcamera &&
                          opts.source != SourceType::GStreamer;
    const auto frame_interval = throttle ? duration<double>(1.0 / std::max<uint32_t>(1, opts.fps))
                                         : duration<double>(0);
    auto next_time = steady_clock::now();

    uint32_t frames_sent = 0;
    while (opts.frame_count == 0 || frames_sent < opts.frame_count) {
        uint8_t* frame_ptr = nullptr;
        std::size_t frame_size = 0;
        uint64_t timestamp_us = 0;
        FrameMeta meta;
        if (!source->get_frame(frame_ptr, frame_size, timestamp_us, meta)) {
            break;
        }

        if (frame_size == 0 || frame_ptr == nullptr) {
            std::cerr << "Empty frame from source\n";
            return 1;
        }
        if (meta.width <= 0 || meta.height <= 0) {
            std::cerr << "Invalid frame dimensions from source\n";
            return 1;
        }

        const uint32_t fragments =
            static_cast<uint32_t>((frame_size + opts.payload_size - 1) / opts.payload_size);

        const uint32_t frame_id = frames_sent;
        if (timestamp_us == 0) {
            timestamp_us = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
        }

        for (uint32_t frag = 0; frag < fragments; ++frag) {
            const uint32_t offset = frag * opts.payload_size;
            const uint32_t remaining = static_cast<uint32_t>(frame_size - offset);
            const uint32_t chunk = std::min(opts.payload_size, remaining);

            RawUdpHeader hdr{};
            build_header(hdr, opts, meta, frame_id, frag, fragments, timestamp_us, offset, chunk);

            uint32_t payload_crc = crc.compute(frame_ptr + offset, chunk);
            hdr.payload_crc32 = host_to_net32(payload_crc);
            hdr.header_crc32 = 0;
            uint32_t header_crc =
                crc.compute(reinterpret_cast<const uint8_t*>(&hdr), sizeof(RawUdpHeader));
            hdr.header_crc32 = host_to_net32(header_crc);

            std::vector<uint8_t> packet(sizeof(RawUdpHeader) + chunk);
            std::memcpy(packet.data(), &hdr, sizeof(RawUdpHeader));
            std::memcpy(packet.data() + sizeof(RawUdpHeader), frame_ptr + offset, chunk);

            ssize_t sent = ::sendto(sock, packet.data(), packet.size(), 0,
                                    reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
            if (sent < 0) {
                std::perror("sendto");
                return 1;
            }
        }

        frames_sent++;
        if (throttle) {
            next_time += duration_cast<steady_clock::duration>(frame_interval);
            std::this_thread::sleep_until(next_time);
        } else {
            next_time = steady_clock::now();
        }
    }

    ::close(sock);
    return 0;
}
