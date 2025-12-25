#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "crc32.h"
#include "preview.h"
#include "protocol.h"
#include "reassembler.h"

using namespace std::chrono;

namespace rawudp {

struct ReceiverOptions {
    uint16_t port = 9000;
    uint32_t expiration_frames = 5;
    bool headless = false;
    ViewMode view_mode = ViewMode::HalfRes;
    uint16_t black_level = 0;
    float wb_r = 1.0f;
    float wb_g = 1.0f;
    float wb_b = 1.0f;
    float gamma = 1.0f;
    std::string record_dir;
    uint32_t flow_id = 0; // 0 = accept all
};

void usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "  --port <port>           UDP listen port (default 9000)\n"
              << "  --expiration <frames>   Drop incomplete frames after this many newer ones (default 5)\n"
              << "  --headless              Disable preview (still validates and records)\n"
              << "  --view-mode <mode>      mono, green, half, bilinear (default half)\n"
              << "  --black-level <value>   Black level subtraction (default 0)\n"
              << "  --wb <r> <g> <b>        White balance gains (default 1 1 1)\n"
              << "  --gamma <value>         Gamma value (default 1.0)\n"
              << "  --record-dir <path>     Save completed frames as .raw files\n"
              << "  --flow-id <id>          Filter specific flow (default accept all)\n";
}

ViewMode parse_view_mode(const std::string& s) {
    if (s == "mono") return ViewMode::Mono;
    if (s == "green") return ViewMode::GreenOnly;
    if (s == "half") return ViewMode::HalfRes;
    if (s == "bilinear") return ViewMode::Bilinear;
    throw std::invalid_argument("Unknown view mode: " + s);
}

bool parse_args(int argc, char** argv, ReceiverOptions& opts) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + name);
            return argv[++i];
        };

        if (arg == "--port") {
            opts.port = static_cast<uint16_t>(std::stoi(require_value(arg)));
        } else if (arg == "--expiration") {
            opts.expiration_frames = static_cast<uint32_t>(std::stoul(require_value(arg)));
        } else if (arg == "--headless") {
            opts.headless = true;
        } else if (arg == "--view-mode") {
            opts.view_mode = parse_view_mode(require_value(arg));
        } else if (arg == "--black-level") {
            opts.black_level = static_cast<uint16_t>(std::stoi(require_value(arg)));
        } else if (arg == "--wb") {
            opts.wb_r = std::stof(require_value(arg));
            if (i + 1 >= argc) throw std::runtime_error("Missing G/B for --wb");
            opts.wb_g = std::stof(argv[++i]);
            if (i + 1 >= argc) throw std::runtime_error("Missing B for --wb");
            opts.wb_b = std::stof(argv[++i]);
        } else if (arg == "--gamma") {
            opts.gamma = std::stof(require_value(arg));
        } else if (arg == "--record-dir") {
            opts.record_dir = require_value(arg);
        } else if (arg == "--flow-id") {
            opts.flow_id = static_cast<uint32_t>(std::stoul(require_value(arg)));
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return false;
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return true;
}

bool validate_packet(const uint8_t* data, std::size_t len, RawUdpHeader& host_hdr,
                     const Crc32& crc) {
    if (len < sizeof(RawUdpHeader)) {
        return false;
    }

    RawUdpHeader net_hdr{};
    std::memcpy(&net_hdr, data, sizeof(RawUdpHeader));
    const uint16_t header_size = net_to_host16(net_hdr.header_size);
    if (header_size != sizeof(RawUdpHeader) || len < header_size) {
        std::cerr << "Invalid header size\n";
        return false;
    }

    if (net_to_host32(net_hdr.magic) != kMagic) {
        return false;
    }
    if (net_to_host16(net_hdr.version) != kVersion) {
        std::cerr << "Unsupported version\n";
        return false;
    }

    RawUdpHeader temp = net_hdr;
    temp.header_crc32 = 0;
    uint32_t calc_hdr_crc = crc.compute(reinterpret_cast<uint8_t*>(&temp), sizeof(RawUdpHeader));
    if (calc_hdr_crc != net_to_host32(net_hdr.header_crc32)) {
        std::cerr << "Header CRC mismatch\n";
        return false;
    }

    const uint32_t payload_size = net_to_host32(net_hdr.payload_size);
    if (header_size + payload_size != len) {
        std::cerr << "Payload size mismatch\n";
        return false;
    }

    uint32_t payload_crc = crc.compute(data + header_size, payload_size);
    if (payload_crc != net_to_host32(net_hdr.payload_crc32)) {
        std::cerr << "Payload CRC mismatch\n";
        return false;
    }

    if (net_hdr.fragment_count == 0) {
        std::cerr << "Fragment count zero\n";
        return false;
    }

    host_hdr.magic = net_to_host32(net_hdr.magic);
    host_hdr.version = net_to_host16(net_hdr.version);
    host_hdr.header_size = header_size;
    host_hdr.flow_id = net_to_host32(net_hdr.flow_id);
    host_hdr.frame_id = net_to_host32(net_hdr.frame_id);
    host_hdr.fragment_id = net_to_host32(net_hdr.fragment_id);
    host_hdr.fragment_count = net_to_host32(net_hdr.fragment_count);
    host_hdr.width = net_to_host16(net_hdr.width);
    host_hdr.height = net_to_host16(net_hdr.height);
    host_hdr.bit_depth = net_hdr.bit_depth;
    host_hdr.pixel_format = net_hdr.pixel_format;
    host_hdr.packing = net_hdr.packing;
    host_hdr.reserved = net_hdr.reserved;
    host_hdr.timestamp_us = net_to_host64(net_hdr.timestamp_us);
    host_hdr.payload_offset = net_to_host32(net_hdr.payload_offset);
    host_hdr.payload_size = payload_size;
    host_hdr.header_crc32 = net_to_host32(net_hdr.header_crc32);
    host_hdr.payload_crc32 = net_to_host32(net_hdr.payload_crc32);
    if (host_hdr.width == 0 || host_hdr.height == 0) {
        std::cerr << "Zero-sized frame\n";
        return false;
    }
    return true;
}

bool write_raw(const std::string& dir, const CompletedFrame& frame) {
    namespace fs = std::filesystem;
    fs::create_directories(dir);
    const auto filename = dir + "/raw_frame_" + std::to_string(frame.header.frame_id) + "_" +
                          std::to_string(frame.header.width) + "x" +
                          std::to_string(frame.header.height) + "_" +
                          std::to_string(frame.header.bit_depth) + "b.raw";
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        std::cerr << "Failed to write " << filename << "\n";
        return false;
    }
    out.write(reinterpret_cast<const char*>(frame.data.data()), frame.data.size());
    return true;
}

} // namespace rawudp

int main(int argc, char** argv) {
    using namespace rawudp;
    ReceiverOptions opts;
    try {
        if (!parse_args(argc, argv, opts)) {
            return 0;
        }
    } catch (const std::exception& e) {
        std::cerr << "Argument error: " << e.what() << "\n";
        usage(argv[0]);
        return 1;
    }

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("socket");
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(opts.port);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        return 1;
    }

    Crc32 crc;
    Reassembler reasm(opts.expiration_frames);
    PreviewConfig preview_cfg;
    preview_cfg.mode = opts.view_mode;
    preview_cfg.black_level = opts.black_level;
    preview_cfg.wb_r = opts.wb_r;
    preview_cfg.wb_g = opts.wb_g;
    preview_cfg.wb_b = opts.wb_b;
    preview_cfg.gamma = opts.gamma;
    preview_cfg.headless = opts.headless;

    uint64_t bytes_received = 0;
    uint64_t frames_interval = 0;
    uint64_t frames_completed_total = 0;
    double rx_age_ms_avg = 0.0;
    double sender_latency_ms_avg = 0.0;
    bool sender_latency_valid = false;
    uint32_t max_reorder = 0;
    std::unordered_map<uint32_t, uint32_t> latest_frame;

    auto last_print = steady_clock::now();
    const auto stats_interval = seconds(1);

    std::vector<uint8_t> buffer(64 * 1024);
    while (true) {
        ssize_t n = ::recvfrom(sock, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::perror("recvfrom");
            break;
        }
        const auto now = steady_clock::now();
        bytes_received += static_cast<uint64_t>(n);

        RawUdpHeader host_hdr{};
        if (!validate_packet(buffer.data(), static_cast<std::size_t>(n), host_hdr, crc)) {
            continue;
        }

        if (opts.flow_id != 0 && host_hdr.flow_id != opts.flow_id) {
            continue;
        }

        uint32_t& latest = latest_frame[host_hdr.flow_id];
        if (host_hdr.frame_id > latest) {
            latest = host_hdr.frame_id;
        } else {
            max_reorder = std::max(max_reorder, latest - host_hdr.frame_id);
        }

        CompletedFrame frame;
        const uint8_t* payload = buffer.data() + host_hdr.header_size;
        if (reasm.add_fragment(host_hdr, payload, host_hdr.payload_size, frame, now)) {
            frames_interval++;
            frames_completed_total++;

            const double receiver_age_ms =
                duration_cast<duration<double, std::milli>>(frame.complete_time -
                                                            frame.first_packet_time)
                    .count();
            rx_age_ms_avg = rx_age_ms_avg * 0.9 + receiver_age_ms * 0.1;

            std::optional<double> sender_latency_sample;
            if (frame.header.timestamp_us != 0) {
                const uint64_t now_us =
                    duration_cast<microseconds>(frame.complete_time.time_since_epoch()).count();
                const double latency_ms =
                    static_cast<double>(now_us - frame.header.timestamp_us) / 1000.0;
                if (std::abs(latency_ms) < 10000.0) {
                    sender_latency_sample = latency_ms;
                    if (sender_latency_valid) {
                        sender_latency_ms_avg = sender_latency_ms_avg * 0.9 + latency_ms * 0.1;
                    } else {
                        sender_latency_ms_avg = latency_ms;
                        sender_latency_valid = true;
                    }
                } else {
                    sender_latency_valid = false;
                }
            } else {
                sender_latency_valid = false;
            }

            if (!opts.record_dir.empty()) {
                write_raw(opts.record_dir, frame);
            }

            std::vector<uint8_t> preview_bgr;
            int out_w = 0, out_h = 0;
            if (render_preview(frame.header, frame.data, preview_cfg, preview_bgr, out_w, out_h)) {
                display_preview(preview_bgr, out_w, out_h, "rawudp", opts.headless);
            }
        }

        const auto now_print = steady_clock::now();
        if (now_print - last_print >= stats_interval) {
            double seconds_elapsed =
                duration_cast<duration<double>>(now_print - last_print).count();
            double mbps = (bytes_received * 8.0) / (seconds_elapsed * 1e6);
            double fps = frames_interval / seconds_elapsed;
            std::string sender_latency_str =
                sender_latency_valid ? std::to_string(sender_latency_ms_avg) : "n/a";
            std::cout << "Stats: "
                      << "fps=" << fps << " "
                      << "mbps=" << mbps << " "
                      << "frames_total=" << frames_completed_total << " "
                      << "dropped=" << reasm.dropped_frames() << " "
                      << "reorder=" << max_reorder << " "
                      << "rx_age_ms~" << rx_age_ms_avg << " "
                      << "sender_latency_ms~" << sender_latency_str << "\n";
            bytes_received = 0;
            frames_interval = 0;
            last_print = now_print;
        }
    }

    ::close(sock);
    return 0;
}
