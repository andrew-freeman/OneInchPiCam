#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>
#include <chrono>

#include "protocol.h"

namespace rawudp {

struct CompletedFrame {
    RawUdpHeader header;
    std::vector<uint8_t> data;
    std::chrono::steady_clock::time_point first_packet_time;
    std::chrono::steady_clock::time_point complete_time;
};

class Reassembler {
public:
    explicit Reassembler(uint32_t expiration_frames);

    // Returns true and fills out when a frame is completed.
    bool add_fragment(const RawUdpHeader& header, const uint8_t* payload, std::size_t payload_size,
                      CompletedFrame& out, std::chrono::steady_clock::time_point recv_time);

    std::size_t dropped_frames() const { return dropped_frames_; }

private:
    struct FrameAssembly {
        RawUdpHeader header;
        std::size_t frame_size;
        std::vector<uint8_t> buffer;
        std::vector<bool> fragments;
        uint32_t received_fragments = 0;
        std::chrono::steady_clock::time_point first_packet_time;
        bool have_first_time = false;
    };

    using FrameMap = std::map<uint32_t, FrameAssembly>;
    std::unordered_map<uint32_t, FrameMap> flows_;
    uint32_t expiration_frames_;
    std::size_t dropped_frames_ = 0;

    std::size_t expected_frame_size(const RawUdpHeader& header) const;
    void drop_expired(FrameMap& frames, uint32_t latest_frame_id);
};

} // namespace rawudp
