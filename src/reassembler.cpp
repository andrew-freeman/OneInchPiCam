#include "reassembler.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <map>

namespace rawudp {

Reassembler::Reassembler(uint32_t expiration_frames) : expiration_frames_(expiration_frames) {}

std::size_t Reassembler::expected_frame_size(const RawUdpHeader& header) const {
    if (static_cast<Packing>(header.packing) != Packing::UNPACKED_16) {
        return 0;
    }
    return static_cast<std::size_t>(header.width) * static_cast<std::size_t>(header.height) * 2;
}

void Reassembler::drop_expired(FrameMap& frames, uint32_t latest_frame_id) {
    while (!frames.empty()) {
        auto it = frames.begin();
        if (latest_frame_id >= it->first + expiration_frames_) {
            frames.erase(it);
            dropped_frames_++;
        } else {
            break;
        }
    }
}

bool Reassembler::add_fragment(const RawUdpHeader& header, const uint8_t* payload,
                               std::size_t payload_size, CompletedFrame& out,
                               std::chrono::steady_clock::time_point recv_time) {
    FrameMap& frame_map = flows_[header.flow_id];
    drop_expired(frame_map, header.frame_id);

    FrameAssembly& assembly = frame_map[header.frame_id];
    if (assembly.buffer.empty()) {
        assembly.header = header;
        assembly.frame_size = expected_frame_size(header);
        if (assembly.frame_size == 0) {
            std::cerr << "Unsupported packing mode or zero-sized frame\n";
            frame_map.erase(header.frame_id);
            return false;
        }
        assembly.buffer.resize(assembly.frame_size, 0);
        assembly.fragments.assign(header.fragment_count, false);
        assembly.first_packet_time = recv_time;
        assembly.have_first_time = true;
    } else {
        // Validate consistency
        if (assembly.header.width != header.width || assembly.header.height != header.height ||
            assembly.header.fragment_count != header.fragment_count) {
            std::cerr << "Fragment metadata mismatch for frame " << header.frame_id << "\n";
            return false;
        }
    }

    if (header.fragment_id >= assembly.fragments.size()) {
        std::cerr << "Invalid fragment id " << header.fragment_id << "\n";
        return false;
    }

    if (assembly.fragments[header.fragment_id]) {
        // Duplicate fragment
        return false;
    }

    const std::size_t offset = header.payload_offset;
    if (offset + payload_size > assembly.buffer.size()) {
        std::cerr << "Payload out of bounds for frame " << header.frame_id << "\n";
        return false;
    }

    std::memcpy(assembly.buffer.data() + offset, payload, payload_size);
    assembly.fragments[header.fragment_id] = true;
    assembly.received_fragments++;

    if (assembly.received_fragments == assembly.fragments.size()) {
        out.header = assembly.header;
        out.data = std::move(assembly.buffer);
        out.complete_time = recv_time;
        out.first_packet_time = assembly.have_first_time ? assembly.first_packet_time : recv_time;
        frame_map.erase(header.frame_id);
        return true;
    }
    return false;
}

} // namespace rawudp
