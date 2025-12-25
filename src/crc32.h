#pragma once

#include <cstddef>
#include <cstdint>

namespace rawudp {

class Crc32 {
public:
    Crc32() { init_table(); }

    uint32_t compute(const uint8_t* data, std::size_t size) const {
        uint32_t crc = 0xffffffffu;
        for (std::size_t i = 0; i < size; ++i) {
            uint8_t index = static_cast<uint8_t>((crc ^ data[i]) & 0xffu);
            crc = (crc >> 8) ^ table_[index];
        }
        return crc ^ 0xffffffffu;
    }

private:
    uint32_t table_[256];

    void init_table() {
        const uint32_t poly = 0xedb88320u;
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int j = 0; j < 8; ++j) {
                if (c & 1) {
                    c = poly ^ (c >> 1);
                } else {
                    c >>= 1;
                }
            }
            table_[i] = c;
        }
    }
};

} // namespace rawudp
