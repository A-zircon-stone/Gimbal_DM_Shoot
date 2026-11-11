#pragma once
#include <cstdint>

namespace shooter_can {
// CAN1/0x200，ID1右摩擦轮、ID2左摩擦轮、ID3未使用、ID4拨盘。
inline void encode_currents(int16_t left, int16_t right, int16_t feeder, uint8_t out[8]) {
    const uint16_t currents[] = {static_cast<uint16_t>(right), static_cast<uint16_t>(left),
                                 0, static_cast<uint16_t>(feeder)};
    for (unsigned i = 0; i < 4; ++i) {
        out[2 * i] = static_cast<uint8_t>(currents[i] >> 8);
        out[2 * i + 1] = static_cast<uint8_t>(currents[i]);
    }
}
}
