#pragma once

#include "GimbalTuning.hpp"
#include <cstddef>
#include <cstdint>

// M_2006_p36：反馈为转子编码器/rpm，发布减速后输出轴的连续角度和rpm。
struct M2006Feedback {
    bool seen = false;
    uint16_t encoder = 0;
    int16_t rotor_speed_rpm = 0;
    int16_t current_raw = 0;
    int64_t total_counts = 0;
    float position_deg = 0;
    float speed_rpm = 0;

    bool update(const uint8_t *data, std::size_t length) {
        if (!data || length != 8) return false;
        const uint16_t next = (uint16_t(data[0]) << 8) | data[1];
        if (next >= 8192) return false;
        if (!seen) total_counts = next;
        else {
            int32_t delta = int32_t(next) - encoder;
            if (delta > 4096) delta -= 8192;
            else if (delta < -4096) delta += 8192;
            total_counts += delta;
        }
        encoder = next;
        rotor_speed_rpm = static_cast<int16_t>((uint16_t(data[2]) << 8) | data[3]);
        current_raw = static_cast<int16_t>((uint16_t(data[4]) << 8) | data[5]);
        speed_rpm = rotor_speed_rpm / gimbal_tuning::shooter::FeederGearRatio;
        position_deg = static_cast<float>(total_counts) *
                       (360.0f / (8192.0f * gimbal_tuning::shooter::FeederGearRatio));
        seen = true;
        return true;
    }
};
