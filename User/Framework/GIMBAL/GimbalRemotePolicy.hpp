#pragma once

#include <cstdint>

namespace gimbal_remote_policy {
constexpr uint8_t SwitchUp = 1;
constexpr uint8_t SwitchDown = 2;
constexpr uint8_t SwitchMiddle = 3;

struct Selection {
    bool remote_mode = false;
    bool keyboard_mode = false;
    bool protected_mode = false;
    bool autoaim_mode = false;
    bool manual_fire_mode = false;
};

inline Selection select(bool remote_online, uint8_t s1, uint8_t s2) {
    Selection result{};
    result.protected_mode = s1 == SwitchUp;
    // S1为右拨杆：下档改为遥控手动发射，不再选择键鼠。
    result.remote_mode = s1 == SwitchUp || s1 == SwitchMiddle || s1 == SwitchDown;
    result.manual_fire_mode = remote_online && s1 == SwitchDown;
    result.autoaim_mode = remote_online && s1 == SwitchMiddle && s2 == SwitchUp;
    return result;
}

}
