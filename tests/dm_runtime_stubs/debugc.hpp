#pragma once
#include <cstdint>
struct DebugStub {
    bool dm_stop_requested=false, dm_enable_requested=false, dm_target_pending=false;
    float dm_target_rad=0;
    float pitch_speed_kp=0, pitch_speed_ki=0, pitch_speed_kd=0, pitch_position_kp=0;
    uint8_t pitch_speed_gain_pending=0;
    bool yaw_stop_requested=false, yaw_enable_requested=false;
    uint8_t yaw_gain_pending=0;
    float yaw_speed_kp=0, yaw_speed_ki=0, yaw_speed_kd=0, yaw_position_kp=0;
};
extern DebugStub debug;
