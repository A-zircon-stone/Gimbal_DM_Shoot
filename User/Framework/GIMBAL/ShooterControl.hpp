#pragma once

#include "GimbalRemotePolicy.hpp"
#include "Pid.hpp"
#include <cstdint>

struct ShooterInput {
    bool remote_online = false;
    uint8_t s1 = 0, s2 = 0;
    int16_t feeder_stick = 0; // DR16 ch[3]，只取正负，不按幅值调速
    bool vision_fire_allowed = false; // 有效自瞄目标且 fire == 1
    bool left_online = false, right_online = false;
    float left_speed_rpm = 0, right_speed_rpm = 0, feeder_speed_rpm = 0;
};

struct ShooterOutput {
    bool friction_enabled = false, friction_ready = false;
    float left_target_rpm = 0, right_target_rpm = 0, feeder_target_rpm = 0;
    int16_t left_current = 0, right_current = 0, feeder_current = 0;
};

class ShooterController {
public:
    ShooterController();
    ShooterOutput step(const ShooterInput &input);
private:
    Pid_t left_pid_{}, right_pid_{}, feeder_pid_{};
    float friction_target_rpm_ = 0;
    float last_feeder_target_rpm_ = 0;
};
