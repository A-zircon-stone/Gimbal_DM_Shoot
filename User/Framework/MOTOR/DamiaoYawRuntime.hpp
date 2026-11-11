#pragma once
#include <cstdint>
#include "DamiaoYawConfig.hpp"
#include "Pid.hpp"
namespace damiao_yaw_runtime {
enum class State : uint8_t { Stopped, Enabling, Running };
struct Snapshot {
    State state = State::Stopped;
    const char *reason = "STARTUP";
    float angle = 0, target = 0, velocity = 0, torque = 0;
    float target_velocity = 0; // 位置外环输出，rad/s
    PidGains position_gains{gimbal_tuning::yaw::PositionKp,
                           gimbal_tuning::yaw::PositionKi, gimbal_tuning::yaw::PositionKd};
    PidGains speed_gains{gimbal_tuning::yaw::SpeedKp,
                        gimbal_tuning::yaw::SpeedKi, gimbal_tuning::yaw::SpeedKd};
    bool feedback_fresh = false, imu_fresh = false;
};
// 发布同一IMU帧的连续Yaw和欧拉角、角速度；角度以度输入。
void publish_imu(float yaw_deg, float roll_deg, float pitch_deg, float gy, float gz, uint32_t now);
void invalidate_imu();
// 保护时失能；退出保护后恢复此前正在运行的轴。视觉目标为绝对Yaw，单位rad。
void step(uint32_t now, float stick_percent, bool protected_mode = false,
          bool vision_target_active = false, float vision_target_rad = 0.0f);
Snapshot snapshot();
}
