#pragma once
#include <cstdint>
#include "DamiaoMit.hpp"
#include "GimbalTuning.hpp"
#include "Pid.hpp"

// Pitch IMU 位置/速度双环；状态机只管理目标、反馈和CAN，不实现PID公式。
namespace damiao_pitch_runtime {
enum class State : uint8_t { Stopped, Enabling, Running };
struct Snapshot {
    State state = State::Stopped;
    const char *reason = "STARTUP";
    bool feedback_fresh = false;
    damiao_mit::Feedback feedback{};
    uint32_t feedback_count = 0;
    uint32_t feedback_age_ms = 0;
    uint32_t tx_errors = 0;
    bool imu_fresh = false;
    uint32_t imu_age_ms = 0;
    float imu_angle = 0;          // IMU 云台角度，rad，抬头为正
    float imu_velocity = 0;       // IMU 角速度，rad/s，抬头为正
    float target_position = 0;    // IMU 的目标角度，rad
    float target_min = 0, target_max = 0; // 本次使能后允许设置的目标范围，rad
    float requested_torque = 0;
    float gravity_torque = 0;      // 重力前馈，Nm；仅运行状态有效，未计入总扭矩限幅
    // PID 停机时锁存，用来查看故障发生的瞬间；重启控制器前不会被后续 IMU 刷新覆盖。
    bool pid_fault_seen = false;
    const char *pid_fault_reason = "NONE";
    float pid_fault_angle = 0, pid_fault_min = 0, pid_fault_max = 0;
    uint32_t pid_fault_dt_ms = 0;
    float target_velocity = 0; // 位置外环输出，rad/s
    PidGains position_gains{gimbal_tuning::pitch::PositionKp,
                           gimbal_tuning::pitch::PositionKi, gimbal_tuning::pitch::PositionKd};
    PidGains speed_gains{gimbal_tuning::pitch::SpeedKp,
                        gimbal_tuning::pitch::SpeedKi, gimbal_tuning::pitch::SpeedKd};
};
// INS 每次成功更新后发布同一帧，控制任务读取完整快照；错误帧立即标记无效。
void publish_imu(float roll_deg, float gyro_y_rad_s, uint32_t now_ms);
void invalidate_imu();
// 由云台任务每 2 ms 调用；snapshot 可从打印任务读取。
// remote_pitch_percent为右摇杆上下，-100到+100；0表示保持当前目标。
// 保护时失能；退出保护后恢复此前正在运行的轴。视觉目标为绝对角，单位rad。
void step(uint32_t now_ms, float remote_pitch_percent = 0.0f,
          bool protected_mode = false, bool vision_target_active = false,
          float vision_target_rad = 0.0f);
Snapshot snapshot();
}
