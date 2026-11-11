#pragma once

#include "DamiaoMit.hpp"
#include "GimbalTuning.hpp"
#include <cmath>

// Pitch 位置 PID：由调用方传入 IMU 角度和角速度，不从电机编码器取闭环反馈。
// 本文件不使能电机，也不调用 CAN；调用方必须先判断反馈新鲜且电机已使能。
namespace damiao_pitch_pid {
struct Config {
    // 这里只是配置结构体的初始值；实际使能时 Runtime 会用 state.kp/ki/kd 覆盖。
    // 例如 Runtime 的 kp=1、kd=0.1 时，控制器实际使用 1 和 0.1，而不是这里的 0。
    float kp = 0.0f;              // 位置误差比例系数，Nm/rad
    float ki = 0.0f;              // 积分系数，Nm/(rad*s)
    float kd = 0.0f;              // 速度阻尼系数，Nm/(rad/s)
    float max_integral = 0.0f;    // 积分项输出绝对值上限，Nm
    float max_torque = 0.0f;      // PID 与前馈之和的绝对值上限，Nm；须不大于电机 TMAX
    float min_position = gimbal_tuning::pitch::MinPositionRad; // IMU 下限，rad
    float max_position = gimbal_tuning::pitch::MaxPositionRad; // IMU 上限，rad
    float max_dt = 0.0f;          // 单次计算的最大允许间隔，s；超过则停止计算
};

inline float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

inline bool valid_config(const Config &c, float torque_encoding_limit) {
    return std::isfinite(c.kp) && c.kp >= 0.0f &&
           std::isfinite(c.ki) && c.ki >= 0.0f &&
           std::isfinite(c.kd) && c.kd >= 0.0f &&
           std::isfinite(c.max_integral) && c.max_integral >= 0.0f &&
           std::isfinite(c.max_torque) && c.max_torque > 0.0f &&
           std::isfinite(torque_encoding_limit) && torque_encoding_limit > 0.0f &&
           c.max_torque <= torque_encoding_limit &&
           c.max_integral <= c.max_torque &&
           std::isfinite(c.min_position) && std::isfinite(c.max_position) &&
           c.min_position < c.max_position &&
           std::isfinite(c.max_dt) && c.max_dt > 0.0f;
}

class Controller {
public:
    // 配置须由实际机械范围和调试结果决定；默认值不能用于启动控制。
    bool configure(const Config &config, float torque_encoding_limit) {
        reset();
        configured_ = valid_config(config, torque_encoding_limit);
        if (configured_) config_ = config;
        return configured_;
    }

    // 在线更新增益，不重置当前目标和积分；下一次 PID 计算立即使用新值。
    bool update_gains(float kp, float ki, float kd) {
        if (!configured_ || !std::isfinite(kp) || kp < 0 ||
            !std::isfinite(ki) || ki < 0 || !std::isfinite(kd) || kd < 0) return false;
        config_.kp = kp; config_.ki = ki; config_.kd = kd;
        return true;
    }

    // 从当前位置启动，避免首次使能就跳到一个远处的固定目标。
    bool start(float measured_position) {
        reset();
        if (!configured_ || !std::isfinite(measured_position) ||
            measured_position < config_.min_position || measured_position > config_.max_position)
            return false;
        target_ = measured_position;
        active_ = true;
        return true;
    }

    // 目标是 IMU 参考坐标的角度，单位 rad；超出调试范围时拒绝。
    bool set_target(float target_position) {
        if (!active_ || !std::isfinite(target_position) ||
            target_position < config_.min_position || target_position > config_.max_position)
            return false;
        target_ = target_position;
        return true;
    }

    // feedback_ready 由调用方检查 CAN 反馈身份、超时和使能状态后提供。
    // feedforward 是与 PID 输出同方向、同单位（Nm）的额外扭矩，例如重力补偿。
    // 返回 false 时 out 不变；调用方应停止发送非零输出并执行自己的失能策略。
    bool calculate(bool feedback_ready, float measured_position, float measured_velocity,
                   float dt_seconds, damiao_mit::Command &out, float feedforward = 0.0f) {
        if (!active_ || !feedback_ready || !std::isfinite(measured_position) ||
            !std::isfinite(measured_velocity) || !std::isfinite(dt_seconds) ||
            !std::isfinite(feedforward) ||
            dt_seconds <= 0.0f || dt_seconds > config_.max_dt ||
            measured_position < config_.min_position || measured_position > config_.max_position) {
            reset();
            return false;
        }
        const float error = target_ - measured_position;
        // 积分以 Nm 保存；例如误差 0.1 rad 持续 0.01 s，Ki=2 时增量是 0.002 Nm。
        const float next_integral = clamp(integral_ + config_.ki * error * dt_seconds,
                                          -config_.max_integral, config_.max_integral);
        // 使用 IMU 角速度计算阻尼；目标位置突变时不会产生误差微分尖峰。
        const float proportional_and_damping = config_.kp * error - config_.kd * measured_velocity;
        const float candidate = proportional_and_damping + next_integral + feedforward;
        if (!std::isfinite(candidate)) { reset(); return false; }
        // PID 加上重力补偿后仍须一起判断饱和。
        // 例：限幅 0.5 Nm，P=0.48、前馈=0.04 时，总和 0.52 已饱和，正误差不能再累积积分。
        if ((candidate <= config_.max_torque && candidate >= -config_.max_torque) ||
            (candidate > config_.max_torque && error < 0.0f) ||
            (candidate < -config_.max_torque && error > 0.0f)) integral_ = next_integral;
        const float torque = clamp(proportional_and_damping + integral_ + feedforward,
                                   -config_.max_torque, config_.max_torque);
        // 按 {位置, 速度, 电机MIT的Kp, 电机MIT的Kd, 扭矩} 顺序组包。
        // MCU 用自己的 PID 和 IMU 算出 torque；电机 MIT 的 Kp/Kd 置零，避免重复闭环。
        out = {0.0f, 0.0f, 0.0f, 0.0f, torque};
        return true;
    }

    // 失联、失能或故障时调用；再次启动必须重新捕获当前位置。
    void reset() { active_ = false; target_ = 0.0f; integral_ = 0.0f; }
    bool active() const { return active_; }
    float target() const { return target_; }
    float integral() const { return integral_; }

private:
    Config config_{};
    bool configured_ = false;
    bool active_ = false;
    float target_ = 0.0f;
    float integral_ = 0.0f;
};
}
