#include "ShooterControl.hpp"
#include "GimbalTuning.hpp"
#include <algorithm>
#include <cmath>

namespace {
namespace tune = gimbal_tuning::shooter;
int16_t friction_current(float torque) {
    // 在转换为int16_t前沿用3508的编码范围，避免大输出先溢出再限幅。
    const float current = torque * tune::FrictionTorqueToCurrent;
    return static_cast<int16_t>(std::max(-float(tune::FrictionCurrentMax),
                                       std::min(float(tune::FrictionCurrentMax), current)));
}
}

ShooterController::ShooterController() {
    left_pid_.Init(tune::FrictionKp, tune::FrictionKi, tune::FrictionKd);
    right_pid_.Init(tune::FrictionKp, tune::FrictionKi, tune::FrictionKd);
    feeder_pid_.Init(tune::FeederKp, tune::FeederKi, tune::FeederKd);
}

ShooterOutput ShooterController::step(const ShooterInput &in) {
    ShooterOutput out;
    const auto mode = gimbal_remote_policy::select(in.remote_online, in.s1, in.s2);
    out.friction_enabled = mode.manual_fire_mode || mode.autoaim_mode;
    if (!out.friction_enabled) {
        friction_target_rpm_ = last_feeder_target_rpm_ = 0;
        left_pid_.Clear(); right_pid_.Clear(); feeder_pid_.Clear();
        return out;
    }

    friction_target_rpm_ = std::min(tune::FrictionTargetRpm,
                                   friction_target_rpm_ + tune::FrictionRampRpmPerCycle);
    out.left_target_rpm = friction_target_rpm_;
    out.right_target_rpm = -friction_target_rpm_;
    left_pid_.Cal(out.left_target_rpm, in.left_speed_rpm, tune::FrictionPidOutputMax);
    right_pid_.Cal(out.right_target_rpm, in.right_speed_rpm, tune::FrictionPidOutputMax);
    out.left_current = friction_current(left_pid_.Output);
    out.right_current = friction_current(right_pid_.Output);
    out.friction_ready = in.left_online && in.right_online &&
        std::fabs(in.left_speed_rpm) >= tune::FrictionReadyRpm &&
        std::fabs(in.right_speed_rpm) >= tune::FrictionReadyRpm;

    if (out.friction_ready && (mode.manual_fire_mode || in.vision_fire_allowed)) {
        if (in.feeder_stick > 0) out.feeder_target_rpm = tune::FeederTargetRpm;
        else if (in.feeder_stick < 0) out.feeder_target_rpm = -tune::FeederTargetRpm;
    }
    if (out.feeder_target_rpm != last_feeder_target_rpm_) feeder_pid_.Clear();
    last_feeder_target_rpm_ = out.feeder_target_rpm;
    feeder_pid_.Cal(out.feeder_target_rpm, in.feeder_speed_rpm, tune::FeederCurrentMax);
    out.feeder_current = static_cast<int16_t>(feeder_pid_.Output);
    return out;
}
