#include "DamiaoPitchRuntime.hpp"
#include "DamiaoPitchPid.hpp"
#include "CANio.hpp"
#include "debugc.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>

namespace damiao_pitch_runtime {
namespace {
// 调参统一修改 GimbalTuning.hpp；这里仅执行控制逻辑。
namespace tune = gimbal_tuning::pitch;
namespace timing = gimbal_tuning::timing;
#if 0 // 之前的使能位置附近 ±0.15 rad 限制，保留参考，当前不再使用。
constexpr float TrialTravelRad = 0.15f;
#endif
#if 0 // 旧 dm_delta 单步限制；现在由绝对目标允许范围检查。
constexpr float MaxStepRad = 0.05f;
#endif
constexpr float RemoteMaxSpeedRadPerSecond =
    tune::RemoteMaxSpeedDegPerSecond * gimbal_tuning::DegreesToRadians;
struct ImuSample {
    bool valid = false;
    float angle_rad = 0;
    float velocity_rad_s = 0;
    uint32_t time_ms = 0;
};
ImuSample imu_sample{};
damiao_pitch_pid::Controller controller;
Snapshot state{};
Snapshot published{};
uint32_t last_disable_ms = 0, last_enable_ms = 0, enable_start_ms = 0;
uint32_t enable_feedback_count = 0, last_control_ms = 0;
uint32_t disable_feedback_count = 0;
bool disable_submitted = false;
bool first_step = true;
bool remote_protected = false;
bool resume_after_remote_protect = false;
// 每次主控启动只自动使能一次。等待IMU/CAN就绪期间保留请求；
// 成功开始使能、手动停止或故障停机后取消，不在后台反复重启电机。
bool startup_enable_pending = false;

void publish() {
    taskENTER_CRITICAL();
    published = state;
    taskEXIT_CRITICAL();
}
void count_result(HAL_StatusTypeDef result) {
    if (result != HAL_OK) ++state.tx_errors;
}
void send_disable(uint32_t now) {
    const HAL_StatusTypeDef result = damiao_pitch_can_send_disable();
    count_result(result);
    if (result == HAL_OK) {
        disable_submitted = true;
        disable_feedback_count = state.feedback_count;
    }
    last_disable_ms = now;
}
void stop(const char *reason, uint32_t now) {
    startup_enable_pending = false;
    controller.reset();
    state.state = State::Stopped;
    state.reason = reason;
    state.requested_torque = 0;
    state.gravity_torque = 0;
    state.target_min = state.target_max = 0; // 停止时没有有效目标设置范围
    // 先撤销未发出的运动帧并发送 FD。总线不通时，下面的停机周期仍会重试。
    disable_submitted = false;
    send_disable(now);
}
void stop_pid_with_diagnostics(const char *reason, uint32_t now, float angle, uint32_t elapsed_ms) {
    // 在 stop() 清空活动范围前先保存，避免事后只能看到 0～0 而无法判断。
    state.pid_fault_seen = true;
    state.pid_fault_reason = reason;
    state.pid_fault_angle = angle;
    state.pid_fault_min = state.target_min;
    state.pid_fault_max = state.target_max;
    state.pid_fault_dt_ms = elapsed_ms;
    stop(reason, now);
}
}

void publish_imu(float roll_deg, float gyro_y_rad_s, uint32_t now_ms) {
    // 延续旧云台的 Pitch 方向：抬头时 INS.Roll 和 Gyro[Y_] 减小。
    // 例如 Roll=-10° 时，-Roll 转为约 +0.1745 rad，表示抬头角为正。
    // Roll 是 asin 得到的角度，接近 ±90° 时不再用它闭环。
    ImuSample next{};
    next.valid = std::isfinite(roll_deg) && std::fabs(roll_deg) < timing::MaxAbsImuRollDeg &&
                 std::isfinite(gyro_y_rad_s);
    next.angle_rad = -roll_deg * gimbal_tuning::DegreesToRadians;
    next.velocity_rad_s = -gyro_y_rad_s;
    next.time_ms = now_ms;
    taskENTER_CRITICAL();
    imu_sample = next;
    taskEXIT_CRITICAL();
}

void invalidate_imu() {
    taskENTER_CRITICAL();
    imu_sample.valid = false;
    taskEXIT_CRITICAL();
}

Snapshot snapshot() {
    taskENTER_CRITICAL();
    const Snapshot copy = published;
    taskEXIT_CRITICAL();
    return copy;
}

void step(uint32_t now, float remote_pitch_percent, bool protected_mode,
          bool vision_target_active, float vision_target_rad) {
    taskENTER_CRITICAL();
    const ImuSample imu = imu_sample;
    taskEXIT_CRITICAL();
    // INS 优先级更高，可能在本次 now 取值后抢占并发布新帧；新帧年龄按 0 算。
    const int32_t imu_elapsed = static_cast<int32_t>(now - imu.time_ms);
    state.imu_age_ms = imu_elapsed < 0 ? 0u : static_cast<uint32_t>(imu_elapsed);
    state.imu_fresh = imu.valid && state.imu_age_ms <= timing::ImuTimeoutMs;
    if (imu.valid) { state.imu_angle = imu.angle_rad; state.imu_velocity = imu.velocity_rad_s; }
    const auto raw = damiao_mit_pitch_feedback_snapshot();
    // CAN 中断也可能在 now 采样后更新反馈，避免把稍新的时间误判成超时。
    const uint32_t feedback_now = raw.seen && static_cast<int32_t>(now - raw.last_ms) < 0 ? raw.last_ms : now;
    const auto assessment = damiao_mit_pitch::assess_snapshot(raw, feedback_now, timing::FeedbackTimeoutMs);
    state.feedback_count = raw.count;
    state.feedback_age_ms = raw.seen ? static_cast<uint32_t>(feedback_now - raw.last_ms) : 0;
    state.feedback_fresh = assessment.health == damiao_feedback::Health::Current;
    if (assessment.health == damiao_feedback::Health::Current ||
        assessment.health == damiao_feedback::Health::Fault ||
        assessment.health == damiao_feedback::Health::UnknownStatus)
        state.feedback = assessment.feedback;

    bool request_stop, request_enable, target_pending;
    float target_rad, kp, ki, kd;
    uint8_t gain_mask;
    taskENTER_CRITICAL();
    request_stop = debug.dm_stop_requested;
    request_enable = debug.dm_enable_requested;
    target_pending = debug.dm_target_pending;
    target_rad = debug.dm_target_rad;
#if 0 // 原相对目标缓存，仅保留历史实现。
    delta_pending = debug.dm_delta_pending;
    delta = debug.dm_delta_rad;
#endif
    gain_mask = debug.dm_gain_pending;
    kp = debug.dm_kp; ki = debug.dm_ki; kd = debug.dm_kd;
    debug.dm_stop_requested = false;
    debug.dm_enable_requested = false;
    debug.dm_target_pending = false;
    debug.dm_gain_pending = 0;
    taskEXIT_CRITICAL();

    if (vision_target_active) target_pending = false;

    // 在线调参只改增益，不重新初始化 PID，也不改变当前目标。
    if (gain_mask) {
        if (((gain_mask & 1) && (!std::isfinite(kp) || kp < 0)) ||
                 ((gain_mask & 2) && (!std::isfinite(ki) || ki < 0)) ||
                 ((gain_mask & 4) && (!std::isfinite(kd) || kd < 0)))
            state.reason = "BAD_GAIN";
        else {
            const float next_kp = gain_mask & 1 ? kp : state.kp;
            const float next_ki = gain_mask & 2 ? ki : state.ki;
            const float next_kd = gain_mask & 4 ? kd : state.kd;
            if (state.state == State::Running && !controller.update_gains(next_kp, next_ki, next_kd))
                state.reason = "BAD_GAIN";
            else {
                state.kp = next_kp; state.ki = next_ki; state.kd = next_kd;
                state.reason = "GAIN_SET";
            }
        }
    }
    if (protected_mode) {
        if (!remote_protected) {
            resume_after_remote_protect = first_step || startup_enable_pending ||
                                          state.state != State::Stopped || request_enable;
            if (request_stop) resume_after_remote_protect = false;
            first_step = false;
            remote_protected = true;
            if (state.state != State::Stopped || startup_enable_pending)
                stop("REMOTE_PROTECT", now);
            else {
                state.reason = "REMOTE_PROTECT";
                state.requested_torque = 0;
                state.gravity_torque = 0;
                send_disable(now);
            }
        } else if (request_stop) resume_after_remote_protect = false;
        else if (request_enable) resume_after_remote_protect = true;
        if (static_cast<uint32_t>(now - last_disable_ms) >= timing::DisablePeriodMs)
            send_disable(now);
        state.reason = "REMOTE_PROTECT";
        state.requested_torque = 0;
        state.gravity_torque = 0;
        publish();
        return;
    }
    if (remote_protected) {
        remote_protected = false;
        if (resume_after_remote_protect && !request_stop) startup_enable_pending = true;
        resume_after_remote_protect = false;
    }

    if (first_step || request_stop) {
        const bool startup = first_step;
        first_step = false;
        stop(request_stop ? "USER_STOP" : "STARTUP", now);
        // 首轮仍先发失能，确认电机状态；随后无需串口dm_enable即可启动。
        // 同一轮若收到dm_stop，则用户停止优先，取消本次上电自动使能。
        startup_enable_pending = startup && !request_stop;
        publish();
        return;
    }

    if (state.state == State::Stopped) {
        if (target_pending) state.reason = "TARGET_NEEDS_ENABLE";
        if (startup_enable_pending &&
            (assessment.health == damiao_feedback::Health::Fault ||
             assessment.health == damiao_feedback::Health::UnknownStatus)) {
            stop("STARTUP_FEEDBACK_FAULT", now);
            publish();
            return;
        }
        // 定期发失能帧用于确认状态和获取反馈；发送成功仅代表进入 CAN 邮箱。
        // 自动等待时也继续询问，允许电机比主控晚完成上电。
        if (!request_enable && static_cast<uint32_t>(now - last_disable_ms) >= timing::DisablePeriodMs) {
            send_disable(now);
        }
        if (request_enable || startup_enable_pending) {
            if (!state.imu_fresh) state.reason = "NEED_IMU";
            else if (imu.angle_rad < tune::MinPositionRad || imu.angle_rad > tune::MaxPositionRad)
                state.reason = "IMU_OUT_OF_RANGE";
            else if (!disable_submitted || !damiao_pitch_can_disable_tx_complete()) state.reason = "WAIT_DISABLE_TX";
            else if (!state.feedback_fresh || state.feedback.status != 0 || raw.count == disable_feedback_count)
                state.reason = "NEED_DISABLED_FEEDBACK";
            else if (damiao_pitch_can_send_enable() != HAL_OK) {
                ++state.tx_errors;
                state.reason = "ENABLE_TX_FAILED";
            } else {
                startup_enable_pending = false;
                state.state = State::Enabling;
                state.reason = target_pending ? "TARGET_NEEDS_RUNNING" : "WAIT_ENABLE";
                enable_start_ms = last_enable_ms = now;
                enable_feedback_count = raw.count;
            }
        }
    } else if (state.state == State::Enabling) {
        if (target_pending) state.reason = "TARGET_NEEDS_RUNNING";
        if (!state.imu_fresh) stop("IMU_LOST", now);
        else if (imu.angle_rad < tune::MinPositionRad || imu.angle_rad > tune::MaxPositionRad)
            stop("IMU_OUT_OF_RANGE", now);
        else if (state.feedback_fresh && raw.count != enable_feedback_count && state.feedback.status == 1) {
            // 之前编码器闭环入口保留作参考；实际控制目标改用同帧 IMU 姿态。
#if 0
            const float p = state.feedback.position;
#endif
            // 自动/手动使能都捕获此刻的IMU角度作为初始目标，不强制归零。
            // 例如启动时imu_rad=+0.12，则target_rad也从+0.12开始保持。
            const float p = imu.angle_rad;
            damiao_pitch_pid::Config config{};
            config.kp = state.kp; config.ki = state.ki; config.kd = state.kd;
            config.max_integral = tune::MaxIntegralNm; config.max_torque = tune::MaxTorqueNm;
            // 控制范围直接使用用户配置的 IMU 上下限，不再跟使能位置绑定。
#if 0 // 之前的临时测试范围，保留参考。
            config.min_position = std::fmax(p - TrialTravelRad, tune::MinPositionRad);
            config.max_position = std::fmin(p + TrialTravelRad, tune::MaxPositionRad);
#endif
            config.min_position = tune::MinPositionRad;
            config.max_position = tune::MaxPositionRad;
            config.max_dt = timing::MaxControlDtSeconds;
            if (!controller.configure(config, damiao_mit_pitch::EncodingLimits.torque) || !controller.start(p))
                stop("PID_INIT_FAILED", now);
            else {
                state.target_position = p;
                state.target_min = config.min_position;
                state.target_max = config.max_position;
                state.pid_fault_seen = false;
                state.pid_fault_reason = "NONE";
                state.state = State::Running;
                state.reason = target_pending ? "TARGET_NEEDS_RUNNING" : "RUNNING";
                last_control_ms = now;
            }
        } else if (static_cast<uint32_t>(now - enable_start_ms) > timing::EnableTimeoutMs ||
                   assessment.health == damiao_feedback::Health::Fault ||
                   assessment.health == damiao_feedback::Health::UnknownStatus) {
            stop("ENABLE_TIMEOUT_OR_FAULT", now);
        } else if (static_cast<uint32_t>(now - last_enable_ms) >= timing::EnablePeriodMs) {
            count_result(damiao_pitch_can_send_enable());
            last_enable_ms = now;
        }
    } else {
        if (!state.imu_fresh) {
            stop("IMU_LOST", now);
        } else if (!state.feedback_fresh || state.feedback.status != 1) {
            stop("FEEDBACK_LOST_OR_FAULT", now);
        } else {
            if (target_pending) {
                // 直接设定绝对角：例如 dm_target=0.10 始终设为 +0.10 rad。
                // set_target 检查用户配置的 IMU 范围。
                if (controller.set_target(target_rad)) {
                    state.target_position = controller.target();
                    state.reason = "RUNNING";
                } else state.reason = "TARGET_REJECTED";
            }
            if (vision_target_active) {
                if (std::isfinite(vision_target_rad) && controller.set_target(vision_target_rad)) {
                    state.target_position = controller.target();
                    state.reason = "VISION_TARGET";
                } else state.reason = "VISION_TARGET_REJECTED";
            }
#if 0 // 原相对角度处理保留，当前不再生效。
            if (delta_pending) {
                // 例：dm_delta=0.02 将 IMU 目标从 0.10 调到 0.12 rad。
                if (std::isfinite(delta) && std::fabs(delta) <= MaxStepRad &&
                    controller.set_target(controller.target() + delta)) {
                    state.target_position = controller.target();
                    state.reason = "RUNNING";
                } else state.reason = "DELTA_REJECTED";
            }
#endif
            const uint32_t elapsed = now - last_control_ms;
            if (elapsed != 0) {
                last_control_ms = now;
                damiao_mit::Command command{};
                // 用户确认水平时 imu_rad≈0，所以此处 β 就是实时 IMU 角度。
                // 例如 β=0，cos(0)=1、sin(0)=0，重力补偿为 -0.04169 Nm。
                // 标定扭矩以电机正方向为准；映射回 PID 方向后再参与限幅和抗饱和。
                const float gravity = tune::GravityCosCoefficientNm * std::cos(imu.angle_rad) +
                                      tune::GravitySinCoefficientNm * std::sin(imu.angle_rad);
                const float control_dt = elapsed * 0.001f;
                // 遥控器给的是目标角速度而非绝对角：目标增量 = 摇杆比例 × 配置速度 × dt。
                // 同周期串口绝对目标优先；摇杆松开时沿用最后的目标。
                if (!target_pending && !vision_target_active && control_dt <= timing::MaxControlDtSeconds &&
                    std::isfinite(remote_pitch_percent) && remote_pitch_percent != 0.0f) {
                    const float stick = std::fmax(-100.0f, std::fmin(100.0f, remote_pitch_percent));
                    const float next = controller.target() + stick * 0.01f * RemoteMaxSpeedRadPerSecond * control_dt;
                    const float bounded = std::fmax(state.target_min, std::fmin(state.target_max, next));
                    if (controller.set_target(bounded)) state.target_position = controller.target();
                }
                if (imu.angle_rad < state.target_min || imu.angle_rad > state.target_max)
                    stop_pid_with_diagnostics("PID_POSITION_LIMIT", now, imu.angle_rad, elapsed);
                else if (control_dt > timing::MaxControlDtSeconds)
                    stop_pid_with_diagnostics("PID_DT_TIMEOUT", now, imu.angle_rad, elapsed);
                else if (!controller.calculate(true, imu.angle_rad, imu.velocity_rad_s,
                                               control_dt, command, gravity * tune::MotorTorqueDirection))
                    stop_pid_with_diagnostics("PID_OR_POSITION_INVALID", now, imu.angle_rad, elapsed);
                else {
                    state.gravity_torque = gravity;
                    command.torque *= tune::MotorTorqueDirection;
                    if (damiao_pitch_can_send_control(command) != HAL_OK) {
                    ++state.tx_errors;
                    stop("CONTROL_TX_FAILED", now);
                    } else state.requested_torque = command.torque;
                }
            }
        }
    }
    publish();
}
}
