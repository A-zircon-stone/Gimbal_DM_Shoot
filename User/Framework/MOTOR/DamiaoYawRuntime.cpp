#include "DamiaoYawRuntime.hpp"
#include "DamiaoYawConfig.hpp"
#include "YawSoftLimit.hpp"
#include "CANio.hpp"
#include "debugc.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>

namespace damiao_yaw_runtime {
namespace {
// 调参统一修改 GimbalTuning.hpp；这里仅执行控制逻辑。
namespace tune = gimbal_tuning::yaw;
namespace timing = gimbal_tuning::timing;
constexpr float RemoteMaxSpeed = tune::RemoteMaxSpeedDegPerSecond * gimbal_tuning::DegreesToRadians;
struct Imu { bool valid=false; float angle=0, velocity=0; uint32_t time=0; };
Imu imu{};
Snapshot state{}, published{};
uint32_t last_disable=0,last_enable=0,enable_start=0,disable_count=0,enable_count=0,last_control=0;
bool first=true, auto_pending=false, disable_submitted=false;
bool remote_protected=false, resume_after_remote_protect=false;
Pid_t yaw_position_pid{};
Pid_t yaw_speed_pid{};
void publish() { taskENTER_CRITICAL(); published=state; taskEXIT_CRITICAL(); }
void disable(uint32_t now, uint32_t count) {
    if (damiao_yaw_can_send_disable()==HAL_OK) { disable_submitted=true; disable_count=count; }
    last_disable=now;
}
void stop(const char *why,uint32_t now,uint32_t count) {
    auto_pending=false; state.state=State::Stopped; state.reason=why;
    state.torque=0; state.target_velocity=0;
    yaw_position_pid.Clear(); yaw_speed_pid.Clear();
    disable_submitted=false; disable(now,count);
}
}
void publish_imu(float yaw_deg,float roll_deg,float pitch_deg,float gy,float gz,uint32_t now) {
    // 沿用旧工程欧拉角速度计算：Yaw导数=(sin(Pitch)*gy+cos(Pitch)*gz)/cos(Roll)。
    const float x=pitch_deg*gimbal_tuning::DegreesToRadians, y=roll_deg*gimbal_tuning::DegreesToRadians;
    const float velocity=(std::sin(x)*gy+std::cos(x)*gz)/std::cos(y);
    Imu next{}; next.valid=std::isfinite(yaw_deg)&&std::isfinite(roll_deg)&&
        std::fabs(roll_deg)<timing::MaxAbsImuRollDeg&&std::isfinite(pitch_deg)&&std::isfinite(velocity);
    next.angle=yaw_deg*gimbal_tuning::DegreesToRadians; next.velocity=velocity; next.time=now;
    taskENTER_CRITICAL(); imu=next; taskEXIT_CRITICAL();
}
void invalidate_imu() { taskENTER_CRITICAL(); imu.valid=false; taskEXIT_CRITICAL(); }
Snapshot snapshot() { taskENTER_CRITICAL(); const auto copy=published; taskEXIT_CRITICAL(); return copy; }
void step(uint32_t now,float stick,bool protected_mode,bool vision_target_active,float vision_target_rad) {
    taskENTER_CRITICAL(); const Imu sample=imu;
    const bool request_stop=debug.yaw_stop_requested;
    const bool request_enable=debug.yaw_enable_requested;
    const uint8_t gain_mask=debug.yaw_gain_pending;
    const float kp=debug.yaw_speed_kp,ki=debug.yaw_speed_ki,kd=debug.yaw_speed_kd;
    const float position_kp=debug.yaw_position_kp;
    debug.yaw_stop_requested=false; debug.yaw_enable_requested=false; debug.yaw_gain_pending=0;
    taskEXIT_CRITICAL();
    const int32_t imu_age=static_cast<int32_t>(now-sample.time);
    state.imu_fresh=sample.valid && (imu_age<0 || static_cast<uint32_t>(imu_age)<=timing::ImuTimeoutMs);
    if (sample.valid) { state.angle=sample.angle; state.velocity=sample.velocity; }
    const auto raw=damiao_mit_yaw_feedback_snapshot();
    const uint32_t fb_now=raw.seen && static_cast<int32_t>(now-raw.last_ms)<0 ? raw.last_ms : now;
    const auto feedback=damiao_mit_yaw::assess_snapshot(raw,fb_now,timing::FeedbackTimeoutMs);
    state.feedback_fresh=feedback.health==damiao_feedback::Health::Current;
    auto speed_gains=state.speed_gains;
    auto position_gains=state.position_gains;
    if (gain_mask&1) speed_gains.kp=kp;
    if (gain_mask&2) speed_gains.ki=ki;
    if (gain_mask&4) speed_gains.kd=kd;
    if (gain_mask&8) position_gains.kp=position_kp;
    // SetGains成功才发布新参数；失败时继续使用上一组有效增益。
    if (yaw_position_pid.SetGains(position_gains)) state.position_gains=position_gains;
    else state.reason="BAD_GAIN";
    if (yaw_speed_pid.SetGains(speed_gains)) state.speed_gains=speed_gains;
    else state.reason="BAD_GAIN";
    if (protected_mode) {
        if (!remote_protected) {
            resume_after_remote_protect=first || auto_pending || state.state!=State::Stopped || request_enable;
            if(request_stop) resume_after_remote_protect=false;
            first=false;
            remote_protected=true;
            if(state.state!=State::Stopped || auto_pending)
                stop("REMOTE_PROTECT",now,raw.count);
            else {
                state.reason="REMOTE_PROTECT";
                state.torque=0;
                disable(now,raw.count);
            }
        } else if(request_stop) resume_after_remote_protect=false;
        else if(request_enable) resume_after_remote_protect=true;
        if(static_cast<uint32_t>(now-last_disable)>=timing::DisablePeriodMs)
            disable(now,raw.count);
        state.reason="REMOTE_PROTECT";
        state.torque=0;
        publish(); return;
    }
    if(remote_protected) {
        remote_protected=false;
        if(resume_after_remote_protect && !request_stop) auto_pending=true;
        resume_after_remote_protect=false;
    }
    if (first || request_stop) {
        const bool startup=first; first=false;
        stop(request_stop ? "USER_STOP" : "STARTUP",now,raw.count);
        auto_pending=startup && !request_stop; publish(); return;
    }
    if (state.state==State::Stopped) {
        if (auto_pending && (feedback.health==damiao_feedback::Health::Fault ||
                             feedback.health==damiao_feedback::Health::UnknownStatus)) {
            stop("STARTUP_FEEDBACK_FAULT",now,raw.count); publish(); return;
        }
        if (!request_enable && static_cast<uint32_t>(now-last_disable)>=timing::DisablePeriodMs) disable(now,raw.count);
        if (request_enable || auto_pending) {
            if (!state.imu_fresh) state.reason="NEED_IMU";
            else if (!disable_submitted || !damiao_yaw_can_disable_tx_complete()) state.reason="WAIT_DISABLE_TX";
            else if (!state.feedback_fresh || feedback.feedback.status!=0 || raw.count==disable_count)
                state.reason="NEED_DISABLED_FEEDBACK";
            else if (damiao_yaw_can_send_enable()!=HAL_OK) state.reason="ENABLE_TX_FAILED";
            else { auto_pending=false; state.state=State::Enabling; state.reason="WAIT_ENABLE";
                   enable_start=last_enable=now; enable_count=raw.count; }
        }
    } else if (state.state==State::Enabling) {
        if (!state.imu_fresh) stop("IMU_LOST",now,raw.count);
        else if (state.feedback_fresh && raw.count!=enable_count && feedback.feedback.status==1) {
            // 上电目标取使能确认这一帧的连续Yaw，不归零、不使用电机编码器。
            state.target=yaw_soft_limit::apply(sample.angle,tune::SoftLimitEnabled,
                                              tune::MinPositionRad,tune::MaxPositionRad);
            yaw_position_pid.Clear(); yaw_speed_pid.Clear();
            state.target_velocity=0; last_control=now;
            state.state=State::Running; state.reason="RUNNING";
        } else if (static_cast<uint32_t>(now-enable_start)>timing::EnableTimeoutMs ||
                   feedback.health==damiao_feedback::Health::Fault ||
                   feedback.health==damiao_feedback::Health::UnknownStatus)
            stop("ENABLE_TIMEOUT_OR_FAULT",now,raw.count);
        else if (static_cast<uint32_t>(now-last_enable)>=timing::EnablePeriodMs) {
            (void)damiao_yaw_can_send_enable(); last_enable=now;
        }
    } else if (!state.imu_fresh) stop("IMU_LOST",now,raw.count);
    else if (!state.feedback_fresh || feedback.feedback.status!=1) stop("FEEDBACK_LOST_OR_FAULT",now,raw.count);
    else {
        const uint32_t elapsed=now-last_control;
        if (elapsed) {
            last_control=now;
            if (elapsed>timing::MaxControlDtMs) { stop("PID_DT_TIMEOUT",now,raw.count); publish(); return; }
            const float dt=elapsed*0.001f;
            // 目标增量 = 摇杆比例 × 配置速度 × 方向 × dt。
            if(vision_target_active) {
                if(std::isfinite(vision_target_rad)) state.target=vision_target_rad;
            } else if (std::isfinite(stick)) state.target+=std::fmax(-100.0f,std::fmin(100.0f,stick))*0.01f*
                                                   RemoteMaxSpeed*tune::RemoteDirection*dt;
            state.target=yaw_soft_limit::apply(state.target,tune::SoftLimitEnabled,
                                              tune::MinPositionRad,tune::MaxPositionRad);
            // 位置环输出目标角速度，速度环输出力矩；两环状态独立且跨周期保存。
            if (!yaw_position_pid.Calculate({state.target,sample.angle,dt,0,sample.velocity}) ||
                !yaw_speed_pid.Calculate({yaw_position_pid.Output,sample.velocity,dt},
                                         {tune::MaxTorqueNm})) {
                stop("PID_INVALID",now,raw.count); publish(); return;
            }
            state.target_velocity=yaw_position_pid.Output;
            const float torque=yaw_speed_pid.Output*tune::MotorTorqueDirection;
            const damiao_mit::Command command{0,0,0,0,torque};
            if (damiao_yaw_can_send_control(command)!=HAL_OK) stop("CONTROL_TX_FAILED",now,raw.count);
            else state.torque=torque;
        }
    }
    publish();
}
}
