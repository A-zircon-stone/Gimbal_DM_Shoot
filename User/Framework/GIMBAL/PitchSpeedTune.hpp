#pragma once
#include "PitchImuHold.hpp"

// 新云台：Pitch ID3，正电流抬头，抬头时原始Roll和gy减小。
// 默认仅速度环；position_enable显式切换串级双环，不自动回位。
struct PitchSpeedTune
{
    static constexpr float RollMinDeg=-35.0f;
    static constexpr float RollMaxDeg=5.0f;
    static constexpr float RemoteMaxSpeedDeg=100.0f; // 右摇杆满量程时，目标俯仰角变化速度°/s
    can_demo::PIDc pid{};
    bool enabled=false;
    bool position_mode=false;
    float target_roll_deg=0;
    float target_speed_rpm=0;
    int16_t command=0;
    uint32_t last_ms=0;
    const char *reason="OFF";
    uint32_t last_can_error=0;

    PitchSpeedTune()
    {
        pid.PID_Init(&pid.PosParam,PitchImuHold::PositionKp,PitchImuHold::PositionKi,PitchImuHold::PositionKd);
        pid.PosParam.PID_OutMax=PitchImuHold::TargetSpeedLimitRpm;
        pid.PosParam.PID_OutStep=10;
        pid.PosParam.PID_ErrAllMax=20000;
        pid.PosParam.PID_Precision=5;
        pid.PID_Init(&pid.SpdParam,PitchImuHold::SpeedKp,PitchImuHold::SpeedKi,PitchImuHold::SpeedKd);
        pid.SpdParam.PID_OutMax=PitchImuHold::CurrentLimit;
        pid.SpdParam.PID_OutStep=15000;
        pid.SpdParam.PID_ErrAllMax=10000;
        pid.SpdParam.PID_Precision=0; // 调速度环时不吞掉低于12°/s的运动
    }
    void stop(const char *why="STOP") { enabled=false; position_mode=false; target_speed_rpm=0; command=0; pid.PID_Clear(); reason=why; }
    void enable(uint32_t now)
    {
        if(enabled && !position_mode) return;
        pid.PID_Clear(); command=0; target_speed_rpm=0; last_ms=now;
        enabled=true; position_mode=false; reason="ENABLED";
    }
    void enable_position(uint32_t now,float roll_deg)
    {
        if(!std::isfinite(roll_deg) || (enabled && position_mode)) return;
        pid.PID_Clear(); command=0; target_speed_rpm=0; last_ms=now;
        target_roll_deg=std::fmax(RollMinDeg,std::fmin(RollMaxDeg,roll_deg));
        enabled=true; position_mode=true; reason="ENABLED";
    }
    bool shift_position(float up_degrees)
    {
        if(!enabled || !position_mode || !std::isfinite(up_degrees)) return false;
        const float next=target_roll_deg-up_degrees; // 抬头正，原始Roll减小
        if(!std::isfinite(next)) return false;
        target_roll_deg=std::fmax(RollMinDeg,std::fmin(RollMaxDeg,next));
        return true;
    }
    bool update_remote_target(float channel_percent,float dt)
    {
        if(!enabled || !position_mode || !std::isfinite(channel_percent) ||
           !std::isfinite(dt) || dt<=0 || channel_percent==0) return false;
        const float stick=std::fmax(-100.0f,std::fmin(100.0f,channel_percent));
        return shift_position(stick*0.01f*RemoteMaxSpeedDeg*dt);
    }
    void step(uint32_t now,float roll_deg,float gyro_y_rad,bool sample_valid,bool apply_angle_limits=true,float speed_ff_rpm=0,float current_ff=0)
    {
        if(!enabled) { command=0; return; }
        if(!sample_valid || !std::isfinite(roll_deg) || !std::isfinite(gyro_y_rad) || !std::isfinite(speed_ff_rpm) || !std::isfinite(current_ff))
        { stop("INVALID_FEEDBACK"); return; }
        if(static_cast<uint32_t>(now-last_ms)<2) return;
        last_ms=now;
        const float up_positive_rpm=-gyro_y_rad*9.549296586f;
        target_speed_rpm=position_mode ? pid.SingleLoop_PID(&pid.PosParam,
            -roll_deg*22.755555556f,-target_roll_deg*22.755555556f) : 0;
        if(position_mode)
            target_speed_rpm=std::fmax(-pid.PosParam.PID_OutMax,
                std::fmin(pid.PosParam.PID_OutMax,target_speed_rpm+speed_ff_rpm));
        // 位置环输出目标rpm；视觉速度前馈叠加后仍使用既有速度限幅。
        const float output=pid.SingleLoop_PID(&pid.SpdParam,up_positive_rpm,target_speed_rpm)+
            (position_mode ? current_ff : 0);
        if(!std::isfinite(output)) { stop("INVALID_OUTPUT"); return; }
        command=static_cast<int16_t>(std::fmax(-pid.SpdParam.PID_OutMax,std::fmin(pid.SpdParam.PID_OutMax,output)));
        reason="RUNNING";
        // 边界只禁止继续向外输出；范围采用用户确认的原始Roll。
        if(apply_angle_limits && ((roll_deg<=RollMinDeg && command>0) ||
           (roll_deg>=RollMaxDeg && command<0))) { command=0; reason="LIMIT"; }
    }
};
