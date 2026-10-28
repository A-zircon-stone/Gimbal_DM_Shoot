#pragma once
#include "legacy/CanDemoPid.hpp"
#include <cmath>
#include <cstdint>

// Yaw ID1：左转角度增大，正电流左转。沿用原PID的counts/rpm单位。
struct YawSpeedTune
{
    // ===== Yaw调参区；速度Kp由ykp=更新并直接启动速度环 =====
    static constexpr float SpeedKp=250.0f;
    static constexpr float SpeedKi=0.15f;
    static constexpr float SpeedKd=0.0f;
    static constexpr float PositionKp=0.5f;
    static constexpr float PositionKi=0.0f;
    static constexpr float PositionKd=15.0f; 
    static constexpr int16_t CurrentLimit=32767;
    static constexpr float TargetSpeedLimitRpm=120.0f;
    static constexpr float RemoteMaxSpeedDeg=250.0f;
    // =========================================================
    can_demo::PIDc pid{};
    bool enabled=false;//决定位置环速度环是否启用
    bool position_mode=false;//enabled=true but position_mode=false只启用速度环
    float target_yaw_deg=0; // 连续Yaw角，无机械角度限位
    float target_speed_rpm=0;
    float actual_speed_rpm=0;
    int16_t command=0;
    uint32_t last_ms=0;
    const char *reason="OFF";

    YawSpeedTune()
    {
        pid.PID_Init(&pid.PosParam,PositionKp,PositionKi,PositionKd);
        pid.PosParam.PID_OutMax=TargetSpeedLimitRpm;
        pid.PosParam.PID_OutStep=10;
        pid.PosParam.PID_ErrAllMax=20000;
        pid.PosParam.PID_Precision=5;
        pid.PID_Init(&pid.SpdParam,SpeedKp,SpeedKi,SpeedKd);
        pid.SpdParam.PID_OutMax=CurrentLimit;
        pid.SpdParam.PID_OutStep=10000;
        pid.SpdParam.PID_ErrAllMax=10000;
        pid.SpdParam.PID_Precision=0;
    }
    void stop(const char *why="STOP")//target_speed_rpm=0、command=0 保证立即生效（Gimbal.cpp 每帧把 command 发 CAN）
    { enabled=false; position_mode=false; command=0; target_speed_rpm=0; pid.PID_Clear(); reason=why; }
    void enable(uint32_t now)
    {
        if(enabled && !position_mode) return;
        pid.PID_Clear(); command=0; target_speed_rpm=0; last_ms=now;//进速度环，目标转速0
        enabled=true; position_mode=false; reason="ENABLED";
    }
    void enable_position(uint32_t now,float yaw_deg)// 进位置环，并且捕获当前角度当目标
    {
        if(!std::isfinite(yaw_deg) || (enabled && position_mode)) return;////已在位置环时不重置目标
        pid.PID_Clear(); command=0; target_speed_rpm=0; last_ms=now;//允许从速度环切进位置环，且切之前 PID_Clear()
        target_yaw_deg=yaw_deg; enabled=true; position_mode=true; reason="ENABLED";
    }
    void apply_gains(uint8_t mask,float kp,float ki,float kd,float kpp,uint32_t now,float yaw_deg,bool switch_mode=true)
    {
        if(mask&1) pid.SpdParam.Kp1=kp;
        if(mask&2) pid.SpdParam.Ki1=ki;
        if(mask&4) pid.SpdParam.Kd1=kd;
        if(mask&8) pid.PosParam.Kp1=kpp;
        if(!switch_mode) return; // 视觉许可期间只改参数，不隐式切换模式或控制权
        if(mask&8) enable_position(now,yaw_deg);
        else if(mask&1) enable(now); // ykp=直接进入零目标速度调试，不需要enable命令
    }
    bool shift_position(float left_degrees)
    {
        const float next=target_yaw_deg+left_degrees;
        if(!enabled || !position_mode || !std::isfinite(next)) return false;
        target_yaw_deg=next; return true;
    }
    void update_remote_target(float channel_percent,float dt)
    {
        if(!std::isfinite(channel_percent) || !std::isfinite(dt) || dt<=0) return;
        shift_position(std::fmax(-100.0f,std::fmin(100.0f,channel_percent))*0.01f*RemoteMaxSpeedDeg*dt);
    }
    // INS旧命名：Roll是标准ZYX的Y俯仰角，Pitch是标准X横滚角。
    // Yaw导数=(sin(X横滚)*gy+cos(X横滚)*gz)/cos(Y俯仰)。
    // 倾斜时不能直接把传感器gz当作Yaw速度。
    static float yaw_rate_rad(float roll_deg,float pitch_deg,float gy,float gz)
    {
        const float x=pitch_deg*0.017453292519943295f;
        const float y=roll_deg*0.017453292519943295f;
        return (std::sin(x)*gy+std::cos(x)*gz)/std::cos(y);
    }
    void step(uint32_t now,float yaw_deg,float yaw_rate,bool valid,float speed_ff_rpm=0,float current_ff=0)
    {
        if(!enabled) { command=0; return; }
        if(!valid || !std::isfinite(yaw_deg) || !std::isfinite(yaw_rate) || !std::isfinite(speed_ff_rpm) || !std::isfinite(current_ff))
        { stop("INVALID_FEEDBACK"); return; }
        if(static_cast<uint32_t>(now-last_ms)<2) return;
        last_ms=now;
        actual_speed_rpm=yaw_rate*9.549296586f;
        // 先减角度再换counts，保持连续Yaw的大角度精度，无+-180跳变。
        target_speed_rpm=position_mode ? pid.SingleLoop_PID(&pid.PosParam,0,
            (target_yaw_deg-yaw_deg)*22.755555556f) : 0;
        if(position_mode)
            target_speed_rpm=std::fmax(-pid.PosParam.PID_OutMax,
                std::fmin(pid.PosParam.PID_OutMax,target_speed_rpm+speed_ff_rpm));
        const float output=pid.SingleLoop_PID(&pid.SpdParam,actual_speed_rpm,target_speed_rpm)+
            (position_mode ? current_ff : 0);
        if(!std::isfinite(output)) { stop("INVALID_OUTPUT"); return; }
        command=static_cast<int16_t>(std::fmax(-pid.SpdParam.PID_OutMax,std::fmin(pid.SpdParam.PID_OutMax,output)));
        reason="RUNNING";
    }
};
