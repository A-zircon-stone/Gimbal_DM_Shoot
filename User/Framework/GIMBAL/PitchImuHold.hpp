#pragma once
#include "legacy/CanDemoPid.hpp"
#include "PitchTilt.hpp"
#include <cmath>
#include <cstdint>

struct PitchHoldInput
{
    bool sample_valid = false; // 已有合法样本；闭环不再按反馈年龄停机
    float angle_rad = 0; // 抬头为正，来自-TY，不使用编码器作角度环反馈
    float speed_rad_s = 0; // 抬头为正，来自-gy
    uint16_t encoder = 0; // 保留电机反馈；闭环限位改用实际IMU角度
};

class PitchImuHold
{
public:
    enum State : uint8_t { Off=0, Holding=1, Complete=2, Rejected=3, Fault=4 };
    // ===== Pitch闭环调参区（沿用CAN_Demo的等效counts/rpm尺度） =====
    static constexpr float PositionKp=0.6f;//0.3f;
    static constexpr float PositionKi=0.0f;
    static constexpr float PositionKd=1.0f;
    static constexpr float SpeedKp=80.0f;
    static constexpr float SpeedKi=0.15f;
    static constexpr float SpeedKd=0.0f;
    static constexpr int16_t CurrentLimit = 32767;
    static constexpr float TargetSpeedLimitRpm=200.0f;
    // 与pitch打印的实际角度一致；到达边界沿用停输出、锁存Fault。
    static constexpr float AngleMinDeg=-90.0f;
    static constexpr float AngleMaxDeg=80.0f;
    static constexpr float AngleMinRad=AngleMinDeg*0.017453292519943295f;
    static constexpr float AngleMaxRad=AngleMaxDeg*0.017453292519943295f;
    static constexpr float BootTargetDeg=-5.0f;
    static constexpr float RemoteSensitivity=0.5f; // 沿用旧RC：[-100,100]通道量乘此值得rad/s
    // ===========================================================
    State state=Off;
    bool bus_owned=false;
    int16_t command=0;
    float target_rad=0;
    float target_speed_rpm=0;
    bool boot_target_pending=true;
    can_demo::PIDc pid{}; // CAN_Demo原PID算法，仅增加命名空间，避免旧工程类型冲突

    PitchImuHold()
    {
        // CAN_Demo/GimbalAxis.hpp中的增益和积分/步幅限制，维持其counts/rpm数值域。
        pid.PID_Init(&pid.PosParam,PositionKp,PositionKi,PositionKd);
        pid.PID_Init(&pid.SpdParam,SpeedKp,SpeedKi,SpeedKd);
        pid.PosParam.PID_OutMax=TargetSpeedLimitRpm;
        pid.PosParam.PID_OutStep=5;
        pid.PosParam.PID_ErrAllMax=20000;
        pid.PosParam.PID_Precision=5;
        pid.SpdParam.PID_OutMax=CurrentLimit;
        pid.SpdParam.PID_OutStep=1500;
        pid.SpdParam.PID_ErrAllMax=10000;
        pid.SpdParam.PID_Precision=2;
    }
    void stop() { boot_target_pending=false; pid.PID_Clear(); command=0; target_speed_rpm=0; state=Off; bus_owned=true; }
    void fail() { stop(); state=Fault; }
    // 初始化后，等待有效反馈和现有使能条件满足，仅自动设置一次目标。
    bool start_boot_target(uint32_t now,const PitchHoldInput &in)
    {
        if(!boot_target_pending || !in.sample_valid) return false;
        if(!enable(now,in)) return false;
        set_target(BootTargetDeg*0.017453292519943295f);
        boot_target_pending=false;
        return true;
    }
    bool set_target(float radians)
    {
        if(state!=Holding || !std::isfinite(radians)) return false;
        target_rad=radians;
        return true;
    }
    bool shift_target_degrees(float delta)
    {
        if(state!=Holding || !std::isfinite(delta)) return false;
        const float next=target_rad+delta*0.017453292519943295f;
        // 只拒绝数值溢出，不额外引入角度幅度停机限制。
        if(!std::isfinite(next*1303.7972938f)) return false;
        return set_target(next);
    }
    bool update_remote_target(float channel_percent,float dt)
    {
        if(state!=Holding || !std::isfinite(channel_percent) ||
           !std::isfinite(dt) || dt<=0 || channel_percent==0) return false;
        const float stick=std::fmax(-100.0f,std::fmin(100.0f,channel_percent));
        const float next=target_rad+stick*RemoteSensitivity*dt;
        return set_target(std::fmax(AngleMinRad,std::fmin(AngleMaxRad,next)));
    }
    bool enable(uint32_t now, const PitchHoldInput &in)
    {
        bus_owned=true;
        if(state==Holding) return true; // 重复启用不改变目标
        if(state==Fault || !valid(in) || std::fabs(in.speed_rad_s)>0.10f)
        { command=0; if(state!=Fault) state=Rejected; return false; }
        pid.PID_Clear();
        target_rad=in.angle_rad; command=0; target_speed_rpm=0;
        last_ms=now; state=Holding;
        return true;
    }
    void step(uint32_t now,const PitchHoldInput &in)
    {
        if(state!=Holding) return;
        const uint32_t elapsed=now-last_ms;
        if(!valid(in))
        { fail(); return; }
        if(elapsed<2) return; // 旧PID按每2ms一次累加积分，避免重复调用
        last_ms=now;
        // 输入来自IMU，只做单位变换，保持原增益适用的counts/rpm数值尺度。
        const float actual_counts=in.angle_rad*1303.7972938f; // 8192/(2pi)
        const float target_counts=target_rad*1303.7972938f;
        const float actual_rpm=in.speed_rad_s*9.549296586f; // 60/(2pi)
        target_speed_rpm=pid.SingleLoop_PID(&pid.PosParam,actual_counts,target_counts);
        const float up_positive=pid.SingleLoop_PID(&pid.SpdParam,actual_rpm,target_speed_rpm);
        // 已实测正电流低头，抬头正坐标的PID输出在唯一出口反号。
        const float raw=-up_positive;
        if(!std::isfinite(raw) || std::fabs(raw)>CurrentLimit) { fail(); return; }
        command=static_cast<int16_t>(raw);
    }
private:
    uint32_t last_ms=0;
    static bool valid(const PitchHoldInput &in)
    {
        return in.sample_valid && std::isfinite(in.angle_rad) && std::isfinite(in.speed_rad_s) &&
            in.angle_rad>AngleMinRad && in.angle_rad<AngleMaxRad;
    }
};
