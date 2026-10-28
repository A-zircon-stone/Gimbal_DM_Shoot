#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cerrno>
#include <cctype>
#include "PitchTilt.hpp"

enum class PitchTestRequest : uint8_t { None, Arm, Plus, Minus, Stop, PositiveOnce, Positive2000, Hold, VisionEnable, PitchDelta, YawPlus, YawMinus, PositionEnable };
inline PitchTestRequest parse_pitch_test_command(const char *line,float *delta_deg=nullptr)
{
    if (!line) return PitchTestRequest::None;
    if (!std::strcmp(line, "position_enable")) return PitchTestRequest::PositionEnable;
    if (!std::strcmp(line, "speed_enable")) return PitchTestRequest::Hold;
    if (!std::strcmp(line, "yaw_plus")) return PitchTestRequest::YawPlus;
    if (!std::strcmp(line, "yaw_minus")) return PitchTestRequest::YawMinus;
    if (!std::strcmp(line, "stop")) return PitchTestRequest::Stop;
    if(line[0]=='p' && line[1]==':')
    {
        char *end=nullptr;
        errno=0;
        const float value=std::strtof(line+2,&end);
        if(end==line+2 || errno==ERANGE || !std::isfinite(value)) return PitchTestRequest::None;
        while(*end && std::isspace(static_cast<unsigned char>(*end))) ++end;
        if(*end) return PitchTestRequest::None;
        if(delta_deg) *delta_deg=value;
        return PitchTestRequest::PitchDelta;
    }
    if (!std::strcmp(line, "pitch_arm")) return PitchTestRequest::Arm;
    if (!std::strcmp(line, "pitch_plus")) return PitchTestRequest::Plus;
    if (!std::strcmp(line, "pitch_minus")) return PitchTestRequest::Minus;
    if (!std::strcmp(line, "pitch_stop")) return PitchTestRequest::Stop;
    if (!std::strcmp(line, "pitch_hold")) return PitchTestRequest::Hold;
    if (!std::strcmp(line, "pitch_enable")) return PitchTestRequest::Hold;
    if (!std::strcmp(line, "pitch_disable")) return PitchTestRequest::Stop;
    if (!std::strcmp(line, "vision_enable")) return PitchTestRequest::VisionEnable;
    if (!std::strcmp(line, "vision_disable")) return PitchTestRequest::Stop;
    if (!std::strcmp(line, "pitch_pos")) return PitchTestRequest::PositiveOnce;
    // 复用CAN_Demo用户命令；方向验证阶段只开放固定小幅短脉冲，不是持续输出。
    if (!std::strcmp(line, "current=600")) return PitchTestRequest::PositiveOnce;
    if (!std::strcmp(line, "current=2000")) return PitchTestRequest::Positive2000;
    if (!std::strcmp(line, "current=0")) return PitchTestRequest::Stop;
    return PitchTestRequest::None;
}

struct PitchTestInput
{
    bool fresh = false; // 同时要求新鲜IMU、两台CAN反馈
    uint16_t encoder = 0;
    int16_t rpm = 0;
    float speed_rad_s = 0;
};

// 只验证驱动方向，不运行PID。电流给定协议沿用用户已提供的CAN_Demo。
class PitchDirectionTest
{
public:
    // 用户要求方向测试忽略位置限位；不改变闭环映射/机械限位配置。
    explicit PitchDirectionTest(bool enforce_position_limits=false)
        : enforce_position_limits_(enforce_position_limits) {}
    enum State : uint8_t { Idle=0, Armed=1, Running=2, Done=3, Rejected=4, Fault=5, Stopped=6 };
    static constexpr uint32_t CommandId = 0x1FE; // CAN_Demo/User/Task/Gimbal.cpp
    static constexpr int16_t Amplitude = 600; // 原始电流命令值；尚未核实到安培的换算
    static constexpr int16_t MaxAmplitude = 2000; // 仅显式current=2000使用，旧600入口不变
    static constexpr uint32_t DurationMs = 500;
    State state = Idle;
    int16_t command = 0;
    int16_t travel_counts = 0;
    bool bus_owned = false; // 一旦用户请求测试，周期发零；默认不发测试帧

    void fail() { command=0; state=Fault; bus_owned=true; fault_latched=true; }
    void step(uint32_t now, PitchTestRequest request, const PitchTestInput &in)
    {
        const bool gap = started && static_cast<uint32_t>(now-last_step) > 10;
        last_step=now; started=true;
        const bool sane = in.fresh && std::isfinite(in.speed_rad_s) &&
            std::fabs(in.speed_rad_s) < 0.35f && in.rpm > -5 && in.rpm < 5 &&
            in.encoder <= 8191 && (!enforce_position_limits_ ||
                (in.encoder > PitchMechanism::SoftMin && in.encoder < PitchMechanism::SoftMax));
        if (request == PitchTestRequest::Stop)
        { command=0; state=fault_latched ? Fault : Stopped; bus_owned=true; return; }
        if(fault_latched) { command=0; state=Fault; return; }
        if (state == Running)
        {
            travel_counts = static_cast<int16_t>(static_cast<int>(in.encoder)-start_encoder);
            if (!sane || gap || (enforce_position_limits_ &&
                (travel_counts <= -40 || travel_counts >= 40)))
            { fail(); return; }
            if (static_cast<uint32_t>(now-pulse_start) >= DurationMs)
            { command=0; state=Done; }
            // 运行中任何命令均不续时/不反转/不重新武装。
            return;
        }
        if (state == Armed && (!sane || gap || static_cast<uint32_t>(now-arm_start) >= 5000))
        { command=0; state=Rejected; }
        if (request == PitchTestRequest::None) return;
        bus_owned=true;
        // 静止/反馈检查始终保留；绝对位置检查由测试配置决定。
        const bool start_safe = sane &&
            std::fabs(in.speed_rad_s) < 0.10f && in.rpm >= -1 && in.rpm <= 1;
        if(request == PitchTestRequest::PositiveOnce || request == PitchTestRequest::Positive2000)
        {
            // 单命令入口：无需先arm；仍检查静止/新鲜度，位置限制由构造参数决定。
            command=0; travel_counts=0;
            if(!start_safe) { state=Rejected; return; }
            command=request == PitchTestRequest::Positive2000 ? MaxAmplitude : Amplitude;
            start_encoder=in.encoder; pulse_start=now; state=Running;
            return;
        }
        if (request == PitchTestRequest::Arm)
        {
            command=0; travel_counts=0;
            // CAN故障锁定，必须重新启动；不能靠重新武装掩盖发送故障。
            if (state == Fault || !start_safe) { if(state!=Fault) state=Rejected; return; }
            arm_start=now; state=Armed; return;
        }
        if ((request == PitchTestRequest::Plus || request == PitchTestRequest::Minus) &&
            state == Armed && start_safe)
        {
            command = request == PitchTestRequest::Plus ? Amplitude : -Amplitude;
            start_encoder=in.encoder; pulse_start=now; state=Running;
        }
        else { command=0; if(state!=Fault) state=Rejected; }
    }
private:
    bool enforce_position_limits_;
    bool started=false;
    bool fault_latched=false;
    uint32_t last_step=0, arm_start=0, pulse_start=0;
    int start_encoder=0;
};

// 沿用CAN_Demo的0x1FE电流给定：ID1=0、ID2=command、ID3/4=0，大端。
inline void encode_pitch_test_frame(int16_t command, uint8_t (&bytes)[8])
{
    std::memset(bytes, 0, sizeof(bytes));
    if (command < -PitchDirectionTest::MaxAmplitude || command > PitchDirectionTest::MaxAmplitude) return;
    const auto word=static_cast<uint16_t>(command);
    bytes[2]=static_cast<uint8_t>(word >> 8);
    bytes[3]=static_cast<uint8_t>(word);
}
