#pragma once
#include "PitchDirectionTest.hpp"

// 方向确认阶段，不使用尚未确认的IMU符号、零点或旧机械限位。
struct GimbalCommissioning
{
    static constexpr bool ContinuousCurrentTest=false;
    static constexpr int16_t ContinuousPitchCurrent=0;
    int16_t yaw_current=0;
    int16_t pitch_current=0;
    uint32_t started_ms=0;
    bool continuous_stopped=false;

    void step_continuous(PitchTestRequest request)
    {
        if(request==PitchTestRequest::Stop) continuous_stopped=true;
        yaw_current=0;
        pitch_current=continuous_stopped ? 0 : ContinuousPitchCurrent;
    }

    void step(uint32_t now,PitchTestRequest request)
    {
        if(static_cast<uint32_t>(now-started_ms)>=PitchDirectionTest::DurationMs)
            yaw_current=pitch_current=0;
        if(request==PitchTestRequest::Stop)
        { yaw_current=pitch_current=0; return; }
        int16_t yaw=0,pitch=0;
        switch(request)
        {
        case PitchTestRequest::YawPlus: yaw=PitchDirectionTest::Amplitude; break;
        case PitchTestRequest::YawMinus: yaw=-PitchDirectionTest::Amplitude; break;
        case PitchTestRequest::Plus:
        case PitchTestRequest::PositiveOnce: pitch=PitchDirectionTest::Amplitude; break;
        case PitchTestRequest::Minus: pitch=-PitchDirectionTest::Amplitude; break;
        case PitchTestRequest::Positive2000: pitch=PitchDirectionTest::MaxAmplitude; break;
        default: return; // enable/p:/视觉等请求不产生输出
        }
        yaw_current=yaw; pitch_current=pitch; started_ms=now;
    }
};
