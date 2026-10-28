#include "PitchDirectionTest.hpp"
#include "GimbalCommissioning.hpp"
#include <cassert>
#include <limits>
#include <initializer_list>

using Req=PitchTestRequest;
using Test=PitchDirectionTest;
int main()
{
    GimbalCommissioning setup;
    GimbalCommissioning continuous;
    continuous.step_continuous(Req::None);
    assert(continuous.yaw_current==0 && continuous.pitch_current==GimbalCommissioning::ContinuousPitchCurrent);
    for(unsigned i=0;i<1000;++i) continuous.step_continuous(Req::None);
    assert(continuous.yaw_current==0 && continuous.pitch_current==GimbalCommissioning::ContinuousPitchCurrent);
    continuous.step_continuous(Req::Stop);
    continuous.step_continuous(Req::None);
    assert(continuous.yaw_current==0 && continuous.pitch_current==0);
    setup.step(0,Req::None);
    assert(setup.yaw_current==0 && setup.pitch_current==0);
    setup.step(100,Req::Hold);
    setup.step(101,Req::VisionEnable);
    setup.step(102,Req::PitchDelta);
    assert(setup.yaw_current==0 && setup.pitch_current==0);
    assert(parse_pitch_test_command("yaw_plus")==Req::YawPlus);
    assert(parse_pitch_test_command("yaw_minus")==Req::YawMinus);
    assert(parse_pitch_test_command("stop")==Req::Stop);
    setup.step(200,Req::YawPlus);
    assert(setup.yaw_current==600 && setup.pitch_current==0);
    setup.step(699,Req::None); assert(setup.yaw_current==600);
    setup.step(700,Req::None); assert(setup.yaw_current==0);
    setup.step(800,Req::YawMinus); assert(setup.yaw_current==-600);
    setup.step(801,Req::Plus); assert(setup.yaw_current==0 && setup.pitch_current==600);
    setup.step(802,Req::Minus); assert(setup.pitch_current==-600);
    setup.step(803,Req::Stop); assert(setup.yaw_current==0 && setup.pitch_current==0);
    setup.step(900,Req::Positive2000); assert(setup.pitch_current==2000);
    setup.step(1400,Req::None); assert(setup.pitch_current==0);
    static_assert(Test::CommandId==0x1FE, "必须复用CAN_Demo电流命令ID");
    assert(parse_pitch_test_command("current=600")==Req::PositiveOnce);
    assert(parse_pitch_test_command("current=0")==Req::Stop);
    assert(parse_pitch_test_command("current=2000")==Req::Positive2000);
    assert(parse_pitch_test_command("current=2001")==Req::None);
    assert(parse_pitch_test_command("current=6000")==Req::None);
    PitchTestInput in{true,5200,0,0};
    // 默认方向测试不执行位置限制，但500ms超时、速度和反馈保护不变。
    Test no_limits;
    auto near_end=in; near_end.encoder=6500;
    no_limits.step(0,Req::Positive2000,near_end);
    assert(no_limits.state==Test::Running && no_limits.command==2000);
    near_end.encoder=6600;
    no_limits.step(2,Req::None,near_end);
    assert(no_limits.state==Test::Running && no_limits.travel_counts==100);
    for(unsigned now=4;now<Test::DurationMs;now+=2) no_limits.step(now,Req::None,near_end);
    assert(no_limits.command==2000);
    no_limits.step(Test::DurationMs,Req::None,near_end);
    assert(no_limits.state==Test::Done && no_limits.command==0);
    Test no_limits_fault;
    no_limits_fault.step(0,Req::Positive2000,in);
    auto lost=in; lost.fresh=false;
    no_limits_fault.step(2,Req::None,lost);
    assert(no_limits_fault.state==Test::Fault && no_limits_fault.command==0);
    Test t{true};
    assert(parse_pitch_test_command("pitch_pos")==Req::PositiveOnce);
    Test once{true};
    once.step(0,Req::PositiveOnce,in);
    assert(once.command==600 && once.state==Test::Running);
    for(unsigned now=2;now<Test::DurationMs;now+=2) once.step(now,Req::None,in);
    assert(once.command==600 && once.state==Test::Running);
    once.step(Test::DurationMs,Req::None,in);
    assert(once.command==0 && once.state==Test::Done);
    auto unsafe=in; unsafe.encoder=PitchMechanism::EncoderMax;
    once.step(Test::DurationMs+2,Req::PositiveOnce,unsafe);
    assert(once.command==0 && once.state==Test::Rejected);
    t.step(0,Req::None,in);
    assert(t.state==Test::Idle && t.command==0 && !t.bus_owned);
    t.step(2,Req::Plus,in);
    assert(t.state==Test::Rejected && t.command==0);
    t.step(4,Req::Arm,in);
    assert(t.state==Test::Armed && t.command==0);
    t.step(6,Req::Plus,in);
    assert(t.state==Test::Running && t.command==600);
    for(unsigned now=8;now<6+Test::DurationMs;now+=2) t.step(now,Req::Plus,in);
    t.step(6+Test::DurationMs,Req::None,in);
    assert(t.state==Test::Done && t.command==0);
    t.step(8+Test::DurationMs,Req::Minus,in); // 不重复触发
    assert(t.command==0);
    t.step(10+Test::DurationMs,Req::Arm,in); t.step(12+Test::DurationMs,Req::Minus,in);
    assert(t.command==-600);
    t.step(14+Test::DurationMs,Req::Stop,in);
    assert(t.state==Test::Stopped && t.command==0);

    // 安全条件变化与调试暂停，都锁定为零，不恢复脉冲。
    for(unsigned mode=0;mode<7;++mode)
    {
        Test s{true}; auto bad=in;
        s.step(0,Req::Arm,in); s.step(2,Req::Plus,in);
        switch(mode)
        {
        case 0: bad.fresh=false; break;
        case 1: bad.encoder=5240; break;
        case 2: bad.encoder=5160; break;
        case 3: bad.rpm=5; break;
        case 4: bad.speed_rad_s=0.35f; break;
        case 5: bad.speed_rad_s=std::numeric_limits<float>::quiet_NaN(); break;
        default: break;
        }
        s.step(mode==6 ? 20 : 4,Req::None,bad);
        assert(s.state==Test::Fault && s.command==0);
        s.step(22,Req::Stop,in); s.step(24,Req::Arm,in); s.step(26,Req::Plus,in);
        assert(s.state==Test::Fault && s.command==0);
    }
    Test expired{true};
    expired.step(0,Req::Arm,in);
    for(unsigned now=2;now<=5000;now+=2) expired.step(now,Req::None,in);
    expired.step(5002,Req::Plus,in);
    assert(expired.command==0 && expired.state==Test::Rejected);
    Test outside{true}; auto bad=in; bad.encoder=6500;
    outside.step(0,Req::Arm,bad);
    assert(outside.command==0 && outside.state==Test::Rejected);
    // 显式启用限位时，两端及保护带仍禁止启动。
    for(uint16_t encoder : {PitchMechanism::EncoderMin,PitchMechanism::SoftMin,
                           PitchMechanism::SoftMax,PitchMechanism::EncoderMax})
    {
        Test edge{true}; auto edge_input=in; edge_input.encoder=encoder;
        edge.step(0,Req::Arm,edge_input);
        assert(edge.state==Test::Rejected && edge.command==0);
    }
    Test relaxed{true}; auto relaxed_input=in; relaxed_input.encoder=6021;
    relaxed.step(0,Req::Arm,relaxed_input);
    assert(relaxed.state==Test::Armed);
    relaxed.step(2,Req::Plus,relaxed_input);
    assert(relaxed.state==Test::Running && relaxed.command==600);
    uint8_t bytes[8];
    encode_pitch_test_frame(600,bytes);
    assert(bytes[0]==0 && bytes[1]==0 && bytes[2]==2 && bytes[3]==88);
    for(unsigned i=4;i<8;++i) assert(bytes[i]==0);
    encode_pitch_test_frame(-600,bytes);
    assert(bytes[2]==0xFD && bytes[3]==0xA8);
    encode_pitch_test_frame(2000,bytes);
    assert(bytes[2]==0x07 && bytes[3]==0xD0);
    assert(bytes[0]==0 && bytes[1]==0);
    for(unsigned i=4;i<8;++i) assert(bytes[i]==0);
    encode_pitch_test_frame(2001,bytes);
    for(auto v:bytes) assert(v==0);
    assert(parse_pitch_test_command("pitch_arm")==Req::Arm);
    assert(parse_pitch_test_command("pitch_plus")==Req::Plus);
    assert(parse_pitch_test_command("pitch_minus")==Req::Minus);
    assert(parse_pitch_test_command("pitch_stop")==Req::Stop);
    assert(parse_pitch_test_command("pitch_plusX")==Req::None);
    Test larger{true};
    larger.step(0,Req::Positive2000,in);
    assert(larger.command==2000 && larger.state==Test::Running);
    for(unsigned now=2;now<Test::DurationMs;now+=2) larger.step(now,Req::None,in);
    assert(larger.command==2000 && larger.state==Test::Running);
    larger.step(Test::DurationMs,Req::None,in);
    assert(larger.command==0 && larger.state==Test::Done);
    larger.step(Test::DurationMs+2,Req::Positive2000,unsafe);
    assert(larger.command==0 && larger.state==Test::Rejected);
}
