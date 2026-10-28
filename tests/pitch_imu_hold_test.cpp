#include "PitchImuHold.hpp"
#include "PitchDirectionTest.hpp"
#include "PitchSpeedPidCommand.hpp"
#include "PitchSpeedTune.hpp"
#include <cassert>
#include <initializer_list>
#include <limits>

// 回归测试使用固定增益，不依赖实机调参区当前是否关闭了角度环。
struct RegressionHold : PitchImuHold
{
    RegressionHold()
    {
        pid.PosParam.Kp1=0.3f; pid.PosParam.Ki1=0.005f; pid.PosParam.Kd1=0;
        pid.SpdParam.Kp1=20; pid.SpdParam.Ki1=0.002f; pid.SpdParam.Kd1=0;
    }
};

int main()
{
    float gain=99;
    assert(parse_pitch_test_command("position_enable")==PitchTestRequest::PositionEnable);
    assert(parse_pitch_test_command("speed_enable")==PitchTestRequest::Hold);
    PitchSpeedTune position;
    position.pid.PosParam.Kp1=0.4f; position.pid.PosParam.Ki1=0; position.pid.PosParam.Kd1=0;
    position.pid.SpdParam.Kp1=35; position.pid.SpdParam.Ki1=0; position.pid.SpdParam.Kd1=0;
    assert(!position.shift_position(2));
    position.enable_position(0,-10);
    assert(position.position_mode && position.target_roll_deg==-10);
    position.step(2,-10,0,true); assert(position.command==0);
    position.step(4,-8,0,true); assert(position.command>0); // 被压低，应抬头
    position.enable_position(6,-8); assert(position.target_roll_deg==-10);
    position.stop(); position.enable_position(8,-10);
    position.step(10,-12,0,true); assert(position.command<0); // 被抬高，应低头
    assert(position.shift_position(2) && position.target_roll_deg==-12);
    assert(position.shift_position(100) && position.target_roll_deg==PitchSpeedTune::RollMinDeg);
    assert(position.shift_position(-100) && position.target_roll_deg==PitchSpeedTune::RollMaxDeg);
    position.enable(12); assert(!position.position_mode && position.enabled);
    position.step(14,12,0,true); assert(position.command==0 && position.target_speed_rpm==0);
    assert(!position.update_remote_target(100,0.002f)); // 单速度模式不接遥控位置输入
    position.enable_position(16,-10);
    assert(position.update_remote_target(100,0.1f));
    assert(std::fabs(position.target_roll_deg-(-10-PitchSpeedTune::RemoteMaxSpeedDeg*0.1f))<1e-6f); // 按当前配置的满杆角速度积分
    assert(!position.update_remote_target(0,0.1f) && std::fabs(position.target_roll_deg-(-10-PitchSpeedTune::RemoteMaxSpeedDeg*0.1f))<1e-6f);
    assert(position.update_remote_target(-100,0.1f));
    assert(std::fabs(position.target_roll_deg-(-10))<1e-6f);
    position.update_remote_target(100,10); assert(position.target_roll_deg==PitchSpeedTune::RollMinDeg);
    position.update_remote_target(-100,10); assert(position.target_roll_deg==PitchSpeedTune::RollMaxDeg);
    position.stop(); assert(!position.update_remote_target(100,0.1f) && !position.enabled);
    // 只有C板：没有电机反馈，只靠有效IMU；不将机械限位套到手持板姿态。
    PitchSpeedTune dry;
    dry.pid.SpdParam.Kp1=35; dry.pid.SpdParam.Ki1=0; dry.pid.SpdParam.Kd1=0;
    dry.enable(0);
    dry.step(2,-90,-0.5f,true,false);
    assert(dry.enabled && dry.command<0);
    dry.stop(); dry.enable(4);
    dry.step(6,90,0.5f,true,false);
    assert(dry.command>0);
    dry.step(8,90,0,true,false); assert(dry.command==0);
    dry.stop(); dry.step(10,10,0.5f,true,false); assert(dry.command==0);
    PitchSpeedTune speed_tune;
    speed_tune.pid.SpdParam.Kp1=35;
    speed_tune.pid.SpdParam.Ki1=0;
    speed_tune.pid.SpdParam.Kd1=0;
    speed_tune.step(2,-10,-1,true);
    assert(speed_tune.command==0 && !speed_tune.enabled);
    speed_tune.enable(2);
    speed_tune.step(4,-10,-1,true); // 抬头角速度为正，应输出负电流阻碍
    assert(speed_tune.command<0);
    speed_tune.stop(); speed_tune.enable(6);
    speed_tune.step(8,10,1,true);
    assert(speed_tune.command>0);
    speed_tune.stop(); speed_tune.enable(10);
    speed_tune.step(12,-10,0,true);
    assert(speed_tune.command==0); // 换角度但静止，不产生角度回位力
    speed_tune.step(14,-20,0,true); assert(speed_tune.command==0);
    speed_tune.step(16,PitchSpeedTune::RollMinDeg,1,true); assert(speed_tune.command==0);
    speed_tune.step(18,PitchSpeedTune::RollMinDeg-1,1,true); assert(speed_tune.command==0 && speed_tune.enabled);
    speed_tune.enable(20); speed_tune.step(22,PitchSpeedTune::RollMaxDeg,-1,true); assert(speed_tune.command==0);
    speed_tune.stop();
    speed_tune.step(24,-10,1,true); assert(speed_tune.command==0);
    // 原始Roll范围-35～5，+1.19在范围内，慢速也应有输出。
    speed_tune.enable(26);
    speed_tune.step(28,1.19f,-0.19f*0.017453292519943295f,true);
    assert(speed_tune.command<0 && speed_tune.enabled);
    speed_tune.step(30,-10,-0.19f*0.017453292519943295f,true);
    assert(speed_tune.command<0 && speed_tune.enabled); // 慢速也应有阻尼
    speed_tune.step(32,1.19f,1,true);
    assert(speed_tune.command>0 && speed_tune.enabled); // 允许向范围内的制动力
    assert(parse_pitch_speed_gain("kp=25.5",gain)==1 && gain==25.5f);
    assert(parse_pitch_speed_gain("ki=0",gain)==2 && gain==0);
    assert(parse_pitch_speed_gain("kd=0.01\t",gain)==4 && gain==0.01f);
    assert(parse_pitch_speed_gain("kpp=0.45",gain)==8 && gain==0.45f);
    assert(parse_pitch_speed_gain("kpp=0",gain)==8 && gain==0);
    assert(parse_pitch_speed_gain("kpp=",gain)==0);
    assert(parse_pitch_speed_gain("kpp=nan",gain)==0);
    assert(parse_pitch_speed_gain("kpp=0.3x",gain)==0);
    for(const char *bad : {"kp=", "ki=nan", "kd=inf", "kp=1oops", "kp=1e999", "p:2"})
        assert(parse_pitch_speed_gain(bad,gain)==0);
    PitchImuHold tuned;
    tuned.pid.SpdParam.Kp1=25.5f;
    tuned.pid.SpdParam.Ki1=0;
    tuned.pid.SpdParam.Kd1=0.01f;
    tuned.stop();
    assert(tuned.pid.SpdParam.Kp1==25.5f && tuned.pid.SpdParam.Ki1==0 && tuned.pid.SpdParam.Kd1==0.01f);
    assert(tuned.pid.PosParam.Kp1==PitchImuHold::PositionKp);
    float delta=0;
    assert(parse_pitch_test_command("p:2.5",&delta)==PitchTestRequest::PitchDelta && delta==2.5f);
    assert(parse_pitch_test_command("p:-1.25",&delta)==PitchTestRequest::PitchDelta && delta==-1.25f);
    assert(parse_pitch_test_command("p:0",&delta)==PitchTestRequest::PitchDelta && delta==0);
    assert(parse_pitch_test_command("p: .5  ",&delta)==PitchTestRequest::PitchDelta && delta==0.5f);
    assert(parse_pitch_test_command("p:1e-2",&delta)==PitchTestRequest::PitchDelta);
    assert(parse_pitch_test_command("p:")==PitchTestRequest::None);
    assert(parse_pitch_test_command("p:nan")==PitchTestRequest::None);
    assert(parse_pitch_test_command("p:inf")==PitchTestRequest::None);
    assert(parse_pitch_test_command("p:1.0abc")==PitchTestRequest::None);
    assert(parse_pitch_test_command("p:1e999")==PitchTestRequest::None);
    assert(parse_pitch_test_command("pitch_hold")==PitchTestRequest::Hold);
    assert(parse_pitch_test_command("pitch_enable")==PitchTestRequest::Hold);
    assert(parse_pitch_test_command("pitch_disable")==PitchTestRequest::Stop);
    PitchHoldInput in{true,-1.2f,0,5200};
    RegressionHold remote;
    assert(!remote.update_remote_target(100,0.002f));
    assert(remote.enable(0,in));
    const float initial_remote=remote.target_rad;
    assert(remote.update_remote_target(100,0.002f));
    assert(std::fabs(remote.target_rad-(initial_remote+100*PitchImuHold::RemoteSensitivity*0.002f))<1e-6f);
    const float remote_moved=remote.target_rad;
    assert(!remote.update_remote_target(0,0.002f) && remote.target_rad==remote_moved);
    assert(remote.update_remote_target(-100,0.002f));
    assert(std::fabs(remote.target_rad-initial_remote)<1e-6f);
    remote.set_target(PitchImuHold::AngleMaxRad-0.001f);
    assert(remote.update_remote_target(100,0.002f) && remote.target_rad==PitchImuHold::AngleMaxRad);
    remote.set_target(PitchImuHold::AngleMinRad+0.001f);
    assert(remote.update_remote_target(-100,0.002f) && remote.target_rad==PitchImuHold::AngleMinRad);
    const float remote_limit=remote.target_rad;
    assert(!remote.update_remote_target(100,std::numeric_limits<float>::quiet_NaN()));
    assert(remote.target_rad==remote_limit);
    remote.stop();
    assert(!remote.update_remote_target(100,0.002f) && remote.state==PitchImuHold::Off);
    RegressionHold startup;
    auto not_ready=in; not_ready.sample_valid=false;
    assert(!startup.start_boot_target(0,not_ready) && startup.command==0);
    assert(startup.boot_target_pending);
    assert(startup.start_boot_target(10,in));
    assert(startup.state==PitchImuHold::Holding);
    assert(std::fabs(startup.target_rad-(-5.0f*0.017453292519943295f))<1e-6f);
    startup.step(12,in);
    assert(startup.command<0); // 从-68度向-5度抬头
    assert(startup.shift_target_degrees(2));
    const float startup_target=startup.target_rad;
    assert(!startup.start_boot_target(14,in) && startup.target_rad==startup_target);
    startup.stop();
    assert(!startup.start_boot_target(16,in) && startup.state==PitchImuHold::Off);
    RegressionHold canceled;
    canceled.stop();
    assert(!canceled.start_boot_target(10,in));
    RegressionHold hold;
    assert(!hold.shift_target_degrees(1));
    RegressionHold shifted;
    assert(shifted.enable(0,in));
    assert(shifted.shift_target_degrees(2.5f));
    assert(std::fabs(shifted.target_rad-(in.angle_rad+2.5f*0.017453292519943295f))<1e-6f);
    assert(shifted.shift_target_degrees(-1.25f));
    assert(std::fabs(shifted.target_rad-(in.angle_rad+1.25f*0.017453292519943295f))<1e-6f);
    const float saved=shifted.target_rad;
    assert(!shifted.shift_target_degrees(std::numeric_limits<float>::quiet_NaN()) && shifted.target_rad==saved);
    assert(!shifted.shift_target_degrees(std::numeric_limits<float>::max()) && shifted.target_rad==saved);
    static_assert(PitchImuHold::AngleMinDeg==-90 && PitchImuHold::AngleMaxDeg==80);
    RegressionHold wider;
    auto edge_inside=in; edge_inside.encoder=1000; edge_inside.angle_rad=-89.9f*0.017453292519943295f;
    assert(wider.enable(0,edge_inside));
    wider.stop(); edge_inside.encoder=7000; edge_inside.angle_rad=79.9f*0.017453292519943295f;
    assert(wider.enable(2,edge_inside));
    hold.step(2,in);
    assert(hold.state==PitchImuHold::Off && hold.command==0 && !hold.bus_owned);
    assert(hold.enable(10,in));
    assert(hold.target_rad==in.angle_rad && hold.command==0);
    hold.step(12,in);
    assert(hold.command==0);
    // 被外力压低2度：IMU角变小，电流必须为负（抬头纠正）。
    auto down=in; down.angle_rad-=0.034906585f;
    hold.step(14,down);
    assert(hold.command<0 && hold.command>=-2000);
    // 同一姿态下改变编码器，不改变PID反馈或闭环角度限位。
    RegressionHold a,b;
    assert(a.enable(0,in) && b.enable(0,in));
    auto enc_changed=down; enc_changed.encoder=7000;
    a.step(2,down); b.step(2,enc_changed);
    assert(a.command==b.command);
    RegressionHold up_hold;
    assert(up_hold.enable(0,in));
    auto up=in; up.angle_rad+=0.034906585f;
    up_hold.step(2,up);
    assert(up_hold.command>0); // 抬高后用正电流低头纠正
    hold.stop();
    assert(hold.state==PitchImuHold::Off && hold.command==0);
    assert(hold.pid.PosParam.PID_Err_all==0 && hold.pid.SpdParam.PID_lastout==0);
    // 实际角度到达边界或超出范围，闭环停输出。
    for(unsigned mode=0;mode<6;++mode)
    {
        RegressionHold s; assert(s.enable(0,in)); auto bad=in;
        switch(mode)
        {
        case 0: bad.sample_valid=false; break;
        case 1: bad.angle_rad=PitchImuHold::AngleMinRad; break;
        case 2: bad.angle_rad=PitchImuHold::AngleMaxRad; break;
        case 3: bad.angle_rad=std::numeric_limits<float>::quiet_NaN(); break;
        case 4: bad.angle_rad=PitchImuHold::AngleMinRad-0.01f; break;
        case 5: bad.angle_rad=PitchImuHold::AngleMaxRad+0.01f; break;
        }
        s.step(2,bad);
        assert(s.state==PitchImuHold::Fault && s.command==0);
        assert(!s.enable(4,in)); // 故障不自动恢复
    }
    RegressionHold paused; paused.enable(0,in); paused.step(20,in);
    assert(paused.state==PitchImuHold::Holding && paused.command==0);
    auto fast=in; fast.speed_rad_s=1.2f;
    paused.step(22,fast);
    assert(paused.state==PitchImuHold::Holding && paused.command>0);
    // 没有反馈年龄字段/超时停机；长时间无新样本仍按最后合法数据运算。
    paused.step(1000,in);
    assert(paused.state==PitchImuHold::Holding);
    // 用户要求删除10度偏差停机：大偏差继续闭环，电流限幅保持。
    RegressionHold large_error; assert(large_error.enable(0,in));
    auto displaced=in; displaced.angle_rad-=0.30f;
    large_error.step(2,displaced);
    assert(large_error.state==PitchImuHold::Holding && large_error.command<0);
    assert(large_error.command>=-PitchImuHold::CurrentLimit);
    RegressionHold trial;
    assert(trial.enable(0,in));
    for(unsigned now=2;now<20000;now+=2)
    {
        trial.step(now,in);
        assert(trial.state==PitchImuHold::Holding && trial.command==0);
        if(now==5000) assert(trial.enable(now,in)); // 重复启用不改变目标
    }
    assert(trial.state==PitchImuHold::Holding && trial.command==0);
    trial.stop();
    assert(trial.state==PitchImuHold::Off && trial.command==0);
    RegressionHold limit; limit.enable(0,in);
    for(unsigned now=2;now<1000;now+=2)
    {
        limit.step(now,down);
        assert(limit.command>=-PitchImuHold::CurrentLimit && limit.command<=PitchImuHold::CurrentLimit);
    }
}
