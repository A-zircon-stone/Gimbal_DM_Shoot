#include "VisionLink.hpp"
#include "PitchSpeedTune.hpp"
#include "YawSpeedTune.hpp"
#include "RemoteControlProtection.hpp"
#include <cassert>
#include <vector>
#include <limits>
#include <initializer_list>

using Packet=VisionReceivePacket2026;
static std::vector<uint8_t> encode(Packet p)
{
    std::vector<uint8_t> b(sizeof(p)); std::memcpy(b.data(),&p,sizeof(p));
    Append_CRC16_Check_Sum(b.data(),b.size()); return b;
}
static unsigned feed(VisionStreamDecoder &d,const std::vector<uint8_t> &bytes,Packet &out)
{
    unsigned count=0; for(auto c:bytes) if(d.push(c,out)) ++count; return count;
}
template<class Axis> static void fixture(Axis &axis)
{
    axis.pid.PosParam.Kp1=1; axis.pid.PosParam.Ki1=0; axis.pid.PosParam.Kd1=0;
    axis.pid.SpdParam.Kp1=100; axis.pid.SpdParam.Ki1=0; axis.pid.SpdParam.Kd1=0;
    axis.pid.PosParam.PID_OutStep=100000; axis.pid.SpdParam.PID_OutStep=100000;
}
int main()
{
    assert(Get_CRC16_Check_Sum(reinterpret_cast<const uint8_t*>("123456789"),9,0xffff)==0x6f91);
    Packet idle; auto idle_bytes=encode(idle);
    assert(idle_bytes[29]==0x4b && idle_bytes[30]==0xac);
    Packet p; p.mode=1;p.yaw=7;p.pitch=-0.2f;p.yaw_vel=0.3f;p.pitch_vel=-0.2f;p.yaw_acc=1;p.pitch_acc=-2;
    auto bytes=encode(p);
    // 每一种31字节的切包位置，以及两帧粘包、垃圾、CRC错误和奇数字节错位。
    for(unsigned split=0;split<=31;++split)
    {
        VisionStreamDecoder d;Packet out;
        unsigned n=feed(d,{bytes.begin(),bytes.begin()+split},out);
        n+=feed(d,{bytes.begin()+split,bytes.end()},out);
        assert(n==1 && out.yaw==7 && out.pitch==-0.2f);
    }
    VisionStreamDecoder d;Packet out;
    auto stream=std::vector<uint8_t>{0,'S','S',0x99};
    auto corrupt=bytes;corrupt[12]^=0x20;
    stream.insert(stream.end(),corrupt.begin(),corrupt.end());stream.push_back(0x42);
    stream.insert(stream.end(),bytes.begin(),bytes.end());stream.insert(stream.end(),idle_bytes.begin(),idle_bytes.end());
    assert(feed(d,stream,out)==2 && out.mode==0 && d.valid_count==2 && d.rejected_count>=1);
    for(float bad:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()})
    {
        Packet invalid=p;invalid.yaw=bad; assert(!vision_command_valid(invalid));
        VisionStreamDecoder decoder;assert(feed(decoder,encode(invalid),out)==0);
        assert(feed(decoder,bytes,out)==1);
    }
    Packet invalid=p;invalid.mode=2;assert(!vision_command_valid(invalid));
    invalid=p;invalid.fire=3;assert(!vision_command_valid(invalid));
    Packet no_target;no_target.pitch=std::numeric_limits<float>::quiet_NaN();
    assert(vision_command_valid(no_target)); // mode0忽略目标，不能把无效/零角度当作回零

    // 使用真实CRC生成协议文档的45字节上行样例。
    VisionAttitudeSample attitude;attitude.q[0]=1;
    uint8_t up[45];assert(build_vision_feedback(attitude,1,2,23,0,0,up));
    assert(up[43]==0xc4 && up[44]==0x9a && Verify_CRC16_Check_Sum(up,45));
    VisionSendPacket2026 feedback;std::memcpy(&feedback,up,45);
    assert(feedback.head[0]=='S' && feedback.head[1]=='P' && feedback.mode==1 && feedback.q[0]==1);
    attitude.q[0]=2;attitude.yaw_deg=450;attitude.pitch_deg=-20;
    attitude.yaw_rate=0.2f;attitude.pitch_rate=-0.3f;
    assert(build_vision_feedback(attitude,3,1,0,0,0,up));std::memcpy(&feedback,up,45);
    assert(feedback.q[0]==1 && std::fabs(feedback.yaw-7.8539816f)<1e-5f);
    assert(feedback.pitch<0 && feedback.pitch_vel==-0.3f && feedback.yaw_vel==0.2f);
    attitude.q[0]=0;assert(!build_vision_feedback(attitude,1,1,0,0,0,up));

    VisionControl control;
    assert(control.uplink_mode()==0);
    assert(control.update(10,1,10,p)==VisionControl::Action::None && !control.active);
    control.set_remote_online(true,10); control.enable(10);assert(control.uplink_mode()==1);
    assert(control.update(20,10,20,p)==VisionControl::Action::None); // 使能前旧帧不采用
    assert(control.update(22,11,22,p)==VisionControl::Action::Target && control.active);
    assert(control.update(222,11,22,p)==VisionControl::Action::None && control.active);
    assert(control.update(223,11,22,p)==VisionControl::Action::Hold && !control.active);
    assert(control.update(224,11,22,p)==VisionControl::Action::None);
    assert(control.update(225,12,225,p)==VisionControl::Action::Target);
    assert(control.update(226,13,226,no_target)==VisionControl::Action::Hold && !control.active);
    assert(control.update(228,14,228,p)==VisionControl::Action::Target);
    control.disable();assert(!control.active && control.uplink_mode()==0);
    assert(control.update(230,15,230,p)==VisionControl::Action::None);
    // 模式切换建立新的接收基线，非法上行模式回退空闲。
    for(uint8_t from=0;from<4;++from) for(uint8_t to=0;to<4;++to)
    {
        VisionControl c;c.set_remote_online(true,0);c.enable(0);c.set_mode(from,1);c.set_mode(to,2);
        assert(c.uplink_mode()==to);
        if(from!=to) assert(c.update(10,2,10,p)==VisionControl::Action::None);
        if(to) assert(c.update(12,3,12,p)==VisionControl::Action::Target);
        else assert(c.update(12,3,12,p)==VisionControl::Action::None);
    }
    control.enable(15);control.set_mode(255,15);assert(control.uplink_mode()==0 && !control.active);
    // 计时器回绕仍以无符号时间差判断200ms超时。
    VisionControl wrap;wrap.set_remote_online(true,0);wrap.enable(0);
    assert(wrap.update(0xfffffff0u,1,0xfffffff0u,p)==VisionControl::Action::Target);
    assert(wrap.update(0x20u,1,0xfffffff0u,p)==VisionControl::Action::None && wrap.active);
    assert(wrap.update(0x100u,1,0xfffffff0u,p)==VisionControl::Action::Hold);

    VisionAxisTargets mapped;
    assert(map_vision_targets(p,-35,5,mapped));
    assert(mapped.yaw_deg>360 && mapped.pitch_deg<0 && mapped.yaw_speed_rpm>0 && mapped.pitch_speed_rpm>0);
    assert(mapped.yaw_current_ff==0 && mapped.pitch_current_ff==0); // 未标定加速度电流模型默认0
    p.pitch=-2;assert(map_vision_targets(p,-35,5,mapped) && mapped.pitch_deg==-35 && mapped.pitch_speed_rpm==0);
    p.pitch=2;assert(map_vision_targets(p,-35,5,mapped) && mapped.pitch_deg==5 && mapped.pitch_speed_rpm==0);
    assert(!map_vision_targets(idle,-35,5,mapped));
    // 在零位置误差时，速度前馈依旧产生相应电流；撤销前馈后输出归零。
    PitchSpeedTune pitch;fixture(pitch);pitch.enable_position(0,-10);
    pitch.step(2,-10,0,true,true,1,0);assert(pitch.command==100);
    pitch.step(4,-10,0,true,true,0,0);assert(pitch.command==0);
    pitch.target_roll_deg=PitchSpeedTune::RollMinDeg;
    pitch.step(6,PitchSpeedTune::RollMinDeg,0,true,true,100,0);assert(pitch.command==0);
    YawSpeedTune yaw;fixture(yaw);yaw.enable_position(0,450);
    yaw.step(2,450,0,true,1,0);assert(yaw.command==100);
    yaw.step(4,450,0,true,10000,0);assert(yaw.target_speed_rpm==yaw.pid.PosParam.PID_OutMax);
    assert(yaw.command<=YawSpeedTune::CurrentLimit);
    yaw.step(6,450,0,true,0,0);assert(yaw.command==0);
    yaw.step(8,450,0,true,0,100000);assert(yaw.command==YawSpeedTune::CurrentLimit);
    yaw.step(10,450,0,true,std::numeric_limits<float>::quiet_NaN());assert(!yaw.enabled && yaw.command==0);
    // 上电视觉许可默认打开，但遥控未连接，帧不能产生视觉控制权。
    VisionControl boot;
    assert(boot.enabled && !boot.remote_online && boot.uplink_mode()==0);
    assert(boot.update(10,1,10,p)==VisionControl::Action::None && !boot.active);
    boot.set_remote_online(true,1);
    assert(boot.enabled && boot.uplink_mode()==1);
    boot.suspend(2,"AXIS_NOT_READY"); // IMU/电机还未就绪不能取消默认许可
    assert(boot.enabled && !boot.active);
    assert(boot.update(12,2,12,p)==VisionControl::Action::None);
    assert(boot.update(14,3,14,p)==VisionControl::Action::Target);
    boot.set_remote_online(false,4);
    assert(boot.enabled && !boot.active && boot.uplink_mode()==0);
    assert(boot.update(16,5,16,p)==VisionControl::Action::None);
    boot.set_remote_online(true,5);
    assert(boot.update(18,5,18,p)==VisionControl::Action::None); // 断连期间旧包丢弃
    assert(boot.update(20,6,20,p)==VisionControl::Action::Target);
    boot.disable();boot.set_remote_online(false,7);boot.set_remote_online(true,8);
    assert(!boot.enabled && boot.update(22,9,22,p)==VisionControl::Action::None); // stop许可不被重连重启
    // 保护执行复用同一个生产代码门控：两轴PID清零、输出归零，重连只请求捕获新位置。
    PitchSpeedTune protected_pitch; YawSpeedTune protected_yaw;
    fixture(protected_pitch); fixture(protected_yaw);
    bool pitch_boot=true,yaw_boot=true;
    RemoteControlProtection gate;
    assert(!gate.step(false,protected_pitch,protected_yaw,pitch_boot,yaw_boot));
    assert(gate.protected_now && !protected_pitch.enabled && !protected_yaw.enabled);
    assert(gate.step(true,protected_pitch,protected_yaw,pitch_boot,yaw_boot));
    protected_pitch.enable_position(0,-10); protected_yaw.enable_position(0,450);
    pitch_boot=yaw_boot=false;
    protected_pitch.step(2,-10,0,true,true,1,0); protected_yaw.step(2,450,0,true,1,0);
    assert(protected_pitch.command!=0 && protected_yaw.command!=0);
    assert(!gate.step(false,protected_pitch,protected_yaw,pitch_boot,yaw_boot));
    assert(protected_pitch.command==0 && protected_yaw.command==0 && !protected_pitch.enabled && !protected_yaw.enabled);
    assert(protected_pitch.pid.SpdParam.PID_Err_all==0 && protected_yaw.pid.SpdParam.PID_Err_all==0);
    assert(!gate.step(false,protected_pitch,protected_yaw,pitch_boot,yaw_boot));
    assert(gate.step(true,protected_pitch,protected_yaw,pitch_boot,yaw_boot) && pitch_boot && yaw_boot);
    // 调参不退出视觉/不切回单速度环；手动模式保留旧的显式调试行为。
    protected_yaw.enable_position(4,450);
    protected_yaw.apply_gains(1,35,0,0,0,6,451,false);
    assert(protected_yaw.position_mode && protected_yaw.target_yaw_deg==450 && protected_yaw.pid.SpdParam.Kp1==35);
    protected_yaw.apply_gains(1,35,0,0,0,8,451,true);
    assert(!protected_yaw.position_mode);
    uint8_t value=0;
    assert(parse_vision_byte_command("vmode=3","vmode=",value) && value==3);
    assert(!parse_vision_byte_command("vmode=3bad","vmode=",value));
    assert(!parse_vision_byte_command("vmode=256","vmode=",value));
}
