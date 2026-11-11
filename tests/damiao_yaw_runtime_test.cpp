#include "DamiaoYawRuntime.hpp"
#include "DamiaoMailboxOwners.hpp"
#include "CANio.hpp"
#include "debugc.hpp"
#include <cassert>
#include <cmath>
#include <cstring>

DebugStub debug;
damiao_mit_yaw::FeedbackSnapshot frame{};
damiao_mit::Command sent{};
unsigned enabled=0, disabled=0, control=0;
damiao_mit_yaw::FeedbackSnapshot damiao_mit_yaw_feedback_snapshot(){return frame;}
HAL_StatusTypeDef damiao_yaw_can_send_enable(){++enabled;return HAL_OK;}
HAL_StatusTypeDef damiao_yaw_can_send_disable(){++disabled;return HAL_OK;}
bool damiao_yaw_can_disable_tx_complete(){return true;}
HAL_StatusTypeDef damiao_yaw_can_send_control(const damiao_mit::Command &c){sent=c;++control;return HAL_OK;}
void feedback(uint32_t t,uint8_t status){
    frame.seen=true; frame.last_ms=t; ++frame.count;
    frame.data[0]=static_cast<uint8_t>((status<<4)|6);
    frame.data[1]=0x80; frame.data[2]=0;
    frame.data[3]=0x80; frame.data[4]=0x08; frame.data[5]=0;
}
int main(int argc,char **argv){
    using namespace damiao_yaw_runtime;
    if(argc==2 && std::strcmp(argv[1],"protect")==0){
        publish_imu(0,0,0,0,0,0); feedback(0,0); step(0,0,true);
        assert(disabled==1 && enabled==0 && snapshot().state==State::Stopped);
        feedback(2,0); step(2,0,true); assert(control==0);
        publish_imu(0,0,0,0,0,4); feedback(4,0); step(4,0,false);
        assert(enabled==1 && snapshot().state==State::Enabling);
        feedback(6,1); step(6,0,false); assert(snapshot().state==State::Running);
        feedback(8,1); step(8,0,false);
        const unsigned before_protect=control;
        feedback(10,1); step(10,0,true);
        assert(disabled==2 && control==before_protect && snapshot().state==State::Stopped);
        return 0;
    }
    DamiaoMailboxOwners owner{};
    owner.pitch_motion=1; owner.yaw_motion=2; owner.reuse(1); owner.yaw_motion|=1;
    assert(owner.pitch_motion==0 && owner.yaw_motion==3);
    owner.pitch_disable=4; owner.reuse(4); assert(owner.pitch_disable==0 && owner.yaw_motion==3);
    // 本用例验证从零增益在线调参；不依赖用户当前的上电调参值。
    debug.yaw_gain_pending=7;
    debug.yaw_speed_kp=debug.yaw_speed_ki=debug.yaw_speed_kd=0;
    step(0,0); assert(disabled==1 && enabled==0);
    // 等待有效IMU和失能反馈后自动使能。
    publish_imu(720,0,0,0,0,2); feedback(2,0); step(2,0);
    assert(enabled==1 && snapshot().state==State::Enabling);
    feedback(4,1); step(4,0);
    assert(snapshot().state==State::Running && std::fabs(snapshot().target-12.5663706f)<0.0001f);
    feedback(6,1); step(6,100);
    assert(control==1 && std::fabs(snapshot().target-12.575097f)<0.0001f);
    assert(sent.kp==0 && sent.kd==0 && sent.torque==0);
    // 在线Kp设置后同一控制循环生效；摇杆回中保持目标。
    debug.yaw_gain_pending=1; debug.yaw_speed_kp=2;
    feedback(8,1); step(8,0);
    assert(std::fabs(sent.torque-0.0174533f)<0.0001f);
    debug.yaw_stop_requested=true; feedback(10,1); step(10,100);
    assert(snapshot().state==State::Stopped);
    feedback(12,0); step(12,100); assert(enabled==1);
    debug.yaw_enable_requested=true; step(14,0); assert(enabled==2);
    feedback(16,1); step(16,0); assert(snapshot().state==State::Running);
    feedback(18,1); step(18,0,false,true,12.8f);
    assert(std::fabs(snapshot().target-12.8f)<0.0001f);
    invalidate_imu(); feedback(20,1); step(20,0);
    assert(snapshot().state==State::Stopped);
}
