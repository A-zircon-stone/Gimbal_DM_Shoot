#include "YawSpeedTune.hpp"
#include "YawPidCommand.hpp"
#include <cassert>
#include <limits>
#include <initializer_list>

int main()
{
    float gain=0;
    assert(parse_yaw_gain("ykp=35",gain)==1 && gain==35);
    assert(parse_yaw_gain("yki=0.02",gain)==2);
    assert(parse_yaw_gain("ykd=2",gain)==4);
    assert(parse_yaw_gain("ykpp=0.4",gain)==8);
    assert(!parse_yaw_gain("kp=35",gain));
    assert(!parse_yaw_gain("ykp=nan",gain));
    assert(!parse_yaw_gain("ykp=35bad",gain));
    assert(!parse_yaw_gain("ykp=",gain));
    assert(parse_yaw_delta("y:-2.5",gain) && gain==-2.5f);
    assert(!parse_yaw_delta("y:inf",gain));
    assert(!parse_yaw_delta("p:2",gain));
    // 对绕世界竖直轴转动的刚体，传感器Y/Z投影应还原Yaw速度。
    for(float tilt : {-35.0f,0.0f,5.0f})
        for(float bank : {-20.0f,0.0f,20.0f})
        {
            const float x=bank*0.01745329252f,y=tilt*0.01745329252f;
            const float gy=std::sin(x)*std::cos(y)*0.8f;
            const float gz=std::cos(x)*std::cos(y)*0.8f;
            assert(std::fabs(YawSpeedTune::yaw_rate_rad(tilt,bank,gy,gz)-0.8f)<1e-5f);
        }
    YawSpeedTune yaw;
    yaw.step(2,0,1,true); assert(!yaw.enabled && yaw.command==0);
    // ykp直接开启零速度环：向左扰动给负电流，向右扰动给正电流。
    yaw.apply_gains(1,35,0,0,0,2,0);
    assert(yaw.enabled && !yaw.position_mode);
    yaw.step(4,0,1,true); assert(yaw.command<0);
    yaw.step(6,0,-1,true); assert(yaw.command>0);
    yaw.step(8,0,0,true); assert(yaw.command==0);
    yaw.apply_gains(6,0,0.01f,0.1f,0,8,0);
    assert(yaw.pid.SpdParam.Kp1==35 && yaw.pid.SpdParam.Ki1==0.01f);
    // ykpp捕获当前连续角度，不突然回零；重复改位置Kp不重置目标。
    yaw.apply_gains(8,0,0,0,1.6f,10,179);
    assert(yaw.position_mode && yaw.target_yaw_deg==179);
    yaw.step(12,179,0,true); assert(yaw.command==0);
    assert(yaw.shift_position(4)); // 穿越180度，目标183而非-177
    yaw.apply_gains(8,0,0,0,1,12,180);
    assert(yaw.target_yaw_deg==183);
    yaw.step(14,181,0,true); assert(yaw.target_speed_rpm>0 && yaw.command>0);
    for(unsigned t=16;t<32;t+=2) yaw.step(t,185,0,true);
    assert(yaw.target_speed_rpm<0 && yaw.command<0);
    const float before=yaw.target_yaw_deg;
    yaw.update_remote_target(100,0.1f);
    assert(std::fabs(yaw.target_yaw_deg-before-YawSpeedTune::RemoteMaxSpeedDeg*0.1f)<1e-5f);
    yaw.update_remote_target(0,0.1f);
    assert(std::fabs(yaw.target_yaw_deg-before-YawSpeedTune::RemoteMaxSpeedDeg*0.1f)<1e-5f);
    yaw.apply_gains(1,10000,0,0,0,32,0); // ykp从位置模式切回速度环
    assert(!yaw.position_mode && yaw.target_speed_rpm==0);
    for(unsigned t=34;t<100;t+=2) yaw.step(t,0,100,true);
    assert(yaw.command==-YawSpeedTune::CurrentLimit);
    yaw.stop(); yaw.step(102,0,1,true); assert(!yaw.enabled && yaw.command==0);
    yaw.apply_gains(1,35,0,0,0,104,0);
    yaw.step(106,0,0,false); assert(!yaw.enabled && yaw.command==0);
    yaw.apply_gains(1,35,0,0,0,108,0);
    yaw.step(110,0,std::numeric_limits<float>::quiet_NaN(),true);
    assert(!yaw.enabled && yaw.command==0);
}
