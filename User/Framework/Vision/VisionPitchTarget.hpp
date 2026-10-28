#pragma once
#include "packet.hpp"
#include <cmath>

// 复用原31字节协议布局。新Pitch接口：绝对IMU角度rad、抬头正，非水平零点。
// mode=0停车；mode=1只更新Pitch。Yaw/开火不接入当前双轴控制。
inline bool vision_pitch_target(const VisionReceivePacket2026 &packet,float &out)
{
    static_assert(sizeof(VisionReceivePacket2026)==31,"视觉包布局必须保持31字节");
    if(packet.head[0]!='S' || packet.head[1]!='P' || packet.mode!=1 ||
       !std::isfinite(packet.pitch)) return false;
    out=packet.pitch;
    return true;
}
