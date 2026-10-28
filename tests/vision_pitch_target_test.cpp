#include "VisionPitchTarget.hpp"
#include "PitchImuHold.hpp"
#include "crc.h"
#include <cassert>
#include <cstring>
#include <limits>

int main()
{
    // 与PowerShell脚本的-1rad固定样本逐字节交叉验证CRC及小端布局。
    uint8_t bytes[31]={0x53,0x50,1};
    const float pitch=-1;
    std::memcpy(bytes+16,&pitch,4);
    Append_CRC16_Check_Sum(bytes,31);
    assert(bytes[29]==0xD5 && bytes[30]==0x1D);
    assert(Verify_CRC16_Check_Sum(bytes,31));
    auto packet=fromVector<VisionReceivePacket2026>(bytes);
    float target=0;
    assert(vision_pitch_target(packet,target) && target==-1);
    bytes[16]^=1;
    assert(!Verify_CRC16_Check_Sum(bytes,31));
    packet.mode=0; assert(!vision_pitch_target(packet,target));
    packet.mode=2; assert(!vision_pitch_target(packet,target));
    packet.mode=1; packet.pitch=std::numeric_limits<float>::quiet_NaN();
    assert(!vision_pitch_target(packet,target));
    packet.pitch=0.5f; packet.head[0]='X'; assert(!vision_pitch_target(packet,target));
    PitchImuHold hold;
    // 固定协议/控制回归夹具，避免依赖实机调参区是否将位置环置零。
    hold.pid.PosParam.Kp1=0.3f; hold.pid.PosParam.Ki1=0.005f; hold.pid.PosParam.Kd1=0;
    hold.pid.SpdParam.Kp1=20; hold.pid.SpdParam.Ki1=0.002f; hold.pid.SpdParam.Kd1=0;
    assert(!hold.set_target(0.2f)); // 关闭时视觉数据不能直接产生输出
    PitchHoldInput in{true,-1,0,5000};
    assert(hold.enable(0,in));
    assert(hold.set_target(-0.965093415f)); // 抬头2度
    hold.step(2,in);
    assert(hold.command<0); // 负电流抬头
    const float old=hold.target_rad;
    assert(!hold.set_target(std::numeric_limits<float>::infinity()) && hold.target_rad==old);
    hold.stop(); assert(!hold.set_target(0));
}
