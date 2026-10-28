#pragma once
#include "packet.hpp"
#include "crc.h"
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdlib>

namespace VisionLinkConfig
{
    constexpr bool Enabled=true; // USB专用视觉二进制通信；USART1保留VOFA
    constexpr uint32_t TxPeriodMs=5;
    constexpr uint32_t RxTimeoutMs=10000; // 首次联调暂定，按有效下行帧间隔实测调整
    constexpr uint8_t EnemyColor=0; // 未知；vcolor=1红/2蓝，填写敌方颜色
    constexpr float BulletSpeedMps=0; // 未接弹速测量，0表示未知，不能伪装为实测弹速
    // 加速度到协议电流值的模型系数尚未标定，默认不施加加速度电流前馈。
    constexpr float YawAccelCurrentPerRadS2=0;
    constexpr float PitchAccelCurrentPerRadS2=0;
    constexpr float RadToDeg=57.295779513f;
    constexpr float RadPerSecToRpm=9.549296586f;
}
static_assert(sizeof(float)==4 && sizeof(VisionSendPacket2026)==45 && sizeof(VisionReceivePacket2026)==31,
              "SP protocol requires 45/31 bytes with IEEE754 float32");
static_assert(offsetof(VisionSendPacket2026,q)==4 && offsetof(VisionSendPacket2026,yaw)==20 &&
              offsetof(VisionSendPacket2026,pitch)==28 && offsetof(VisionSendPacket2026,crc16)==43,
              "SP uplink field offsets");
static_assert(offsetof(VisionReceivePacket2026,yaw)==4 && offsetof(VisionReceivePacket2026,pitch)==16 &&
              offsetof(VisionReceivePacket2026,shoot_count)==28 && offsetof(VisionReceivePacket2026,crc16)==29,
              "SP downlink field offsets");

inline bool vision_command_valid(const VisionReceivePacket2026 &p)
{
    if(p.head[0]!='S' || p.head[1]!='P' || p.mode>1 || p.fire>2) return false;
    if(p.mode==0) return true; // 撤销控制时角度/速度/计数全部忽略
    return std::isfinite(p.yaw) && std::isfinite(p.yaw_vel) && std::isfinite(p.yaw_acc) &&
           std::isfinite(p.pitch) && std::isfinite(p.pitch_vel) && std::isfinite(p.pitch_acc) &&
           std::isfinite(p.yaw*VisionLinkConfig::RadToDeg) &&
           std::isfinite(p.pitch*VisionLinkConfig::RadToDeg) &&
           std::isfinite(p.yaw_vel*VisionLinkConfig::RadPerSecToRpm) &&
           std::isfinite(p.pitch_vel*VisionLinkConfig::RadPerSecToRpm);
}

// 每字节缓存；半包跨回调保留，粘包逐帧处理，CRC错误后滑动一字节重新找SP。
struct VisionStreamDecoder
{
    uint8_t bytes[31]{};
    unsigned used=0;
    uint32_t byte_count=0,valid_count=0,rejected_count=0;
    void discard_one()
    { --used; if(used) std::memmove(bytes,bytes+1,used); }
    void align()
    {
        while(used && (bytes[0]!='S' || (used>=2 && bytes[1]!='P'))) discard_one();
    }
    bool push(uint8_t byte,VisionReceivePacket2026 &out)
    {
        ++byte_count; bytes[used++]=byte; align();
        if(used<sizeof(bytes)) return false;
        VisionReceivePacket2026 candidate;
        std::memcpy(&candidate,bytes,sizeof(candidate));
        if(Verify_CRC16_Check_Sum(bytes,sizeof(bytes)) && vision_command_valid(candidate))
        { out=candidate; used=0; ++valid_count; return true; }
        ++rejected_count; discard_one(); align(); return false;
    }
};

struct VisionAttitudeSample
{
    float q[4]{}; // 机体系到惯性系，wxyz
    float yaw_deg=0; // 与q同源的连续Yaw
    float pitch_deg=0; // 原始INS.Roll，抬头为负
    float yaw_rate=0;
    float pitch_rate=0;
};
inline bool build_vision_feedback(const VisionAttitudeSample &s,uint8_t mode,uint8_t enemy,
                                  float bullet_speed,uint16_t bullet_count,uint8_t shoot_count,
                                  uint8_t out[45])
{
    float norm2=0;
    for(float x:s.q) { if(!std::isfinite(x)) return false; norm2+=x*x; }
    if(!std::isfinite(norm2) || norm2<=0 || !std::isfinite(s.yaw_deg) ||
       !std::isfinite(s.pitch_deg) || !std::isfinite(s.yaw_rate) ||
       !std::isfinite(s.pitch_rate) || !std::isfinite(bullet_speed)) return false;
    VisionSendPacket2026 p{};
    p.mode=mode<=3 ? mode : 0;
    p.enemy_color=enemy<=2 ? enemy : 0;
    const float inv=1.0f/std::sqrt(norm2);
    for(unsigned i=0;i<4;++i) p.q[i]=s.q[i]*inv;
    p.yaw=s.yaw_deg*0.017453292519943295f;
    p.pitch=s.pitch_deg*0.017453292519943295f;
    p.yaw_vel=s.yaw_rate; p.pitch_vel=s.pitch_rate;
    p.bullet_speed=bullet_speed; p.bullet_count=bullet_count;
    p.already_shoot_count=shoot_count;
    std::memcpy(out,&p,sizeof(p));
    Append_CRC16_Check_Sum(out,sizeof(p));
    return true;
}

struct VisionControl
{
    enum class Action { None, Target, Hold };
    bool enabled=true,active=false; // 上电默认允许视觉；实际输出仍受遥控在线保护约束
    bool remote_online=false;
    uint8_t requested_mode=1;
    uint8_t enemy_color=VisionLinkConfig::EnemyColor;
    uint32_t last_sequence=0;
    VisionReceivePacket2026 target{};
    const char *reason="RC_OFFLINE";
    uint8_t uplink_mode() const { return enabled && remote_online ? requested_mode : 0; }
    void set_remote_online(bool online,uint32_t sequence)
    {
        if(!online || online!=remote_online)
        {
            active=false; last_sequence=sequence;
            reason=online ? (enabled ? "WAIT_TARGET" : "MANUAL") : "RC_OFFLINE";
        }
        remote_online=online;
    }
    void enable(uint32_t sequence)
    {
        enabled=true; active=false; last_sequence=sequence;
        if(requested_mode==0) requested_mode=1;
        reason="WAIT_TARGET"; // 丢弃使能前的包，必须收到新帧
    }
    void disable(const char *why="MANUAL") { enabled=false; active=false; reason=why; }
    void suspend(uint32_t sequence,const char *why)
    { active=false; last_sequence=sequence; reason=why; } // 保留视觉许可，等待轴就绪和新帧
    void set_mode(uint8_t mode,uint32_t sequence)
    {
        mode=mode<=3 ? mode : 0;
        if(mode==requested_mode) return;
        requested_mode=mode; active=false; last_sequence=sequence;
        reason=mode ? "WAIT_TARGET" : "IDLE";
    }
    Action update(uint32_t now,uint32_t sequence,uint32_t rx_ms,const VisionReceivePacket2026 &p)
    {
        if(!remote_online || !enabled || requested_mode==0) return Action::None;
        const bool was_active=active;
        if(static_cast<uint32_t>(now-rx_ms)>VisionLinkConfig::RxTimeoutMs)
        { active=false; reason="RX_TIMEOUT"; return was_active ? Action::Hold : Action::None; }
        if(sequence==last_sequence) return Action::None;
        last_sequence=sequence;
        if(!vision_command_valid(p)) return Action::None;
        if(p.mode==0)
        { active=false; reason="NO_TARGET"; return was_active ? Action::Hold : Action::None; }
        target=p; active=true; reason="TRACKING"; return Action::Target;
    }
};

struct VisionAxisTargets
{
    float yaw_deg=0,pitch_deg=0,yaw_speed_rpm=0,pitch_speed_rpm=0;
    float yaw_current_ff=0,pitch_current_ff=0;
};
inline bool map_vision_targets(const VisionReceivePacket2026 &p,float pitch_min,float pitch_max,
                              VisionAxisTargets &out)
{
    if(p.mode!=1 || !vision_command_valid(p)) return false;
    out=VisionAxisTargets{};
    out.yaw_deg=p.yaw*VisionLinkConfig::RadToDeg;
    const float raw_pitch=p.pitch*VisionLinkConfig::RadToDeg;
    out.pitch_deg=std::fmax(pitch_min,std::fmin(pitch_max,raw_pitch));
    out.yaw_speed_rpm=p.yaw_vel*VisionLinkConfig::RadPerSecToRpm;
    out.yaw_current_ff=p.yaw_acc*VisionLinkConfig::YawAccelCurrentPerRadS2;
    // 协议Pitch抬头为负，控制器角速度和电流抬头为正；越界目标只钳角度、不带前馈。
    if(raw_pitch>=pitch_min && raw_pitch<=pitch_max)
    {
        out.pitch_speed_rpm=-p.pitch_vel*VisionLinkConfig::RadPerSecToRpm;
        out.pitch_current_ff=-p.pitch_acc*VisionLinkConfig::PitchAccelCurrentPerRadS2;
    }
    return true;
}

inline bool parse_vision_byte_command(const char *line,const char *prefix,uint8_t &value)
{
    const auto n=std::strlen(prefix);
    if(std::strncmp(line,prefix,n)) return false;
    char *end=nullptr;
    const unsigned long parsed=std::strtoul(line+n,&end,10);
    if(end==line+n || *end || parsed>255) return false;
    value=static_cast<uint8_t>(parsed); return true;
}
