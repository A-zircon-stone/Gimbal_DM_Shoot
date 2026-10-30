#pragma once

#include <cstddef>
#include <cstdint>

// 当前CAN1：Yaw ID1反馈0x205，Pitch ID3反馈0x207。
// 此标志禁用旧控制链路；人工触发的Pitch方向测试/IMU首测为独立输出例外。
// 不得仅关闭此标志就运行尚未适配的旧闭环。
namespace GimbalHardware
{
constexpr bool FeedbackOnly = true;
constexpr bool SerialTestOnly = false; // 串口测试结束，恢复速度环调试和周期打印
// 新云台测量阶段：上电零输出，PID/回位/遥控/视觉不驱动电机。
constexpr bool CommissioningMode = false;
constexpr bool PitchSpeedTuneMode = true; // 两轴上电捕获当前位置进入遥控位置环，仍支持串口切换速度环
constexpr bool PitchSpeedDryRun = true; // 达妙迁移期间禁止旧6020电流帧输出
// 独立、人工触发的短脉冲测试例外；不解锁旧控制链路。
constexpr bool PitchDirectionTestEnabled = false;
constexpr uint8_t YawId = 1;
constexpr uint8_t PitchId = 3;
constexpr uint32_t FeedbackTimeoutMs = 100;
}

// 当前0x1FE电流报文：ID1..4各占两字节，未使用通道写零。
inline void gm6020_encode_gimbal_current(int16_t yaw,int16_t pitch,uint8_t bytes[8])
{
    static_assert(GimbalHardware::YawId>=1 && GimbalHardware::YawId<=4);
    static_assert(GimbalHardware::PitchId>=1 && GimbalHardware::PitchId<=4);
    static_assert(GimbalHardware::YawId!=GimbalHardware::PitchId);
    for(unsigned i=0;i<8;++i) bytes[i]=0;
    const unsigned yaw_index=2*(GimbalHardware::YawId-1);
    const unsigned pitch_index=2*(GimbalHardware::PitchId-1);
    bytes[yaw_index]=static_cast<uint16_t>(yaw)>>8;
    bytes[yaw_index+1]=static_cast<uint16_t>(yaw)&0xff;
    bytes[pitch_index]=static_cast<uint16_t>(pitch)>>8;
    bytes[pitch_index+1]=static_cast<uint16_t>(pitch)&0xff;
}

struct Gm6020Feedback
{
    uint16_t encoder = 0;       // 0..8191，单圈原始位置，不是机械零点
    int16_t speed_rpm = 0;
    int16_t current_raw = 0;    // 原始转矩电流，不冒充 Nm
    uint8_t temperature_c = 0;
};

inline int16_t gm6020_signed_word(uint8_t high, uint8_t low)
{
    const int32_t word = (static_cast<uint16_t>(high) << 8) | low;
    return static_cast<int16_t>(word < 32768 ? word : word - 65536);
}

// 失败时不修改上一次有效数据。CAN帧类型与总线由调用方校验。
inline bool gm6020_decode(uint32_t std_id, uint8_t motor_id,
                          const uint8_t *bytes, std::size_t length,
                          Gm6020Feedback &out)
{
    if (!bytes || length != 8 || motor_id < 1 || motor_id > 7 ||
        std_id != 0x204u + motor_id)
        return false;

    Gm6020Feedback next{};
    next.encoder = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
    if (next.encoder > 8191)
        return false;
    next.speed_rpm = gm6020_signed_word(bytes[2], bytes[3]);
    next.current_raw = gm6020_signed_word(bytes[4], bytes[5]);
    next.temperature_c = bytes[6];
    out = next;
    return true;
}

inline bool gm6020_feedback_fresh(bool seen, uint32_t now_ms, uint32_t last_ms)
{
    return seen && static_cast<uint32_t>(now_ms - last_ms) <=
                       GimbalHardware::FeedbackTimeoutMs;
}
