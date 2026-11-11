#pragma once

#include <cstddef>
#include <cstdint>

// 达妙 V41 一控四协议：周期反馈与四路电流报文的数据转换。
// 根据达妙《固件更新日志-3.1》V41 一控四部分；仅做数据处理，不发送 CAN。
namespace damiao_v41 {

constexpr uint8_t PitchMotorId = 5; // pitch 实际读到的 CAN ID：0x05
constexpr uint8_t YawMotorId = 0;   // 0 表示尚未配置，读到实际 ID 后再填写 1..8

struct Feedback {
    uint16_t position_raw = 0; // 0..8191 对应电机反馈的一圈位置
    int16_t speed_raw = 0;     // 有符号值；实际速度为 speed_raw / 100 rpm
    int16_t current_ma = 0;    // 有符号扭矩电流反馈，单位 mA
    uint8_t coil_temperature_c = 0;
    uint8_t error_status = 0;  // 保留原始状态值，具体含义按对应固件手册确认
};

// 高字节在前：FF F6 拼成 0xFFF6（65526），按 16 位补码得到 -10。
inline int16_t signed_big_endian(uint8_t high, uint8_t low) {
    const uint32_t word = (static_cast<uint32_t>(high) << 8) | low;
    return static_cast<int16_t>(word < 0x8000u ? static_cast<int32_t>(word)
                                               : static_cast<int32_t>(word) - 65536);
}

// 根据帧头识别电机；例如 ID 5 的反馈 ID = 0x300 + 5 = 0x305。
// 不能从 D[0] 的低 4 位取电机 ID：V41 的 D[0] 是位置高字节。
inline bool decode_feedback(uint8_t motor_id, uint32_t frame_id, const uint8_t *data,
                            std::size_t length, Feedback &output) {
    if (motor_id < 1 || motor_id > 8 || frame_id != 0x300u + motor_id ||
        !data || length != 8) return false;
    Feedback next{};
    next.position_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (next.position_raw > 8191) return false;
    next.speed_raw = signed_big_endian(data[2], data[3]);
    next.current_ma = signed_big_endian(data[4], data[5]);
    next.coil_temperature_c = data[6];
    next.error_status = data[7];
    output = next; // 校验全部通过后才更新，失败时保留上次有效数据
    return true;
}

inline float speed_rpm(const Feedback &feedback) {
    return static_cast<float>(feedback.speed_raw) / 100.0f;
}

struct CurrentFrame {
    uint16_t standard_id = 0;
    uint8_t data[8]{};
};

// 构造四台电机共用的电流帧：group 1 对应 ID 1..4，group 2 对应 ID 5..8。
// 每个 16 位值是调用者已经按 V41 协议编码的原始电流标幺值，不是 mA 或 Nm。
// 例：ID5 的原始值为 0x1234 时，0x4FE 的 D[0]=0x34，D[1]=0x12（低字节在前）。
// 四个通道都必须显式提供：零也会作用于对应通道，不可假设其他电机不存在。
// 此函数不决定电流范围/方向，不做硬件发送；实机前需核对固件和限制。
inline bool make_current_frame(uint8_t group, const uint16_t (&encoded_currents)[4],
                               CurrentFrame &output) {
    if (group != 1 && group != 2) return false;
    CurrentFrame next{};
    next.standard_id = group == 1 ? 0x3fe : 0x4fe;
    for (unsigned i = 0; i < 4; ++i) {
        next.data[2 * i] = static_cast<uint8_t>(encoded_currents[i] & 0xffu);
        next.data[2 * i + 1] = static_cast<uint8_t>(encoded_currents[i] >> 8);
    }
    output = next;
    return true;
}
}
