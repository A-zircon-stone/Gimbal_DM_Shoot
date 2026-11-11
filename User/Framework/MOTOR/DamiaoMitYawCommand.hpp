#pragma once

#include "DamiaoMitYawMonitor.hpp"

namespace damiao_mit_yaw {
// 一帧标准 CAN 数据：帧头 ID 和 8 字节载荷。这里只组装，不执行发送。
struct CanFrame {
    uint16_t standard_id = 0;
    uint8_t data[8]{};
};
enum class SpecialCommand : uint8_t { Enable = 0xfc, Disable = 0xfd };

// MIT 特殊帧：ID 0x06，前七字节 FF；最后一字节 FC 为使能，FD 为失能。
// 例：失能帧为 0x06 / FF FF FF FF FF FF FF FD。
// 此函数只是生成数据；实际发送要先有明确的使能条件和停机策略。
inline bool make_special_frame(SpecialCommand command, CanFrame &output) {
    if (command != SpecialCommand::Enable && command != SpecialCommand::Disable) return false;
    CanFrame next{};
    next.standard_id = MotorCanId;
    for (unsigned i = 0; i < 7; ++i) next.data[i] = 0xff;
    next.data[7] = static_cast<uint8_t>(command);
    output = next;
    return true;
}

// 对应 Yaw 配置的 P_MAX/V_MAX/T_MAX，将 MIT 控制量转换成标准帧。
// 编码范围不是安全限幅；角度、速度、扭矩及控制状态须在发送层另行检查。
inline bool make_control_frame(const damiao_mit::Command &command, CanFrame &output) {
    CanFrame next{};
    next.standard_id = MotorCanId;
    if (!damiao_mit::encode_command(EncodingLimits, command, next.data, 8)) return false;
    // FF FF FF FF FF FF FF xx 可能与特殊命令冲突，不把它作为普通控制帧。
    bool reserved_prefix = true;
    for (unsigned i = 0; i < 7; ++i) reserved_prefix = reserved_prefix && next.data[i] == 0xff;
    if (reserved_prefix) return false;
    output = next;
    return true;
}
}
