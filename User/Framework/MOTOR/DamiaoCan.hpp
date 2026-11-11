#pragma once

#include "DamiaoMit.hpp"
#include <cstddef>
#include <cstdint>

// 达妙 MIT 的 CAN 报文构造与反馈识别。这里不访问 STM32 外设，也不发送报文。
namespace damiao_can {
struct AxisIds {
    uint16_t motor_id;    // 控制帧使用的电机 CAN ID
    uint16_t feedback_id; // 电机返回反馈时使用的 Master ID
};

// 计划中的 ID，需在达妙上位机中设置到对应电机；现阶段不代表实物已有此配置。
constexpr AxisIds Yaw{0x01, 0x11};
constexpr AxisIds Pitch{0x03, 0x13};

// 本工程限定两台电机的电机编号为 1..15，避免 D[0] 编号位数的歧义。
// 两个 Master ID 非零且互不重复，也不能与任一电机的 CAN ID 重复。
inline bool valid_pair(AxisIds a, AxisIds b) {
    return a.motor_id >= 1 && a.motor_id <= 15 && b.motor_id >= 1 && b.motor_id <= 15 &&
           a.motor_id != b.motor_id && a.feedback_id >= 1 && a.feedback_id <= 0x7fe &&
           b.feedback_id >= 1 && b.feedback_id <= 0x7fe && a.feedback_id != b.feedback_id &&
           a.feedback_id != a.motor_id && a.feedback_id != b.motor_id &&
           b.feedback_id != a.motor_id && b.feedback_id != b.motor_id;
}

// 完整的标准 CAN 数据帧所需内容；帧类型和长度由实际 CAN 发送层设置。
struct DM_can_frame { uint16_t id; uint8_t data[8]; };
enum class SpecialCommand : uint8_t { Enable = 0xfc, Disable = 0xfd };

// 例如电机 CAN ID 为 0x01 时，使能帧 ID 为 0x01，数据为：
// FF FF FF FF FF FF FF FC。失能只将最后一个字节换成 FD。
// 本函数仅组装报文；固件版本、模式及发送时机均须在实际使用前确认。
inline bool make_special(AxisIds axis, SpecialCommand command, DM_can_frame &out) {
    if (axis.motor_id < 1 || axis.motor_id > 15 ||
        (command != SpecialCommand::Enable && command != SpecialCommand::Disable)) return false;
    DM_can_frame next{};
    next.id = axis.motor_id;
    for (unsigned i = 0; i < 7; ++i) next.data[i] = 0xff;
    next.data[7] = static_cast<uint8_t>(command);
    out = next;
    return true;
}

// 构造 MIT 控制帧；编码范围须取自对应电机的实际配置。
// 还需要在控制层单独实现使能状态、超时、机械限位和实际扭矩限幅。
inline bool make_control(AxisIds axis, const damiao_mit::Limits &limits,
                         const damiao_mit::Command &command, DM_can_frame &out) {
    if (axis.motor_id < 1 || axis.motor_id > 15) return false;
    DM_can_frame next{};
    next.id = axis.motor_id;
    if (!damiao_mit::encode_command(limits, command, next.data, sizeof(next.data))) return false;
    // 避免控制数据与 FF FF FF FF FF FF FF xx 特殊命令格式发生歧义。
    bool special_prefix = true;
    for (unsigned i = 0; i < 7; ++i) special_prefix = special_prefix && next.data[i] == 0xff;
    if (special_prefix) return false;
    out = next;
    return true;
}

enum class Axis { None, Yaw, Pitch }; 
// 先按帧头的 Master ID 找对应轴，再核对数据区 D[0] 低 4 位的电机编号。
// 例：yaw 的反馈帧头 ID=0x11，D[0]=0x81 时，低 4 位为 1，可识别为 yaw；
// 高 4 位 8 是状态/故障码，仍须由上层检查，不能因匹配成功就认为电机正常。
// standard_data 表示调用方已经确认这是标准数据帧、而非扩展帧或远程帧。
inline Axis identify(AxisIds yaw, AxisIds pitch, bool standard_data, uint32_t frame_id,
                     const uint8_t *data, std::size_t length) {
    if (!valid_pair(yaw, pitch) || !standard_data || !data || length != 8) return Axis::None;
    const auto motor_low = static_cast<uint8_t>(data[0] & 0x0f);
    if (frame_id == yaw.feedback_id && motor_low == yaw.motor_id) return Axis::Yaw;
    if (frame_id == pitch.feedback_id && motor_low == pitch.motor_id) return Axis::Pitch;
    return Axis::None;
}
}
