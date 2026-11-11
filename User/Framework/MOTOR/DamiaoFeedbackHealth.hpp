#pragma once

#include "DamiaoMit.hpp"
#include <cstddef>
#include <cstdint>

// 与电机 ID、CAN 外设无关的反馈检查；只供观察和后续控制层使用，不发送报文。
namespace damiao_feedback {
enum class Health { NoFeedback, Stale, Invalid, Current, Fault, UnknownStatus };

struct Assessment {
    Health health = Health::NoFeedback;
    damiao_mit::Feedback feedback{}; // 仅 Current/Fault/UnknownStatus 时是有效的本帧数据
};

// 输入必须是已经完成帧类型、长度、参数应答和电机身份检查的反馈快照。
// 本函数本身不检查 CAN ID，也不能证明电机安全或允许启动闭环。
// timeout_ms 为调用者根据发送/反馈周期设置的超时，不是固定电机参数。
inline Assessment assess(bool feedback_seen, uint32_t last_feedback_ms, uint32_t now_ms,
                         uint32_t timeout_ms, const uint8_t *data, std::size_t length,
                         const damiao_mit::Limits &limits) {
    Assessment result{};
    if (!feedback_seen) return result;
    // 无符号减法处理系统计数回绕：例如 FFFFFFFE 到 2 的间隔是 4ms。
    if (static_cast<uint32_t>(now_ms - last_feedback_ms) > timeout_ms) {
        result.health = Health::Stale;
        return result;
    }
    if (!damiao_mit::decode_feedback(limits, data, length, result.feedback)) {
        result.health = Health::Invalid;
        return result;
    }
    // D[0]=0x85 时，高四位 8 为状态码；手册定义 8..E 为故障。
    // 0/1 按常见固件视为可识别状态；2..7 和 F 未确认时按未知处理。
    // Current 只说明反馈新鲜且状态已识别，并不代表已经使能或允许输出。
    const uint8_t status = result.feedback.status;
    if (status >= 8 && status <= 14) result.health = Health::Fault;
    else if (status <= 1) result.health = Health::Current;
    else result.health = Health::UnknownStatus;
    return result;
}
}
