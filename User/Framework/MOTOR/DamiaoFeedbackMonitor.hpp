#pragma once

#include "DamiaoCan.hpp"
#include <cstddef>
#include <cstdint>

// 接收观察器：仅记录符合所选 ID 的原始反馈，不发送任何报文。
// 尚未拿到 P_MAX/V_MAX/T_MAX，因此这里不把编码值换算成角度或扭矩。
namespace damiao_can {
struct RawFeedback {
    bool seen = false;           // 从启动至今是否收到过匹配报文
    uint32_t count = 0;          // 累计收到的匹配报文数
    uint32_t last_ms = 0;        // 最近收到报文时的系统毫秒计数
    uint16_t frame_id = 0;       // CAN 帧头 ID（反馈 Master ID）
    uint8_t data[8]{};           // 原始 D[0]..D[7]，便于调试器检查
};
struct FeedbackSnapshot { RawFeedback yaw, pitch; };

// 中断和任务间读取时，调用者必须用临界区保护整个快照的复制。
class FeedbackMonitor {
public:
    bool receive(bool standard_data, uint32_t frame_id, const uint8_t *data,
                 std::size_t length, uint32_t now_ms) {
        const Axis axis = identify(Yaw, Pitch, standard_data, frame_id, data, length);
        if (axis == Axis::None) return false;
        RawFeedback &record = axis == Axis::Yaw ? state_.yaw : state_.pitch;
        record.frame_id = static_cast<uint16_t>(frame_id);
        for (unsigned i = 0; i < 8; ++i) record.data[i] = data[i];
        record.last_ms = now_ms;
        ++record.count;
        record.seen = true;
        return true;
    }
    FeedbackSnapshot snapshot() const { return state_; }
private:
    FeedbackSnapshot state_{};
};

// now_ms 和 last_ms 均为 32 位毫秒计数；无符号减法可处理计数器回绕。
// 例如 last_ms=0xfffffffe、now_ms=2，相差 4ms，而非一个很大的负数。
inline bool fresh(const RawFeedback &record, uint32_t now_ms, uint32_t timeout_ms) {
    return record.seen && static_cast<uint32_t>(now_ms - record.last_ms) <= timeout_ms;
}
}
