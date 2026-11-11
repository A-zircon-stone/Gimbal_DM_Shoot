#pragma once

#include "DamiaoV41.hpp"

namespace damiao_v41 {
struct AxisFeedback {
    Feedback feedback{};
    bool seen = false;
    uint32_t count = 0;
    uint32_t last_ms = 0;
};
struct FeedbackSnapshot { AxisFeedback yaw, pitch; };

// 在 CAN 接收中断写入；任务读取整份快照时需要由调用方保护临界区。
class FeedbackMonitor {
public:
    bool receive(uint32_t frame_id, const uint8_t *data, std::size_t length,
                 uint32_t now_ms) {
        Feedback decoded{};
        AxisFeedback *axis = nullptr;
        if (decode_feedback(PitchMotorId, frame_id, data, length, decoded)) axis = &state_.pitch;
        else if (YawMotorId != PitchMotorId &&
                 decode_feedback(YawMotorId, frame_id, data, length, decoded)) axis = &state_.yaw;
        if (!axis) return false;
        axis->feedback = decoded;
        axis->last_ms = now_ms;
        ++axis->count;
        axis->seen = true;
        return true;
    }
    FeedbackSnapshot snapshot() const { return state_; }
private:
    FeedbackSnapshot state_{};
};

inline bool fresh(const AxisFeedback &axis, uint32_t now_ms, uint32_t timeout_ms) {
    // 无符号减法处理计数器回绕：0xfffffffe 到 2 的间隔为 4ms。
    return axis.seen && static_cast<uint32_t>(now_ms - axis.last_ms) <= timeout_ms;
}
}
