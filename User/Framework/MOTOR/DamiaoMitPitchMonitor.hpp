#pragma once

#include "DamiaoMit.hpp"
#include "GimbalTuning.hpp"
#include "DamiaoFeedbackHealth.hpp"
#include <cstddef>
#include <cstdint>

namespace damiao_mit_pitch {
// 电机地址和 MIT 编码范围统一在 GimbalTuning.hpp 配置。
constexpr uint8_t MotorCanId = gimbal_tuning::pitch::MotorCanId;
constexpr uint16_t FeedbackMasterId = gimbal_tuning::pitch::FeedbackMasterId;
constexpr damiao_mit::Limits EncodingLimits = gimbal_tuning::pitch::EncodingLimits;

struct FeedbackSnapshot {
    bool seen = false;        // 是否收到过可识别的电机反馈
    uint32_t count = 0;       // 已接收的反馈帧数量
    uint32_t last_ms = 0;     // 最近一次反馈时的毫秒计数
    uint8_t data[8]{};        // 原始反馈；任务层可用 decode_snapshot() 换算
};

// 参数读写应答也可能使用 Master ID 0x63，不能直接当作电机反馈。
// 例如 05 00 33 15 ... 是向 ID 5 读 PMAX 的应答，而非位置反馈。
// 只根据载荷做保守排除：如恰有位置反馈与此字节序列相同，也会被丢弃。
inline bool looks_like_parameter_reply(const uint8_t *data) {
    return data[0] == MotorCanId && data[1] == 0 &&
           (data[2] == 0x33 || data[2] == 0x55 || data[2] == 0xaa);
}

class FeedbackMonitor {
public:
    // 调用方须先确认标准数据帧、CAN1；这里只检查 ID、长度与内容。
    bool receive(uint32_t frame_id, const uint8_t *data, std::size_t length, uint32_t now_ms) {
        if (frame_id != FeedbackMasterId || !data || length != 8 ||
            (data[0] & 0x0f) != MotorCanId || looks_like_parameter_reply(data)) return false;
        for (unsigned i = 0; i < 8; ++i) state_.data[i] = data[i];
        state_.last_ms = now_ms;
        ++state_.count;
        state_.seen = true;
        return true;
    }
    FeedbackSnapshot snapshot() const { return state_; }
private:
    FeedbackSnapshot state_{};
};

// 数值换算在任务层执行，而不是在 CAN 接收中断里做浮点运算。
inline bool decode_snapshot(const FeedbackSnapshot &snapshot, damiao_mit::Feedback &output) {
    return snapshot.seen && damiao_mit::decode_feedback(EncodingLimits, snapshot.data, 8, output);
}
inline bool fresh(const FeedbackSnapshot &snapshot, uint32_t now_ms, uint32_t timeout_ms) {
    // 无符号减法允许毫秒计数器回绕，例如 FFFFFFFE 到 2 相差 4ms。
    return snapshot.seen && static_cast<uint32_t>(now_ms - snapshot.last_ms) <= timeout_ms;
}
// 将中断中保存的完整快照交给任务层检查；不在这里发送或使能电机。
// timeout_ms 由调用方根据实际反馈周期指定，不能仅以 Current 允许电机输出。
inline damiao_feedback::Assessment assess_snapshot(const FeedbackSnapshot &snapshot,
                                                    uint32_t now_ms, uint32_t timeout_ms) {
    return damiao_feedback::assess(snapshot.seen, snapshot.last_ms, now_ms, timeout_ms,
                                   snapshot.data, sizeof(snapshot.data), EncodingLimits);
}
}
