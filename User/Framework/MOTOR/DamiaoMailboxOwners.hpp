#pragma once
#include <cstdint>

// 两轴由同一个云台任务提交CAN帧；每次邮箱重新分配时更新归属。
// 例如邮箱1原属Pitch，发送完成后被Yaw复用，Pitch停止不能撤销Yaw帧。
struct DamiaoMailboxOwners {
    uint32_t pitch_motion = 0, pitch_disable = 0;
    uint32_t yaw_motion = 0, yaw_disable = 0;
    void reuse(uint32_t mailbox) {
        pitch_motion &= ~mailbox; yaw_motion &= ~mailbox;
        if (pitch_disable == mailbox) pitch_disable = 0;
        if (yaw_disable == mailbox) yaw_disable = 0;
    }
};
