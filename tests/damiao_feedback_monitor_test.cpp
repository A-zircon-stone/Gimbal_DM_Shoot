#include "DamiaoFeedbackMonitor.hpp"
#include <cassert>
#include <cstdint>

int main() {
    using namespace damiao_can;
    FeedbackMonitor monitor;
    const uint8_t yaw[8] = {0x11, 1, 2, 3, 4, 5, 6, 7};
    const uint8_t pitch[8] = {0x13, 8, 9, 10, 11, 12, 13, 14};
    assert(!monitor.snapshot().yaw.seen);
    assert(!monitor.receive(false, 0x11, yaw, 8, 1));
    assert(!monitor.receive(true, 0x11, pitch, 8, 1)); // 帧头匹配但编号不匹配
    assert(!monitor.receive(true, 0x11, yaw, 7, 1));
    assert(!monitor.snapshot().yaw.seen);
    assert(monitor.receive(true, 0x11, yaw, 8, 0xfffffffeu));
    auto s = monitor.snapshot();
    assert(s.yaw.seen && s.yaw.count == 1 && s.yaw.frame_id == 0x11);
    assert(s.yaw.data[1] == 1 && s.yaw.data[7] == 7 && !s.pitch.seen);
    assert(fresh(s.yaw, 2, 4)); // 经过 32 位计时器回绕后相差 4ms
    assert(!fresh(s.yaw, 3, 4));
    assert(monitor.receive(true, 0x13, pitch, 8, 5));
    assert(monitor.receive(true, 0x11, yaw, 8, 6));
    s = monitor.snapshot();
    assert(s.yaw.count == 2 && s.yaw.last_ms == 6);
    assert(s.pitch.count == 1 && s.pitch.last_ms == 5 && s.pitch.data[7] == 14);
}
