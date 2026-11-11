#include "DamiaoV41Monitor.hpp"
#include <cassert>
#include <cmath>

int main() {
    using namespace damiao_v41;
    // 用户实际抓到的 pitch 反馈：位置 6674，速度 -0.10 rpm，电流 -3 mA，温度 22℃。
    const uint8_t pitch[] = {0x1a, 0x12, 0xff, 0xf6, 0xff, 0xfd, 0x16, 0x00};
    Feedback decoded{};
    assert(decode_feedback(5, 0x305, pitch, 8, decoded));
    assert(decoded.position_raw == 6674 && decoded.speed_raw == -10);
    assert(std::fabs(speed_rpm(decoded) + 0.1f) < 0.0001f);
    assert(decoded.current_ma == -3 && decoded.coil_temperature_c == 22 && decoded.error_status == 0);
    assert(!decode_feedback(5, 0x306, pitch, 8, decoded));
    assert(!decode_feedback(0, 0x300, pitch, 8, decoded));
    assert(!decode_feedback(5, 0x305, nullptr, 8, decoded));
    assert(!decode_feedback(5, 0x305, pitch, 7, decoded));
    const uint8_t invalid[] = {0x20, 0, 0, 0, 0, 0, 0, 0}; // 8192 超出单圈范围
    assert(!decode_feedback(5, 0x305, invalid, 8, decoded));
    assert(decoded.position_raw == 6674); // 失败时不覆盖上次有效反馈

    FeedbackMonitor monitor;
    assert(!monitor.receive(0x301, pitch, 8, 1)); // yaw 尚未确认，不能误认成 yaw
    assert(monitor.receive(0x305, pitch, 8, 0xfffffffeu));
    auto s = monitor.snapshot();
    assert(!s.yaw.seen && s.pitch.seen && s.pitch.count == 1);
    assert(fresh(s.pitch, 2, 4) && !fresh(s.pitch, 3, 4)); // 时间回绕
    assert(monitor.receive(0x305, pitch, 8, 8));
    s = monitor.snapshot();
    assert(s.pitch.count == 2 && s.pitch.last_ms == 8);

    const uint16_t values[] = {0x1234, 0xffff, 0x0000, 0xabcd};
    CurrentFrame frame{};
    assert(make_current_frame(2, values, frame));
    const uint8_t expected[] = {0x34, 0x12, 0xff, 0xff, 0, 0, 0xcd, 0xab};
    assert(frame.standard_id == 0x4fe);
    for (unsigned i=0; i<8; ++i) assert(frame.data[i] == expected[i]);
    assert(make_current_frame(1, values, frame) && frame.standard_id == 0x3fe);
    assert(!make_current_frame(3, values, frame) && frame.standard_id == 0x3fe);
}
