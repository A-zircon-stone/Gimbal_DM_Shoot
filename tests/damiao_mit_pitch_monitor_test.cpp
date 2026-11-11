#include "DamiaoMitPitchMonitor.hpp"
#include <cassert>
#include <cmath>

int main() {
    using namespace damiao_mit_pitch;
    FeedbackMonitor monitor;
    // 0x15 表示状态 1、电机编号低四位 5；三个数值均接近编码范围中点。
    const uint8_t feedback[8] = {0x15, 0x80, 0x00, 0x80, 0x08, 0x00, 42, 37};
    const uint8_t parameter_reply[8] = {0x05, 0x00, 0x33, 0x15, 0, 0, 0, 0};
    assert(!monitor.receive(0x305, feedback, 8, 1)); // V41 帧 ID 不再用于 MIT
    assert(!monitor.receive(0x63, nullptr, 8, 1));
    assert(!monitor.receive(0x63, feedback, 7, 1));
    assert(!monitor.receive(0x63, parameter_reply, 8, 1));
    assert(!monitor.snapshot().seen);
    assert(monitor.receive(0x63, feedback, 8, 0xfffffffeu));
    auto snapshot = monitor.snapshot();
    assert(snapshot.seen && snapshot.count == 1 && snapshot.data[0] == 0x15);
    assert(fresh(snapshot, 2, 4) && !fresh(snapshot, 3, 4));
    assert(assess_snapshot(snapshot, 2, 4).health == damiao_feedback::Health::Current);
    assert(assess_snapshot(snapshot, 3, 4).health == damiao_feedback::Health::Stale);
    damiao_mit::Feedback decoded{};
    assert(decode_snapshot(snapshot, decoded));
    assert(decoded.motor_id_low == 5 && decoded.status == 1);
    assert(std::fabs(decoded.position) < 0.001f);
    assert(std::fabs(decoded.velocity) < 0.01f);
    assert(std::fabs(decoded.torque) < 0.003f);
    assert(decoded.mos_temperature == 42 && decoded.rotor_temperature == 37);
    assert(monitor.receive(0x63, feedback, 8, 12));
    assert(monitor.snapshot().count == 2 && monitor.snapshot().last_ms == 12);
}
