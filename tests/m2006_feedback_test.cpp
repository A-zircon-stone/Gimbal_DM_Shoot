#include "M2006Feedback.hpp"
#include <cassert>
#include <cmath>

int main()
{
    M2006Feedback feedback;
    uint8_t bytes[] = {0x1f, 0xf0, 0x15, 0x18, 0xff, 0x9c, 40, 0};
    assert(feedback.update(bytes, 8));
    assert(feedback.speed_rpm == 150); // 5400 / 36
    assert(feedback.current_raw == -100);
    const float before = feedback.position_deg;
    bytes[0] = 0; bytes[1] = 16; // 8176 -> 16，跨零实际前进32计数。
    assert(feedback.update(bytes, 8));
    assert(std::fabs(feedback.position_deg - before - 32.0f * 360 / (8192 * 36)) < 0.0001f);
    bytes[0] = 0x1f; bytes[1] = 0xf0;
    bytes[2] = 0xea; bytes[3] = 0xe8; // -5400 rpm
    assert(feedback.update(bytes, 8));
    assert(feedback.speed_rpm == -150);
    assert(std::fabs(feedback.position_deg - before) < 0.0001f);
    assert(!feedback.update(bytes, 7));
    assert(!feedback.update(nullptr, 8));
    bytes[0] = 0x20; bytes[1] = 0; // 编码器有效计数0..8191
    assert(!feedback.update(bytes, 8));
    assert(feedback.speed_rpm == -150);
}
