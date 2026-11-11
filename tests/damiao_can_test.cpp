#include "DamiaoCan.hpp"
#include <cassert>

int main() {
    using namespace damiao_can;
    assert(valid_pair(Yaw, Pitch));
    assert(!valid_pair({1, 0x11}, {3, 0x11})); // 两台反馈 ID 冲突
    assert(!valid_pair({1, 3}, Pitch));         // Master ID 与另一台电机 ID 冲突
    assert(!valid_pair({1, 0}, Pitch));          // 本方案不使用共享 Master ID 0
    DM_can_frame f{};
    assert(make_special(Yaw, SpecialCommand::Enable, f));
    assert(f.id == 1 && f.data[7] == 0xfc);
    for (unsigned i = 0; i < 7; ++i) assert(f.data[i] == 0xff);
    assert(make_special(Pitch, SpecialCommand::Disable, f));
    assert(f.id == 3 && f.data[7] == 0xfd);
    const uint8_t yaw[] = {0x81, 0, 0, 0, 0, 0, 0, 0};
    const uint8_t pitch[] = {0x13, 0, 0, 0, 0, 0, 0, 0};
    assert(identify(Yaw, Pitch, true, 0x11, yaw, 8) == Axis::Yaw);
    assert(identify(Yaw, Pitch, true, 0x13, pitch, 8) == Axis::Pitch);
    assert(identify(Yaw, Pitch, true, 0x13, yaw, 8) == Axis::None);
    assert(identify(Yaw, Pitch, false, 0x11, yaw, 8) == Axis::None);
    assert(identify(Yaw, Pitch, true, 0x11, yaw, 7) == Axis::None);
    assert(identify(Yaw, Pitch, true, 0x11, nullptr, 8) == Axis::None);
    const damiao_mit::Limits limits{12.5f, 30.f, 10.f}; // 测试数据，不是实物配置
    assert(make_control(Yaw, limits, {0, 0, 0, 0, 0}, f));
    assert(f.id == 1 && f.data[0] == 0x7f && f.data[1] == 0xff);
    assert(!make_control(Pitch, limits, {12.5f, 30.f, 500.f, 5.f, 10.f}, f));
    assert(f.id == 1); // 失败时保留原来的输出
}
