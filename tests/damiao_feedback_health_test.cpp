#include "DamiaoFeedbackHealth.hpp"
#include <cassert>

int main() {
    using damiao_feedback::Health;
    const damiao_mit::Limits limits{12.5f, 30.f, 10.f}; // 测试值，不是实物配置
    const uint8_t current[] = {0x15, 0x80, 0, 0x80, 0x08, 0, 35, 40};
    const uint8_t fault[] = {0x85, 0x80, 0, 0x80, 0x08, 0, 35, 40};
    const uint8_t unknown[] = {0x25, 0x80, 0, 0x80, 0x08, 0, 35, 40};
    assert(damiao_feedback::assess(false, 0, 10, 100, current, 8, limits).health == Health::NoFeedback);
    assert(damiao_feedback::assess(true, 10, 111, 100, current, 8, limits).health == Health::Stale);
    assert(damiao_feedback::assess(true, 10, 110, 100, nullptr, 8, limits).health == Health::Invalid);
    assert(damiao_feedback::assess(true, 10, 110, 100, current, 7, limits).health == Health::Invalid);
    assert(damiao_feedback::assess(true, 10, 110, 100, current, 8, {0,30,10}).health == Health::Invalid);
    const auto result = damiao_feedback::assess(true, 10, 110, 100, current, 8, limits);
    assert(result.health == Health::Current && result.feedback.motor_id_low == 5);
    assert(result.feedback.mos_temperature == 35 && result.feedback.rotor_temperature == 40);
    assert(damiao_feedback::assess(true, 10, 11, 100, fault, 8, limits).health == Health::Fault);
    assert(damiao_feedback::assess(true, 10, 11, 100, unknown, 8, limits).health == Health::UnknownStatus);
    assert(damiao_feedback::assess(true, 0xfffffffeu, 2, 4, current, 8, limits).health == Health::Current);
    assert(damiao_feedback::assess(true, 0xfffffffeu, 3, 4, current, 8, limits).health == Health::Stale);
}
