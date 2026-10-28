#include "PitchTilt.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <initializer_list>

int main()
{
    PitchTilt tilt{};
    auto sample = [&tilt](float deg) {
        const float half = deg * 0.017453292519943295f * 0.5f;
        const float q[] = {std::cos(half), 0, std::sin(half), 0};
        assert(update_pitch_tilt(&tilt, q));
        return map_pitch_feedback(tilt, -0.5f, 5000, true);
    };
    auto a = sample(89);
    auto b = sample(91);
    assert(a.valid && b.valid);
    assert(b.angle_rad < a.angle_rad); // 越过90度不折返
    assert(std::fabs(b.speed_rad_s - 0.5f) < 1e-6f);
    sample(179);
    b = sample(181);
    assert(std::fabs(b.angle_rad + 181 * 0.017453292519943295f) < 1e-5f);
    assert(b.inside_soft_limits && b.allow_raise && b.allow_lower);
    auto low = map_pitch_feedback(tilt, 0, PitchMechanism::SoftMin, true);
    assert(low.valid && !low.inside_soft_limits && !low.allow_raise && low.allow_lower);
    auto high = map_pitch_feedback(tilt, 0, PitchMechanism::SoftMax, true);
    assert(high.valid && !high.inside_soft_limits && high.allow_raise && !high.allow_lower);
    for (uint16_t ep : {uint16_t(3499), uint16_t(6401), uint16_t(8191)})
    {
        auto outside = map_pitch_feedback(tilt, 0, ep, true);
        assert(!outside.allow_raise && !outside.allow_lower && !outside.inside_soft_limits);
    }
    assert(!map_pitch_feedback(tilt, 0, 5000, false).valid);
    assert(!map_pitch_feedback(tilt, std::numeric_limits<float>::quiet_NaN(), 5000, true).valid);
    assert(!map_pitch_feedback(tilt, 0, 8192, true).valid);
    tilt.valid = false;
    assert(!map_pitch_feedback(tilt, 0, 5000, true).valid);
}
