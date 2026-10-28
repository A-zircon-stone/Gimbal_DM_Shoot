#include "PitchTilt.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

constexpr float pi = 3.14159265358979323846f;
void make_q(float pitch_deg, float yaw_deg, float q[4])
{
    const float p = pitch_deg * pi / 360.0f;
    const float y = yaw_deg * pi / 360.0f;
    // q_z(yaw) ⊗ q_y(pitch)
    q[0] = std::cos(y)*std::cos(p);
    q[1] = -std::sin(y)*std::sin(p);
    q[2] = std::cos(y)*std::sin(p);
    q[3] = std::sin(y)*std::cos(p);
}
void near(float actual, float expected, float tolerance = 0.003f)
{
    assert(std::fabs(actual - expected) < tolerance);
}
int main()
{
    PitchTilt s{};
    float q[4];
    // 跨过欧拉±90°仍单调；跨过±180°时连续解绕。
    for (int i = 0; i <= 1440; ++i)
    {
        const float angle = i * 0.5f;
        make_q(angle, 37.0f, q);
        assert(update_pitch_tilt(&s, q) && s.valid);
        near(s.continuous_deg, angle, 0.02f);
    }
    for (int i = 1440; i >= -720; --i)
    {
        const float angle = i * 0.5f;
        make_q(angle, -73.0f, q);
        assert(update_pitch_tilt(&s, q));
        near(s.continuous_deg, angle, 0.03f);
    }
    for (float angle : {-179.f, -100.f, -90.f, -30.f, 0.f, 30.f, 90.f, 100.f, 179.f})
    {
        PitchTilt once{};
        make_q(angle, 123.f, q);
        for (float &v : q) v *= -3.f;
        assert(update_pitch_tilt(&once, q));
        near(once.continuous_deg, angle);
    }
    // 传感器Y轴竖直时，XZ重力投影消失，不允许发布有效角度。
    const float quarter_turn_x[4] = {std::sqrt(0.5f), std::sqrt(0.5f), 0, 0};
    const float previous = s.continuous_deg;
    assert(!update_pitch_tilt(&s, quarter_turn_x));
    assert(!s.valid && !s.initialized);
    near(s.continuous_deg, previous);
    make_q(20, 0, q);
    assert(update_pitch_tilt(&s, q));
    near(s.continuous_deg, 20); // 恢复时不猜测失效期间转过的圈数
    const float zero[4] = {};
    assert(!update_pitch_tilt(&s, zero));
    assert(!update_pitch_tilt(&s, nullptr));
    assert(!update_pitch_tilt(nullptr, q));
    q[0] = std::numeric_limits<float>::quiet_NaN();
    assert(!update_pitch_tilt(&s, q));
    q[0] = std::numeric_limits<float>::infinity();
    assert(!update_pitch_tilt(&s, q));
    make_q(30, 0, q);
    assert(update_pitch_tilt(&s, q));
    s.continuous_deg = std::numeric_limits<float>::infinity();
    assert(!update_pitch_tilt(&s, q));
    std::cout << "Pitch Y tilt tests passed\n";
}
