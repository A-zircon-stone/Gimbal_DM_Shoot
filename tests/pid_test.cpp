#include "Pid.hpp"
#include <cassert>
#include <cmath>
#include <limits>

static bool near(float a, float b) { return std::fabs(a-b) < 0.0001f; }

int main() {
    // 四个对象必须保持独立的积分与测量历史。
    Pid_t yaw_position{}, yaw_speed{}, pitch_position{}, pitch_speed{};
    assert(yaw_position.SetGains({60,0,0}));
    assert(yaw_speed.SetGains({0.5f,0,0}));
    assert(pitch_position.SetGains({100,0,0}));
    assert(pitch_speed.SetGains({0.5f,0,0}));
    assert(yaw_position.Calculate({0.1f,0,0.002f,0,0.2f}));
    assert(yaw_speed.Calculate({yaw_position.Output,0.2f,0.002f}, {10}));
    assert(near(yaw_position.Output,6) && near(yaw_speed.Output,2.9f));
    assert(pitch_position.Calculate({0.1f,0,0.002f,0,0.2f}));
    assert(pitch_speed.Calculate({pitch_position.Output,0.2f,0.002f,-0.04f}, {10,0.2f}));
    assert(near(pitch_speed.Output,4.86f));
    // 位置环没有私加速度上限（允许连续Yaw的大角差）。
    assert(yaw_position.Calculate({20,0,0.002f}));
    assert(near(yaw_position.Output,1200));

    Pid_t fine{}, coarse{};
    fine.Init(0,2,0); coarse.Init(0,2,0);
    for (int i=0; i<100; ++i) assert(fine.Calculate({1,0,0.001f}));
    for (int i=0; i<10; ++i) assert(coarse.Calculate({1,0,0.01f}));
    assert(near(fine.Output,0.2f) && near(fine.Output,coarse.Output));
    assert(yaw_speed.Integral == 0 && pitch_speed.Integral == 0);
    // 在线修改增益不清积分；停止/使能时Clear清历史但保留增益。
    assert(fine.SetGains({0,3,0}));
    assert(near(fine.Integral,0.2f));
    fine.Clear();
    assert(fine.Integral == 0 && fine.Output == 0 && fine.Ki == 3);
    assert(!fine.SetGains({-1,0,0}) && fine.Ki == 3);

    Pid_t derivative{};
    derivative.Init(0,0,0.5f);
    assert(derivative.Calculate({0,100,0.01f}));
    assert(derivative.Output == 0); // 首帧无历史，不能产生虚假微分。
    assert(derivative.Calculate({0,100.02f,0.01f}));
    assert(std::fabs(derivative.Output+1) < 0.001f);
    assert(derivative.Calculate({100,100.02f,0.01f}));
    assert(derivative.Output == 0); // 目标突变不造成微分冲击。
    // 外环可直接用IMU角速度作为角度导数。
    derivative.Clear();
    assert(derivative.Calculate({1,0,0.002f,0,2}));
    assert(near(derivative.Output,-1));

    Pid_t saturated{};
    saturated.Init(4,1,0);
    assert(saturated.Calculate({0.12f,0,0.01f,0.04f}, {0.5f,0.4f}));
    assert(near(saturated.Output,0.5f) && saturated.Integral == 0);
    saturated.Init(0,1,0);
    assert(saturated.Calculate({1,0,0.1f}, {1,0.2f}));
    assert(near(saturated.Integral,0.1f));
    assert(saturated.Calculate({1,0,0.5f}, {1,0.2f}));
    assert(near(saturated.Integral,0.2f));
    assert(saturated.Calculate({-1,0,0.1f}, {1,0.2f}));
    assert(near(saturated.Integral,0.1f));
    // 饱和期间允许反向误差释放积分。
    assert(saturated.Calculate({-0.1f,0,0.1f,2}, {1,0.2f}));
    assert(near(saturated.Integral,0.09f) && saturated.Output == 1);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float previous=saturated.Output, previous_i=saturated.Integral;
    assert(!saturated.Calculate({nan,0,0.01f}));
    assert(!saturated.Calculate({0,inf,0.01f}));
    assert(!saturated.Calculate({0,0,0}));
    assert(!saturated.Calculate({0,0,-0.01f}));
    assert(!saturated.Calculate({0,0,0.01f,nan}));
    assert(!saturated.Calculate({0,0,0.01f,0,inf}));
    assert(!saturated.Calculate({0,0,0.01f}, {-1}));
    assert(!saturated.Calculate({0,0,0.01f}, {nan}));
    assert(saturated.Output == previous && saturated.Integral == previous_i);

    // 原Cal接口保持每周期累计和0.2测量差滤波：发射参数无需转换。
    Pid_t legacy{};
    legacy.Init(2,0.5f,1);
    legacy.Cal(3,1,100);
    assert(near(legacy.Output,4.8f));
    legacy.Cal(3,2,100);
    assert(near(legacy.Output,3.14f));
    legacy.Init(20,1,0);
    legacy.Cal(150,0,9600);
    assert(legacy.Output == 3150);
}
