#include "DamiaoPitchPid.hpp"
#include <cassert>
#include <cmath>
#include <limits>

int main() {
    using namespace damiao_pitch_pid;
    Controller controller;
    Config config{};
    assert(!controller.configure(config, 10.0f));
    config = {2.0f, 1.0f, 0.2f, 0.4f, 1.0f, -2.0f, 2.0f, 0.1f};
    assert(controller.configure(config, 10.0f));
    assert(controller.start(-1.0f));
    assert(controller.target() == -1.0f);
    assert(!controller.set_target(3.0f));
    assert(controller.set_target(-0.9f));
    damiao_mit::Command command{};
    assert(controller.calculate(true, -1.0f, 0.0f, 0.01f, command));
    assert(command.kp == 0.0f && command.kd == 0.0f);
    assert(std::fabs(command.torque - 0.201f) < 0.0001f);
    assert(controller.set_target(2.0f));
    assert(controller.calculate(true, -1.0f, 0.0f, 0.01f, command));
    assert(command.torque <= 1.0f && command.torque >= -1.0f);
    assert(!controller.calculate(false, -1.0f, 0.0f, 0.01f, command));
    assert(!controller.active() && controller.integral() == 0.0f);
    assert(!controller.calculate(true, -1.0f, 0.0f, 0.01f, command));
    assert(controller.start(-1.0f));
    assert(!controller.calculate(true, -1.0f, 0.0f, 0.2f, command));
    assert(controller.start(-1.0f));
    assert(!controller.calculate(true, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.01f, command));
    assert(controller.start(-1.0f));
    assert(!controller.calculate(true, 2.1f, 0.0f, 0.01f, command));

    // 前馈单独施加时，PID 误差和速度为零，输出应等于前馈。
    Config feedforward_config{4.0f, 1.0f, 0.0f, 0.4f, 0.5f, -1.0f, 1.0f, 0.1f};
    assert(controller.configure(feedforward_config, 10.0f));
    assert(controller.start(0.0f));
    assert(controller.calculate(true, 0.0f, 0.0f, 0.01f, command, -0.04f));
    assert(std::fabs(command.torque + 0.04f) < 0.00001f);
    // P=0.48，前馈=0.04，两者加起来超出 0.5；总输出限幅，积分不得继续增大。
    assert(controller.set_target(0.12f));
    assert(controller.calculate(true, 0.0f, 0.0f, 0.01f, command, 0.04f));
    assert(std::fabs(command.torque - 0.5f) < 0.00001f && controller.integral() == 0.0f);
    // 非法前馈不能产生新输出，也不能保持 PID 运行状态。
    command.torque = 0.123f;
    assert(!controller.calculate(true, 0.0f, 0.0f, 0.01f, command,
                                 std::numeric_limits<float>::quiet_NaN()));
    assert(!controller.active() && command.torque == 0.123f);
}
