#include "ShooterControl.hpp"
#include "ShooterCan.hpp"
#include "VisionLink.hpp"
#include "DamiaoMailboxOwners.hpp"
#include <cassert>
#include <cmath>
#include <initializer_list>

int main()
{
    using namespace gimbal_remote_policy;
    ShooterInput in{};
    in.remote_online = true;
    in.s1 = SwitchDown;
    in.s2 = SwitchMiddle;
    in.left_online = in.right_online = true;
    in.left_speed_rpm = 5800;
    in.right_speed_rpm = -5800;
    in.feeder_stick = 1;
    ShooterController controller;
    for (uint8_t s2 : {SwitchUp, SwitchMiddle, SwitchDown}) {
        in.s2 = s2;
        auto out = controller.step(in);
        assert(out.friction_enabled && out.friction_ready);
        assert(out.feeder_target_rpm == 150 && out.feeder_current > 0);
        in.feeder_stick = -1;
        out = controller.step(in);
        assert(out.feeder_target_rpm == -150 && out.feeder_current < 0);
        in.feeder_stick = 0;
        out = controller.step(in);
        assert(out.feeder_target_rpm == 0 && out.feeder_current == 0);
        in.feeder_stick = 660;
    }
    // 任意一轮未到位、离线，都不能拨弹；摩擦轮继续启动。
    in.left_speed_rpm = 3999;
    auto out = controller.step(in);
    assert(out.friction_enabled && !out.friction_ready && out.feeder_target_rpm == 0);
    in.left_speed_rpm = 4000;
    in.right_speed_rpm = -3999;
    assert(controller.step(in).feeder_target_rpm == 0);
    in.right_speed_rpm = -4000;
    assert(controller.step(in).feeder_target_rpm == 150);
    in.left_online = false;
    assert(controller.step(in).feeder_target_rpm == 0);
    in.left_online = true;
    in.right_online = false;
    assert(controller.step(in).feeder_target_rpm == 0);
    in.right_online = true;

    // 使用真实 VisionControl 验证进模式要新帧、fire 撤销、mode=0、超时。
    in.s1 = SwitchMiddle;
    in.s2 = SwitchUp;
    VisionControl vision;
    vision.set_remote_online(true, 10);
    vision.enable(10);
    VisionReceivePacket2026 packet;
    packet.mode = packet.fire = 1;
    auto run_vision = [&](uint32_t now, uint32_t seq, uint32_t rx_ms) {
        vision.update(now, seq, rx_ms, packet);
        in.vision_fire_allowed = vision.active && vision.target.fire == 1;
        return controller.step(in);
    };
    out = run_vision(100, 10, 100);
    assert(out.friction_enabled && out.feeder_target_rpm == 0);
    assert(run_vision(102, 11, 102).feeder_target_rpm == 150);
    in.left_speed_rpm = 3999;
    assert(run_vision(103, 11, 102).feeder_target_rpm == 0);
    in.left_speed_rpm = 4000;
    in.feeder_stick = -660;
    assert(run_vision(104, 11, 102).feeder_target_rpm == -150);
    packet.fire = 0;
    assert(run_vision(106, 12, 106).feeder_target_rpm == 0);
    packet.fire = 2; // 旧打符值不是本次协议约定的 fire=1。
    assert(run_vision(108, 13, 108).feeder_target_rpm == 0);
    packet.fire = 1;
    assert(run_vision(110, 14, 110).feeder_target_rpm == -150);
    assert(run_vision(111 + VisionLinkConfig::RxTimeoutMs, 14, 110).feeder_target_rpm == 0);
    packet.mode = 0;
    assert(run_vision(112, 15, 112).feeder_target_rpm == 0);
    for (uint8_t s2 : {SwitchMiddle, SwitchDown}) {
        in.s2 = s2;
        out = controller.step(in);
        assert(!out.friction_enabled && out.feeder_target_rpm == 0);
        assert(out.left_current == 0 && out.right_current == 0 && out.feeder_current == 0);
    }
    in.s1 = SwitchUp;
    in.s2 = SwitchUp;
    assert(!controller.step(in).friction_enabled);
    in.s1 = SwitchDown;
    in.remote_online = false;
    assert(!controller.step(in).friction_enabled);

    // 摩擦轮沿用现有目标、爬升与方向；满速时继续维持目标。
    ShooterController ramp;
    in.remote_online = true;
    in.left_speed_rpm = in.right_speed_rpm = 0;
    for (int i = 0; i < 2000; ++i) out = ramp.step(in);
    assert(out.left_target_rpm == 5800 && out.right_target_rpm == -5800);
    assert(out.left_current > 0 && out.right_current < 0);
    assert(out.left_current <= 16384 && out.right_current >= -16384);
    assert(out.feeder_target_rpm == 0);

    // CAN 0x200：ID1/2为摩擦轮、ID3留空、ID4为拨盘，均为大端有符号电流。
    uint8_t bytes[8]{};
    shooter_can::encode_currents(0x1234, -2, -150, bytes);
    const uint8_t expected[] = {0xff, 0xfe, 0x12, 0x34, 0, 0, 0xff, 0x6a};
    for (unsigned i = 0; i < 8; ++i) assert(bytes[i] == expected[i]);
    // 射击帧复用邮箱后，两台达妙停止不能误撤销它。
    DamiaoMailboxOwners owners;
    owners.pitch_motion = 1;
    owners.yaw_disable = 1;
    owners.reuse(1);
    assert(owners.pitch_motion == 0 && owners.yaw_disable == 0);
}
