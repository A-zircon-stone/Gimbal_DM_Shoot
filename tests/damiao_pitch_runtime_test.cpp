#include "DamiaoPitchRuntime.hpp"
#include "DamiaoPitchPid.hpp"
#include "DamiaoMitPitchMonitor.hpp"
#include "CANio.hpp"
#include "debugc.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
DebugStub debug;
damiao_mit_pitch::FeedbackSnapshot frame{};
unsigned enables=0, disables=0, controls=0;
damiao_mit::Command last_command{};
bool fail_control=false;
bool disable_complete=true;
damiao_mit_pitch::FeedbackSnapshot damiao_mit_pitch_feedback_snapshot() { return frame; }
HAL_StatusTypeDef damiao_pitch_can_send_enable() { ++enables; return HAL_OK; }
HAL_StatusTypeDef damiao_pitch_can_send_disable() { ++disables; return HAL_OK; }
bool damiao_pitch_can_disable_tx_complete() { return disable_complete; }
HAL_StatusTypeDef damiao_pitch_can_send_control(const damiao_mit::Command &c) {
    ++controls; last_command=c; return fail_control ? HAL_ERROR : HAL_OK;
}
void feedback(uint32_t now, uint8_t status, float position) {
    using namespace damiao_mit;
    const auto p=encode(position,-12.5f,12.5f,16);
    const auto v=encode(0,-30,30,12), t=encode(0,-10,10,12);
    frame.seen=true; ++frame.count; frame.last_ms=now;
    frame.data[0]=static_cast<uint8_t>((status<<4)|5);
    frame.data[1]=p>>8; frame.data[2]=p;
    frame.data[3]=v>>4; frame.data[4]=static_cast<uint8_t>((v<<4)|(t>>8));
    frame.data[5]=t; frame.data[6]=25; frame.data[7]=26;
}
int main(int argc, char **argv) {
    using namespace damiao_pitch_runtime;
    // 沿用用户当前角度配置，边界测试取上下限内外的点，不固定旧调试范围。
    const damiao_pitch_pid::Config limits{};
    const float inside_upper_roll = -(limits.max_position-0.01f)*57.295779513f;
    const float outside_upper_roll = -(limits.max_position+0.01f)*57.295779513f;
    if (argc == 2 && std::strcmp(argv[1], "remote")==0) {
        step(0); publish_imu(0,0,2); feedback(2,0,0); step(2);
        feedback(4,1,0); step(4);
        assert(snapshot().state==State::Running);
        feedback(6,1,0); step(6,50.0f);
        // 速度由用户配置，检查半摇杆在2ms产生正向变化，不固定配置值。
        assert(snapshot().target_position>0);
        const float previous=snapshot().target_position;
        feedback(8,1,0); step(8,0);
        assert(snapshot().target_position==previous);
        debug.dm_target_pending=true; debug.dm_target_rad=0.1f;
        feedback(10,1,0); step(10,100);
        assert(std::fabs(snapshot().target_position-0.1f)<0.00001f);
        debug.dm_stop_requested=true; feedback(12,1,0); step(12,100);
        assert(snapshot().state==State::Stopped);
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "protect")==0) {
        publish_imu(0,0,0); feedback(0,0,0); step(0,0,true);
        assert(disables==1 && enables==0 && snapshot().state==State::Stopped);
        feedback(2,0,0); step(2,0,true);
        assert(enables==0 && controls==0);
        publish_imu(0,0,4); feedback(4,0,0); step(4,0,false);
        assert(enables==1 && snapshot().state==State::Enabling);
        feedback(6,1,0); step(6,0,false);
        assert(snapshot().state==State::Running);
        feedback(8,1,0); step(8,0,false);
        const unsigned before_protect=controls;
        feedback(10,1,0); step(10,0,true);
        assert(disables==2 && controls==before_protect && snapshot().state==State::Stopped);
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "vision")==0) {
        step(0); publish_imu(0,0,2); feedback(2,0,0); step(2);
        feedback(4,1,0); step(4);
        feedback(6,1,0); step(6,0,false,true,0.2f);
        assert(snapshot().state==State::Running &&
               std::fabs(snapshot().target_position-0.2f)<0.00001f);
        feedback(8,1,0); step(8,0,false,true,0.25f);
        assert(std::fabs(snapshot().target_position-0.25f)<0.00001f);
        return 0;
    }
    // 每个场景独立进程运行，模拟一次真正的主控启动，避免人为重置运行时状态。
    if (argc == 2) {
        step(0);
        assert(disables==1 && enables==0);
        if (std::strcmp(argv[1], "startup-stop")==0) {
            debug.dm_stop_requested=true;
            debug.dm_enable_requested=true; // 同时收到停止和使能时停止优先。
            step(2);
            assert(std::strcmp(snapshot().reason,"USER_STOP")==0);
        } else if (std::strcmp(argv[1], "startup-fault")==0) {
            publish_imu(-11.4591559f,0,2); feedback(2,8,-1.0f); step(2);
            assert(std::strcmp(snapshot().reason,"STARTUP_FEEDBACK_FAULT")==0);
        } else {
            assert(std::strcmp(argv[1], "startup-late-feedback")==0);
            // 无IMU/反馈时不能使能；等待期间仍定期发失能来获取反馈。
            step(100); step(200);
            assert(enables==0 && disables==3);
            publish_imu(-11.4591559f,0,202); feedback(202,0,-1.0f); step(202);
            assert(enables==1 && snapshot().state==State::Enabling);
            feedback(204,1,-1.0f); step(204);
            assert(snapshot().state==State::Running &&
                   std::fabs(snapshot().target_position-0.2f)<0.001f);
            return 0;
        }
        // 停止/故障后即使数据恢复，不能自行重新使能；显式命令仍可以恢复。
        publish_imu(-11.4591559f,0,4); feedback(4,0,-1.0f); step(4);
        assert(enables==0 && snapshot().state==State::Stopped);
        publish_imu(-11.4591559f,0,104); feedback(104,0,-1.0f); step(104);
        feedback(106,0,-1.0f); step(106);
        assert(enables==0 && snapshot().state==State::Stopped);
        debug.dm_enable_requested=true; step(108);
        assert(enables==1 && snapshot().state==State::Enabling);
        feedback(110,1,-1.0f); step(110);
        assert(snapshot().state==State::Running &&
               std::fabs(snapshot().target_position-0.2f)<0.001f);
        return 0;
    }
    step(0); assert(disables==1 && snapshot().state==State::Stopped);
    // 显式配置测试增益，不依赖用户在工程里选择的上电默认值。
    debug.dm_gain_pending=5; debug.dm_kp=1.0f; debug.dm_kd=0.1f;
    step(2); // 不发dm_enable：上电自动请求等待IMU和失能反馈。
    assert(enables==0 && std::strcmp(snapshot().reason,"NEED_IMU")==0);
    publish_imu(5.72957795f,0,2); step(2);
    assert(enables==0 && std::strcmp(snapshot().reason,"NEED_DISABLED_FEEDBACK")==0);
    feedback(4,0,-1.0f); disable_complete=false; step(4);
    assert(enables==0 && std::strcmp(snapshot().reason,"WAIT_DISABLE_TX")==0);
    disable_complete=true; step(4);
    assert(enables==1 && snapshot().state==State::Enabling);
    step(6); assert(controls==0); // 未确认使能前不发闭环命令
    // 使能反馈到达的同一轮若提前收到目标，需提示尚未接受，不悄悄当成已设置。
    debug.dm_target_pending=true; debug.dm_target_rad=-0.05f;
    feedback(8,1,-1.0f); step(8);
    assert(snapshot().state==State::Running && std::fabs(snapshot().target_position+0.1f)<0.001f);
    assert(std::strcmp(snapshot().reason,"TARGET_NEEDS_RUNNING")==0);
    assert(std::fabs(snapshot().target_min-limits.min_position)<0.001f &&
           std::fabs(snapshot().target_max-limits.max_position)<0.001f);
    feedback(10,1,-1.0f); step(10);
    assert(controls==1 && last_command.kp==0 && last_command.kd==0);
    const float gravity_at_minus_point_one = -0.04168749f*std::cos(-0.1f) +
                                              0.01554734f*std::sin(-0.1f);
    assert(std::fabs(last_command.torque - gravity_at_minus_point_one) < 0.0001f);
    assert(std::fabs(snapshot().gravity_torque - gravity_at_minus_point_one) < 0.0001f);
    debug.dm_target_pending=true; debug.dm_target_rad=-0.08f; feedback(12,1,-1.0f); step(12);
    assert(snapshot().target_position > -0.09f &&
           std::fabs(last_command.torque - (gravity_at_minus_point_one + 0.02f)) < 0.0001f);
    publish_imu(5.72957795f,0.1f,14);
    debug.dm_target_pending=true; debug.dm_target_rad=limits.max_position+0.01f; feedback(14,1,-1.0f); step(14);
    assert(std::strcmp(snapshot().reason,"TARGET_REJECTED")==0 &&
           std::fabs(last_command.torque - (gravity_at_minus_point_one + 0.03f)) < 0.0001f);
    // 相同绝对目标发两次，目标仍是 -0.08，不会像增量那样累加。
    debug.dm_target_pending=true; debug.dm_target_rad=-0.08f; feedback(16,1,-1.0f); step(16);
    assert(std::fabs(snapshot().target_position + 0.08f) < 0.0001f);
    // 在线修改 Kp：保持使能和目标，下一帧输出按新增益计算。
    debug.dm_gain_pending=1; debug.dm_kp=2.0f; feedback(18,1,-1.0f); step(18);
    assert(snapshot().state==State::Running && snapshot().kp==2.0f &&
           std::fabs(snapshot().target_position+0.08f)<0.0001f &&
           std::fabs(last_command.torque-(gravity_at_minus_point_one+0.05f))<0.0001f);
    // 在线增益可以超过原来的 50/10/5，并立即用于控制，不重置目标或失能。
    debug.dm_gain_pending=7; debug.dm_kp=200.0f; debug.dm_ki=20.0f; debug.dm_kd=10.0f;
    feedback(20,1,-1.0f); step(20);
    assert(snapshot().state==State::Running && snapshot().kp==200.0f &&
           snapshot().ki==20.0f && snapshot().kd==10.0f &&
           std::fabs(snapshot().target_position+0.08f)<0.0001f &&
           std::strcmp(snapshot().reason,"GAIN_SET")==0);
    // 误差0.02rad，角速度-0.1rad/s；P=4、I=0.0008、D=1，再加重力补偿。
    assert(std::fabs(last_command.torque-(gravity_at_minus_point_one+5.0008f))<0.0005f);
    publish_imu(5.72957795f,0,200); step(200);
    assert(snapshot().state==State::Stopped && std::strcmp(snapshot().reason,"FEEDBACK_LOST_OR_FAULT")==0);
    assert(snapshot().gravity_torque==0 && snapshot().requested_torque==0);
    feedback(202,0,-1.0f); step(202);
    assert(snapshot().state==State::Stopped && enables==1); // 故障恢复不自动重启。
    debug.dm_enable_requested=true; step(202);
    feedback(204,1,-1.0f); step(204); assert(snapshot().state==State::Running);
    fail_control=true; feedback(206,1,-1.0f); step(206);
    assert(snapshot().state==State::Stopped && snapshot().tx_errors==1);
    feedback(208,0,-1.0f); debug.dm_enable_requested=true; debug.dm_stop_requested=true;
    debug.dm_target_pending=true; debug.dm_target_rad=0.1f; step(208);
    assert(snapshot().state==State::Stopped && std::strcmp(snapshot().reason,"USER_STOP")==0);
    feedback(210,0,3.0f); debug.dm_enable_requested=true; step(210);
    feedback(212,1,3.0f); step(212);
    assert(snapshot().state==State::Running && std::fabs(snapshot().target_position+0.1f)<0.001f);
    feedback(236,1,3.0f); step(236); // 电机有反馈，但 IMU 不再更新
    assert(snapshot().state==State::Stopped && std::strcmp(snapshot().reason,"IMU_LOST")==0);
    publish_imu(0,0,240); invalidate_imu(); debug.dm_enable_requested=true; step(240);
    assert(snapshot().state==State::Stopped && std::strcmp(snapshot().reason,"NEED_IMU")==0);
    // 比配置下限低0.01rad时，不能发使能。
    publish_imu(-(limits.min_position-0.01f)*57.295779513f,0,242); feedback(242,0,-1.0f);
    const unsigned before = enables;
    debug.dm_enable_requested=true; step(242);
    assert(enables==before && std::strcmp(snapshot().reason,"IMU_OUT_OF_RANGE")==0);
    // 先在上限内0.01rad处启动，再验证目标不能越过当前配置上限。
    fail_control=false; publish_imu(inside_upper_roll,0,244); feedback(244,0,-1.0f);
    debug.dm_enable_requested=true; step(244);
    feedback(246,1,-1.0f); step(246);
    assert(snapshot().state==State::Running);
    debug.dm_target_pending=true; debug.dm_target_rad=limits.max_position+0.01f;
    publish_imu(inside_upper_roll,0,248); feedback(248,1,-1.0f); step(248);
    assert(std::strcmp(snapshot().reason,"TARGET_REJECTED")==0);
    // 运行中实际 IMU 超过配置上限时失能。
    publish_imu(outside_upper_roll,0,250); feedback(250,1,-1.0f); step(250);
    assert(snapshot().state==State::Stopped && std::strcmp(snapshot().reason,"PID_POSITION_LIMIT")==0);
    assert(snapshot().pid_fault_seen && snapshot().pid_fault_angle>limits.max_position &&
           std::fabs(snapshot().pid_fault_max-limits.max_position)<0.001f && snapshot().pid_fault_dt_ms==2);
    // 发送使能后、等待电机确认期间发生越界，同样必须停机。
    publish_imu(0,0,252); feedback(252,0,-1.0f);
    debug.dm_enable_requested=true; step(252);
    assert(snapshot().state==State::Enabling);
    publish_imu(outside_upper_roll,0,254); feedback(254,1,-1.0f); step(254);
    assert(snapshot().state==State::Stopped && std::strcmp(snapshot().reason,"IMU_OUT_OF_RANGE")==0);
    // 水平、静止时立即输出拟合的重力前馈；IMU 失效后不能再提交控制帧。
    publish_imu(0,0,256); feedback(256,0,-1.0f);
    debug.dm_enable_requested=true; step(256);
    feedback(258,1,-1.0f); step(258);
    publish_imu(0,0,260); feedback(260,1,-1.0f); step(260);
    assert(snapshot().state==State::Running && std::fabs(last_command.torque + 0.04168749f)<0.0001f);
    const unsigned controls_before_failure=controls;
    invalidate_imu(); feedback(262,1,-1.0f); step(262);
    assert(snapshot().state==State::Stopped && controls==controls_before_failure && snapshot().gravity_torque==0);
    // 模拟控制任务被暂停 54ms；故障现场应保留间隔与原限位。
    publish_imu(0,0,264); feedback(264,0,-1.0f); debug.dm_enable_requested=true; step(264);
    feedback(266,1,-1.0f); step(266);
    assert(snapshot().state==State::Running && !snapshot().pid_fault_seen);
    // 从 0 使能后转到 -0.30，仍在用户范围内，不能因旧 ±0.15 限制而停机。
    publish_imu(17.188734f,0,268); feedback(268,1,-1.0f); step(268);
    assert(snapshot().state==State::Running);
    publish_imu(0,0,270); feedback(270,1,-1.0f); step(270);
    publish_imu(0,0,324); feedback(324,1,-1.0f); step(324);
    assert(snapshot().state==State::Stopped && std::strcmp(snapshot().reason,"PID_DT_TIMEOUT")==0);
    assert(snapshot().pid_fault_seen && snapshot().pid_fault_dt_ms==54 &&
           std::fabs(snapshot().pid_fault_min-limits.min_position)<0.001f &&
           std::fabs(snapshot().pid_fault_max-limits.max_position)<0.001f);
}
