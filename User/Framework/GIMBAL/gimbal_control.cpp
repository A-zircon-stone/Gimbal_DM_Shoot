#include "gimbal_control.hpp"
#include "DMMotor.hpp"
#include "GimbalTuning.hpp"


/*控制流程
PID计算目标力矩
    ↓
pitch_motor.BuildTorqueFrame(...)  生成报文
    ↓
damiao_can_send_torque_frame(...)   提交CAN发送

*/
namespace gimbal_control {

// 创建两个电机对象
damiao::Motor pitch_motor;
damiao::Motor yaw_motor;

void Init()
{
    // 准备Pitch电机配置
    damiao::MotorConfig pitch_config{};

    pitch_config.motor_id =
        gimbal_tuning::pitch::MotorCanId;

    pitch_config.master_id =
        gimbal_tuning::pitch::FeedbackMasterId;

    pitch_config.position_max =
        gimbal_tuning::pitch::EncodingLimits.position;

    pitch_config.velocity_max =
        gimbal_tuning::pitch::EncodingLimits.velocity;

    pitch_config.torque_max =
        gimbal_tuning::pitch::EncodingLimits.torque;

    // 保存到Pitch对象内部
    pitch_motor.Init(pitch_config);

    // 准备Yaw电机配置
    damiao::MotorConfig yaw_config{};

    yaw_config.motor_id =
        gimbal_tuning::yaw::MotorCanId;

    yaw_config.master_id =
        gimbal_tuning::yaw::FeedbackMasterId;

    yaw_config.position_max =
        gimbal_tuning::yaw::EncodingLimits.position;

    yaw_config.velocity_max =
        gimbal_tuning::yaw::EncodingLimits.velocity;

    yaw_config.torque_max =
        gimbal_tuning::yaw::EncodingLimits.torque;

    // 保存到Yaw对象内部
    yaw_motor.Init(yaw_config);
}

}