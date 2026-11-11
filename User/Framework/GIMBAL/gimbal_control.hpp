#pragma once

#include "DamiaoMit.hpp"
#include <cstdint>


namespace gimbal_control {

void Init();

}
// 达妙双轴云台：集中调参入口（当前 Task_Gimbal 正在使用）。
// 修改本文件后需要重新编译、烧录；串口在线修改 PID 只在本次上电期间有效。
// PID 是 STM32 的 IMU 位置闭环增益；发给电机的 MIT Kp/Kd 仍为 0。
// 旧 6020 的 PitchSpeedTune/YawSpeedTune 属于保留代码，不是当前双轴的调参入口。
namespace gimbal_tuning {
constexpr float DegreesToRadians = 0.017453292519943295f; // π / 180，单位换算

namespace pitch {
// ===== Pitch：上电 PID =====
// 在线命令：dm_kp=数值、dm_ki=数值、dm_kd=数值，每条以换行结束。
constexpr float InitialKp = 50.0f; // N·m/rad，位置误差比例
constexpr float InitialKi = 0.0f;   // N·m/(rad·s)，积分
constexpr float InitialKd = 0.5f;   // N·m/(rad/s)，IMU 角速度阻尼

// ===== Pitch：遥控灵敏度 =====
// 右摇杆上下；满杆每秒改变多少度目标角，半杆为一半，松手保持目标。
constexpr float RemoteMaxSpeedDegPerSecond = 200.0f; // °/s

// ===== Pitch：角度范围、输出与方向 =====
constexpr float MinPositionRad = -0.50f; // IMU 抬头为正，rad
constexpr float MaxPositionRad = 0.65f;  // rad
constexpr float MaxIntegralNm = 0.2f;   // 积分项输出绝对值上限，N·m
constexpr float MaxTorqueNm = 10.0f;    // PID + 重力补偿的总输出限幅，N·m
constexpr float MotorTorqueDirection = 1.0f; // 已确认正扭矩抬头；反装时核对方向

// ===== Pitch：重力补偿 =====
// β = 实际 IMU Pitch（rad），水平为 0；q = A*cos(β) + B*sin(β)。
constexpr float GravityCosCoefficientNm = -0.04168749f; // A，N·m
constexpr float GravitySinCoefficientNm =  0.01554734f; // B，N·m

// ===== Pitch：电机上位机参数（需与实际电机一致） =====
constexpr uint8_t MotorCanId = 0x05;
constexpr uint16_t FeedbackMasterId = 0x63;
// PMAX(rad)、VMAX(rad/s)、TMAX(N·m)：MIT 编码范围，不是云台机械限位。
constexpr damiao_mit::Limits EncodingLimits{12.5f, 30.0f, 10.0f};
}

namespace yaw {
// ===== Yaw：上电 PID =====
// 在线命令：ykp=数值、yki=数值、ykd=数值，每条以换行结束。
// 默认全为 0，沿用当前调试设置；实际位置纠正力矩由在线调参或修改默认值产生。
constexpr float InitialKp = 10.0f; // N·m/rad
constexpr float InitialKi = 0.0f; // N·m/(rad·s)
constexpr float InitialKd = 0.0f; // N·m/(rad/s)

// ===== Yaw：遥控灵敏度与方向 =====
// 右摇杆左右；控制任务已沿用旧工程的 -portGetCh0() 通道方向。
constexpr float RemoteMaxSpeedDegPerSecond = 250.0f; // °/s
constexpr float RemoteDirection = 1.0f; // 左右反向时改为 -1，与电机扭矩方向独立
// 暂按正扭矩使 IMU 连续 Yaw 增大；尚未实测，确认相反时改为 -1。
constexpr float MotorTorqueDirection = 1.0f;

// ===== Yaw：电机上位机参数与输出 =====
constexpr uint8_t MotorCanId = 0x06;
constexpr uint16_t FeedbackMasterId = 0x62;
constexpr damiao_mit::Limits EncodingLimits{12.5f, 30.0f, 10.0f};
// 沿用原先按 TMAX 限幅的关系；需要独立调小时可在这里填 N·m 数值。
constexpr float MaxTorqueNm = EncodingLimits.torque;
// ===== Yaw：软件目标角限位（默认关闭） =====
// 参照 INS.YawTotalAngle 转成 rad 的连续角，不是每次使能位置的相对角。
// 0/0 是未填写占位值，不生效。先填写上下限，再将开关设为 true。
// 开启后目标夹在区间内；越界不直接失能，上电在区间外时目标取最近边界。
constexpr bool SoftLimitEnabled = false;
constexpr float MinPositionRad = -0.7f;
constexpr float MaxPositionRad = 0.7f;
static_assert(!SoftLimitEnabled ||
              (MinPositionRad < MaxPositionRad &&
               MinPositionRad >= -3.402823466e+38F && MaxPositionRad <= 3.402823466e+38F),
              "Fill finite Yaw MinPositionRad < MaxPositionRad before enabling soft limits");
}

namespace shooter {
// ===== 发射：摩擦轮 =====
// 沿用本工程旧 Update_Fric 的转速、爬升、到位阈值和 PID。
// CAN1：右轮0x201（负转），左轮0x202（正转）。速度单位为电机轴rpm。
constexpr uint16_t RightFeedbackId = 0x201;
constexpr uint16_t LeftFeedbackId = 0x202;
constexpr float FrictionTargetRpm = 5800.0f;
constexpr float FrictionRampRpmPerCycle = 3.0f; // 控制周期2ms
constexpr float FrictionReadyRpm = 4000.0f;    // 两轮实测绝对转速均达到此值
constexpr float FrictionKp = 0.0031f;
constexpr float FrictionKi = 0.0f;
constexpr float FrictionKd = 0.000010f;
constexpr float FrictionPidOutputMax = 100.0f; // 原 Pid_t::Cal 参数
constexpr float FrictionTorqueToCurrent = 819.2f / 0.3f; // 原3508换算
constexpr int16_t FrictionCurrentMax = 16384; // 原3508电流编码范围

// ===== 发射：M2006 + 36:1 拨盘 =====
// ch[3]为左摇杆上下；目标、反馈均为减速后的输出轴rpm。
constexpr uint16_t FeederFeedbackId = 0x204;
constexpr uint16_t CommandId = 0x200;
constexpr float FeederGearRatio = 36.0f;
constexpr float FeederTargetRpm = 150.0f;
// 沿用本机 Shoot/User/Shooter/ShooterMotors.cpp 的速度环初值。
// Ki按每个2ms控制周期累计，PID输出直接为CAN电流编码值。
constexpr float FeederKp = 20.0f;
constexpr float FeederKi = 1.0f;
constexpr float FeederKd = 0.0f;
constexpr float FeederCurrentMax = 9600.0f;
}

namespace timing {
// 两轴当前使用的超时和重试参数，整理时保持原值。
constexpr uint32_t ImuTimeoutMs = 20;
constexpr uint32_t FeedbackTimeoutMs = 100;
constexpr uint32_t EnableTimeoutMs = 300;
constexpr uint32_t EnablePeriodMs = 20;
constexpr uint32_t DisablePeriodMs = 100;
constexpr uint32_t MaxControlDtMs = 50;
constexpr float MaxControlDtSeconds = MaxControlDtMs / 1000.0f;
constexpr float MaxAbsImuRollDeg = 85.0f; // 原欧拉角有效性判定，边界不包含
constexpr uint32_t TelemetryPeriodMs = 20; // VOFA+ 四通道输出周期
}
}
