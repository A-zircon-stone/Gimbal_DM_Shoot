#pragma once
#include <cstddef>
#include <cstdint>

constexpr std::size_t PrintTelemetryCapacity = 512;
constexpr uint32_t PrintTelemetryPeriodMs = 100;
// 用户通过USART1连接VOFA+，默认只发FireWater。false仅用于恢复人工标签日志。
constexpr bool PrintUart1FireWater = true;
struct PrintTelemetry
{
    uint32_t time_ms = 0;
    uint32_t imu_sequence = 0;
    bool imu_fresh = false;
    float roll_deg = 0;
    float pitch_deg = 0;
    float yaw_deg = 0;
    float gyro_rad[3] = {};
    float q[4] = {}; // Hamilton wxyz，仅用于显示快照，不修改INS状态
    float pitch_y_deg = 0; // TY：传感器Y轴倾斜辅助角，不是已标定机械Pitch
    bool pitch_y_valid = false;
    uint16_t yaw_encoder = 0;
    uint16_t pitch_encoder = 0;
    int16_t yaw_rpm = 0;
    int16_t pitch_rpm = 0;
    bool yaw_online = false;
    bool pitch_online = false;
    // 追加诊断，不改变VOFA现有0..20列绑定。
    float ekf_bias[3] = {};
    float ekf_gyro_rad[3] = {};
    float temperature_c = 0;
    float sample_dt_s = 0;
    float accel_mps2[3] = {};
    float quaternion_norm = 0;
    float accel_norm_mps2 = 0;
    uint8_t pitch_test_state = 0; // 34: 0闲置1武装2运行3完成4拒绝5故障6停止
    int16_t pitch_test_command = 0; // 35: CAN_Demo的原始电流给定，未核实安培换算
    int16_t pitch_test_travel = 0; // 36: EP相对脉冲起点的变化
    uint8_t pitch_hold_state=0; // 37: 0关闭1闭环2保留3拒绝4故障
    float pitch_control_deg=0; // 38: 抬头为正，尚未机械归零
    float pitch_target_deg=0; // 39: 使能时捕获的IMU角度
    float pitch_speed_deg_s=0; // 40: 抬头为正
    int16_t pitch_current_command=0; // 41: 本轮最终命令
    int16_t pitch_current_feedback=0; // 42: 电机回报的原始转矩电流
    int16_t yaw_current_command=0;
    int16_t yaw_current_feedback=0;
    float yaw_control_deg=0; // 连续Yaw角，与当前控制器反馈一致
    float yaw_target_deg=0; // 当前Yaw控制器的位置目标

};
// FireWater六通道：Yaw当前/目标/误差、Pitch当前/目标/误差，误差=target-actual，单位deg。
// Pitch沿用原始IMU Roll符号（抬头减小），与target_roll_deg直接比较。
std::size_t format_gimbal_angles(char *out,std::size_t capacity,const PrintTelemetry &data);
std::size_t format_commissioning_telemetry(char *out,std::size_t capacity,
                                         const PrintTelemetry &data);
// 返回有效字节数（不含终止零）；参数非法或空间不足时返回0，不发送截断行。
std::size_t format_print_telemetry(char *out, std::size_t capacity,
                                 const PrintTelemetry &data);
// USART1/USB默认精简FireWater CSV：Pitch角度/目标/给定电流/反馈电流/EP/角速度/状态。
// 不再输出原43列四元数等诊断；状态-1反馈无效（非有限角值替换0，不能视为真零）。
std::size_t format_vofa_telemetry(char *out, std::size_t capacity,
                                const PrintTelemetry &data);
// INS任务完成一整帧后调用；仅复制数据，不做格式化或I/O。
// imucheck 三通道：原始 INS Roll、Pitch、Yaw，单位度；无有效新帧时不输出。
std::size_t format_imu_attitude(char *out, std::size_t capacity, const PrintTelemetry &data);
void publish_print_imu(float roll_deg, float pitch_deg, float yaw_deg,
                       const float gyro_rad[3], const float q[4]);
