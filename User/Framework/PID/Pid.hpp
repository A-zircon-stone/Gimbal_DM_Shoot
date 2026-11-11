//
// Created by 曹锐 on 2025/12/16.
//

#ifndef PID_HPP
#define PID_HPP
#include <limits>

struct PidGains {
    float kp = 0, ki = 0, kd = 0;
};

struct PidLimits {
    // 默认不限制；具体轴显式传入已有的输出/积分范围。
    float output = std::numeric_limits<float>::infinity();
    float integral = std::numeric_limits<float>::infinity();
};

struct PidInput {
    float target = 0;
    float measured = 0;
    float dt = 0; // 秒
    float feedforward = 0; // 与PID输出同单位
    // 有同源导数时直接提供，例如位置外环使用IMU角速度。
    // 未提供时，由连续两次measured和dt计算；首帧导数为0。
    float measured_rate = std::numeric_limits<float>::quiet_NaN();
};


class Pid_t
{
public:
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;

    float Error = 0;
    float Error_tot = 0;
    float Val_last = 0;
    float delta = 0;
    float Output = 0;
    float Integral = 0; // Calculate接口的积分输出，已包含Ki

    void Init(float tkp, float tki, float tkd);
    // 原有逐周期接口：保留摩擦轮/M2006的增益与滤波语义。
    void Cal(float purpose, float val_now, float maxx);
    // 实际dt接口：P + I - Kd*测量导数 + 前馈；包含总输出抗积分饱和。
    // 失败时不修改历史和输出，调用方沿用其故障处理。
    bool Calculate(const PidInput &input, const PidLimits &limits = {});
    bool SetGains(const PidGains &gains); // 在线调参不重置积分/目标
    void Clear();
private:
    float previous_measurement_ = 0;
    bool measurement_ready_ = false;
};


#endif //PID_HPP
