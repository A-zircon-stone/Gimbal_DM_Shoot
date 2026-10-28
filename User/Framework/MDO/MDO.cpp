//
// Created by RedogJrome on 2026/4/25.
//

#include "MDO.hpp"

SimpleMDO Yaw_MDO;
SimpleMDO Pitch_MDO;
SimpleMDO Left_Fric_MDO,Right_Fric_MDO;


void SimpleMDO::Init(float J_, float b_, float dt_, float bandwidth_hz)
{
    J = J_;
    b = b_;
    dt = dt_;
    // 将 Hz 转换为 rad/s 作为观测器增益
    // 增益越大，观测越快，但高频噪声越大
    K_obs = 2.0f * 3.14159265f * bandwidth_hz;

    Reset();
}

void SimpleMDO::Reset()
{
    I_state = 0.0f;
    d_est = 0.0f;
}

float SimpleMDO::Step(float omega, float last_tau_cmd)
{
    // 1. 计算当前系统实际角动量
    float p = J * omega;

    // 2. 根据实际动量与内部积分状态的偏差，更新扰动估计
    // 公式: d_est = Ko * (p - I_state)
    d_est = K_obs * (p - I_state);

    // 物理防爆限幅：防止云台被卡死时估算出一个能烧毁电机的扰动
    if (d_est > dist_limit)       d_est = dist_limit;
    else if (d_est < -dist_limit) d_est = -dist_limit;

    // 3. 更新内部积分状态 (欧拉前向积分)
    // 模拟纯理论情况下的动量变化律：I(k+1) = I(k) + dt * (tau - b*omega + d_est)
    I_state += dt * (last_tau_cmd - b * omega + d_est);

    return d_est;
}