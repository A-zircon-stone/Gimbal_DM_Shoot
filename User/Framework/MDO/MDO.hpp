//
// Created by RedogJrome on 2026/4/25.
//

#ifndef INC_2026_BALANCED_GIMBAL_MDO_HPP
#define INC_2026_BALANCED_GIMBAL_MDO_HPP

class SimpleMDO {
public:
    // 物理参数 (必须与 LQR 保持绝对一致)
    float J = 0.01f;
    float b = 0.1f;
    float dt = 0.001f;

    // 观测器参数
    float K_obs = 100.0f;       // 观测器增益 (带宽, rad/s)
    float dist_limit = 3.0f;    // 扰动估算值的物理限幅

    // 内部状态
    float I_state = 0.0f;       // 内部积分动量状态 (Gamma)
    float d_est = 0.0f;         // 当前估算出的外部扰动力矩

public:
    // 初始化，bandwidth_hz 建议设置在 10Hz - 40Hz 之间
    void Init(float J_, float b_, float dt_, float bandwidth_hz);

    // 观测器步进
    // @param omega: 当前实际角速度
    // @param last_tau_cmd: 上一控制周期实际下发给电机的总力矩指令
    // @return: 估算出的扰动力矩 d_est
    float Step(float omega, float last_tau_cmd);

    // 云台重新使能或复位时调用，防止积分状态爆炸
    void Reset();
};

extern SimpleMDO Yaw_MDO;
extern SimpleMDO Pitch_MDO;
extern SimpleMDO Left_Fric_MDO,Right_Fric_MDO;


#endif //INC_2026_BALANCED_GIMBAL_MDO_HPP