//
// Created by RedogJrome on 2026/3/31.
//
#pragma once

#ifndef INC_2026_BALANCED_GIMBAL_LQR_HPP
#define INC_2026_BALANCED_GIMBAL_LQR_HPP

#pragma once

class SimpleLQR {
public:
    // 你可以直接在外面改
    float Q[2] = {1.0f, 1.0f};   // Q = diag(Q[0], Q[1])
    float R    = 1.0f;            // 标量

    // 算出来的增益
    float K[2] = {0.0f, 0.0f};

    // 参数
    float J = 0.01f;
    float b = 0.1f;
    float dt = 0.001f;
    float torque_limit = 4.0f;

public:
    // 初始化 or 改完Q/R后重新调用
    void Init(float J_, float b_, float dt_);

    float Step(float theta, float omega, float theta_ref, float omega_ref = 0.0f);
};

extern SimpleLQR Yaw_LQR;
extern SimpleLQR Pitch_LQR;


#endif //INC_2026_BALANCED_GIMBAL_LQR_HPP