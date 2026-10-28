//
// Created by RedogJrome on 2026/3/31.
//

#include "Lqr.hpp"

SimpleLQR Yaw_LQR;
SimpleLQR Pitch_LQR;

void SimpleLQR::
Init(float J_, float b_, float dt_)
{
    J = J_;
    b = b_;
    dt = dt_;

    // 离散系统
    // x = [theta, omega]
    // theta(k+1) = theta + dt * omega
    // omega(k+1) = omega + dt * ( -b/J * omega + 1/J * tau )
    float A00 = 1.0f;
    float A01 = dt;
    float A10 = 0.0f;
    float A11 = 1.0f - dt * b / J;

    float B0 = 0.0f;
    float B1 = dt / J;

    // Riccati 迭代
    float P00 = Q[0];
    float P01 = 0.0f;
    float P10 = 0.0f;
    float P11 = Q[1];

    for (int i = 0; i < 200; i++) {
        float BtP0 = B0 * P00 + B1 * P10;
        float BtP1 = B0 * P01 + B1 * P11;
        float S = R + BtP0 * B0 + BtP1 * B1;
        float invS = 1.0f / S;

        float AtP00 = A00 * P00 + A10 * P10;
        float AtP01 = A00 * P01 + A10 * P11;
        float AtP10 = A01 * P00 + A11 * P10;
        float AtP11 = A01 * P01 + A11 * P11;

        float AtP_A00 = AtP00 * A00 + AtP01 * A10;
        float AtP_A01 = AtP00 * A01 + AtP01 * A11;
        float AtP_A10 = AtP10 * A00 + AtP11 * A10;
        float AtP_A11 = AtP10 * A01 + AtP11 * A11;

        float AtP_B0 = AtP00 * B0 + AtP01 * B1;
        float AtP_B1 = AtP10 * B0 + AtP11 * B1;

        float newP00 = AtP_A00 - AtP_B0 * invS * AtP_B0 + Q[0];
        float newP01 = AtP_A01 - AtP_B0 * invS * AtP_B1;
        float newP10 = AtP_A10 - AtP_B1 * invS * AtP_B0;
        float newP11 = AtP_A11 - AtP_B1 * invS * AtP_B1 + Q[1];

        P00 = newP00;
        P01 = newP01;
        P10 = newP10;
        P11 = newP11;
    }

    // K = (R + B'PB)^-1 * B'PA
    float BtP0 = B0 * P00 + B1 * P10;
    float BtP1 = B0 * P01 + B1 * P11;
    float S = R + BtP0 * B0 + BtP1 * B1;
    float invS = 1.0f / S;

    float BtPA0 = BtP0 * A00 + BtP1 * A10;
    float BtPA1 = BtP0 * A01 + BtP1 * A11;

    K[0] = invS * BtPA0;
    K[1] = invS * BtPA1;
}


float SimpleLQR::Step(float theta, float omega, float theta_ref, float omega_ref)
{
    float e_theta = theta - theta_ref;
    float e_omega = omega - omega_ref;


    float u = -(K[0] * e_theta + K[1] * e_omega);

    if (u > torque_limit)  u = torque_limit;
    if (u < -torque_limit) u = -torque_limit;

    return u;
}