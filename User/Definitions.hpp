//
// Created by RedogJrome on 2026/3/6.
//

#ifndef INC_2026_BALANCED_GIMBAL_DEFINITIONS_HPP
#define INC_2026_BALANCED_GIMBAL_DEFINITIONS_HPP

#ifndef PI
#define PI 3.14159265358979323846f

#define PITCH_ANGLE_MAX 0.40f
#define PITCH_ANGLE_MIN -0.30f //  都是弧度

#define PITCH_ENCODER_MAX 0.0f;
#define PITCH_ENCODER_MIN 0.0f;



#endif
#include "cstdint"


enum class OnlineState_t :int8_t { OFFLINE = 0, ONLINE = 1 };

inline float LowPassFilter(float x, float last_x, float gamma)
{
    return gamma * x + (1.0f - gamma) * last_x;
}

inline float Limit(float x,float a,float b)
{
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

//角度底噪不超过0.01

#endif //INC_2026_BALANCED_GIMBAL_DEFINITIONS_HPP