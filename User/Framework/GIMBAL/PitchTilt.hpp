#pragma once
#include <cstdint>

// 轴向验证辅助量：参考系+Z在传感器XZ平面的投影角。
// 仅当机械Pitch轴与传感器Y轴对齐且该轴不接近竖直时，才是Pitch候选量。
// 不是通用ZYX欧拉Pitch，不是机械零位；当前不参与电机输出。
struct PitchTilt
{
    float wrapped_deg = 0;
    float continuous_deg = 0;
    bool initialized = false;
    bool valid = false;
};

// Hamilton wxyz，q为传感器->参考系旋转。相邻有效采样角变化须<180°。
// 失败保留上次角值、清有效性及连续性；恢复后从单圈角重新建立参考。
bool update_pitch_tilt(PitchTilt *state, const float q[4]);

namespace PitchMechanism
{
constexpr uint16_t EncoderMin = 3500;
constexpr uint16_t EncoderMax = 6400;
// 临时保守保护带，单位为编码器计数；不是已验证的制动距离。
constexpr uint16_t GuardCounts = 50;
constexpr uint16_t SoftMin = EncoderMin + GuardCounts;
constexpr uint16_t SoftMax = EncoderMax - GuardCounts;
}

// 映射预览，不是电机使能条件，也不产生电机命令。
// 抬头正：angle=-TY（无机械零偏），speed=-gy。
struct PitchAxisFeedback
{
    float angle_rad = 0;
    float speed_rad_s = 0;
    uint16_t encoder = 0;
    bool valid = false;
    bool inside_soft_limits = false;
    bool allow_raise = false; // 抬头使编码器减小
    bool allow_lower = false;
};

PitchAxisFeedback map_pitch_feedback(const PitchTilt &tilt, float gyro_y_rad_s,
                                      uint16_t encoder, bool encoder_fresh);
