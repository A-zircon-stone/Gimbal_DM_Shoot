#include "PitchTilt.hpp"
#include <cmath>

PitchAxisFeedback map_pitch_feedback(const PitchTilt &tilt, float gyro_y_rad_s,
                                      uint16_t encoder, bool encoder_fresh)
{
    PitchAxisFeedback result{};
    result.encoder = encoder;
    if (!tilt.valid || !tilt.initialized || !std::isfinite(tilt.continuous_deg) ||
        !std::isfinite(gyro_y_rad_s) || !encoder_fresh || encoder > 8191)
        return result;
    result.angle_rad = -tilt.continuous_deg * 0.017453292519943295f;
    result.speed_rad_s = -gyro_y_rad_s;
    if (!std::isfinite(result.angle_rad)) return PitchAxisFeedback{};
    result.valid = true;
    result.inside_soft_limits = encoder > PitchMechanism::SoftMin &&
                                encoder < PitchMechanism::SoftMax;
    // 超出用户给定机械范围时，两方向都禁止，不能自动尝试回位。
    if (encoder >= PitchMechanism::EncoderMin && encoder <= PitchMechanism::EncoderMax)
    {
        result.allow_raise = encoder > PitchMechanism::SoftMin;
        result.allow_lower = encoder < PitchMechanism::SoftMax;
    }
    return result;
}

bool update_pitch_tilt(PitchTilt *state, const float q[4])
{
    if (!state) return false;
    auto reject = [state]() {
        state->valid = false;
        state->initialized = false;
        return false;
    };
    if (!q) return reject();
    for (unsigned i = 0; i < 4; ++i)
        if (!std::isfinite(q[i])) return reject();
    const float norm2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (!std::isfinite(norm2) || norm2 < 1e-12f) return reject();
    // g_b = R(q)^T [0,0,1]：取融合姿态方向，不直接用瞬时加速度。
    const float gx = 2.0f * (q[1]*q[3] - q[0]*q[2]) / norm2;
    const float gz = (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]) / norm2;
    const float projection2 = gx*gx + gz*gz;
    // Y轴接近重力方向时XZ投影不可靠；0.25为几何门限，不是机械限位。
    if (!std::isfinite(projection2) || projection2 < 0.25f*0.25f) return reject();
    const float wrapped = std::atan2(-gx, gz) * 57.295779513f;
    float continuous = wrapped;
    if (state->initialized)
    {
        if (!std::isfinite(state->wrapped_deg) || !std::isfinite(state->continuous_deg))
            return reject();
        float delta = wrapped - state->wrapped_deg;
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        continuous = state->continuous_deg + delta;
    }
    if (!std::isfinite(wrapped) || !std::isfinite(continuous)) return reject();
    state->wrapped_deg = wrapped;
    state->continuous_deg = continuous;
    state->initialized = true;
    state->valid = true;
    return true;
}
