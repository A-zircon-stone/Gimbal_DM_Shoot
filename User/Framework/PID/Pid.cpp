//
// Created by 曹锐 on 2025/12/16.
//

#include "Pid.hpp"
#include "Definitions.hpp"
#include <algorithm>
#include <cmath>

namespace {
bool valid_gains(const PidGains &g) {
    return std::isfinite(g.kp) && g.kp >= 0 && std::isfinite(g.ki) && g.ki >= 0 &&
           std::isfinite(g.kd) && g.kd >= 0;
}
float bounded(float value, float limit) {
    return std::max(-limit, std::min(limit, value));
}
}

bool Pid_t::SetGains(const PidGains &gains) {
    if (!valid_gains(gains)) return false;
    Kp = gains.kp; Ki = gains.ki; Kd = gains.kd;
    return true;
}

bool Pid_t::Calculate(const PidInput &in, const PidLimits &limits) {
    if (!valid_gains({Kp,Ki,Kd}) || !std::isfinite(in.target) ||
        !std::isfinite(in.measured) || !std::isfinite(in.dt) || in.dt <= 0 ||
        !std::isfinite(in.feedforward) ||
        (!std::isnan(in.measured_rate) && !std::isfinite(in.measured_rate)) ||
        !(limits.output >= 0) || !(limits.integral >= 0)) return false;

    const float error = in.target - in.measured;
    const float rate = !std::isnan(in.measured_rate) ? in.measured_rate :
        (measurement_ready_ && Kd != 0 ? (in.measured-previous_measurement_)/in.dt : 0);
    const float next_integral_raw = Integral + Ki * error * in.dt;
    if (!std::isfinite(error) || !std::isfinite(rate) || !std::isfinite(next_integral_raw)) return false;
    const float next_integral = bounded(next_integral_raw, limits.integral);
    const float pd = Kp * error - Kd * rate;
    const float candidate = pd + next_integral + in.feedforward;
    if (!std::isfinite(candidate)) return false;
    const bool accept_integral = std::fabs(candidate) <= limits.output ||
        (candidate > limits.output && error < 0) || (candidate < -limits.output && error > 0);
    const float integral = accept_integral ? next_integral : Integral;
    const float total = pd + integral + in.feedforward;
    if (!std::isfinite(total)) return false;

    Error = error;
    Integral = integral;
    Output = bounded(total, limits.output);
    previous_measurement_ = in.measured;
    measurement_ready_ = true;
    return true;
}

void Pid_t::Cal(float purpose, float val_now, float maxx)
{
    Error = purpose - val_now;
    Error_tot += Error;
    delta = LowPassFilter(val_now - Val_last, delta, 0.2);
    Output = Kp * Error + Ki * Error_tot - Kd * delta;
    if (Output > maxx)
    {
        Output = maxx;
        Error_tot -= Error;
    }
    else if (Output < -maxx)
    {
        Output = -maxx;
        Error_tot -= Error;
    }
    Val_last = val_now;
}

void Pid_t::Clear()
{
    Error = 0.0f;
    Error_tot = 0.0f;
    Val_last = 0.0f;
    Output = 0.0f;
    delta = 0.0f;
    Integral = 0.0f;
    previous_measurement_ = 0.0f;
    measurement_ready_ = false;
}

void Pid_t::Init(float tkp, float tki, float tkd)
{
    Clear();
    Kp = tkp;
    Ki = tki;
    Kd = tkd;
}

