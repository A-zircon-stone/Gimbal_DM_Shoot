//
// Created by 曹锐 on 2025/12/16.
//

#include "Pid.hpp"
#include "Definitions.hpp"

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
}

void Pid_t::Init(float tkp, float tki, float tkd)
{
    Clear();
    Kp = tkp;
    Ki = tki;
    Kd = tkd;
}

