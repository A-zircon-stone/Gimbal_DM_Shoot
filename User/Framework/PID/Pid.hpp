//
// Created by 曹锐 on 2025/12/16.
//

#ifndef PID_HPP
#define PID_HPP


class Pid_t
{
public:
    float Kp;
    float Ki;
    float Kd;

    float Error;
    float Error_tot;
    float Val_last;
    float delta;
    float Output;

    void Init(float tkp, float tki, float tkd);
    void Cal(float purpose, float val_now, float maxx);
    void Clear();
};


#endif //PID_HPP