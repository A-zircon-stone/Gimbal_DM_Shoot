//
// Created by RedogJrome on 2026/3/13.
//

#ifndef INC_2026_BALANCED_GIMBAL_AUTOAIM_HPP
#define INC_2026_BALANCED_GIMBAL_AUTOAIM_HPP
#include "Wdg.hpp"


class AutoAim_t
{
    public:
    Wdg_t wdg;

    // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    uint8_t Symbol = 0;//打符用的标志位;
    uint8_t sy_tim;
    uint8_t Mode = 0;     //0:不控制 1:控制 （使能）
    uint8_t Fire = 0;     //0:不开火 1:开火

    float Yaw_Angle;
    float Yaw_Spe;
    float Yaw_Acc;


    float Pitch_Angle;
    float Pitch_Spe;
    float Pitch_Acc;

    uint8_t shoot_count_From_MiniPC = 0;
    uint8_t shoot_count_Now = 0;

};

extern AutoAim_t AutoAim;

#endif //INC_2026_BALANCED_GIMBAL_AUTOAIM_HPP