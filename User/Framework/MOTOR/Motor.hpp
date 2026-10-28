//
// Created by RedogJrome on 2026/3/6.
//

#ifndef INC_2026_BALANCED_GIMBAL_MOTOR_HPP
#define INC_2026_BALANCED_GIMBAL_MOTOR_HPP
#include "Wdg.hpp"
#include "cstdint"
#include "Definitions.hpp"
#include "Gm6020Feedback.hpp"
class Motor_6020_t
{
public:

    uint32_t ID = 0;

    int16_t RawEcd = 0;
    uint8_t RawTemperature = 0;
    int16_t RawTorque = 0; // 兼容旧字段名；6020这里是原始转矩电流
    int16_t RawSpe = 0;    // rpm

    float Torque = 0; // 未标定，不由原始电流直接换算
    float Spe = 0;    // 电机轴 rad/s，不套用旧减速比

    float TargetTorque = 0;

    int16_t Output = 0;
    volatile uint32_t FeedbackCount = 0;
    volatile uint32_t LastFeedbackMs = 0;
    volatile bool FeedbackSeen = false;


    float ZeroPoint;
    float Angle = 0; // 编码器单圈 rad，尚未标定机械零点

    Motor_6020_t(float zp): ZeroPoint(zp)
    {}
    void Update_Info(uint8_t* data);
    bool Update_Feedback(uint32_t std_id, const uint8_t *data, uint32_t length);
    void Set_Target_Torque(float target_torque);
};

class Motor_3508_t
{
public:

    Wdg_t Wdg;

    uint32_t ID;
    int16_t RawEcd;
    int8_t RawTemperature;
    int16_t RawTorque;
    int16_t RawSpe;

    float Torque;
    float Spe;

    float TargetTorque;

    int16_t Output;

    void Update_Info(uint8_t* data);
    void Set_Target_Torque(float target_torque);
};

class Motor_4310_t
{

    public:
    float TargetTorque;

    int16_t RawEcd;
    int8_t RawTemperature;
    int16_t RawTorque;
    int16_t RawSpe;


    int16_t Output;
    void Update_Info(uint8_t* data);
    void Set_Target_Torque(float target_torque);

};

#endif //INC_2026_BALANCED_GIMBAL_MOTOR_HPP
