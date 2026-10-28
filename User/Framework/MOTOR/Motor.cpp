//
// Created by RedogJrome on 2026/3/6.
//

#include "Motor.hpp"
#include "CANio.hpp"
#include "Definitions.hpp"
#include "usart.h"
#include "Motor.hpp"

void Motor_3508_t::Update_Info(uint8_t* data)
{
    RawEcd = (data[0] << 8) | data[1];
    RawSpe = (data[2] << 8) | data[3];
    RawTorque = (data[4] << 8) | data[5];
    RawTemperature = data[6];

    Spe = LowPassFilter(RawSpe, Spe, 0.05f);
    Torque = LowPassFilter(RawTorque ,Torque,0.2f);
    Wdg.clear_wdg();
}

void Motor_3508_t::Set_Target_Torque(float target_torque)
{
    Output = static_cast<int16_t> (target_torque / 0.3 *819.2);
    if (Output > 16384) Output = 16384;
    if (Output < -16384) Output = -16384;
}

void Motor_6020_t::Update_Info(uint8_t* data)
{
    Update_Feedback(0x204u + ID, data, 8);
}

bool Motor_6020_t::Update_Feedback(uint32_t std_id, const uint8_t *data, uint32_t length)
{
    if (ID < 1 || ID > 7) return false;
    Gm6020Feedback feedback{};
    if (!gm6020_decode(std_id, static_cast<uint8_t>(ID), data, length, feedback))
        return false;
    RawEcd = static_cast<int16_t>(feedback.encoder);
    RawSpe = feedback.speed_rpm;
    RawTorque = feedback.current_raw;
    RawTemperature = feedback.temperature_c;
    Spe = RawSpe * (2.0f * PI / 60.0f);
    Angle = RawEcd * (2.0f * PI / 8192.0f);
    LastFeedbackMs = HAL_GetTick();
    FeedbackSeen = true;
    ++FeedbackCount;
    return true;
}

void Motor_6020_t::Set_Target_Torque(float target_torque)
{
    if (GimbalHardware::FeedbackOnly)
    {
        TargetTorque = 0;
        Output = 0;
        return;
    }
    Output = static_cast<int16_t> (target_torque / 0.3 *819.2);
    if (Output > 16384) Output = 16384;
    if (Output < -16384) Output = -16384;
}

void Motor_4310_t::Set_Target_Torque(float target_torque)
{
    TargetTorque = Limit(target_torque,-4.0,4.0);//限幅度4.0nm
}
void Motor_4310_t::Update_Info(uint8_t* data)
{

    RawEcd = (data[0] << 8) | data[1];
    RawSpe = (data[2] << 8) | data[3];
    RawTorque = (data[4] << 8) | data[5];
    RawTemperature = data[6];

    ///每1000能产生0.59157Nm
    Output = static_cast<int16_t> (TargetTorque / 0.59157f * 1000.0f);
    Cmd_Gimbal_Pitch(Output);
}
