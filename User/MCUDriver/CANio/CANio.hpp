//
// Created by RedogJrome on 2026/3/6.
//

#ifndef INC_2026_BALANCED_GIMBAL_CANIO_HPP
#define INC_2026_BALANCED_GIMBAL_CANIO_HPP
#include "can.h"

void can_filter_init();
HAL_StatusTypeDef SendMessage_8(CAN_HandleTypeDef *hcan,uint32_t stdid,uint8_t *data);
HAL_StatusTypeDef Cmd_Gimbal_Pitch(int16_t motor);
// 只允许反馈模式内人工短脉冲测试。不是旧电机力矩接口。
HAL_StatusTypeDef Cmd_Pitch_Direction_Test(int16_t raw_current);
HAL_StatusTypeDef Cmd_Gimbal_Raw_Current(int16_t yaw_current,int16_t pitch_current);


#endif //INC_2026_BALANCED_GIMBAL_CANIO_HPP
