//
// Created by RedogJrome on 2026/3/6.
//
#include "CANio.hpp"

#include <cstring>

#include "Motor.hpp"
#include "can.h"
#include "Gimbal.hpp"
#include "usart.h"
#include "PitchDirectionTest.hpp"

HAL_StatusTypeDef Cmd_Pitch_Direction_Test(int16_t raw_current)
{
    return Cmd_Gimbal_Raw_Current(0,raw_current);
}

HAL_StatusTypeDef Cmd_Gimbal_Raw_Current(int16_t yaw_current,int16_t raw_current)
{
    if(GimbalHardware::PitchSpeedTuneMode && GimbalHardware::PitchSpeedDryRun)
        return HAL_OK; // 空板演算禁止提交电流帧，返回值仅表示已跳过发送
    constexpr int16_t pitch_limit=GimbalHardware::PitchSpeedTuneMode ? PitchImuHold::CurrentLimit :
        (GimbalHardware::CommissioningMode && GimbalCommissioning::ContinuousCurrentTest ?
         GimbalCommissioning::ContinuousPitchCurrent : PitchDirectionTest::MaxAmplitude);
    constexpr int16_t yaw_limit=GimbalHardware::PitchSpeedTuneMode ? YawSpeedTune::CurrentLimit : PitchDirectionTest::MaxAmplitude;
    if (!GimbalHardware::FeedbackOnly || !GimbalHardware::PitchDirectionTestEnabled ||
        raw_current < -pitch_limit || raw_current > pitch_limit ||
        yaw_current < -yaw_limit || yaw_current > yaw_limit)
        return HAL_ERROR;
    // 零命令前先取消仍在邮箱中的旧脉冲，避免停车后延迟发出旧非零命令。
    if(raw_current==0 && yaw_current==0)
        (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
    if(HAL_CAN_GetError(&hcan1)!=HAL_CAN_ERROR_NONE ||
       HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0)
    {
        (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
        // 仍尝试发送停车帧；非零帧出错则交给上层锁定。
        if(raw_current!=0 || yaw_current!=0) return HAL_ERROR;
    }
    uint8_t bytes[8];
    gm6020_encode_gimbal_current(yaw_current,raw_current,bytes);
    CAN_TxHeaderTypeDef header{};
    header.StdId=PitchDirectionTest::CommandId;
    header.IDE=CAN_ID_STD; header.RTR=CAN_RTR_DATA; header.DLC=8;
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(&hcan1, &header, bytes, &mailbox);
}

void can_filter_init()
{
    // 在打开接收中断前绑定电机ID，避免启动早期反馈被错误分配。
    Gimbal.Motor_Yaw.ID = GimbalHardware::YawId;
    Gimbal.Motor_Pitch.ID = GimbalHardware::PitchId;
    CAN_FilterTypeDef can_filter_st{};
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank=0;
    can_filter_st.FilterFIFOAssignment=CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();

    if (GimbalHardware::FeedbackOnly)
        return; // 当前硬件两台电机均在CAN1，不启动旧CAN2命令链路

    can_filter_st.FilterBank=14;

    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

HAL_StatusTypeDef Cmd_Gimbal_Pitch(int16_t motor)
{
    if (GimbalHardware::FeedbackOnly) return HAL_ERROR;
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef motor_tx_message;
    uint8_t motor_can_send_data[8]={0,};

    motor_tx_message.StdId=0x4FE;
    motor_tx_message.IDE=CAN_ID_STD;
    motor_tx_message.RTR=CAN_RTR_DATA;
    motor_tx_message.DLC=0X02;

    motor_can_send_data[0]=motor;
    motor_can_send_data[1]=motor>>8;

    return HAL_CAN_AddTxMessage(&hcan1,&motor_tx_message,motor_can_send_data,&send_mail_box);
}

HAL_StatusTypeDef Cmd_Gimbal_Fric(uint32_t stdid,int16_t motor1,int16_t motor2)
{
    if (GimbalHardware::FeedbackOnly) return HAL_ERROR;
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef motor_tx_message;
    uint8_t motor_can_send_data[8]={0,};

    motor_tx_message.StdId=stdid;
    motor_tx_message.IDE=CAN_ID_STD;
    motor_tx_message.RTR=CAN_RTR_DATA;
    motor_tx_message.DLC=0X04;

    motor_can_send_data[0]=motor1>>8;
    motor_can_send_data[1]=motor1;
    motor_can_send_data[2]=motor2>>8;
    motor_can_send_data[3]=motor2;

    return HAL_CAN_AddTxMessage(&hcan1,&motor_tx_message,motor_can_send_data,&send_mail_box);
}

HAL_StatusTypeDef SendMessage_8(CAN_HandleTypeDef *hcan,uint32_t stdid,uint8_t *data)
{
    if (GimbalHardware::FeedbackOnly) return HAL_ERROR;
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t can_send_data[8]={0,};

    tx_message.StdId=stdid;
    tx_message.IDE=CAN_ID_STD;
    tx_message.RTR=CAN_RTR_DATA;
    tx_message.DLC=8;
    memcpy(can_send_data,data,8);
    return HAL_CAN_AddTxMessage(hcan,&tx_message,can_send_data,&send_mail_box);
}

void  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return;
    if (rx_header.IDE != CAN_ID_STD || rx_header.RTR != CAN_RTR_DATA ||
        rx_header.DLC != 8)
        return;

    if (hcan == &hcan1)
    {
        if (rx_header.StdId == 0x204u + GimbalHardware::YawId)
            Gimbal.Motor_Yaw.Update_Feedback(rx_header.StdId, rx_data, rx_header.DLC);
        else if (rx_header.StdId == 0x204u + GimbalHardware::PitchId)
            Gimbal.Motor_Pitch.Update_Feedback(rx_header.StdId, rx_data, rx_header.DLC);
    }
    if (GimbalHardware::FeedbackOnly)
        return; // 接收中断绝不顺带发送旧摩擦轮命令
    if (rx_header.StdId == 0x202) //左边摩擦轮
    {
        Gimbal.Motor_Left_Fric.Update_Info(rx_data);
    }
    else if (rx_header.StdId == 0x201) //右边摩擦轮
    {
        Gimbal.Motor_Right_Fric.Update_Info(rx_data);
    }

    else if (rx_header.StdId == 0x305) //Pitch电机
    {
        Gimbal.Motor_Pitch.Update_Info(rx_data);
    }

    else if (rx_header.StdId == 0x777) //底盘给头发的东西
    {
        Gimbal.Chassis_Status = rx_data[0];//底盘的状态
        Gimbal.Robot_Color = rx_data[1];
        Gimbal.Robot_HP = rx_data[2] | (rx_data[3]<<8);
        Gimbal.Head_Direction = rx_data[4];
        Gimbal.Refree_Elc = rx_data[5];
    }
    Cmd_Gimbal_Fric(0x200,Gimbal.Motor_Right_Fric.Output,Gimbal.Motor_Left_Fric.Output);
}
