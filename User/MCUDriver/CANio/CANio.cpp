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
#include "DamiaoMailboxOwners.hpp"
#include "ShooterCan.hpp"

namespace {
// MIT 的接收观察器保留，但不参与 V41 一控四的接收流程。
#if 0
damiao_can::FeedbackMonitor damiao_feedback_monitor;
#endif
// pitch 已换成标准固件；保留 V41 接收器以供后续参考。
#if 0
damiao_v41::FeedbackMonitor damiao_v41_feedback_monitor;
#endif
damiao_mit_pitch::FeedbackMonitor damiao_mit_pitch_feedback_monitor;
damiao_mit_yaw::FeedbackMonitor damiao_mit_yaw_feedback_monitor;
}

#if 0 // 之前的 MIT 快照实现，保留供以后参考。
damiao_can::FeedbackSnapshot damiao_feedback_snapshot()
{
    // 保存原中断状态，避免调用方原本已关中断时被错误地重新打开。
    const uint32_t previous_mask = __get_PRIMASK();
    __disable_irq();
    const auto snapshot = damiao_feedback_monitor.snapshot();
    __set_PRIMASK(previous_mask);
    return snapshot;
}
#endif

#if 0 // V41 快照实现保留，不用于现在的 pitch 电机。
damiao_v41::FeedbackSnapshot damiao_v41_feedback_snapshot()
{
    // 防止 CAN 中断在复制一半时更新反馈；恢复调用前的中断状态。
    const uint32_t previous_mask = __get_PRIMASK();
    __disable_irq();
    const auto snapshot = damiao_v41_feedback_monitor.snapshot();
    __set_PRIMASK(previous_mask);
    return snapshot;
}
#endif

damiao_mit_pitch::FeedbackSnapshot damiao_mit_pitch_feedback_snapshot()
{
    const uint32_t previous_mask = __get_PRIMASK();
    __disable_irq(); // 防止接收中断在复制快照期间更新数据
    const auto snapshot = damiao_mit_pitch_feedback_monitor.snapshot();
    __set_PRIMASK(previous_mask); // 恢复调用前状态，不能无条件开中断
    return snapshot;
}

damiao_mit_yaw::FeedbackSnapshot damiao_mit_yaw_feedback_snapshot()
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const auto snapshot = damiao_mit_yaw_feedback_monitor.snapshot();
    __set_PRIMASK(mask);
    return snapshot;
}

namespace {
DamiaoMailboxOwners owners;
// 单帧非阻塞提交，成功仅代表进入邮箱；电机是否使能要以反馈状态确认。
HAL_StatusTypeDef damiao_pitch_submit(const damiao_mit_pitch::CanFrame &frame, bool is_motion)
{
    CAN_TxHeaderTypeDef header{};
    header.StdId = frame.standard_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    uint32_t mailbox = 0;
    // HAL 接口要求可写指针，但本函数不修改调用方的数据。
    uint8_t bytes[8];
    memcpy(bytes, frame.data, sizeof(bytes));
    const HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan1, &header, bytes, &mailbox);
    if (result == HAL_OK) {
        owners.reuse(mailbox);
        if (is_motion) owners.pitch_motion |= mailbox;
        else owners.pitch_disable = mailbox;
    }
    return result;
}
}

HAL_StatusTypeDef damiao_pitch_can_send_enable()
{
    if (!damiao_pitch_can_disable_tx_complete()) return HAL_BUSY;
    damiao_mit_pitch::CanFrame frame{};
    if (!damiao_mit_pitch::make_special_frame(damiao_mit_pitch::SpecialCommand::Enable, frame))
        return HAL_ERROR;
    return damiao_pitch_submit(frame, true);
}

HAL_StatusTypeDef damiao_pitch_can_send_disable()
{
    if (!damiao_pitch_can_disable_tx_complete()) return HAL_BUSY;
    // 先取消邮箱里的旧控制帧，避免失能之后旧命令才发出。
    if (owners.pitch_motion != 0) {
        (void)HAL_CAN_AbortTxRequest(&hcan1, owners.pitch_motion);
        owners.pitch_motion = 0;
    }
    damiao_mit_pitch::CanFrame frame{};
    if (!damiao_mit_pitch::make_special_frame(damiao_mit_pitch::SpecialCommand::Disable, frame))
        return HAL_ERROR;
    // 如邮箱仍忙或总线错误，返回失败，调用方应保持失能状态并周期重试。
    return damiao_pitch_submit(frame, false);
}

bool damiao_pitch_can_disable_tx_complete()
{
    // 只检查 CAN 外设邮箱状态，不能将邮箱空闲等同于电机确认收到。
    if (owners.pitch_disable == 0) return true;
    if (HAL_CAN_IsTxMessagePending(&hcan1, owners.pitch_disable)) return false;
    owners.pitch_disable = 0;
    return true;
}

HAL_StatusTypeDef damiao_pitch_can_send_control(const damiao_mit::Command &command)
{
    damiao_mit_pitch::CanFrame frame{};
    if (!damiao_mit_pitch::make_control_frame(command, frame)) return HAL_ERROR;
    return damiao_pitch_submit(frame, true);
}

namespace {
HAL_StatusTypeDef damiao_yaw_submit(const damiao_mit_yaw::CanFrame &frame, bool motion)
{
    CAN_TxHeaderTypeDef header{};
    header.StdId=frame.standard_id; header.IDE=CAN_ID_STD; header.RTR=CAN_RTR_DATA; header.DLC=8;
    uint32_t mailbox=0;
    uint8_t bytes[8]; std::memcpy(bytes,frame.data,sizeof(bytes));
    const auto result=HAL_CAN_AddTxMessage(&hcan1,&header,bytes,&mailbox);
    if (result == HAL_OK) {
        owners.reuse(mailbox);
        if (motion) owners.yaw_motion |= mailbox;
        else owners.yaw_disable = mailbox;
    }
    return result;
}
}
bool damiao_yaw_can_disable_tx_complete()
{
    if (!owners.yaw_disable) return true;
    if (HAL_CAN_IsTxMessagePending(&hcan1,owners.yaw_disable)) return false;
    owners.yaw_disable=0;
    return true;
}
HAL_StatusTypeDef damiao_yaw_can_send_enable()
{
    if (!damiao_yaw_can_disable_tx_complete()) return HAL_BUSY;
    damiao_mit_yaw::CanFrame frame{};
    if (!damiao_mit_yaw::make_special_frame(damiao_mit_yaw::SpecialCommand::Enable,frame)) return HAL_ERROR;
    return damiao_yaw_submit(frame,true);
}
HAL_StatusTypeDef damiao_yaw_can_send_disable()
{
    if (!damiao_yaw_can_disable_tx_complete()) return HAL_BUSY;
    if (owners.yaw_motion) {
        (void)HAL_CAN_AbortTxRequest(&hcan1,owners.yaw_motion);
        owners.yaw_motion=0;
    }
    damiao_mit_yaw::CanFrame frame{};
    if (!damiao_mit_yaw::make_special_frame(damiao_mit_yaw::SpecialCommand::Disable,frame)) return HAL_ERROR;
    return damiao_yaw_submit(frame,false);
}
HAL_StatusTypeDef damiao_yaw_can_send_control(const damiao_mit::Command &command)
{
    damiao_mit_yaw::CanFrame frame{};
    if (!damiao_mit_yaw::make_control_frame(command,frame)) return HAL_ERROR;
    return damiao_yaw_submit(frame,true);
}
HAL_StatusTypeDef damiao_can_send_torque_frame(
    const damiao::CanFrame& frame)
{
    // 1. 判断报文发给哪台电机
    bool is_pitch =
        frame.id == gimbal_tuning::pitch::MotorCanId;

    bool is_yaw =
        frame.id == gimbal_tuning::yaw::MotorCanId;

    if (!is_pitch && !is_yaw)
    {
        return HAL_ERROR;
    }

    // 2. 沿用现有逻辑，等待之前的失能帧发送完成
    if (is_pitch && !damiao_pitch_can_disable_tx_complete())
    {
        return HAL_BUSY;
    }

    if (is_yaw && !damiao_yaw_can_disable_tx_complete())
    {
        return HAL_BUSY;
    }

    // 3. 填写STM32 HAL需要的CAN帧头
    CAN_TxHeaderTypeDef header{};

    header.StdId = frame.id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;

    // 4. 复制8个数据字节
    uint8_t bytes[8]{};

    for (int i = 0; i < 8; i++)
    {
        bytes[i] = frame.data[i];
    }

    // 5. 提交到CAN1发送邮箱
    uint32_t mailbox = 0;

    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(
        &hcan1,
        &header,
        bytes,
        &mailbox
    );

    // 6. 沿用现有的邮箱归属记录
    if (result == HAL_OK)
    {
        owners.reuse(mailbox);

        if (is_pitch)
        {
            owners.pitch_motion |= mailbox;
        }
        else
        {
            owners.yaw_motion |= mailbox;
        }
    }

    return result;
}
HAL_StatusTypeDef shooter_can_send_currents(int16_t left, int16_t right, int16_t feeder)
{
    CAN_TxHeaderTypeDef header{};
    header.StdId = gimbal_tuning::shooter::CommandId;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    uint8_t bytes[8];
    shooter_can::encode_currents(left, right, feeder, bytes);
    uint32_t mailbox = 0;
    const auto result = HAL_CAN_AddTxMessage(&hcan1, &header, bytes, &mailbox);
    // 邮箱可能曾被达妙轴使用，必须清除旧归属，避免轴失能误撤销发射帧。
    if (result == HAL_OK) owners.reuse(mailbox);
    return result;
}

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

    // 之前的 MIT 反馈分发保留，但 V41 一控四不使用 Master ID 作为周期反馈 ID。
#if 0
    if (hcan == &hcan1)
        (void)damiao_feedback_monitor.receive(true, rx_header.StdId, rx_data,
                                             rx_header.DLC, HAL_GetTick());
#endif
    // 保留 V41 路径；pitch 已换用标准 MIT 固件，不能按此格式解析。
#if 0
    // 仅观察 CAN1 上的 V41 反馈。标准数据帧和 8 字节已在上方检查。
    // 例如 pitch ID=5，帧头 ID=0x305；不在接收中断中发送任何电流命令。
    if (hcan == &hcan1)
        (void)damiao_v41_feedback_monitor.receive(rx_header.StdId, rx_data,
                                                   rx_header.DLC, HAL_GetTick());
#endif
    // pitch 的 Master ID 是 0x63；只记录反馈，不在中断中发送或使能电机。
    if (hcan == &hcan1) {
        (void)damiao_mit_pitch_feedback_monitor.receive(rx_header.StdId, rx_data,
                                                        rx_header.DLC, HAL_GetTick());
        (void)damiao_mit_yaw_feedback_monitor.receive(rx_header.StdId, rx_data,
                                                      rx_header.DLC, HAL_GetTick());
        // 发射反馈单独接入，接收中断不发送电机命令。
        if (rx_header.StdId == gimbal_tuning::shooter::LeftFeedbackId)
            Gimbal.Motor_Left_Fric.Update_Info(rx_data);
        else if (rx_header.StdId == gimbal_tuning::shooter::RightFeedbackId)
            Gimbal.Motor_Right_Fric.Update_Info(rx_data);
        else if (rx_header.StdId == gimbal_tuning::shooter::FeederFeedbackId)
            (void)Gimbal.Motor_Feeder.update(rx_data, rx_header.DLC);
    }
    return;
#if 0 // 旧 6020/发射链路暂不运行，后续发射机构单独适配后再整理。
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
#endif
}
