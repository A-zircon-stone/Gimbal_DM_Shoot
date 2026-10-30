//
// Created by RedogJrome on 2026/3/6.
//

#ifndef INC_2026_BALANCED_GIMBAL_CANIO_HPP
#define INC_2026_BALANCED_GIMBAL_CANIO_HPP
#include "can.h"
#include "DamiaoMitPitchMonitor.hpp"
#include "DamiaoMitPitchCommand.hpp"
// 之前的 MIT 接收代码留作参考，V41 一控四暂不使用。
#if 0
#include "DamiaoFeedbackMonitor.hpp"
#include "DamiaoV41Monitor.hpp"
#endif

void can_filter_init();
// 从任务中取得 pitch 原始反馈快照；内部短暂屏蔽中断，防止读到半帧。
damiao_mit_pitch::FeedbackSnapshot damiao_mit_pitch_feedback_snapshot();

// 从同一个控制任务调用：HAL_OK 只表示已经放入 CAN 发送邮箱，不代表电机收到。
// 电机初始化采用先失能再按控制任务状态机使能；此处不自动启动电机。
HAL_StatusTypeDef damiao_pitch_can_send_enable();
HAL_StatusTypeDef damiao_pitch_can_send_disable();
HAL_StatusTypeDef damiao_pitch_can_send_control(const damiao_mit::Command &command);
// 等待上一帧失能命令离开发送邮箱后，再提交使能命令。
bool damiao_pitch_can_disable_tx_complete();
#if 0 // 之前的 V41 快照接口保留；pitch 已刷为标准 MIT 固件。
damiao_v41::FeedbackSnapshot damiao_v41_feedback_snapshot();
#endif
#if 0 // 之前的 MIT 快照接口，保留但不参与编译
damiao_can::FeedbackSnapshot damiao_feedback_snapshot();
#endif
HAL_StatusTypeDef SendMessage_8(CAN_HandleTypeDef *hcan,uint32_t stdid,uint8_t *data);
HAL_StatusTypeDef Cmd_Gimbal_Pitch(int16_t motor);
// 只允许反馈模式内人工短脉冲测试。不是旧电机力矩接口。
HAL_StatusTypeDef Cmd_Pitch_Direction_Test(int16_t raw_current);
HAL_StatusTypeDef Cmd_Gimbal_Raw_Current(int16_t yaw_current,int16_t pitch_current);


#endif //INC_2026_BALANCED_GIMBAL_CANIO_HPP
