#pragma once
#include "DamiaoMitPitchMonitor.hpp"
#include "DamiaoMitYawMonitor.hpp"
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
damiao_mit_pitch::FeedbackSnapshot damiao_mit_pitch_feedback_snapshot();
HAL_StatusTypeDef damiao_pitch_can_send_enable();
HAL_StatusTypeDef damiao_pitch_can_send_disable();
HAL_StatusTypeDef damiao_pitch_can_send_control(const damiao_mit::Command &);
bool damiao_pitch_can_disable_tx_complete();
damiao_mit_yaw::FeedbackSnapshot damiao_mit_yaw_feedback_snapshot();
HAL_StatusTypeDef damiao_yaw_can_send_enable();
HAL_StatusTypeDef damiao_yaw_can_send_disable();
HAL_StatusTypeDef damiao_yaw_can_send_control(const damiao_mit::Command &);
bool damiao_yaw_can_disable_tx_complete();
