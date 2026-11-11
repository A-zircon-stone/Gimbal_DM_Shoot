#include "ShooterCan.hpp"
#include "M2006Feedback.hpp"
#include "DamiaoMailboxOwners.hpp"
#include "DamiaoMitPitchMonitor.hpp"
#include "DamiaoMitYawMonitor.hpp"
#include <cassert>
#include <cstring>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
struct CAN_HandleTypeDef {} hcan1, hcan2;
struct CAN_TxHeaderTypeDef { uint32_t StdId = 0, IDE = 0, RTR = 0, DLC = 0; };
using CAN_RxHeaderTypeDef = CAN_TxHeaderTypeDef;
constexpr uint32_t CAN_ID_STD = 0, CAN_RTR_DATA = 0, CAN_RX_FIFO0 = 0;
CAN_TxHeaderTypeDef tx_header;
CAN_RxHeaderTypeDef rx_header;
uint8_t tx_data[8]{}, rx_bytes[8]{};
unsigned tx_count = 0;
HAL_StatusTypeDef tx_result = HAL_OK, rx_result = HAL_OK;
DamiaoMailboxOwners owners;
damiao_mit_pitch::FeedbackMonitor damiao_mit_pitch_feedback_monitor;
damiao_mit_yaw::FeedbackMonitor damiao_mit_yaw_feedback_monitor;
struct FrictionMotorStub {
    unsigned frames = 0;
    void Update_Info(uint8_t *) { ++frames; }
};
struct {
    FrictionMotorStub Motor_Left_Fric, Motor_Right_Fric;
    M2006Feedback Motor_Feeder;
} Gimbal;

uint32_t HAL_GetTick() { return 123; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t,
                                      CAN_RxHeaderTypeDef *header, uint8_t *bytes) {
    *header = rx_header;
    std::memcpy(bytes, rx_bytes, 8);
    return rx_result;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can, CAN_TxHeaderTypeDef *header,
                                      uint8_t *bytes, uint32_t *mailbox) {
    assert(can == &hcan1);
    tx_header = *header;
    std::memcpy(tx_data, bytes, 8);
    *mailbox = 1;
    ++tx_count;
    return tx_result;
}

// CMake提取实际CANio.cpp函数，测试实际路由/提交代码而不是重写一份。
#include "shooter_can_actual.inc"

int main() {
    owners.pitch_motion = 1;
    owners.yaw_disable = 1;
    assert(shooter_can_send_currents(0x1234, -2, -150) == HAL_OK);
    assert(tx_header.StdId == 0x200 && tx_header.DLC == 8);
    assert(tx_header.IDE == CAN_ID_STD && tx_header.RTR == CAN_RTR_DATA);
    const uint8_t expected[] = {0xff, 0xfe, 0x12, 0x34, 0, 0, 0xff, 0x6a};
    assert(std::memcmp(tx_data, expected, 8) == 0);
    assert(owners.pitch_motion == 0 && owners.yaw_disable == 0);
    tx_result = HAL_ERROR;
    owners.yaw_motion = 1;
    assert(shooter_can_send_currents(0, 0, 0) == HAL_ERROR);
    assert(owners.yaw_motion == 1); // 提交失败不得清除别人的邮箱归属。
    const auto sends_before_rx = tx_count;

    rx_header.DLC = 8;
    rx_header.StdId = 0x201;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(Gimbal.Motor_Right_Fric.frames == 1 && Gimbal.Motor_Left_Fric.frames == 0);
    rx_header.StdId = 0x202;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(Gimbal.Motor_Left_Fric.frames == 1);
    rx_header.StdId = 0x204;
    rx_bytes[2] = 0x15; rx_bytes[3] = 0x18;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    assert(!Gimbal.Motor_Feeder.seen);
    rx_header.DLC = 7;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(!Gimbal.Motor_Feeder.seen);
    rx_header.DLC = 8;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(Gimbal.Motor_Feeder.speed_rpm == 150);
    assert(tx_count == sends_before_rx); // RX中断只接收，不能带出命令。
}
