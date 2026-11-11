//
// Created by Andrew on 2025/4/5.
//

#ifndef RM2025_INFANTRY_GIMBAL_POWERRUNE_DEBUGC_HPP
#define RM2025_INFANTRY_GIMBAL_POWERRUNE_DEBUGC_HPP
#include "usartio.hpp"
#include "PitchDirectionTest.hpp"

// 定义接收缓冲区大小
#define DEBUG_RVSIZE 512
#define DEBUG_TXSIZE 512

class cDebug : public Class_Usart
{
public:
    cDebug(UART_HandleTypeDef* local_uart, uint16_t buf_size, eUSART_type type) :
        Class_Usart(local_uart, type, DR16), buf_size_(buf_size)
    {
    }

    void open();
    void close();
    void rxCallback(UART_HandleTypeDef *huart, uint16_t size); // 覆盖基类方法以防调用到双缓冲相关引发卡死
    void usart_printf(const char* format, ...);
    // IRQ写入；GIMBAL任务在临界区取走。停止命令优先，不被后续命令覆盖。
    // 达妙 Pitch 命令由 UART 中断写入，控制任务必须在临界区复制并清除。
    // stop 优先：尚未被控制任务取走时，后续 enable 和 target 不会覆盖它。
    volatile bool dm_stop_requested = false;
    volatile bool dm_enable_requested = false;
    volatile bool dm_target_pending = false;
    volatile float dm_target_rad = 0.0f; // 绝对 IMU 目标角度，单位 rad
    volatile bool dm_status_requested = false;
    volatile PitchTestRequest pitch_request = PitchTestRequest::None;
    volatile uint32_t pitch_request_ms = 0;
    volatile float pitch_delta_deg = 0;
    // 达妙双环：kp/ki/kd及dm_kp/ki/kd写同一速度环邮箱；kpp写位置Kp。
    volatile float pitch_speed_kp = 0;
    volatile float pitch_speed_ki = 0;
    volatile float pitch_speed_kd = 0;
    volatile float pitch_position_kp = 0;
    volatile uint8_t pitch_speed_gain_pending = 0;
    volatile float yaw_speed_kp=0;
    volatile float yaw_speed_ki=0;
    volatile float yaw_speed_kd=0;
    volatile float yaw_position_kp=0;
    volatile uint8_t yaw_gain_pending=0;
    volatile float yaw_delta_deg=0;
    volatile bool yaw_delta_pending=false;
    volatile bool yaw_stop_requested=false;
    volatile bool yaw_enable_requested=false;
    volatile bool vision_disable_requested=false;
    volatile bool vision_mode_pending=false;
    volatile uint8_t vision_mode=0;
    volatile bool vision_color_pending=false;
    volatile uint8_t vision_color=0;
    volatile bool vision_status_requested=false;
    volatile bool pitch_status_requested = false;
    volatile bool test_reply_requested = false;
    volatile bool imu_check_enabled = false; // imucheck 每次完整命令切换姿态打印
    volatile uint32_t pitch_rx_lines = 0;
    volatile uint32_t uart_rx_bytes = 0;
    volatile uint32_t uart_rx_events = 0;
    volatile uint32_t uart_rx_start_status = HAL_ERROR;

    // --- 用户接口: 直接访问成员变量 ---
    float A = 0.0f;
    float B = 0.0f;
    float C = 0.0f;
    float D = 0.0f;

    // 检查是否有数据更新
    bool isUpdate() const { return data_updated; }
    void clearUpdateFlag() { data_updated = false; }

private:
    uint16_t buf_size_;
    bool data_updated = false; // 数据更新标志位
    uint8_t debug_tx_buf[DEBUG_TXSIZE]{};
};

extern cDebug debug;

#endif //RM2025_INFANTRY_GIMBAL_POWERRUNE_DEBUGC_HPP
