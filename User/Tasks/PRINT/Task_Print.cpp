#include "cmsis_os.h"
#include "Gimbal.hpp"
#include "usart.h"
#include "AutoAim.hpp"
#include "INSTask.hpp"
#include "PrintTelemetry.hpp"
#include "PitchTilt.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "debugc.hpp"
#include "VisionLink.hpp"
#include <cstring>
#include <cstdio>

namespace
{
// 仅publish_print_imu写入；读写均用短临界区，避免一行混用不同IMU帧。
PrintTelemetry imu_frame{};
uint32_t imu_last_ms = 0;
PitchTilt pitch_tilt{}; // 仅INS任务在发布时更新
// 独立、持久缓冲区：异步发送完成前不改写相应内容。
char text[PrintTelemetryCapacity];
char vofa_text[PrintTelemetryCapacity];
uint8_t uart_tx[PrintTelemetryCapacity];
uint8_t usb_tx[PrintTelemetryCapacity];

PrintTelemetry capture_frame()
{
    taskENTER_CRITICAL();
    PrintTelemetry data = imu_frame;
    data.time_ms = HAL_GetTick();
    data.imu_fresh = data.imu_sequence != 0 &&
        static_cast<uint32_t>(data.time_ms - imu_last_ms) <= 100;
    data.pitch_y_valid = data.pitch_y_valid && data.imu_fresh;
    data.yaw_encoder = Gimbal.Motor_Yaw.RawEcd;
    data.pitch_encoder = Gimbal.Motor_Pitch.RawEcd;
    data.yaw_rpm = Gimbal.Motor_Yaw.RawSpe;
    data.pitch_rpm = Gimbal.Motor_Pitch.RawSpe;
    data.yaw_online = gm6020_feedback_fresh(Gimbal.Motor_Yaw.FeedbackSeen,
        data.time_ms, Gimbal.Motor_Yaw.LastFeedbackMs);
    data.pitch_online = gm6020_feedback_fresh(Gimbal.Motor_Pitch.FeedbackSeen,
        data.time_ms, Gimbal.Motor_Pitch.LastFeedbackMs);
    data.pitch_test_state = Gimbal.Pitch_Test.state;
    data.pitch_test_command = Gimbal.Pitch_Test.command;
    data.pitch_test_travel = Gimbal.Pitch_Test.travel_counts;
    data.pitch_hold_state=Gimbal.Pitch_Hold.state;
    data.pitch_control_deg=-Gimbal.Pitch_Tilt.continuous_deg;
    data.pitch_target_deg=Gimbal.Pitch_Hold.target_rad*57.295779513f;
    data.pitch_speed_deg_s=-INS.Gyro[Y_]*57.295779513f;
    data.pitch_current_command=Gimbal.Pitch_Current_Command;
    data.pitch_current_feedback=GimbalHardware::PitchSpeedTuneMode && GimbalHardware::PitchSpeedDryRun ?
        0 : Gimbal.Motor_Pitch.RawTorque;
    data.yaw_current_command=GimbalHardware::PitchSpeedTuneMode ? Gimbal.Yaw_Current_Command : Gimbal.Commissioning.yaw_current;
    data.yaw_current_feedback=Gimbal.Motor_Yaw.RawTorque;
    if(GimbalHardware::PitchSpeedTuneMode)
    {
        // 实际控制反馈与目标在同一临界区快照；不用旧Pitch_Hold/TY的目标。
        data.yaw_control_deg=INS.YawTotalAngle;
        data.yaw_target_deg=Gimbal.Yaw_Speed_Tune.target_yaw_deg;
        data.pitch_control_deg=INS.Roll;
        data.pitch_target_deg=Gimbal.Pitch_Speed_Tune.target_roll_deg;
    }
    taskEXIT_CRITICAL();
    return data;
}

bool transmit_frame(const char *bytes, std::size_t size,
                    const char *vofa_bytes, std::size_t vofa_size,volatile bool *reply_requested=nullptr)
{
    // 周期波形忙时可丢；命令回复保留pending，下轮重试，不覆盖DMA在用缓冲区。
    const char *uart_bytes = PrintUart1FireWater ? vofa_bytes : bytes;
    const std::size_t uart_size = PrintUart1FireWater ? vofa_size : size;
    bool uart_started=false;
    taskENTER_CRITICAL();
    if (uart_size != 0 && huart1.gState == HAL_UART_STATE_READY)
    {
        std::memcpy(uart_tx, uart_bytes, uart_size);
        // 在临界区内先取走请求，失败则放回；成功后新到的请求不会被事后清除。
        if(reply_requested) *reply_requested=false;
        uart_started=HAL_UART_Transmit_DMA(&huart1, uart_tx, static_cast<uint16_t>(uart_size))==HAL_OK;
        if(!uart_started && reply_requested) *reply_requested=true;
    }
    taskEXIT_CRITICAL();

    // USB反馈版同样只发FireWater采样行，不与普通日志混发。
    // USB IRQ优先级5，临界区保护拔线时pClassData的生命周期。
    if (!VisionLinkConfig::Enabled && GimbalHardware::FeedbackOnly && vofa_size != 0)
    {
        taskENTER_CRITICAL();
        auto *cdc = static_cast<USBD_CDC_HandleTypeDef *>(hUsbDeviceFS.pClassData);
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc && cdc->TxState == 0)
        {
            std::memcpy(usb_tx, vofa_bytes, vofa_size);
            (void)CDC_Transmit_FS(usb_tx, static_cast<uint16_t>(vofa_size));
        }
        taskEXIT_CRITICAL();
    }
    return uart_started;
}
}

void publish_print_imu(float roll_deg, float pitch_deg, float yaw_deg,
                       const float gyro_rad[3], const float q[4])
{
    // 三角函数在临界区外计算，仍按INS连续帧更新，不能只在10Hz打印时解绕。
    const bool tilt_valid = update_pitch_tilt(&pitch_tilt, q);
    taskENTER_CRITICAL();
    imu_frame.roll_deg = roll_deg;
    imu_frame.pitch_deg = pitch_deg;
    imu_frame.yaw_deg = yaw_deg;
    for (unsigned i = 0; i < 3; ++i) imu_frame.gyro_rad[i] = gyro_rad[i];
    for (unsigned i = 0; i < 4; ++i) imu_frame.q[i] = q[i];
    for (unsigned i = 0; i < 3; ++i)
    {
        imu_frame.ekf_bias[i] = QEKF_INS.GyroBias[i];
        imu_frame.ekf_gyro_rad[i] = QEKF_INS.Gyro[i];
        imu_frame.accel_mps2[i] = INS.Accel[i];
    }
    imu_frame.temperature_c = BMI088.Temperature;
    imu_frame.sample_dt_s = QEKF_INS.dt;
    imu_frame.quaternion_norm = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    imu_frame.accel_norm_mps2 = QEKF_INS.accl_norm;
    imu_frame.pitch_y_deg = pitch_tilt.continuous_deg;
    imu_frame.pitch_y_valid = tilt_valid;
    ++imu_frame.imu_sequence;
    imu_last_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void WatchDogFunction()
{
    INS.Wdg.wdgf();
    AutoAim.wdg.wdgf();
    Gimbal.Motor_Left_Fric.Wdg.wdgf();
    Gimbal.Motor_Right_Fric.Wdg.wdgf();
}

void Task_Print(void *argument)
{
    (void)argument;
    if(GimbalHardware::SerialTestOnly)
    {
        static uint8_t reply[]="ok\r\n";
        for(;;)
        {
            WatchDogFunction();
            if(debug.test_reply_requested)
            {
                // 串口测试独占发送，不启动周期日志/DMA发送。
                debug.test_reply_requested=false;
                if(HAL_UART_Transmit(&huart1,reply,4,50)!=HAL_OK)
                    debug.test_reply_requested=true;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_print = last_wake;
    for (;;)
    {
        // 保留看门狗原来的2ms调用周期；打印独立降到10Hz。
        WatchDogFunction();
        const TickType_t now = xTaskGetTickCount();
        const uint32_t period_ms=GimbalHardware::CommissioningMode || GimbalHardware::PitchSpeedTuneMode ? 20 : PrintTelemetryPeriodMs;
        if (static_cast<TickType_t>(now - last_print) >= pdMS_TO_TICKS(period_ms))
        {
            last_print = now;
            const PrintTelemetry data = capture_frame();
            const std::size_t size = PrintUart1FireWater ? 0 :
                format_print_telemetry(text, sizeof(text), data);
            const std::size_t vofa_size = GimbalHardware::PitchSpeedTuneMode ?
                format_gimbal_angles(vofa_text,sizeof(vofa_text),data) :
                GimbalHardware::CommissioningMode ?
                format_commissioning_telemetry(vofa_text,sizeof(vofa_text),data) :
                format_vofa_telemetry(vofa_text, sizeof(vofa_text), data);
            taskENTER_CRITICAL();
            const bool vision_status_requested=debug.vision_status_requested;
            const auto vision_control=Gimbal.Vision_Control;
            const uint32_t vision_sequence=vision_rx_sequence,vision_ms=vision_rx_tick_ms;
            const auto vision_packet=vision_rv_pkt_2026;
            const auto yaw_tune=Gimbal.Yaw_Speed_Tune;
            const bool status_requested=debug.pitch_status_requested;
            const bool test_requested=debug.test_reply_requested;
            const auto tune=Gimbal.Pitch_Speed_Tune;
            const uint32_t rx_lines=debug.pitch_rx_lines;
            const uint32_t rx_bytes=debug.uart_rx_bytes;
            const uint32_t rx_events=debug.uart_rx_events;
            const uint32_t rx_start=debug.uart_rx_start_status;
            const uint32_t uart_error=huart1.ErrorCode;
            const uint32_t rx_remaining=__HAL_DMA_GET_COUNTER(huart1.hdmarx);
            const bool imu_valid=QEKF_INS.LastUpdateValid;
            const bool motor_seen=Gimbal.Motor_Pitch.FeedbackSeen;
            const float gyro_y=INS.Gyro[Y_];
            taskEXIT_CRITICAL();
            if(test_requested)
            {
                // 只由打印任务发送，避免与周期日志共用UART DMA时互相覆盖。
                // UART忙时保留请求，下一轮重试。
                transmit_frame("ok\r\n",4,"ok\r\n",4,&debug.test_reply_requested);
            }
            else if(vision_status_requested)
            {
                const int n=std::snprintf(vofa_text,sizeof(vofa_text),
                    "vision_status:enabled=%u,active=%u,RC_online=%u,up_mode=%u,enemy=%u,rx=%lu,age_ms=%lu,"
                    "down_mode=%u,fire=%u,reason=%s,Y_enabled=%u,P_enabled=%u,Y_target=%.2f,P_target=%.2f\r\n",
                    static_cast<unsigned>(vision_control.enabled),static_cast<unsigned>(vision_control.active),
                    static_cast<unsigned>(vision_control.remote_online),
                    static_cast<unsigned>(vision_control.uplink_mode()),static_cast<unsigned>(vision_control.enemy_color),
                    static_cast<unsigned long>(vision_sequence),static_cast<unsigned long>(HAL_GetTick()-vision_ms),
                    static_cast<unsigned>(vision_packet.mode),static_cast<unsigned>(vision_packet.fire),vision_control.reason,
                    static_cast<unsigned>(yaw_tune.enabled),static_cast<unsigned>(tune.enabled),
                    static_cast<double>(yaw_tune.target_yaw_deg),static_cast<double>(tune.target_roll_deg));
                if(n>0 && static_cast<std::size_t>(n)<sizeof(vofa_text))
                    transmit_frame(vofa_text,n,vofa_text,n,&debug.vision_status_requested);
                else
                {
                    constexpr char error[]="vision_status:format_error\r\n";
                    transmit_frame(error,sizeof(error)-1,error,sizeof(error)-1,&debug.vision_status_requested);
                }
            }
            else if(status_requested)
            {
                const int n=std::snprintf(vofa_text,sizeof(vofa_text),
                    "pitch_status:v=RX2,enabled=%u,reason=%s,rx_lines=%lu,kp=%.4f,ki=%.4f,kd=%.4f,"
                    "R_deg=%.2f,gy_dps=%.3f,input_rpm=%.4f,error_rpm=%.4f,PID_out=%.3f,"
                    "IP_cmd=%d,imu_valid=%u,motor_seen=%u,CAN_error=0x%lX,"
                    "rx_bytes=%lu,rx_events=%lu,rx_start=%lu,rx_remaining=%lu,UART_error=0x%lX\r\n",
                    static_cast<unsigned>(tune.enabled),tune.reason,static_cast<unsigned long>(rx_lines),
                    static_cast<double>(tune.pid.SpdParam.Kp1),static_cast<double>(tune.pid.SpdParam.Ki1),
                    static_cast<double>(tune.pid.SpdParam.Kd1),static_cast<double>(data.roll_deg),
                    static_cast<double>(gyro_y*57.295779513f),static_cast<double>(tune.pid.SpdParam.PID_Input),
                    static_cast<double>(tune.pid.SpdParam.PID_Err_now),static_cast<double>(tune.pid.SpdParam.PID_Out),
                    static_cast<int>(tune.command),static_cast<unsigned>(imu_valid),static_cast<unsigned>(motor_seen),
                    static_cast<unsigned long>(tune.last_can_error),static_cast<unsigned long>(rx_bytes),
                    static_cast<unsigned long>(rx_events),static_cast<unsigned long>(rx_start),
                    static_cast<unsigned long>(rx_remaining),static_cast<unsigned long>(uart_error));
                if(n>0 && static_cast<std::size_t>(n)<sizeof(vofa_text))
                    transmit_frame(vofa_text,n,vofa_text,n,&debug.pitch_status_requested);
                else
                {
                    constexpr char error[]="pitch_status:format_error\r\n";
                    transmit_frame(error,sizeof(error)-1,error,sizeof(error)-1,&debug.pitch_status_requested);
                }
            }
            else transmit_frame(text, size, vofa_text, vofa_size);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
    }
}
