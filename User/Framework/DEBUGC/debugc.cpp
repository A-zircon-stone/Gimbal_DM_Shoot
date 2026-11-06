//
// Created by Andrew on 2025/4/5.
//
#include "debugc.hpp"
#include "PitchSpeedPidCommand.hpp"
#include "YawPidCommand.hpp"
#include "VisionLink.hpp"
#include <cstring>
#include <cstdlib> // 包含 strtof
#include <cstdio>
#include <cstdarg>
#include <cmath>
#include "usart.h"

// 只接受完整、有限的数值，避免 nan/inf 或尾部乱码进入控制器。
static bool parse_dm_float(const char *line, const char *prefix, float &value)
{
    const std::size_t prefix_length = std::strlen(prefix);
    if (std::strncmp(line, prefix, prefix_length) != 0) return false;
    const char *start = line + prefix_length;
    char *end = nullptr;
    const float parsed = std::strtof(start, &end);
    if (end == start || *end != '\0' || !std::isfinite(parsed)) return false;
    value = parsed;
    return true;
}

// 将接收缓冲区放到特定的 RAM 区域
uint8_t rx_buffer[DEBUG_RVSIZE];

// 实例化对象
cDebug debug(&huart1, DEBUG_RVSIZE, DMA_IDLE_IT);

void cDebug::open()
{
    uart_rx_start_status=HAL_UARTEx_ReceiveToIdle_DMA(which_uart_, rx_buffer, buf_size_);
}

void cDebug::close()
{
    HAL_UART_DMAStop(which_uart_);
}

void cDebug::rxCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    // 1. 获取本次接收到的数据长度 (现在由外层HAL计算并传入 size)
    uint16_t len = size;
    ++uart_rx_events;
    uart_rx_bytes+=size;

    // 新命令仅按完整CR/LF行识别，支持DMA空闲分包；不改变旧A/B/C/D命令。
    static char line[32]{};
    static unsigned used=0;
    static bool overflow=false;
    for (unsigned i=0; i<len && i<DEBUG_RVSIZE; ++i)
    {
        const char c=static_cast<char>(rx_buffer[i]);
        if(c=='\r' || c=='\n')
        {
            line[used]='\0';
            if(used && !overflow) ++pitch_rx_lines;
            // 一条命令以回车或换行结束，可分多次 DMA 接收；空行会忽略。
            if (!overflow && !std::strcmp(line, "dm_stop")) {
                dm_stop_requested = true;
                dm_enable_requested = false;
                dm_target_pending = false;
            }
            if (!overflow && !std::strcmp(line, "dm_enable") && !dm_stop_requested)
                dm_enable_requested = true;
            if (!overflow && !std::strcmp(line, "dm_status")) dm_status_requested = true;
            float dm_value = 0.0f;
            // 例如 dm_target=0.10：绝对目标为 0.10 rad，不是增加 0.10 rad。
            if (!overflow && !dm_stop_requested && parse_dm_float(line, "dm_target=", dm_value)) {
                dm_target_rad = dm_value;
                dm_target_pending = true;
            }
#if 0 // 原相对目标命令保留，但不再接收 dm_delta。
            if (!overflow && !dm_stop_requested && parse_dm_float(line, "dm_delta=", dm_value)) {
                dm_delta_rad = dm_value;
                dm_delta_pending = true;
            }
#endif
            // PID 数值先存入邮箱，由控制任务检查具体允许范围后使用。
            if (!overflow && parse_dm_float(line, "dm_kp=", dm_value) && dm_value >= 0) {
                dm_kp = dm_value; dm_gain_pending |= 1u;
            }
            if (!overflow && parse_dm_float(line, "dm_ki=", dm_value) && dm_value >= 0) {
                dm_ki = dm_value; dm_gain_pending |= 2u;
            }
            if (!overflow && parse_dm_float(line, "dm_kd=", dm_value) && dm_value >= 0) {
                dm_kd = dm_value; dm_gain_pending |= 4u;
            }
            if(!overflow && !std::strcmp(line,"pitch_status")) pitch_status_requested=true;
            if(!overflow && !std::strcmp(line,"test")) test_reply_requested=true;
            if(!overflow && !std::strcmp(line,"imucheck")) imu_check_enabled=!imu_check_enabled;
            if(!overflow && !std::strcmp(line,"vision_status")) vision_status_requested=true;
            if(!overflow && !std::strcmp(line,"vision_disable")) vision_disable_requested=true;
            uint8_t vision_byte=0;
            if(!overflow && parse_vision_byte_command(line,"vmode=",vision_byte))
            { vision_mode=vision_byte; vision_mode_pending=true; }
            if(!overflow && parse_vision_byte_command(line,"vcolor=",vision_byte) && vision_byte<=2)
            { vision_color=vision_byte; vision_color_pending=true; }
            float gain=0;
            const uint8_t gain_mask=overflow ? 0 : parse_pitch_speed_gain(line,gain);
            if(gain_mask)
            {
                if(gain_mask==1) pitch_speed_kp=gain;
                if(gain_mask==2) pitch_speed_ki=gain;
                if(gain_mask==4) pitch_speed_kd=gain;
                if(gain_mask==8) pitch_position_kp=gain;
                pitch_speed_gain_pending |= gain_mask;
            }
            const uint8_t yaw_mask=overflow ? 0 : parse_yaw_gain(line,gain);
            if(yaw_mask)
            {
                if(yaw_mask==1) yaw_speed_kp=gain;
                if(yaw_mask==2) yaw_speed_ki=gain;
                if(yaw_mask==4) yaw_speed_kd=gain;
                if(yaw_mask==8) yaw_position_kp=gain;
                yaw_gain_pending |= yaw_mask;
            }
            if(!overflow && (!std::strcmp(line,"ystop") || !std::strcmp(line,"stop")))
            { yaw_stop_requested=true; yaw_enable_requested=false; }
            if(!overflow && !std::strcmp(line,"yenable") && !yaw_stop_requested)
                yaw_enable_requested=true;
            float yaw_delta=0;
            if(!overflow && parse_yaw_delta(line,yaw_delta))
            { yaw_delta_deg=yaw_delta; yaw_delta_pending=true; }
            float delta=0;
            // 新链路vision_disable只交还遥控，不沿用旧版Stop关闭Pitch的语义。
            const auto req=overflow || !std::strcmp(line,"vision_disable") ? PitchTestRequest::None : parse_pitch_test_command(line,&delta);
            if(req!=PitchTestRequest::None && pitch_request!=PitchTestRequest::Stop)
            { pitch_delta_deg=delta; pitch_request=req; pitch_request_ms=HAL_GetTick(); }
            used=0; overflow=false;
        }
        else if(used<sizeof(line)-1 && !overflow) line[used++]=c;
        else overflow=true;
    }

    // 2. 有数据才处理
    if (len > 0)
    {
        // 强制添加结束符
        if (len < DEBUG_RVSIZE) rx_buffer[len] = 0;
        else rx_buffer[DEBUG_RVSIZE - 1] = 0;

        char* pCurrent = (char*)rx_buffer;
        bool has_valid_cmd = false;

        // 3. 扫描缓冲区
        while (*pCurrent != '\0')
        {
            // 检查当前字符是否为 A-D，且下一个字符是 : (冒号)
            if ((*pCurrent >= 'A' && *pCurrent <= 'D') && *(pCurrent + 1) == ':')
            {
                char key = *pCurrent;
                char* pValStart = pCurrent + 2; // 跳过 "X:" 指向数字
                char* pEnd = nullptr;

                // 解析浮点数
                float val = strtof(pValStart, &pEnd);

                // 如果 pEnd 移动了，说明解析到了数字
                if (pEnd != pValStart)
                {
                    // 直接更新对应成员变量
                    switch (key)
                    {
                    case 'A': this->A = val;
                        break;
                    case 'B': this->B = val;
                        break;
                    case 'C': this->C = val;
                        break;
                    case 'D': this->D = val;
                        break;
                    default: break;
                    }
                    has_valid_cmd = true;

                    // 继续处理
                    pCurrent = pEnd;
                }
                else
                {
                    pCurrent++;
                }
            }
            else
            {
                pCurrent++;
            }
        }

        if (has_valid_cmd)
        {
            data_updated = true;
        }
    }

    // 4. 恢复接收状态之前，需要先调用HAL的Abort来重置内部的RxState标志位
    HAL_UART_AbortReceive(which_uart_);

    // 4. 清空 buffer
    memset(rx_buffer, 0, len);

    // 5. 重新开启接收
    uart_rx_start_status=HAL_UARTEx_ReceiveToIdle_DMA(which_uart_, rx_buffer, buf_size_);
}

void cDebug::usart_printf(const char* format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);
    length = vsnprintf((char*)debug_tx_buf, DEBUG_TXSIZE, (const char*)format, args);
    va_end(args);

    HAL_UART_Transmit_DMA(which_uart_, (uint8_t*)debug_tx_buf, length);
}
