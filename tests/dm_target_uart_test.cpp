#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "PitchSpeedPidCommand.hpp"
#include "YawPidCommand.hpp"

// 以下仅模拟硬件和与本测试无关的旧命令；待测串口解析使用真实源码。
#define DEBUG_RVSIZE 512
struct UART_HandleTypeDef {};
enum class PitchTestRequest { None, Stop };
uint8_t rx_buffer[DEBUG_RVSIZE]{};
uint32_t HAL_GetTick() { return 0; }
void HAL_UART_AbortReceive(UART_HandleTypeDef *) {}
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t) { return 0; }
bool parse_vision_byte_command(const char *, const char *, uint8_t &) { return false; }
PitchTestRequest parse_pitch_test_command(const char *, float *) { return PitchTestRequest::None; }

struct cDebug {
    UART_HandleTypeDef *which_uart_ = nullptr;
    uint16_t buf_size_ = DEBUG_RVSIZE;
    uint32_t uart_rx_events=0, uart_rx_bytes=0, pitch_rx_lines=0, uart_rx_start_status=0;
    bool dm_stop_requested=false, dm_enable_requested=false, dm_target_pending=false;
    float dm_target_rad=0;
    bool dm_status_requested=false, pitch_status_requested=false, test_reply_requested=false;
    bool imu_check_enabled=false;
    bool vision_status_requested=false, vision_disable_requested=false;
    uint8_t vision_mode=0, vision_color=0;
    bool vision_mode_pending=false, vision_color_pending=false;
    float pitch_speed_kp=0,pitch_speed_ki=0,pitch_speed_kd=0,pitch_position_kp=0;
    uint8_t pitch_speed_gain_pending=0;
    float yaw_speed_kp=0,yaw_speed_ki=0,yaw_speed_kd=0,yaw_position_kp=0,yaw_delta_deg=0;
    uint8_t yaw_gain_pending=0;
    bool yaw_stop_requested=false, yaw_delta_pending=false, yaw_enable_requested=false;
    PitchTestRequest pitch_request=PitchTestRequest::None;
    float pitch_delta_deg=0;
    uint32_t pitch_request_ms=0;
    float A=0,B=0,C=0,D=0;
    bool data_updated=false;
    void rxCallback(UART_HandleTypeDef *, uint16_t);
};

#include "debugc_actual.inc"

void receive(cDebug &debug, const char *part) {
    const auto len = std::strlen(part);
    assert(len < DEBUG_RVSIZE);
    std::memcpy(rx_buffer,part,len);
    debug.rxCallback(nullptr,static_cast<uint16_t>(len));
}

int main() {
    cDebug debug;
    receive(debug,"kpp=100\nkp=0.5\nki=0.1\nkd=0\n");
    assert(debug.pitch_speed_gain_pending==15 && debug.pitch_position_kp==100);
    assert(debug.pitch_speed_kp==0.5f && debug.pitch_speed_ki==0.1f && debug.pitch_speed_kd==0);
    debug.pitch_speed_gain_pending=0;
    receive(debug,"dm_kp=2\nkp=3\n");
    assert(debug.pitch_speed_kp==3 && debug.pitch_speed_gain_pending==1);
    receive(debug,"kp=4\ndm_kp=5\ndm_ki=0.2\ndm_kd=0.3\n");
    assert(debug.pitch_speed_kp==5 && debug.pitch_speed_ki==0.2f && debug.pitch_speed_kd==0.3f);
    receive(debug,"ykpp=60\nykp=0.5\nyki=0\nykd=0\n");
    assert(debug.yaw_gain_pending==15 && debug.yaw_position_kp==60 && debug.yaw_speed_kp==0.5f);
    receive(debug,"imuch");
    assert(!debug.imu_check_enabled);
    receive(debug,"eck\r\n");
    assert(debug.imu_check_enabled);
    receive(debug,"imucheck_bad\n");
    assert(debug.imu_check_enabled);
    receive(debug,"imucheck\n");
    assert(!debug.imu_check_enabled);
    receive(debug,"dm_tar");
    assert(!debug.dm_target_pending);
    receive(debug,"get=-0.12\r\n");
    assert(debug.dm_target_pending && std::fabs(debug.dm_target_rad+0.12f)<0.00001f);
    debug.dm_target_pending=false;
    receive(debug,"dm_target=nan\ndm_target=inf\ndm_target=0.1bad\ndm_target=\n");
    assert(!debug.dm_target_pending);
    receive(debug,"dm_delta=0.10\n");
    assert(!debug.dm_target_pending);
    receive(debug,"dm_target=0.1\ndm_stop\ndm_enable\ndm_target=0.2\n");
    assert(debug.dm_stop_requested && !debug.dm_enable_requested && !debug.dm_target_pending);
    debug.dm_stop_requested=false;
    receive(debug,"dm_target=0.3\n");
    assert(debug.dm_target_pending && std::fabs(debug.dm_target_rad-0.3f)<0.00001f);
}
