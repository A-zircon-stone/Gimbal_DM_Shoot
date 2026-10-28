#include "cmsis_os.h"
#include "Gimbal.hpp"
#include "INSTask.hpp"
#include "VisionLink.hpp"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "usb_device.h"

VisionSendPacket2026 vision_sd_pkt_2026{};
NavSendPacket nav_sd_pkt{};
namespace { uint8_t usb_vision_tx[45]{}; }

void VisionChattingLoop()
{
    uint8_t bytes[45];
    VisionAttitudeSample s{};
    uint8_t mode,enemy;
    taskENTER_CRITICAL();
    const bool valid=QEKF_INS.LastUpdateValid;
    std::memcpy(s.q,INS.q,sizeof(s.q));
    s.yaw_deg=INS.YawTotalAngle;
    s.pitch_deg=INS.Roll; // 抬头为负，与视觉和现有控制的raw Roll一致
    s.yaw_rate=YawSpeedTune::yaw_rate_rad(INS.Roll,INS.Pitch,INS.Gyro[Y_],INS.Gyro[Z_]);
    s.pitch_rate=INS.Gyro[Y_];
    mode=Gimbal.Vision_Control.uplink_mode();
    enemy=Gimbal.Vision_Control.enemy_color;
    taskEXIT_CRITICAL();
    // 首帧必须是真实有效姿态；弹速及发射计数尚未测量/接入，发送未知或0。
    if(!valid || !build_vision_feedback(s,mode,enemy,VisionLinkConfig::BulletSpeedMps,0,0,bytes)) return;
    taskENTER_CRITICAL();
    auto *cdc=static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
    if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED && cdc && cdc->TxState==0)
    {
        std::memcpy(usb_vision_tx,bytes,sizeof(bytes));
        std::memcpy(&vision_sd_pkt_2026,bytes,sizeof(bytes));
        (void)CDC_Transmit_FS(usb_vision_tx,sizeof(usb_vision_tx));
    }
    taskEXIT_CRITICAL(); // 忙时丢弃本次状态，下轮发送最新帧，不补发积压旧姿态
}
void Task_Vision(void *argument)
{
    TickType_t wake=xTaskGetTickCount();
    for(;;)
    {
        if(VisionLinkConfig::Enabled && !GimbalHardware::SerialTestOnly) VisionChattingLoop();
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(VisionLinkConfig::TxPeriodMs));
    }
}
