#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

constexpr int HAL_UART_STATE_READY=0,HAL_OK=0,HAL_BUSY=1;
constexpr int USBD_STATE_CONFIGURED=1;
constexpr bool PrintUart1FireWater=true;
namespace VisionLinkConfig { constexpr bool Enabled=true; }
namespace GimbalHardware { constexpr bool FeedbackOnly=true; }
struct UART { int gState=HAL_UART_STATE_READY; } huart1;
struct USBD_CDC_HandleTypeDef { unsigned TxState=0; };
struct Usb { void *pClassData=nullptr; int dev_state=0; } hUsbDeviceFS;
uint8_t uart_tx[512]{},usb_tx[512]{};
int depth=0,uart_calls=0,usb_calls=0,hal_result=HAL_OK;
volatile bool *request_arriving_during_send=nullptr;
std::string sent;
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (--depth)
int HAL_UART_Transmit_DMA(UART *,uint8_t *data,uint16_t length)
{
    assert(depth==1); ++uart_calls;
    if(request_arriving_during_send) *request_arriving_during_send=true;
    if(hal_result==HAL_OK) sent.assign(reinterpret_cast<char*>(data),length);
    return hal_result;
}
int CDC_Transmit_FS(uint8_t *,uint16_t) { ++usb_calls; return 0; }
#include "print_uart_send.inc"

int main()
{
    constexpr char reply[]="vision_status:enabled=1,active=1\r\n";
    volatile bool vision_pending=true,pitch_pending=true;
    std::memcpy(uart_tx,"DMA-active",10);
    huart1.gState=HAL_BUSY;
    assert(!transmit_frame(reply,sizeof(reply)-1,reply,sizeof(reply)-1,&vision_pending));
    assert(vision_pending && pitch_pending && uart_calls==0 && std::memcmp(uart_tx,"DMA-active",10)==0);
    assert(depth==0);
    huart1.gState=HAL_UART_STATE_READY;hal_result=HAL_BUSY;
    assert(!transmit_frame(reply,sizeof(reply)-1,reply,sizeof(reply)-1,&vision_pending));
    assert(vision_pending && pitch_pending && uart_calls==1); // DMA start failure must retry
    hal_result=HAL_OK;
    assert(transmit_frame(reply,sizeof(reply)-1,reply,sizeof(reply)-1,&vision_pending));
    assert(!vision_pending && pitch_pending && sent==reply && uart_calls==2);
    // A later request must survive a successful start; do not clear flags after HAL returns.
    vision_pending=true;request_arriving_during_send=&vision_pending;
    assert(transmit_frame(reply,sizeof(reply)-1,reply,sizeof(reply)-1,&vision_pending));
    assert(vision_pending);
    request_arriving_during_send=nullptr;
    assert(transmit_frame(reply,sizeof(reply)-1,reply,sizeof(reply)-1,&vision_pending));
    assert(!vision_pending);
    assert(transmit_frame("ok\r\n",4,"ok\r\n",4,&pitch_pending));
    assert(!pitch_pending && sent=="ok\r\n");
    // Periodic telemetry may still drop on busy without corrupting the active DMA buffer.
    huart1.gState=HAL_BUSY;
    const auto previous=sent;
    assert(!transmit_frame("wave",4,"wave",4));
    assert(sent==previous && usb_calls==0 && depth==0); // never send text to vision USB
}
