#ifndef USARTIO_HPP
#define USARTIO_HPP
#include "main.h"
#include "remote_struct.h"

void remote_init(void);

typedef enum
{
    DMA_IDLE_IT = 0,
    DMA_CPLT_IT = 1
}eUSART_type;

typedef enum
{
    DR16,
    VT03,
}eRemote_type;

class Class_Usart{
public:
    Class_Usart(UART_HandleTypeDef *uart,eUSART_type type,eRemote_type remote_type)
    {
        which_uart_ = uart;
        type_ = type;
        remote_type_ = remote_type;
    }
    void open(uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength);
    void rxCallback(UART_HandleTypeDef *huart,uint8_t size);

protected:
    UART_HandleTypeDef *which_uart_;
    eUSART_type type_;
    eRemote_type remote_type_;
};

class Class_Dr16 : public Class_Usart
{
public:
    Class_Dr16(UART_HandleTypeDef *uart,eUSART_type type,eRemote_type remote_type):Class_Usart(uart,type,remote_type){}

    RC_ctrl_t &value();

    int64_t live_time = 0;
    bool online = false;
private:
    RC_ctrl_t dr16_value;
};

class Class_VT03 : public Class_Usart
{
public:
    Class_VT03(UART_HandleTypeDef *uart,eUSART_type type,eRemote_type remote_type):Class_Usart(uart,type,remote_type){}

    VT03_ctrl_t &value();

    int64_t live_time = -1;
    bool online = false;
private:
    VT03_ctrl_t vt03_value;
};

extern Class_Dr16 dr16;
extern Class_VT03 vt03;

#endif //USARTIO_HPP
