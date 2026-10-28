#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "usb_device.h"
#include "tim.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "bsp_delay.h"
#include "bsp_dwt.h"

#include "spi.h"
#include "CANio.hpp"
#include "usartio.hpp"
#include "debugc.hpp"

void BSP_Init()
{
    SystemClock_Config();
    HAL_Init();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_TIM5_Init();
    MX_TIM10_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_USB_DEVICE_Init();
}

void os_Init()
{
    osKernelInitialize();
    MX_FREERTOS_Init();
    osKernelStart();
}

void User_Init()
{
    delay_init();
    DWT_Init(168);
    debug.open();
    remote_init();
    can_filter_init();
}

int main(void)
{
    BSP_Init();
    User_Init();
    os_Init();

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
}