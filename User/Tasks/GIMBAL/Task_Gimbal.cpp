//
// Created by RedogJrome on 2026/3/6.
//

#include "Task_Gimbal.hpp"
#include "cmsis_os.h"
#include "Gimbal.hpp"
#include "usart.h"


void Task_Gimbal(void *argument)
{
    Gimbal.Init();
    Gimbal.Manual_Protect_Flag = true;
    for(;;)
    {
        TickType_t t = xTaskGetTickCount();

        Gimbal.Gimbal_Loop();

        vTaskDelayUntil(&t,2);
    }
}
