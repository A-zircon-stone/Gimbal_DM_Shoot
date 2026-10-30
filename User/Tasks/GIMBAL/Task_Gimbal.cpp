//
// Created by RedogJrome on 2026/3/6.
//

#include "Task_Gimbal.hpp"
#include "cmsis_os.h"
#include "Gimbal.hpp"
#include "usart.h"
#include "DamiaoPitchRuntime.hpp"


void Task_Gimbal(void *argument)
{
    Gimbal.Init();
    Gimbal.Manual_Protect_Flag = true;
    for(;;)
    {
        TickType_t t = xTaskGetTickCount();

        // 达妙 Pitch 首轮：运行新的 IMU 角度闭环调试链路。
        // 旧 Gimbal_Loop 保留在工程中；不能把 6020 输出直接发给达妙电机。
        damiao_pitch_runtime::step(HAL_GetTick());
        // Gimbal.Gimbal_Loop();

        vTaskDelayUntil(&t,2);
    }
}
