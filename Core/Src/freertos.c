/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PRINT */
osThreadId_t PRINTHandle;
uint32_t PRINTBuffer[ 512 ];
osStaticThreadDef_t PRINTControlBlock;
const osThreadAttr_t PRINT_attributes = {
  .name = "PRINT",
  .cb_mem = &PRINTControlBlock,
  .cb_size = sizeof(PRINTControlBlock),
  .stack_mem = &PRINTBuffer[0],
  .stack_size = sizeof(PRINTBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for INS_Task */
osThreadId_t INS_TaskHandle;
uint32_t INS_TaskBuffer[ 512 ];
osStaticThreadDef_t INS_TaskControlBlock;
const osThreadAttr_t INS_Task_attributes = {
  .name = "INS_Task",
  .cb_mem = &INS_TaskControlBlock,
  .cb_size = sizeof(INS_TaskControlBlock),
  .stack_mem = &INS_TaskBuffer[0],
  .stack_size = sizeof(INS_TaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for GIMBAL */
osThreadId_t GIMBALHandle;
uint32_t GIMBALBuffer[ 1024 ];
osStaticThreadDef_t GIMBALControlBlock;
const osThreadAttr_t GIMBAL_attributes = {
  .name = "GIMBAL",
  .cb_mem = &GIMBALControlBlock,
  .cb_size = sizeof(GIMBALControlBlock),
  .stack_mem = &GIMBALBuffer[0],
  .stack_size = sizeof(GIMBALBuffer),
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for VISION */
osThreadId_t VISIONHandle;
uint32_t VISIONBuffer[ 1024 ];
osStaticThreadDef_t VISIONControlBlock;
const osThreadAttr_t VISION_attributes = {
  .name = "VISION",
  .cb_mem = &VISIONControlBlock,
  .cb_size = sizeof(VISIONControlBlock),
  .stack_mem = &VISIONBuffer[0],
  .stack_size = sizeof(VISIONBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Task_Print(void *argument);
void INSTask(void *argument);
void Task_Gimbal(void *argument);
void Task_Vision(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PRINT */
  PRINTHandle = osThreadNew(Task_Print, NULL, &PRINT_attributes);

  /* creation of INS_Task */
  INS_TaskHandle = osThreadNew(INSTask, NULL, &INS_Task_attributes);

  /* creation of GIMBAL */
  GIMBALHandle = osThreadNew(Task_Gimbal, NULL, &GIMBAL_attributes);

  /* creation of VISION */
  VISIONHandle = osThreadNew(Task_Vision, NULL, &VISION_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Task_Print */
/**
* @brief Function implementing the PRINT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Print */
__weak void Task_Print(void *argument)
{
  /* USER CODE BEGIN Task_Print */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Print */
}

/* USER CODE BEGIN Header_INSTask */
/**
* @brief Function implementing the INS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INSTask */
__weak void INSTask(void *argument)
{
  /* USER CODE BEGIN INSTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INSTask */
}

/* USER CODE BEGIN Header_Task_Gimbal */
/**
* @brief Function implementing the GIMBAL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Gimbal */
__weak void Task_Gimbal(void *argument)
{
  /* USER CODE BEGIN Task_Gimbal */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Gimbal */
}

/* USER CODE BEGIN Header_Task_Vision */
/**
* @brief Function implementing the VISION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Vision */
__weak void Task_Vision(void *argument)
{
  /* USER CODE BEGIN Task_Vision */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Vision */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

