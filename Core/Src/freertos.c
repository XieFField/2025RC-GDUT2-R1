/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* Definitions for CAN1_Send */
osThreadId_t CAN1_SendHandle;
const osThreadAttr_t CAN1_Send_attributes = {
  .name = "CAN1_Send",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassic */
osThreadId_t chassicHandle;
const osThreadAttr_t chassic_attributes = {
  .name = "chassic",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN2_Send */
osThreadId_t CAN2_SendHandle;
const osThreadAttr_t CAN2_Send_attributes = {
  .name = "CAN2_Send",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for user_debug */
osThreadId_t user_debugHandle;
const osThreadAttr_t user_debug_attributes = {
  .name = "user_debug",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Air_Joy */
osThreadId_t Air_JoyHandle;
const osThreadAttr_t Air_Joy_attributes = {
  .name = "Air_Joy",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LaserPositioning */
osThreadId_t LaserPositioningHandle;
const osThreadAttr_t LaserPositioning_attributes = {
  .name = "LaserPositioning",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for loraTask1 */
osThreadId_t loraTask1Handle;
const osThreadAttr_t loraTask1_attributes = {
  .name = "loraTask1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for relocate */
osThreadId_t relocateHandle;
const osThreadAttr_t relocate_attributes = {
  .name = "relocate",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LED */
osThreadId_t LEDHandle;
const osThreadAttr_t LED_attributes = {
  .name = "LED",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CAN1_Send_Task(void *argument);
extern void Chassis_Task(void *argument);
extern void CAN2_Send_Task(void *argument);
extern void User_Debug_Task(void *argument);
extern void Air_Joy_Task(void *argument);
extern void LaserPositioning_Task(void *argument);
extern void Lora_Task1(void *argument);
extern void relocate_task(void *argument);
extern void LED_Task(void *argument);

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
  /* creation of CAN1_Send */
  CAN1_SendHandle = osThreadNew(CAN1_Send_Task, NULL, &CAN1_Send_attributes);

  /* creation of chassic */
  chassicHandle = osThreadNew(Chassis_Task, NULL, &chassic_attributes);

  /* creation of CAN2_Send */
  CAN2_SendHandle = osThreadNew(CAN2_Send_Task, NULL, &CAN2_Send_attributes);

  /* creation of user_debug */
  user_debugHandle = osThreadNew(User_Debug_Task, NULL, &user_debug_attributes);

  /* creation of Air_Joy */
  Air_JoyHandle = osThreadNew(Air_Joy_Task, NULL, &Air_Joy_attributes);

  /* creation of LaserPositioning */
  LaserPositioningHandle = osThreadNew(LaserPositioning_Task, NULL, &LaserPositioning_attributes);

  /* creation of loraTask1 */
  loraTask1Handle = osThreadNew(Lora_Task1, NULL, &loraTask1_attributes);

  /* creation of relocate */
  relocateHandle = osThreadNew(relocate_task, NULL, &relocate_attributes);

  /* creation of LED */
  LEDHandle = osThreadNew(LED_Task, NULL, &LED_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_CAN1_Send_Task */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */


/* USER CODE END Header_CAN1_Send_Task */
__weak void CAN1_Send_Task(void *argument)
{
  /* USER CODE BEGIN CAN1_Send_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN1_Send_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

