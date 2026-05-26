/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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

#include "ADC_Battery.h"
#include "buzzer.h"
#include "CallBack_Task.h"
#include "IMU.h"
#include "Init_Config.h"
#include "TotalControl.h"
#include "surface_control_task.h"
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

float ADC_Voltage_Real;
/* USER CODE END Variables */
osThreadId Init_TaskHandle;
osThreadId SelfTest_TaskHandle;
osThreadId IMU_TaskHandle;
osThreadId Total_Control_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void InitTask(void const * argument);
void SelfTestTask(void const * argument);
void IMUTask(void const * argument);
void TotalControlTask(void const * argument);

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
  /* definition and creation of Init_Task */
  osThreadDef(Init_Task, InitTask, osPriorityNormal, 0, 512);
  Init_TaskHandle = osThreadCreate(osThread(Init_Task), NULL);

  /* definition and creation of SelfTest_Task */
  osThreadDef(SelfTest_Task, SelfTestTask, osPriorityIdle, 0, 512);
  SelfTest_TaskHandle = osThreadCreate(osThread(SelfTest_Task), NULL);

  /* definition and creation of IMU_Task */
  osThreadDef(IMU_Task, IMUTask, osPriorityIdle, 0, 512);
  IMU_TaskHandle = osThreadCreate(osThread(IMU_Task), NULL);

  /* definition and creation of Total_Control_Task */
  osThreadDef(Total_Control_Task, TotalControlTask, osPriorityIdle, 0, 512);
  Total_Control_TaskHandle = osThreadCreate(osThread(Total_Control_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_InitTask */
/**
* @brief Function implementing the Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InitTask */
void InitTask(void const * argument)
{
  /* USER CODE BEGIN InitTask */
  /* Infinite loop */
  for(;;)
  {
    TotalInitTask();
    osThreadTerminate(Init_TaskHandle);
    osDelay(1);
  }
  /* USER CODE END InitTask */
}

/* USER CODE BEGIN Header_SelfTestTask */
/**
* @brief Function implementing the SelfTest_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SelfTestTask */
void SelfTestTask(void const * argument)
{
  /* USER CODE BEGIN SelfTestTask */
  /* Infinite loop */
  for(;;)
  {
	// Total_Power_Control(Power_ON);
    if (Guidance_State == Self_Text_State)
    {
        Self_Text_Task();
    }
	  Buzzer_play_task(&Buzzer_message);
    osDelay(100);
  }
  /* USER CODE END SelfTestTask */
}

/* USER CODE BEGIN Header_IMUTask */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUTask */
void IMUTask(void const * argument)
{
  /* USER CODE BEGIN IMUTask */
  /* Infinite loop */
  for(;;)
  {
    IMU_Task();
    osDelay( 1 );
  }
  /* USER CODE END IMUTask */
}

/* USER CODE BEGIN Header_TotalControlTask */
/**
* @brief Function implementing the Total_Control_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TotalControlTask */
void TotalControlTask(void const * argument)
{
  /* USER CODE BEGIN TotalControlTask */
  /* Infinite loop */
  for(;;)
  {
    TotalControl();
    // control_cnt++;
    osDelay(1);
  }
  /* USER CODE END TotalControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

