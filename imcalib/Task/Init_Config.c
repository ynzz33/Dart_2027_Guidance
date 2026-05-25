#include "adc.h"
#include "ADC_Battery.h"
#include "CallBack_Task.h"
//#include "cmsis_os.h"
#include "filter.h"
#include "main.h"
#include "user_lib.h"
#include "tim.h"
#include "IMU.h"
#include "pid.h"
#include "surface_control_task.h"
#include "usart.h"
#include "PNG_Task.h"


void TotalInitTask(void)
{

    ALL_CS_Free();
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    PWM_Init();
    pid_init();

		IMU_Data.Q[NOW][0] = 1;

//    HAL_GPIO_WritePin( GPIOA,GPIO_PIN_4,GPIO_PIN_SET );//led



		HAL_UART_Receive_DMA(&huart3, Vision_Rx_Buf, sizeof(Vision_Rx_Buf));
    HAL_UART_Receive_DMA(&huart2, Rx_Buf, sizeof(Rx_Buf));
    HAL_HalfDuplex_EnableTransmitter( &huart1 );//镖头通信发送初始化使能

    ADC_Init();//adc_battery输入

		IMU_Init();

		PNG_Init(&PNG_Data);

		Kalman_Vel_Init();

		Total_Power_Control(Power_ON);
}


