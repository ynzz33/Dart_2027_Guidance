#include "adc.h"
#include "ADC_Battery.h"
#include "CallBack_Task.h"
//#include "cmsis_os.h"
#include "filter.h"
#include "vision_ins.h"
#include "main.h"
#include "user_lib.h"
#include "tim.h"
#include "IMU.h"
#include "pid.h"
#include "adrc.h"              /* ADRC自抗扰控制器 */
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



    HAL_HalfDuplex_EnableTransmitter( &huart1 );                                 //镖头通信:半双工先置发送模式
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rx_Buf, sizeof(Rx_Buf));               //调试:空闲不定长接收
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Vision_Rx_Buf, sizeof(Vision_Rx_Buf));//视觉:空闲不定长接收
		__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);                              //关掉半传输中断,避免半满时误回调

    ADC_Init();//adc_battery输入

		IMU_Init();

		PNG_Init(&PNG_Data);

		VisInsEKF_Init();   /* 视觉/IMU 紧耦合速度 EKF 初始化(取代纯积分速度,见 Tool/vision_ins.c) */

		ADRC_Init_All();    /* ADRC控制器初始化(3通道×2环) */

		Total_Power_Control(Power_ON);
}


