#include "user_lib.h"

#include "main.h"
#include "tim.h"

/****************************************************************************
* Name : ECO & IMU Universal pid initialization
* Feature : 
* Details : 
*****************************************************************************/
void PidInit(void)
{

}	

void PidCalc(void)
{

}

void PWM_Init(void)
{
    HAL_TIM_PWM_Init(&htim2);
    HAL_TIM_PWM_Init(&htim3);
    HAL_TIM_PWM_Init(&htim4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);//涵道
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);//buzzer
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);//Servo右1
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//Servo右2
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//Servo右1
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);//Servo右2
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);//Servo右3
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);//Servo右4
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//Servo右5
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);//Servo右6
}

