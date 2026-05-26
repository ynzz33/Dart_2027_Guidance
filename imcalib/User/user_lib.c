#include "user_lib.h"

#include "main.h"
#include "tim.h"
#include "surface_control_task.h"
#include "IMU.h"    
#include "CallBack_Task.h"

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

void Self_Text_Process(void)
{
    switch(Self_Text.Self_Text_Process)
    {
        case Self_Text_Vision:
        {
            switch(Self_Text.Vision_Self_Text_flag)
            {
                case Self_Text_Failure:
                {
                    // Self_Text.Self_Text_Process = Self_Text_OK;
                    // Self_Text.Self_Text_Process = Self_Text_Dart_Trigeer;
                }break;
            }
        }break;
        case Self_Text_Dart_Trigeer:
        {
            switch(Self_Text.Dart_Trigger_Self_Text_flag)
            {
                case Self_Text_Failure:
                {
                    // Self_Text.Self_Text_Process = Self_Text_OK;
                }break;
                case Self_Text_Success:
                {
                    Self_Text.Self_Text_Process = Self_Text_OK;
                }break;
            }
        }break;
        case Self_Text_OK:
        {
        }break;
    }
}

void Self_Text_Task(void)
{
    if (Self_Text.Vision_Self_Text_flag == Self_Text_Failure)
    {
        Vision_Self_Text();  
        Vision_Rx_Data.Vision_Receive_Cnt++;
        return;
    }
    if (Self_Text.Dart_Trigger_Self_Text_flag == Self_Text_Failure)
    {
        // 之后每 ~300ms 询问一次,而不是 100ms 狂发
        static uint8_t poll_cnt = 0;
        if (++poll_cnt >= 3)
        {
            poll_cnt = 0;
            Dart_Trigger_State_Check();
            Dart_Trigger_Data.Dart_Trigger_Receive_Cnt++;
        }
    }
    Self_Text_Process();
}


// void My_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart) 
// {
//     uint32_t DMA_FLAGS;

//     if (__HAL_UART_GET_FLAG(huart, UART_CLEAR_TCF)) 
//     {
//         __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF);
//         huart->gState = HAL_UART_STATE_READY;
//         return;
//     }
//     if (__HAL_UART_GET_FLAG(huart, UART_F]LAG_PE)) 
//     {
//         __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE);
//     }
//     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) 
//     {
//         __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
//     }
//     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) 
//     {
//         __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);
//     }
//     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) 
//     {
//         __HAL_UART_CLEAR_OREFLAG(huart);
//     }
//     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) 
//     {
//         __HAL_UART_CLEAR_IDLEFLAG(huart);
//         DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);
//         // 失能DMA
//         __HAL_DMA_DISABLE(huart->hdmarx);
//         __HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);

//         // uint16_t ResivesLen = huart->RxXferSize - (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);

//         huart->hdmarx->Instance->CNDTR = huart->RxXferSize;
//         __HAL_DMA_ENABLE(huart->hdmarx);    
//         if (huart->Instance == USART1)
//         {
//             Dart_Trigger_Data.Communicate_Flag = Receive;
//             Dart_Trigger_Communicate(Dart_Trigger_Data.Communicate_Flag);
//             Dart_Trigger_Data.Frame_Cmd = NONE;
//         }
//         else if (huart->Instance == PC_UART_Instance)
//         {
//             HAL_UART_Receive_DMA(&PC_UART_Handle, Rx_Buf, sizeof(Rx_Buf));
//         }
//         else if (huart->Instance == Vision_UART_Instance)
//         {
//             Vision_Receive();
//         }

//     }
// }


