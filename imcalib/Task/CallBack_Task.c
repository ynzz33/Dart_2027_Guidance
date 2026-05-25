//
// Created by ynz on 2025/12/3.
//
#include "ADC_Battery.h"
#include "Button.h"
#include "IMU.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "CallBack_Task.h"

#include "buzzer.h"
#include "FreeRTOS.h"
#include "surface_control_task.h"
#include "task.h"
#include "TotalControl.h"
#include "Vofa_send.h"
uint8_t Rx_Buf[7],Tx_Buf[7],Vision_Rx_Buf[6],Vision_Tx_Buf[3] = {0x5A,0,0xA5},Trigger_Rx_Buf[10],Trigger_Tx_Buf[5],flag = 0;
Dart_Trigger_Data_t Dart_Trigger_Data = {.Frame_Head = 0xAA,.Frame_Tail = 0x00};
Vision_Rx_Buf_t Vision_Rx_Data ;
static uint8_t bit_reverse(uint8_t byte) {
    byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1);
    byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2);
    byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4);
    return byte;
}

// CRC-8-MAXIM（输入反转+输出反转+初始值0x00+多项式0x31）
uint8_t crc8_maxim_with_reflect(uint8_t* data, uint16_t len) {
    uint8_t crc = 0x00;        // 初始值（协议要求）
    const uint8_t poly = 0x31; // 多项式x^8+x^5+x^4+1（0x31）

    for (uint16_t i = 0; i < len; i++) {
        // 步骤1：输入反转（每个字节先位反转）
        uint8_t reversed_byte = bit_reverse(data[i]);
        crc ^= reversed_byte;

        // 步骤2：逐bit处理（和之前逻辑一致）
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ poly) : (crc << 1);
        }
    }

    // 步骤3：输出反转（最终CRC结果位反转）
    crc = bit_reverse(crc);
    return crc;
}
void Dart_Trigger_Receive(Dart_Trigger_Data_t* Data)
{
    if (Trigger_Rx_Buf[0] == 0XAA)
    {
        switch (Data->Frame_Cmd)
        {
            case Status_Cheak:
            {
                Data->Borad_Version             = Trigger_Rx_Buf[3];
                Data->Borad_Temp                = Trigger_Rx_Buf[4];
                Data->Borad_State_Team          = Trigger_Rx_Buf[5]>>3&0x01;
                Data->Borad_State_Light_ON      = Trigger_Rx_Buf[5]>>2&0x01;
                Data->Borad_State_Voltage       = Trigger_Rx_Buf[5]>>1&0x01;
                Data->Borad_State_Light_Error   = Trigger_Rx_Buf[5]&0x01;

            }break;
            case Color_Set:
            {
                Data->Rx_Set_Team_Color = Trigger_Rx_Buf[3];
            }break;
        }
    }
    if (Self_Text.Dart_Trigger_Self_Text_flag==Self_Text_Failure)
    {
        if (Data->Borad_State_Voltage==0&&Data->Borad_State_Light_Error==0)
        {
            Self_Text.Dart_Trigger_Self_Text_flag=Self_Text_Success;
        }
    }
}
void Dart_Trigger_Transmit(Dart_Trigger_Data_t* Data)
{
    HAL_HalfDuplex_EnableTransmitter(&huart1);//单线串口:发送前置发送模式
    memset(Trigger_Tx_Buf, 0, sizeof(Trigger_Tx_Buf));
    switch (Data->Frame_Cmd)
    {
        case Status_Cheak:
        {
            Trigger_Tx_Buf[0] = 0XAA;
            Trigger_Tx_Buf[1] = Data->Frame_Cmd;
            Trigger_Tx_Buf[2] = 0x00;
            Data->Frame_Tail = crc8_maxim_with_reflect(Trigger_Tx_Buf,3);
            Trigger_Tx_Buf[3] = Data->Frame_Tail;
            HAL_UART_Transmit_DMA(&huart1, Trigger_Tx_Buf, 4);
        }break;
        case Color_Set:
        {
            Trigger_Tx_Buf[0] = 0XAA;
            Trigger_Tx_Buf[1] = Data->Frame_Cmd;
            Trigger_Tx_Buf[2] = 0x01;
            Trigger_Tx_Buf[3] = Data->Tx_Set_Team_Color;
            Data->Frame_Tail = crc8_maxim_with_reflect(Trigger_Tx_Buf,4);
            Trigger_Tx_Buf[4] = Data->Frame_Tail;
            HAL_UART_Transmit_DMA(&huart1, Trigger_Tx_Buf, 5);
        }break;
    }

}
void Dart_Trigger_Communicate(uint8_t Flag)
{
    switch (Flag)
    {
        case Receive:
        {
            Dart_Trigger_Receive(&Dart_Trigger_Data);
        }break;
        case Send:
        {
            Dart_Trigger_Transmit(&Dart_Trigger_Data);
        }break;

    }
}
void Dart_Trigger_State_Check(void)
{
    Dart_Trigger_Data.Communicate_Flag = Send;
    Dart_Trigger_Data.Frame_Cmd = Status_Cheak;
    Dart_Trigger_Communicate(Dart_Trigger_Data.Communicate_Flag);
}
void Dart_Trriger_Color_Set(uint8_t Team_Color)
{
    Dart_Trigger_Data.Communicate_Flag = Send;
    Dart_Trigger_Data.Tx_Set_Team_Color = Team_Color;
    Dart_Trigger_Data.Frame_Cmd = Color_Set;
    Dart_Trigger_Communicate(Dart_Trigger_Data.Communicate_Flag);
}
void Dart_Trigger_Power_Control(uint8_t Power_State)
{
    HAL_GPIO_WritePin( HEAD_PORT ,HEAD_PIN ,Power_State);
}
void Dart_Trigger_Self_Text(void)
{
    Dart_Trigger_Power_Control(Power_ON);
    Dart_Trigger_State_Check();
}


void Vision_Handle(uint8_t* Buf)
{
    Vision_Rx_Data.Vision_Head = Buf[0];
    Vision_Rx_Data.Vision_Tail = Buf[5];
    if (Buf[0]==0x5A&&Buf[5]==0xA5)
    {
    Vision_Rx_Data.x[NOW] = (int16_t)(Buf[1]<<8|Buf[2]);
    Vision_Rx_Data.y[NOW] = -(int16_t)(Buf[3]<<8|Buf[4]);
        Vision_Rx_Data.Vision_recognize_flag = RECOGNIZE_SUCCESS;
        HAL_GPIO_WritePin( LED_PORT,LED_PIN,GPIO_PIN_SET );
    if (Guidance_State==Terminal)
        Buzzer_play_song(song_ni);
    }
    else if(Buf[0]==0x7A&&Buf[5]==0xA7)
    {
        Vision_Rx_Data.x[NOW] = 0;
        Vision_Rx_Data.y[NOW] = 0;
        Vision_Rx_Data.Vision_recognize_flag = RECOGNIZE_FAILURE;
        HAL_GPIO_WritePin( LED_PORT,LED_PIN,GPIO_PIN_SET );
    }
    else if(Buf[0]==0x9A&&Buf[5]==0xA9)
    {
        Vision_Rx_Data.Record_State[0] = (int16_t)(Buf[1]<<8|Buf[2]);
        Vision_Rx_Data.Record_State[1] = (int16_t)(Buf[3]<<8|Buf[4]);
    }
}
void Vision_Receive(void)
{
    if (Self_Text.Vision_Self_Text_flag==Self_Text_Failure)
    {
        Vision_Rx_Data.Vision_Self_Text_Data = Vision_Rx_Buf[1];
        if (Vision_Rx_Data.Vision_Self_Text_Data==Vision_Cmd_Self_Text)
        {
            Self_Text.Vision_Self_Text_flag = Self_Text_Success;
            Self_Text.Self_Text_Process = Self_Text_Dart_Trigeer;
        }
    }
    else
    {
        Vision_Handle(Vision_Rx_Buf);
    }
    if (Self_Text.Vision_Self_Text_flag==0)
    {
        HAL_UART_Receive_DMA(&huart3, Vision_Rx_Buf, sizeof(Vision_Rx_Buf)-4);
    }
    else
    {
        HAL_UART_Receive_DMA(&huart3,Vision_Rx_Buf,sizeof(Vision_Rx_Buf) );
    }
}
void Vision_Transmit(uint8_t Cmd)
{
    Vision_Tx_Buf[1] = Cmd;
    HAL_UART_Transmit_DMA( &huart3,Vision_Tx_Buf,sizeof(Vision_Tx_Buf) );
}
void Vision_Self_Text(void)
{
    Vision_Transmit(Vision_Cmd_Self_Text);
}                      


void Total_Power_Control(uint8_t Power_State)
{
    HAL_GPIO_WritePin( POWER_PORT ,POWER_PIN ,Power_State);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        Dart_Trigger_Data.Communicate_Flag = Receive;
        Dart_Trigger_Communicate(Dart_Trigger_Data.Communicate_Flag);
        Dart_Trigger_Data.Frame_Cmd = NONE;
    }
    else if (huart->Instance == USART2)
    {
        HAL_UART_Receive_DMA(&huart2, Rx_Buf, sizeof(Rx_Buf));
    }
    else if (huart->Instance == USART3)
    {
        flag++;
        Vision_Receive();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_HalfDuplex_EnableReceiver(&huart1);//单线串口：发送后置接收模式
        switch (Dart_Trigger_Data.Frame_Cmd)
        {
            case Status_Cheak:
            {
                HAL_UART_Receive_DMA(&huart1, Trigger_Rx_Buf, 7);
            }break;
            case Color_Set:
            {
                HAL_UART_Receive_DMA(&huart1, Trigger_Rx_Buf, 5);
            }break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == Button_PIN)
    {
        if (Button_Data.TIM_INIT_FLAG==0)
        {
            HAL_TIM_Base_Start_IT(&htim15);
            Button_Data.Start_Time = HAL_GetTick();
            Button_Data.Press_State[LAST] = STATE_RELEASE;
            Button_Data.Press_Stable_State[LAST] = STATE_RELEASE;
            Button_Data.Press_State[NOW] = STATE_RELEASE;
            Button_Data.Press_Stable_State[NOW] = STATE_RELEASE;
            Button_Data.TIM_INIT_FLAG=1;
        }
        __HAL_GPIO_EXTI_CLEAR_IT(Button_PIN);
    }

}

 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
     /* USER CODE BEGIN Callback 0 */

     /* USER CODE END Callback 0 */
     if (htim->Instance == TIM1)
     {
         HAL_IncTick();
     }
     else if (htim->Instance == TIM6)
     {
         // Vofa_Send_Data2(control_cnt,IMU_Cnt);
         // IMU_Cnt = 0;
         // control_cnt = 0;
     }
     else if (htim->Instance == TIM7)
     {
         // IMU_Task();
         // IMU_Cnt++;
     }
     else if (htim->Instance == TIM15)
     {
          Button_Detect();
     }
     /* USER CODE BEGIN Callback 1 */
     /* USER CODE END Callback 1 */
 }
#include "CallBack_Task.h"
