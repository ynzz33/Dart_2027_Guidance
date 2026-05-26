//
// Created by ynz on 2025/12/3.
//
#include "stm32g4xx_hal.h"
#ifndef CALLBACK_TASK_H
#define CALLBACK_TASK_H

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_4

#define BUZZER_TIM_HANDLE        htim2
#define BUZZER_TIM_CHANNEL       TIM_CHANNEL_3

#define HEAD_PORT GPIOB
#define HEAD_PIN GPIO_PIN_6

#define Button_PORT GPIOA
#define Button_PIN GPIO_PIN_7

#define POWER_PORT GPIOA
#define POWER_PIN GPIO_PIN_6

#define DART_UART_Instance USART1
#define DART_UART_Handle huart1

#define PC_UART_Instance USART2
#define PC_UART_Handle huart2

#define Vision_UART_Instance USART3
#define Vision_UART_Handle huart3
enum
{
	None,
	Receive,
	Send,
};
enum
{
	Status_Cheak = 0x01,
	Color_Set = 0x02,
};
enum
{
	Team_RED = 0x00,
	Team_Blue = 0x01,
};
enum
{
	RECOGNIZE_FAILURE,
	RECOGNIZE_SUCCESS
};

enum
{
	Vision_Cmd_Work ,
	Vision_Cmd_Stop ,
	Vision_Cmd_Self_Text ,
	Vision_Cmd_Record_Start ,
	Vision_Cmd_Record_Stop
};
enum
{
	Power_OFF = GPIO_PIN_RESET,
	Power_ON = GPIO_PIN_SET,
};

typedef struct
{
	uint8_t Communicate_Flag;
	uint8_t Frame_Head;
	uint8_t Frame_Cmd;
	uint8_t Tx_Set_Team_Color;
	uint8_t Rx_Set_Team_Color;
	uint8_t Borad_Version;
	uint8_t Borad_Temp;
	uint8_t Borad_State_Team;
	uint8_t Borad_State_Light_ON;
	uint8_t Borad_State_Voltage;
	uint8_t Borad_State_Light_Error;
	uint8_t Frame_Tail;
	uint32_t Dart_Trigger_Receive_Cnt;
}Dart_Trigger_Data_t;

typedef struct
{
	uint8_t Vision_Head;
	int16_t x[2];
	int16_t y[2];
	uint8_t Vision_Tail;
	uint8_t Vision_recognize_flag;
	uint8_t Vision_Self_Text_Data;
	uint8_t Record_State[2];
	uint32_t Vision_Receive_Cnt;
} Vision_Rx_Buf_t;

extern uint8_t Rx_Buf[7],Tx_Buf[7],Vision_Rx_Buf[6],Trigger_Rx_Buf[10],Trigger_Tx_Buf[5],flag;
extern float ADC_Voltage_Real;
extern Dart_Trigger_Data_t Dart_Trigger_Data;
extern Vision_Rx_Buf_t Vision_Rx_Data;

void Dart_Trigger_Communicate(uint8_t Flag);
void Vision_Transmit(uint8_t Cmd);
void Vision_Self_Text(void);
void Dart_Trigger_State_Check(void);
void Dart_Trriger_Color_Set(uint8_t Team_Color);
void Dart_Trigger_Power_Control(uint8_t Power_State);
void Dart_Trigger_Self_Text(void);
void Total_Power_Control(uint8_t Power_State);
#endif //CALLBACK_TASK_H
