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

/* 镖头触发板通信(huart1 单线半双工, CRC8-MAXIM)状态与回读 */
typedef struct
{
	uint8_t Communicate_Flag;          /* 通信状态 None/Receive/Send */
	uint8_t Frame_Head;                /* 帧头 */
	uint8_t Frame_Cmd;                 /* 命令字 Status_Cheak/Color_Set */
	uint8_t Tx_Set_Team_Color;         /* 下发的队伍颜色 */
	uint8_t Rx_Set_Team_Color;         /* 回读的队伍颜色 */
	uint8_t Borad_Version;             /* 触发板固件版本 */
	uint8_t Borad_Temp;                /* 板载温度 */
	uint8_t Borad_State_Team;          /* 当前队伍 */
	uint8_t Borad_State_Light_ON;      /* 指示灯开 */
	uint8_t Borad_State_Voltage;       /* 电压状态 */
	uint8_t Borad_State_Light_Error;   /* 灯故障 */
	uint8_t Frame_Tail;                /* 帧尾 */
	uint32_t Dart_Trigger_Receive_Cnt; /* 收帧计数 */
}Dart_Trigger_Data_t;

/* 视觉(OpenMV, huart3 空闲DMA, 6字节帧)接收缓存。ISR(~20Hz)写,Guidance_Terminal(1kHz)读 */
typedef struct
{
	uint8_t Vision_Head;             /* 帧头:0x5A识别成功 / 0x5B距离+面积 / 0x7A丢目标 / 0x9A录制状态 */
	int16_t x[2];                    /* 视觉原始像素 x [NOW,LAST] */
	int16_t y[2];                    /* 视觉原始像素 y [NOW,LAST] */
	uint16_t dist_cm;                /* 目标距离 cm(0x5B 包,视觉端 DIST_K/sqrt(blob像素));越小=越近,末制导俯冲调度的剩余距离代理 */
	float area;                   /* 当前目标 blob 像素面积(0x5B 包,辅助/观测,不直接进调度) */
	float radius;                 /* 当前目标 blob 等效半径(像素,0x5B 包);用于归一化视线角误差:远小近大 */
	float Euler[2][2];  /* 像素→度 后的视线角 [NOW/LAST][0=pitch,1=yaw];供 Guidance_Terminal 锁存世界系视线目标 */
	float Euler_norm[2];            /* 半径归一化后的视线角 [0=pitch,1=yaw];= Euler × (REF_RADIUS/radius) */
	uint8_t Vision_Tail;             /* 帧尾 */
	uint8_t Vision_recognize_flag;   /* RECOGNIZE_SUCCESS / RECOGNIZE_FAILURE */
	uint8_t Vision_Self_Text_Data;   /* 视觉自检数据 */
	uint8_t Record_State[2];         /* 录制状态 [NOW,LAST] */
	uint32_t Vision_Receive_Cnt;     /* 收帧总计数 */
	uint32_t Vision_Recog_Cnt;       /* 仅"识别成功"帧递增,纯统计用 */
	uint8_t  Vision_New_Data_flag;   /* 视觉新有效数据到达置1; Guidance_Terminal 消费后清0,据此判新帧 → 锁存世界系视线目标 */
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
void Vision_Transmit_Debug(void);
#endif //CALLBACK_TASK_H
