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
#include <math.h>
#include "TotalControl.h"
#include "Vofa_send.h"
#include "adrc.h"
#include "../lqr_tool/lqr.h"
uint8_t Rx_Buf[7],Tx_Buf[7],Vision_Rx_Buf[6],Vision_Tx_Buf[3] = {0x5A,0,0xA5},Vision_TxDebug_Buf[50],Trigger_Rx_Buf[10],Trigger_Tx_Buf[5],flag = 0;
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
/* 解析触发板回包:Status_Cheak 拆出版本/温度/队伍/灯/电压状态位,电压与灯均正常则置自检通过;Color_Set 回读颜色 */
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
                if (Self_Text.Dart_Trigger_Self_Text_flag==Self_Text_Failure&&Data->Borad_State_Voltage==0&&Data->Borad_State_Light_Error==0)
                {
                    Self_Text.Dart_Trigger_Self_Text_flag=Self_Text_Success;
                }

            }break;
            case Color_Set:
            {
                Data->Rx_Set_Team_Color = Trigger_Rx_Buf[3];
            }break;
        }
    }
}
void Dart_Trigger_Transmit(Dart_Trigger_Data_t* Data)
{
    HAL_UART_AbortReceive(&DART_UART_Handle); 
    HAL_HalfDuplex_EnableTransmitter(&DART_UART_Handle);//单线串口:发送前置发送模式
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
            HAL_UART_Transmit_DMA(&DART_UART_Handle, Trigger_Tx_Buf, 4);
        }break;
        case Color_Set:
        {
            Trigger_Tx_Buf[0] = 0XAA;
            Trigger_Tx_Buf[1] = Data->Frame_Cmd;
            Trigger_Tx_Buf[2] = 0x01;
            Trigger_Tx_Buf[3] = Data->Tx_Set_Team_Color;
            Data->Frame_Tail = crc8_maxim_with_reflect(Trigger_Tx_Buf,4);
            Trigger_Tx_Buf[4] = Data->Frame_Tail;
            HAL_UART_Transmit_DMA(&DART_UART_Handle, Trigger_Tx_Buf, 5);
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
void Dart_Trigger_Self_Text(void)
{
    Dart_Trigger_State_Check();
}
// void Dart_Trigger_Power_Control(uint8_t Power_State)
// {
//     HAL_GPIO_WritePin( HEAD_PORT ,HEAD_PIN ,Power_State);
// }

void Vision_Receive(uint8_t* Buf)
{
    Vision_Rx_Data.Vision_Head = Buf[0];
    Vision_Rx_Data.Vision_Tail = Buf[5];
    if (Buf[0]==0x5A&&Buf[5]==0xA5)            /* 识别成功(6字节):像素 x,y */
    {
        Vision_Rx_Data.x[NOW] = (int16_t)(Buf[1]<<8|Buf[2]);
        Vision_Rx_Data.y[NOW] = (int16_t)(Buf[3]<<8|Buf[4]);
        // Vision_Rx_Data.Euler[NOW][0] = Vision_Rx_Data.y[NOW]/16.00f*72.0f;
        // Vision_Rx_Data.Euler[NOW][1] = Vision_Rx_Data.x[NOW]/12.00f*54.0f;
        Vision_Rx_Data.Euler[NOW][0] = Vision_Rx_Data.y[NOW]/160.0f*36.0f;
        Vision_Rx_Data.Euler[NOW][1] = Vision_Rx_Data.x[NOW]/120.0f*27.0f;
        Vision_Rx_Data.Vision_recognize_flag = RECOGNIZE_SUCCESS;
        Vision_Rx_Data.Vision_Recog_Cnt++;          /* 识别成功帧计数(纯统计) */
        Vision_Rx_Data.Vision_New_Data_flag = 1;    /* 视觉新有效数据到达 → 控制端 Guidance_Terminal 据此锁存世界系视线,消费后清0 */
        HAL_GPIO_WritePin( LED_PORT,LED_PIN,GPIO_PIN_SET );
        if (Guidance_State==Terminal)
        {
            Buzzer_Remind();
        }
    }
    else if(Buf[0]==0x5B&&Buf[5]==0xA6)        /* 距离+面积扩展包(6字节,独立于识别包):dist_cm/area 均 uint16 大端 */
    {
        Vision_Rx_Data.dist_cm = (uint16_t)(Buf[1]<<8|Buf[2]);   /* 目标距离 cm(视觉端 DIST_K/sqrt(blob像素)) */
        Vision_Rx_Data.area    = (float)(Buf[3]<<8|Buf[4]);   /* 当前目标 blob 像素面积 */
        /* 由面积算等效半径:r = sqrt(area/π),供视线角归一化用 */
        Vision_Rx_Data.radius  = ((float)sqrtf(Vision_Rx_Data.area / M_PI));
    }
    else if(Buf[0]==0x7A&&Buf[5]==0xA7)
    {
        Vision_Rx_Data.x[NOW] = 0;
        Vision_Rx_Data.y[NOW] = 0;
        Vision_Rx_Data.Vision_recognize_flag = RECOGNIZE_FAILURE;
        Vision_Rx_Data.Vision_New_Data_flag = 1;    /* 丢目标也是一帧新数据 → Guidance_Terminal 据此回中/解锁/请求视觉继续 */
        HAL_GPIO_WritePin( LED_PORT,LED_PIN,GPIO_PIN_SET );
    }
    else if(Buf[0]==0x9A&&Buf[5]==0xA9)
    {
        Vision_Rx_Data.Record_State[0] = (int16_t)(Buf[1]<<8|Buf[2]);
        Vision_Rx_Data.Record_State[1] = (int16_t)(Buf[3]<<8|Buf[4]);
    }
}
void Vision_Transmit(uint8_t Cmd)
{
    Vision_Tx_Buf[1] = Cmd;
    HAL_UART_Transmit_DMA( &Vision_UART_Handle,Vision_Tx_Buf,sizeof(Vision_Tx_Buf) );
}
void Vision_Self_Text(void)
{
    Vision_Transmit(Vision_Cmd_Self_Text);
}  
void Vision_Transmit_Debug(void)
{
    /* 12 个待发量,顺序固定(气动辨识用):
     * [0] 时间(ms→s)  [1] 速度V_lqr(m/s)  [2] roll_err(deg)
     * [3] pitch_err    [4] yaw_err         [5] p(deg/s)
     * [6] q            [7] r               [8] delta1 UL(deg)
     * [9] delta2 UR    [10] delta3 DR       [11] delta4 DL */
    float    val[12];
    uint8_t *src;
    uint8_t  i;

    // val[0]  = (Vision_Rx_Data.Vision_Recog_Cnt%10)*1000+Guidance_State*100+Surface.current_gyro_Euler[NOW][ROLL]/10.0f;
    // val[1]  = Vision_Rx_Data.x[NOW]*1000.0f+ADC_Voltage_Real;
    // val[2]  = Vision_Rx_Data.y[NOW]*1000.0f+IMU_Data.Velocity[Body][NOW][YAW]*10;
    // val[3]  = Surface.target_angle_Euler[NOW][PITCH] - Surface.current_angle_Euler[NOW][PITCH];
    // val[4]  = Surface.target_angle_Euler[NOW][ROLL]  - Surface.current_angle_Euler[NOW][ROLL]; 
    // val[5]  = Surface.target_angle_Euler[NOW][YAW]   - Surface.current_angle_Euler[NOW][YAW];
    // val[6]  = 11;
    // val[7]  = 11;
    // val[8]  = lqr_ctrl.u_servo_deg[0];
    // val[9]  = lqr_ctrl.u_servo_deg[1];
    // val[10] = lqr_ctrl.u_servo_deg[2];
    // val[11] = lqr_ctrl.u_servo_deg[3];
    
    val[0]  = IMU_Data.Velocity[Body][NOW][X];                          /* 时间 s */
    val[1]  = IMU_Data.Velocity[Body][NOW][Y];                                   /* 速度 m/s */
    val[2]  = IMU_Data.Velocity[Body][NOW][Z]    ; /* delta1 rad */
    val[3]  = Surface.output_angle_Servo[NOW][UP_RIGHT]   ; /* delta2 rad */
    val[4]  = Surface.output_angle_Servo[NOW][DOWN_RIGHT] ; /* delta3 rad */
    val[5]  = Surface.output_angle_Servo[NOW][DOWN_LEFT]  ; /* delta4 rad */
    val[6]  =  Surface.target_angle_Euler[NOW][ROLL]  - Surface.current_angle_Euler[NOW][ROLL]  ;
    val[7]  =  Surface.target_angle_Euler[NOW][PITCH] - Surface.current_angle_Euler[NOW][PITCH] ;
    val[8]  =  Surface.target_angle_Euler[NOW][YAW]   - Surface.current_angle_Euler[NOW][YAW]   ;
    val[9]  = Vision_Rx_Data.x[NOW]*1000.0f+ADC_Voltage_Real;
    val[10] = Vision_Rx_Data.y[NOW]*1000.0f+V_DART;
    val[11] = (Vision_Rx_Data.Vision_Recog_Cnt%10)*1000+Guidance_State*100+Surface.current_gyro_Euler[NOW][ROLL]/10.0f;






    // val[3]  = Surface.current_angle_Euler[NOW][YAW];
    // val[4]  = Surface.target_angle_Euler[NOW][YAW]; 
    // val[5]  = Surface.current_angle_Euler[NOW][PITCH];
    // val[6]  = surface_control_pid[Angle][YAW].pout;
    // val[7]  = surface_control_pid[Angle][YAW].iout;
    // val[8]  = surface_control_pid[Angle][YAW].dout;
    // val[9]  = Surface.current_gyro_Euler[NOW][YAW];
    // val[10] = temp[YAW];
    // val[11] = Surface.output_Body_Euler[NOW][YAW];


    // val[4]  = lqr_ctrl.err_deg[1] ;        
    // val[3]  = lqr_ctrl.err_deg[0] ;                    
    // val[5]  = lqr_ctrl.err_deg[2] ;
    // val[6]  = lqr_ctrl.axis_cmd_deg[0] ;
    // val[7]  = lqr_ctrl.axis_cmd_deg[1] ;   
    // val[8]  = lqr_ctrl.axis_cmd_deg[2] ;

    // val[3]  = surface_control_pid[Angle][PITCH].err[NOW]; 
    // val[4]  = surface_control_pid[Angle][ROLL].err[NOW];   
    // val[5]  = surface_control_pid[Angle][YAW].err[NOW];
    // val[6]  = Surface.output_Body_Euler[NOW][PITCH];
    // val[7]  = Surface.output_Body_Euler[NOW][ROLL];
    // val[8]  = Surface.output_Body_Euler[NOW][YAW];    

    // val[3]  = IMU_Data.Velocity[Body][NOW][PITCH];
    // val[4]  = IMU_Data.Velocity[Body][NOW][ROLL];
    // val[5]  = IMU_Data.Velocity[Body][NOW][YAW];o
    // val[9]  = lqr_ctrl.axis_cmd_deg[0] ;
    // val[10] = lqr_ctrl.axis_cmd_deg[1] ;
    // val[11] = lqr_ctrl.axis_cmd_deg[2] ;
    
    // val[0]  = IMU_Data.A_Normed[NOW][Y];
    // val[1]  = IMU_Data.A[NOW][Y];
    // val[2]  = Surface.current_gyro_Euler[NOW][ROLL];
    // val[3]  = ladrc_ctrl[LADRC_ROLL].z1;
    // val[4]  = ladrc_ctrl[LADRC_ROLL].z2;
    // val[5]  = ladrc_ctrl[LADRC_ROLL].z3;
    // val[0]  = temp[PITCH]; 
    // val[1]  = Surface.current_gyro_Euler[NOW][YAW];
    // val[2]  = temp[YAW];

    // val[0]  = IMU_Data.A[NOW][X];
    // val[1]  = IMU_Data.A[NOW][Y];
    // val[2]  = IMU_Data.A[NOW][Z];
    // val[3]  = IMU_Data.A_Normed[NOW][X];
    // val[4]  = IMU_Data.A_Normed[NOW][Y];
    // val[5]  = IMU_Data.A_Normed[NOW][Z];
    // val[6]  = Surface.current_angle_Euler[NOW][PITCH];
    // val[7]  = Surface.current_angle_Euler[NOW][ROLL];
    // val[8]  = Surface.current_angle_Euler[NOW][YAW];
    // val[9]  = Surface.current_gyro_Euler[NOW][PITCH];
    // val[10] = Surface.current_gyro_Euler[NOW][ROLL];
    // val[11] = Surface.current_gyro_Euler[NOW][YAW];

    /* 帧头0x57 + 12×float(大端,MSB先,每个4字节) + 帧尾0x75, 共50字节(Vision_TxDebug_Buf[50])。
     * 视觉端定长收50字节,校验首0x57/尾0x75后大端解析,如 Python: struct.unpack('>12f', buf[1:49])。 */
    Vision_TxDebug_Buf[0] = 0x57;
    for (i = 0; i < 12; i++)
    {
        src = (uint8_t *)&val[i];                       /* STM32 小端: src[0]=LSB ... src[3]=MSB */
        Vision_TxDebug_Buf[1 + i * 4 + 0] = src[3];     /* 大端: 高字节先发 */
        Vision_TxDebug_Buf[1 + i * 4 + 1] = src[2];
        Vision_TxDebug_Buf[1 + i * 4 + 2] = src[1];
        Vision_TxDebug_Buf[1 + i * 4 + 3] = src[0];
    }
    Vision_TxDebug_Buf[49] = 0x75;

    HAL_UART_Transmit_DMA(&Vision_UART_Handle, Vision_TxDebug_Buf, sizeof(Vision_TxDebug_Buf));
}                    


void Total_Power_Control(uint8_t Power_State)
{
    HAL_GPIO_WritePin( POWER_PORT ,POWER_PIN ,Power_State);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DART_UART_Instance)
    {
        HAL_HalfDuplex_EnableReceiver(&DART_UART_Handle);   //单线串口:发送后置接收模式
        //不定长接收:任意 IDLE 或收满 buffer 都会触发 RxEventCallback
        HAL_UARTEx_ReceiveToIdle_DMA(&DART_UART_Handle, Trigger_Rx_Buf, sizeof(Trigger_Rx_Buf));
        __HAL_DMA_DISABLE_IT(DART_UART_Handle.hdmarx, DMA_IT_HT);  //关掉半传输中断,只在 IDLE/TC 触发
    }
}

//------------------ 空闲接收事件回调:替换原来的 HAL_UART_RxCpltCallback ------------------
//触发时机: 1) 收到 IDLE (任意空闲间隔)  2) DMA 收满整个 buffer
//Size 是本次实际接收到的字节数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == DART_UART_Instance)
    {
        //触发板 Status_Cheak 回包 7 字节, Color_Set 回包 5 字节
        if (Size >= 4 && Trigger_Rx_Buf[0] == 0xAA)
        {
            Dart_Trigger_Receive(&Dart_Trigger_Data);
            Dart_Trigger_Data.Dart_Trigger_Receive_Cnt++;
        }
        Dart_Trigger_Data.Frame_Cmd = NONE;
        //半双工不在这里重启接收, 等下一次 TxCpltCallback 启动
    }
    else if (huart->Instance == PC_UART_Instance)
    {
        //调试串口收到数据先丢弃, 立即重启监听
        HAL_UARTEx_ReceiveToIdle_DMA(&PC_UART_Handle, Rx_Buf, sizeof(Rx_Buf));
        __HAL_DMA_DISABLE_IT(PC_UART_Handle.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == Vision_UART_Instance)
    {
        if (Self_Text.Vision_Self_Text_flag == Self_Text_Failure)
        {
            //自检包: head + cmd, 至少 2 字节
            Vision_Rx_Data.Vision_Head           = Vision_Rx_Buf[0];
            Vision_Rx_Data.Vision_Self_Text_Data = Vision_Rx_Buf[1];
            if (Size >= 2 && Vision_Rx_Data.Vision_Self_Text_Data == Vision_Cmd_Self_Text)
            {
                Self_Text.Vision_Self_Text_flag = Self_Text_Success;
                Self_Text.Self_Text_Process     = Self_Text_Dart_Trigeer;
                Self_Text.Self_Text_Process = Self_Text_OK;
            }
        }
        else if (Size == 6)
        {
            //正常包(全为6字节):识别0x5A / 距离面积0x5B / 丢目标0x7A / 录制0x9A,
            //Vision_Receive 内按 帧头Buf[0]+帧尾Buf[5] 校验,错位/错包自动丢弃
            Vision_Receive(Vision_Rx_Buf);
        }
        Vision_Rx_Data.Vision_Receive_Cnt++;
        HAL_UARTEx_ReceiveToIdle_DMA(&Vision_UART_Handle, Vision_Rx_Buf, sizeof(Vision_Rx_Buf));
        __HAL_DMA_DISABLE_IT(Vision_UART_Handle.hdmarx, DMA_IT_HT);
    }
}

//UART 错误回调:出错时也要把接收重新拉起来,否则会卡死
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DART_UART_Instance)
    {
        HAL_UART_AbortReceive(&DART_UART_Handle);
        //半双工不主动重启,等下一次发送
    }
    else if (huart->Instance == PC_UART_Instance)
    {
        HAL_UART_AbortReceive(&PC_UART_Handle);
        HAL_UARTEx_ReceiveToIdle_DMA(&PC_UART_Handle, Rx_Buf, sizeof(Rx_Buf));
        __HAL_DMA_DISABLE_IT(PC_UART_Handle.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == Vision_UART_Instance)
    {
        HAL_UART_AbortReceive(&Vision_UART_Handle);
        HAL_UARTEx_ReceiveToIdle_DMA(&Vision_UART_Handle, Vision_Rx_Buf, sizeof(Vision_Rx_Buf));
        __HAL_DMA_DISABLE_IT(Vision_UART_Handle.hdmarx, DMA_IT_HT);
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
         LQR_Gain_Update50Hz();   /* 50Hz 更新 LQR 速度调度增益 K_d(V) */
     }
     else if (htim->Instance == TIM15)
     {
          Button_Detect();
     }
     /* USER CODE BEGIN Callback 1 */
     /* USER CODE END Callback 1 */
 }
#include "CallBack_Task.h"
