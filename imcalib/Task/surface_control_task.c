/******************************************************************************
*** @File           : surface_control_task.c
*** @Description    : None
*** @Attention      : None
*** @Author         : Lmy
*** @Date           : 2025/3/2
*** @版权归属:
***                  ██╗███╗   ███╗ ██████╗ █████╗
***                  ██║████╗ ████║██╔════╝██╔══██╗
***                  ██║██╔████╔██║██║     ███████║
***                  ██║██║╚██╔╝██║██║     ██╔══██║
***                  ██║██║ ╚═╝ ██║╚██████╗██║  ██║
***                  ╚═╝╚═╝     ╚═╝ ╚═════╝╚═╝  ╚═╝
******************************************************************************/
/* 头文件 include(s) BEGIN */
#include "surface_control_task.h"

#include "buzzer.h"
#include "CallBack_Task.h"
#include "pid.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "IMU.h"
#include "tim.h"
#include "filter.h"
#include "PNG_Task.h"

/* 头文件 include(s) END */
/*---------------------------------------------------------------------------*/
/* 静态变量定义 static variable(s) BEGIN */

/* 静态变量定义 static variable(s) END */
/*---------------------------------------------------------------------------*/
/* 全局变量定义 global variable(s) BEGIN */
uint8_t DART_TYPE = VECTOR_NOZZLE;
uint8_t Guidance_State;
Surface_t Surface;
Self_Text_t Self_Text;
uint8_t Wing_Servo_Control_Flag = 1;//舵机控制标志位
float Stable_Pitch,Stable_Yaw;
uint16_t end_cnt = 0,Guidance_cnt[3] = {0};
float servo_lat_scale = 1.0f;   /* 横侧(roll/yaw)保留比例 k,Pitch 优先分配时被缩放,Vofa 可观测 */
/* 全局变量定义 global variable(s) END */
/*---------------------------------------------------------------------------*/


void Euler_Updata(void)
{
    //Updata
    for (int i = 0;i<3;i++)
    {
        Surface.target_angle_Euler [LLAST][i] = Surface.target_angle_Euler [LAST][i];
        Surface.target_angle_Euler [LAST ][i] = Surface.target_angle_Euler [NOW ][i];
        Surface.current_angle_Euler[LLAST][i] = Surface.current_angle_Euler[LAST][i];
        Surface.current_angle_Euler[LAST ][i] = Surface.current_angle_Euler[NOW ][i];
    }
}
void Servo_Updata(void)
{
    /* X 翼跑 4 个舵机,十字翼跑 3 个;output_angle_Servo 共用 [3][4] 缓存 */
    int n = (DART_TYPE == VECTOR_NOZZLE) ? 4 : 3;
    for (int i = 0; i < n; i++)
    {
        Surface.output_angle_Servo[LLAST][i] = Surface.output_angle_Servo[LAST][i];
        Surface.output_angle_Servo[LAST ][i] = Surface.output_angle_Servo[NOW ][i];
    }
}
                     
void Data_Updata(void)
{
    Euler_Updata();
    Servo_Updata();
}

void Guidance_Start(void)//自检后的判断
{
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];//pitch 靠机械/动力稳,目标=当前角:只阻尼,不死保绝对 0
        Surface.target_angle_Euler[NOW][ROLL]  = Surface.current_angle_Euler[NOW][ROLL];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
}
void Guidance_Stable(void)//自稳
{
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];//pitch 靠机械/动力稳,目标=当前角:只阻尼,不死保绝对 0
        Surface.target_angle_Euler[NOW][ROLL]  = Surface.current_angle_Euler[NOW][ROLL];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
        // Buzzer_play_song(song_ni);
        if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE)
        {
            Vision_Transmit(Vision_Cmd_Work);
        }
}
void Guidance_Terminal(void)//制导段
{
    taskENTER_CRITICAL();
    Vision_Rx_Buf_t v = Vision_Rx_Data;
    taskEXIT_CRITICAL();       
    // Surface.target_angle_Euler[NOW][PITCH]  = 0;
    // Surface.target_angle_Euler[NOW][ROLL]  = 0;
    // Surface.target_angle_Euler[NOW][YAW]  = 0;

    Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
    Surface.target_angle_Euler[NOW][ROLL]  = 0;
    Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];

    if (v.Vision_recognize_flag == RECOGNIZE_SUCCESS)
    {
        // PNG_Guidance(&Vision_Rx_Data,&PNG_Data,&Surface,&IMU_Data);
        if(Surface.current_angle_Euler[NOW][PITCH]<-10)
        {
            Surface.target_angle_Euler[NOW][PITCH] = v.x[NOW]+Surface.current_angle_Euler[NOW][PITCH]  ;
        }
        else
        {
            Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        }
        Surface.target_angle_Euler[NOW][PITCH] = v.x[NOW]+Surface.current_angle_Euler[NOW][PITCH]  ;
        Surface.target_angle_Euler[NOW][YAW]   = v.y[NOW]+Surface.current_angle_Euler[NOW][YAW]    ;
    }
    if (v.Vision_recognize_flag == RECOGNIZE_FAILURE)
    {
        Vision_Transmit(Vision_Cmd_Work);
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
    }  
}
void Guidance_End(void)
{
    // Dart_Trigger_Power_Control( Power_OFF );
    if (end_cnt++>50)
    {
        Vision_Transmit(Vision_Cmd_Record_Stop);
        Guidance_State = PROCESS_OK;
    }
    Wing_Servo_Control_Flag = 0;
    // Total_Power_Control( Power_OFF )//不能太快掉电，不然openmv保存不了视频等等
    // Buzzer_stop();
}


void get_current_Target(void)
{
        // Guidance_State = Stable;
        switch(Guidance_State)
        {
            case Start:
            {
                Guidance_Start();
            }break;
            case Stable:
            {
                if(IMU_Data.Euler[NOW][PITCH]<=20.0f)
                {
                    Guidance_Stable();
                }
            }break;
            case Terminal:
            {
                Guidance_Terminal();
            }break;
            case End:
            {
                Guidance_End();
            }break;
        }
}
void get_current_State(void)
{
    if (Self_Text.Self_Text_Process==Self_Text_OK)
    {
        Guidance_State = Start;
        if (Self_Text.Self_Text_Process<5)
        {
            Vision_Transmit( Vision_Cmd_Record_Start );
            Self_Text.Self_Text_Process = 5;
        }
        // static uint8_t cnt = 0 ;
        // if (cnt++>20&&cnt<30)
        // {
        //     Stable_Pitch = Surface.current_angle_Euler[NOW][PITCH];
        //     Stable_Yaw   = Surface.current_angle_Euler[NOW][YAW];
        //     cnt = 40 ;
        // }
    }
    else if (Guidance_State == Start && (IMU_Data.A_Normed[NOW][X] >= 0.80f||IMU_Data.A[NOW][X] <= -1.50f))
    {
        if (Guidance_cnt[0]++>5)
        {
            Guidance_State = Stable;
            Guidance_cnt[0] = 0;
            // Stable_Pitch = IMU_Data.A_Normed[NOW][PITCH];
            // Vision_Transmit( Vision_Cmd_Record_Start );
        }
    }
    else if (Guidance_State == Stable && IMU_Data.Euler[NOW][PITCH]<=0.0f)
    {
        if(Guidance_cnt[1]++>5)
        {
            Guidance_State = Terminal;
            Guidance_cnt[1] = 0;
        }
    }
    else if (Guidance_State == Terminal && IMU_Data.A[NOW][X]>= 1.50f&&IMU_Data.A_Normed[NOW][X] >= 0.80f)
    {
        if(Guidance_cnt[2]++>5)
        {
            Guidance_State = End;
            Vision_Transmit( Vision_Cmd_Record_Stop );
            Guidance_cnt[2] = 0;
        }
    }
}

#if 0 // DART_TYPE == FIXED_WING  (飞翼路径保留作参考,当前主程序只走 X 翼,故不编译)
void Wing_left_Control(float data)
{
    //变大向上，变小向下
    Surface.Finally_Angle[NOW][Wing_left]  =  (data/90.0f*1000.0f);
    if (Surface.Finally_Angle[NOW][Wing_left]>=Wing_up_Change_Angle)
    {
        Surface.Finally_Angle[NOW][Wing_left]=Wing_up_Change_Angle;
    }
    else if (Surface.Finally_Angle[NOW][Wing_left]<=Wing_down_Change_Angle)
    {
        Surface.Finally_Angle[NOW][Wing_left]=Wing_down_Change_Angle;
    }
    Surface.Finally_Angle[NOW][Wing_left] = Wing_left_ZERO_POINT - Surface.Finally_Angle[NOW][Wing_left] ;
    if (Guidance_State==Stable)
    {
        Surface.Finally_Angle[NOW][Wing_left] = Wing_left_ZERO_POINT;
    }
    // __HAL_TIM_SET_COMPARE( &htim4,Wing_left_Channel   ,Surface.Finally_Angle[NOW][Wing_left]);
    __HAL_TIM_SET_COMPARE( &htim4,Wing_left_Channel,Wing_left_ZERO_POINT);

} 
void Wing_right_Control(float data)
{
    //变小向上，变大向下
    Surface.Finally_Angle[NOW][Wing_right]  =  (data/90*1000);
    if (Surface.Finally_Angle[NOW][Wing_right]>=Wing_up_Change_Angle )
    {
        Surface.Finally_Angle[NOW][Wing_right]=Wing_up_Change_Angle;
    }
    else if (Surface.Finally_Angle[NOW][Wing_right]<=Wing_down_Change_Angle)
    {
        Surface.Finally_Angle[NOW][Wing_right]=Wing_down_Change_Angle;
    }
    Surface.Finally_Angle[NOW][Wing_right] = Wing_right_ZERO_POINT + Surface.Finally_Angle[NOW][Wing_right];
    if (Guidance_State==Stable)
    {
        Surface.Finally_Angle[NOW][Wing_right] = Wing_right_ZERO_POINT;
    }
    __HAL_TIM_SET_COMPARE( &htim4,Wing_right_Channel  ,Surface.Finally_Angle[NOW][Wing_right]);
    // __HAL_TIM_SET_COMPARE( &htim4,Wing_right_Channel,Wing_right_ZERO_POINT);
}
void Vertical_fin_Control(float data)
{
    float Vertical_fin_Angle = data/90*1000;
    if (Vertical_fin_Angle>=Vertical_fin_Change_Angle)
    {
        Vertical_fin_Angle= Vertical_fin_Change_Angle ;
    }
    else if (Vertical_fin_Angle<= -Vertical_fin_Change_Angle)
    {
        Vertical_fin_Angle = -Vertical_fin_Change_Angle ;
    }
    Surface.Finally_Angle[NOW][Vertical_fin] = Vertical_fin_ZERO_POINT + Vertical_fin_Angle;
    if (Guidance_State==Stable)
    {
        Surface.Finally_Angle[NOW][Vertical_fin] = Vertical_fin_ZERO_POINT;
    }
    // __HAL_TIM_SET_COMPARE( &htim4,Vertical_fin_Channel,Surface.Finally_Angle[NOW][Vertical_fin]);
    __HAL_TIM_SET_COMPARE( &htim4,Vertical_fin_Channel,Vertical_fin_ZERO_POINT);
}
void Wing_Control_FIXED_WING(void)
{
    // if (Guidance_State==End||(Guidance_State==Terminal&&Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE))
    // {
    //
    // }
    // else
    // {

        // Surface.output_angle_Servo[NOW][Wing_right]     = Near_By_Process(Surface.output_angle_Servo[NOW][Wing_right]   ,Surface.output_angle_Servo[LAST][Wing_right]   ,180);
        // Surface.output_angle_Servo[NOW][Wing_left]      = Near_By_Process(Surface.output_angle_Servo[NOW][Wing_left]    ,Surface.output_angle_Servo[LAST][Wing_left]    ,180);
        // Surface.output_angle_Servo[NOW][Vertical_fin]   = Near_By_Process(Surface.output_angle_Servo[NOW][Vertical_fin] ,Surface.output_angle_Servo[LAST][Vertical_fin] ,180);
        Wing_right_Control(  Surface.output_angle_Servo[NOW][Wing_right]  );
        Wing_left_Control(   Surface.output_angle_Servo[NOW][Wing_left]   );
        Vertical_fin_Control(Surface.output_angle_Servo[NOW][Vertical_fin]);
    // }
}
#endif
#if 1 //DART_TYPE == VECTOR_NOZZLE
/* X 翼 4 个舵机 PWM 写入:接 TIM3 CH2 TIM4 CH2-CH4 (PB6/PB7/PB8/PB9)
 * 输入 data 单位是度(±90 内), 内部映射到 PWM 微秒并做 ZERO 偏置 + 限幅
 */
void Wing_UL_Control(float data)//硬件原因导致左上舵机接在了 TIM3 上，所以单独写函数控制
{
    Surface.Finally_Angle[NOW][UP_LEFT]  = Servo_UL_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Surface.Finally_Angle[NOW][UP_LEFT]);
    
    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_UL_ZERO);
}
void Wing_UR_Control(float data)
{ 
    Surface.Finally_Angle[NOW][UP_RIGHT] = Servo_UR_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_UR_Channel, Surface.Finally_Angle[NOW][UP_RIGHT]);
    
    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_UR_ZERO);
}
void Wing_DL_Control(float data)
{
    Surface.Finally_Angle[NOW][DOWN_LEFT]  = Servo_DL_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_DL_Channel, Surface.Finally_Angle[NOW][DOWN_LEFT]);

    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_DR_ZERO);
}
void Wing_DR_Control(float data)
{
    Surface.Finally_Angle[NOW][DOWN_RIGHT] = Servo_DR_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_DR_Channel, Surface.Finally_Angle[NOW][DOWN_RIGHT]);

    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_DL_ZERO);
}
/* Pitch 优先最小能量控制分配:
 * pitch 全额保留,roll/yaw 等比缩到舵机余量内 → 调 roll/yaw 不污染 pitch。
 * 写入 Surface.output_angle_Servo[NOW][0..3](已含 SIGN,已在 ±SERVO_ANGLE_LIMIT 内)。*/
void Servo_Mix_PitchPriority(float p, float r, float y)
{
    float pitch_limit = SERVO_ANGLE_LIMIT-20;                   //pitch优先，但也有限幅
    abs_limit(&p, pitch_limit);                           /* 1) pitch 分量优先*/

    float P[4], L[4];                                           /* 2) 拆 pitch 分量 / 横侧分量(逻辑符号阵) */
    P[UP_LEFT]    = +p;  L[UP_LEFT]    = +r - y;
    P[UP_RIGHT]   = +p;  L[UP_RIGHT]   = -r + y;
    P[DOWN_LEFT]  = -p;  L[DOWN_LEFT]  = +r + y;
    P[DOWN_RIGHT] = -p;  L[DOWN_RIGHT] = -r - y;

    float k = 1.0f;                                             /* 3) 求最大横侧比例 k */
    for (int i = 0; i < 4; i++)
    {
        float aL = (L[i] < 0.0f) ? -L[i] : L[i];                // 绝对值
        if (aL < 1e-6f) continue;                               /* 该片无横侧分量,不构成约束 */
        float sgnL = (L[i] < 0.0f) ? -1.0f : 1.0f;              //拿到原来的符号以便复原
        float ki = aL / (SERVO_ANGLE_LIMIT - sgnL * P[i]) ;      //总限幅减去pitch原始再除以横侧分量，得到比例
        if (ki < k) k = ki;                                     //在比例内就按原样，不动，如果超出比例就按比例缩放，保证在限幅内
    }
    if (k < 0.0f) k = 0.0f;
    servo_lat_scale = k;

    float SGN[4];                                               /* 4) 最终的合成 + 物理方向 SIGN 写入 */
    SGN[UP_LEFT]   = SIGN_UL;  SGN[UP_RIGHT]   = SIGN_UR;
    SGN[DOWN_LEFT] = SIGN_DL;  SGN[DOWN_RIGHT] = SIGN_DR;
    for (int i = 0; i < 4; i++)
        Surface.output_angle_Servo[NOW][i] = SGN[i] * (P[i] + k * L[i]);
}

void Wing_Control_VECTOR_NOZZLE(void)
{
    if (Guidance_State == Terminal||Guidance_State == Self_Text_State)
    {
        Wing_UL_Control(Surface.output_angle_Servo[NOW][UP_LEFT]    );
        Wing_UR_Control(Surface.output_angle_Servo[NOW][UP_RIGHT]   );
        Wing_DL_Control(Surface.output_angle_Servo[NOW][DOWN_LEFT]  );
        Wing_DR_Control(Surface.output_angle_Servo[NOW][DOWN_RIGHT] );
    }
    else
    {
        // 末端失去目标或制导结束,舵面回中
        Wing_UL_Control(0.0f);
        Wing_UR_Control(0.0f);
        Wing_DL_Control(0.0f);
        Wing_DR_Control(0.0f);
        return;
    }
        // Wing_UL_Control(0.0f);
        // Wing_UR_Control(0.0f);
        // Wing_DL_Control(0.0f);
        // Wing_DR_Control(0.0f);
}
#endif
/*---- 线程区 ----*/
void surface_control_task(void)
{
  /* USER CODE BEGIN surface_control_task */
    static uint32_t prev_tick = 0;
    static uint8_t  first_run = 1;
    uint32_t current_tick = xTaskGetTickCount();
    float delta_time;
    if (first_run)
    {
        delta_time = (float)CTRL_PERIOD_MS * 0.001f;
        first_run = 0;
    }
    else
    {
        delta_time = (float)(current_tick - prev_tick) * 0.001f;
        if (delta_time < 1e-4f) delta_time = (float)CTRL_PERIOD_MS * 0.001f;
    }
    prev_tick = current_tick;
    /*状态机，拿目标值*/
    get_current_State();
    get_current_Target();
    /*pid，算输出值*/
    Surface.pid_cale_flag = 0;
    if (Guidance_State==Stable||Guidance_State==Terminal)
    {
        Surface.pid_cale_flag = 1;
        Euler_pid_Cale(delta_time);//这部分先根据当前欧拉角计算差值，坐标系为东北天
    }
    /*解算到舵面*/
    if(DART_TYPE == VECTOR_NOZZLE   )//x翼
    {
        if (Guidance_State==Stable||Guidance_State==Terminal)
        {
            /* Pitch 优先最小能量分配:pitch 全保,roll/yaw 等比缩进限幅,不污染 pitch。
             * SIGN_xx 仍在函数内逐片乘上,标定流程不变。 */
            Servo_Mix_PitchPriority(Surface.output_gyro_Euler[NOW][PITCH],
                                    Surface.output_gyro_Euler[NOW][ROLL],
                                    Surface.output_gyro_Euler[NOW][YAW]);
        }
        for (int i = 0; i < 4; i++)                    /* 安全网:分配已保证在限内,此处仅兜底 FP 误差 */
            abs_limit(&Surface.output_angle_Servo[NOW][i], SERVO_ANGLE_LIMIT);
    }
    if(DART_TYPE == FIXED_WING    )//飞翼
    {
        /*东北天坐标系前*/
        /*这部分，伯努利原理去想，流速快气压小，其他被舵面撞击所以流速减小，也就是舵面往哪里转，就会有一个反方向的力*/
        /*pitch的话，因为上面pid计算出来的是差值，也就是会向目标方向转，所以直接进行*/
        if (Guidance_State==Stable||Guidance_State==Terminal)
        {
                    Surface.output_angle_Servo[NOW][Vertical_fin] =  +Surface.output_gyro_Euler[NOW][YAW];
            // if (Surface.current_angle_Euler[NOW][PITCH]<=-5)
            //     {
                    Surface.output_angle_Servo[NOW][Wing_left]    =(-Surface.output_gyro_Euler[NOW][ROLL]*0.6f + Surface.output_gyro_Euler[NOW][PITCH]*0.0f);
                    Surface.output_angle_Servo[NOW][Wing_right]   =(+Surface.output_gyro_Euler[NOW][ROLL]*0.6f + Surface.output_gyro_Euler[NOW][PITCH]*0.0f);
                // }
        }
        Low_Pass_Filter(Surface.output_angle_Servo[NOW][Wing_left]    ,Surface.output_angle_Servo[LAST][Wing_left]    ,0.7);
        Low_Pass_Filter(Surface.output_angle_Servo[NOW][Wing_right]   ,Surface.output_angle_Servo[LAST][Wing_right]   ,0.7);
        Low_Pass_Filter(Surface.output_angle_Servo[NOW][Vertical_fin] ,Surface.output_angle_Servo[LAST][Vertical_fin] ,0.7);


        abs_limit(&Surface.output_angle_Servo[NOW][Wing_left]    ,60);
        abs_limit(&Surface.output_angle_Servo[NOW][Wing_right]   ,60);
        abs_limit(&Surface.output_angle_Servo[NOW][Vertical_fin] ,60);
        if (Guidance_State == Start)
        {
            Surface.output_angle_Servo[NOW][Wing_left]    = Surface.target_angle_Euler[NOW][PITCH];
            Surface.output_angle_Servo[NOW][Wing_right]   = Surface.target_angle_Euler[NOW][ROLL];
            Surface.output_angle_Servo[NOW][Vertical_fin] = Surface.target_angle_Euler[NOW][YAW];
        }
    }
    if (Guidance_State == End || Guidance_State == PROCESS_OK)
    {
        for (int i = 0; i < 4; i++) Surface.output_angle_Servo[NOW][i] = 0;
    }
    if (Wing_Servo_Control_Flag == 1)
    {      
        if(Guidance_State == Self_Text_State)
        {
            Surface.output_angle_Servo[NOW][UP_LEFT]    = 30;
            Surface.output_angle_Servo[NOW][UP_RIGHT]   = 30;
            Surface.output_angle_Servo[NOW][DOWN_LEFT]  = 30;
            Surface.output_angle_Servo[NOW][DOWN_RIGHT] = 30;
        }
        else if(Guidance_State == Start)
        {
            Surface.output_angle_Servo[NOW][UP_LEFT]    = 0;
            Surface.output_angle_Servo[NOW][UP_RIGHT]   = 0;
            Surface.output_angle_Servo[NOW][DOWN_LEFT]  = 0;
            Surface.output_angle_Servo[NOW][DOWN_RIGHT] = 0; 
        }
        if(DART_TYPE == VECTOR_NOZZLE   )//x翼
        {
            Wing_Control_VECTOR_NOZZLE();
        }
        else if(DART_TYPE == FIXED_WING    )//飞翼
        {
            // Wing_Control_FIXED_WING();
        }
    }
    Data_Updata();
    if (Self_Text.Self_Text_Process == Self_Text_Dart_Trigeer&&Self_Text.Dart_Trigger_Self_Text_flag == Self_Text_Failure && Self_Text.Vision_Self_Text_flag == Self_Text_Success)
    {
        osDelay( 300 );
    }
  /* USER CODE END surface_control_task */
}

//@Author:Liko                     .~---------~.
//                                /'  | === \  '\
//                                |'  | |_/ /  '|
//                                |'  | |\ /   '|
//                                |'__\_| \_\__'|
//                                |>----(+)----<|
//                             _~/|'###########'|\~_
//                          _~/   |'###########'|   \~_
//                       _~/      |'###########'|      \~_
//                    _~/         |'###########'|         \~_
//                 _~/            |'###########'|            \~_
//              _~/               |'###########'|               \~_
//           _~/     '          ' |'###########'| '          '     \~_
//        _~/           '     '   |'###########'|   '     '           \~_
//     _~/                 '      |'###########'|      '                 \~_
//   _/    '            '     '   ||___-|_|-___||   '     '            '    \_
//  //         '     '          ' ||  /_| |_\  || '          '     '         \\
// /'             '               || // | | \\ ||               '             '\
// |'          '     '            ||//  | |  \\||            '     '          '|
// |'       '           '         |' /~-| |-~\ '|         '           '       '|
// |'    '                 '     /''   |===|   ''\     '                 '    '|
// |'  '                      ' / ''           '' \ '                       ' '|
// |'   ____________________-__/ /''           ''\ \__-____________________   '|
// \'   |          ||_+_____| /  |==||==| |==||==|  \ |_____+_||          |   '/
// ^\.._|          '~\V/~~~/  |-----'_ _ _ _ _'-----|  \~~~\V/~'          |_../^
//      ^~--..__          /       |____|===|____|       \          __..--~^
//              ^~--...__/^                             ^\__...--~^

/*---- 函数区 ----*/

