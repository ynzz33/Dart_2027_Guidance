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
uint8_t DART_TYPE = FIXED_WING;
uint8_t Guidance_State;
Surface_t Surface;
Self_Text_t Self_Text;
uint8_t Wing_Servo_Control_Flag = 1;//舵机控制标志位
float Stable_Pitch,Stable_Yaw;
uint16_t end_cnt = 0;
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
    //Updata
    for (int i = 0;i<3;i++)
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
                    Surface.target_angle_Euler[NOW][PITCH] = 30;
                    Surface.target_angle_Euler[NOW][ROLL]  = 30;
                    Surface.target_angle_Euler[NOW][YAW]   = 30;
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
                    Surface.target_angle_Euler[NOW][PITCH] = -30;
                    Surface.target_angle_Euler[NOW][ROLL]  = -30;
                    Surface.target_angle_Euler[NOW][YAW]   = -30;
                    // Self_Text.Self_Text_Process = Self_Text_OK;
                }break;
                case Self_Text_Success:
                {
                    Surface.target_angle_Euler[NOW][PITCH] = 0;
                    Surface.target_angle_Euler[NOW][ROLL]  = 0;
                    Surface.target_angle_Euler[NOW][YAW]   = 0;
                    Self_Text.Self_Text_Process = Self_Text_OK;
                }break;
            }
        }break;
        case Self_Text_OK:
        {
        }break;
    }
}

void Guidance_Start(void)
{
    if (Self_Text.Vision_Self_Text_flag==Self_Text_Failure)
    {
        // if (Self_Text.Vision_Self_Text_flag_cnt<=10)
        // {
            // Self_Text.Vision_Self_Text_flag_cnt++;
            Vision_Self_Text();
        // }
    }
    else if (Self_Text.Dart_Trigger_Self_Text_flag==Self_Text_Failure)
    {
        Dart_Trigger_Self_Text();
        Self_Text.Dart_Trigger_Self_Text_flag_cnt++;
    }
    Self_Text_Process();
}
void Guidance_Stable(void)
{
        // Surface.target_angle_Euler[NOW][PITCH] = Low_Pass_Filter(Surface.current_angle_Euler[NOW][PITCH],Surface.current_angle_Euler[LAST][PITCH],0.7f);//(Stable_Pitch - Surface.current_angle_Euler[NOW][PITCH])/2;
        // Surface.target_angle_Euler[NOW][ROLL]  =  0;//Surface.current_angle_Euler[NOW][ROLL];
        // Surface.target_angle_Euler[NOW][YAW]   = Low_Pass_Filter(Surface.current_angle_Euler[NOW][YAW],Surface.current_angle_Euler[LAST][YAW],0.7f);  //Surface.current_angle_Euler[NOW][YAW];
    // Buzzer_play_song(song_ni);
        if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE)
        {
            Vision_Transmit(Vision_Cmd_Work);
        }
}
void Guidance_Terminal(void)
{
    // static uint8_t flag = 0;
    // if (flag==0)
    // {
    //     flag = 1;
    //
    // }
    if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_SUCCESS)
    {
        // PNG_Guidance(&Vision_Rx_Data,&PNG_Data,&Surface,&IMU_Data);


        Surface.target_angle_Euler[NOW][PITCH] = Vision_Rx_Data.x[NOW]+Surface.current_angle_Euler[NOW][PITCH]  ;                       //Surface.current_angle_Euler[NOW][PITCH];;
        Surface.target_angle_Euler[NOW][YAW]   = Vision_Rx_Data.y[NOW]+Surface.current_angle_Euler[NOW][YAW]    ;                       //Surface.current_angle_Euler[NOW][YAW]  ;;
        if (Vision_Rx_Data.y[NOW]<10&&Vision_Rx_Data.y[NOW]>-10)
        {
            Surface.target_angle_Euler[NOW][ROLL]  = Surface.current_angle_Euler[NOW][ROLL]/2;//Surface.current_angle_Euler[NOW][ROLL]
        }
        else
        {
            Surface.target_angle_Euler[NOW][ROLL]  = Vision_Rx_Data.y[NOW]*0.8;
        }
    }
    if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE)
    {
        Vision_Transmit(Vision_Cmd_Work);
        // PNG_Data.FOV_GYRO[X] = 0;
        // PNG_Data.FOV_GYRO[Y] = 0;
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH]  ;                                           ;//Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][ROLL]  = 0                                        ;                                           ;//Surface.current_angle_Euler[NOW][ROLL] ;
        Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW]    ;                                           ;//Surface.current_angle_Euler[NOW][YAW]  ;
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
    if (Surface.Text_Flag==1)
    {
        Surface.target_angle_Euler[NOW][PITCH] = 0;
        Surface.target_angle_Euler[NOW][ROLL]  = 0;
        Surface.target_angle_Euler[NOW][YAW]   = 0;
    }
    else
    {
        switch(Guidance_State)
        {
            case Start:
            {
                // if (Self_Text.Self_Text_Process==Self_Text_Start)
                // {
                    Guidance_Start();
                // }
            }break;
            case Stable:
            {
                Guidance_Stable();
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
}
void get_current_State(void)
{
    if (Self_Text.Self_Text_Process==Self_Text_OK)
    {
        Guidance_State = Start;
        if (Self_Text.Self_Text_Process<5)
        {
            // Vision_Transmit( Vision_Cmd_Record_Start );
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
    else if (Guidance_State == Start && IMU_Data.A[NOW][X]<=-5.0f&&Self_Text.Self_Text_Process==5)
    {
        Guidance_State = Stable;
        if (Self_Text.Self_Text_Process<5)
        {
            Vision_Transmit( Vision_Cmd_Record_Start );
            Self_Text.Self_Text_Process = 5;
        }
        // Stable_Pitch = IMU_Data.A_Normed[NOW][PITCH];
        // Vision_Transmit( Vision_Cmd_Record_Start );
    }
    else if (Guidance_State == Stable && IMU_Data.Euler[NOW][PITCH]<=0.0f)
    {
        Guidance_State = Terminal;
    }
    else if (Guidance_State == Terminal && IMU_Data.A[NOW][X]>=5.0f)
    {
        Guidance_State = End;
        Vision_Transmit( Vision_Cmd_Record_Stop );
    }
}


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
void Wing_Control(void)
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

/*---- 线程区 ----*/
void surface_control_task(void)
{
  /* USER CODE BEGIN surface_control_task */
  static uint32_t prev_tick;
  prev_tick = xTaskGetTickCount();

    /* 计算时间间隔（单位：秒）*/
    uint32_t current_tick = xTaskGetTickCount();
    float delta_time = (float)(current_tick - prev_tick) * 0.001f; // 1tick = 1ms
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
#if (DART_TYPE == FIXED_WING)//飞翼
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
#elif (DART_TYPE == VECTOR_NOZZLE)//应该是x翼的解算，先不动，后面再改
    float up_left_output    = -axis_roll.output - axis_pitch.output;
    float up_right_output   = -axis_roll.output - axis_pitch.output;
    float down_left_output  =  axis_roll.output - axis_pitch.output;
    float down_right_output =  axis_roll.output - axis_pitch.output;

    send_Servo_Command(SERVO_UP_LEFT,    up_left_output);
    send_Servo_Command(SERVO_UP_RIGHT,   up_right_output);
    send_Servo_Command(SERVO_DOWN_LEFT,  down_left_output);
    send_Servo_Command(SERVO_DOWN_RIGHT, down_right_output);
#endif
    if (Guidance_State == Start)
    {
        Surface.output_angle_Servo[NOW][Wing_left]    = Surface.target_angle_Euler[NOW][PITCH];
        Surface.output_angle_Servo[NOW][Wing_right]   = Surface.target_angle_Euler[NOW][ROLL];
        Surface.output_angle_Servo[NOW][Vertical_fin] = Surface.target_angle_Euler[NOW][YAW];
    }
    else if (Guidance_State == End||Guidance_State == PROCESS_OK)
    {
        Surface.output_angle_Servo[NOW][Wing_left]    = 0;
        Surface.output_angle_Servo[NOW][Wing_right]   = 0;
        Surface.output_angle_Servo[NOW][Vertical_fin] = 0;
    }
    if (Wing_Servo_Control_Flag == 1)
    {
        Wing_Control();
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

