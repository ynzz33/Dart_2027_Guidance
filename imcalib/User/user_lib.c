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
    // if(DART_TYPE == FIXED_WING    )//飞翼
    // {
    //     /*东北天坐标系前*/
    //     /*这部分，伯努利原理去想，流速快气压小，其他被舵面撞击所以流速减小，也就是舵面往哪里转，就会有一个反方向的力*/
    //     /*pitch的话，因为上面pid计算出来的是差值，也就是会向目标方向转，所以直接进行*/
    //     if (Guidance_State==Stable||Guidance_State==Terminal)
    //     {
    //                 Surface.output_angle_Servo[NOW][Vertical_fin] =  +Surface.output_gyro_Euler[NOW][YAW];
    //         // if (Surface.current_angle_Euler[NOW][PITCH]<=-5)
    //         //     {
    //                 Surface.output_angle_Servo[NOW][Wing_left]    =(-Surface.output_gyro_Euler[NOW][ROLL]*0.6f + Surface.output_gyro_Euler[NOW][PITCH]*0.0f);
    //                 Surface.output_angle_Servo[NOW][Wing_right]   =(+Surface.output_gyro_Euler[NOW][ROLL]*0.6f + Surface.output_gyro_Euler[NOW][PITCH]*0.0f);
    //             // }
    //     }
    //     Low_Pass_Filter(Surface.output_angle_Servo[NOW][Wing_left]    ,Surface.output_angle_Servo[LAST][Wing_left]    ,0.7);
    //     Low_Pass_Filter(Surface.output_angle_Servo[NOW][Wing_right]   ,Surface.output_angle_Servo[LAST][Wing_right]   ,0.7);
    //     Low_Pass_Filter(Surface.output_angle_Servo[NOW][Vertical_fin] ,Surface.output_angle_Servo[LAST][Vertical_fin] ,0.7);


    //     abs_limit(&Surface.output_angle_Servo[NOW][Wing_left]    ,60);
    //     abs_limit(&Surface.output_angle_Servo[NOW][Wing_right]   ,60);
    //     abs_limit(&Surface.output_angle_Servo[NOW][Vertical_fin] ,60);
    //     if (Guidance_State == Start)
    //     {
    //         Surface.output_angle_Servo[NOW][Wing_left]    = Surface.target_angle_Euler[NOW][PITCH];
    //         Surface.output_angle_Servo[NOW][Wing_right]   = Surface.target_angle_Euler[NOW][ROLL];
    //         Surface.output_angle_Servo[NOW][Vertical_fin] = Surface.target_angle_Euler[NOW][YAW];
    //     }
    // }
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



