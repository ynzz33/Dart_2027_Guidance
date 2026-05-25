 //
// Created by ynz on 2025/11/27.
//
#include <stdint.h>

#ifndef SURFACE_CONTROL_TSAK_H
#define SURFACE_CONTROL_TSAK_H

#define  Wing_left_Channel    TIM_CHANNEL_2
#define  Wing_right_Channel   TIM_CHANNEL_3
#define  Vertical_fin_Channel TIM_CHANNEL_4

#define  ZERO_POINT 1500

#define  Wing_left_ZERO_POINT      1400  //变大向上，变小向下
#define  Wing_right_ZERO_POINT     1480  //变小向上，变大向下
#define  Vertical_fin_ZERO_POINT   1520

#define  Wing_up_Change_Angle       50.0f  // 250.0f
#define  Wing_down_Change_Angle    -100.0f  //-400.0f
#define  Vertical_fin_Change_Angle  250.0f  // 800.0f

 enum
{
    Wing_left,
    Wing_right,
    Vertical_fin
};

enum
{
    FIXED_WING,
    VECTOR_NOZZLE
};

enum
{
    Start,
    Stable,
    Terminal,
    End,
    PROCESS_OK
};

enum
{
    Angle,
    Gyro
};


enum
{
    Self_Text_Failure,
    Self_Text_Success,
};

enum
{
    Self_Text_Vision,
    Self_Text_Dart_Trigeer,
    Self_Text_OK,
    Self_Text_Start, 
};
typedef struct
{
    float output_angle_Servo [3][3];
    float current_angle_Euler[3][3];
    float target_angle_Euler [3][3];
    float current_gyro_Euler [3][3];
    float output_gyro_Euler  [3][3];
    float Finally_Angle[3][3];
    uint8_t pid_cale_flag;
    uint8_t Text_Flag;
}Surface_t;

typedef struct
{
    uint8_t Vision_Self_Text_flag;
    uint8_t Dart_Trigger_Self_Text_flag;
    uint16_t Vision_Self_Text_flag_cnt;
    uint16_t Dart_Trigger_Self_Text_flag_cnt;
    uint8_t Self_Text_Process;
}Self_Text_t;

extern Surface_t Surface;
extern Self_Text_t Self_Text;
extern uint8_t Guidance_State;
extern uint8_t Wing_Servo_Control_Flag;
extern uint16_t end_cnt ;
void surface_control_task(void);


#endif //SURFACE_CONTROL_TSAK_H
