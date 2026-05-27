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
/* === X 翼 4 舵机布局(VECTOR_NOZZLE 模式使用),不影响原飞翼路径 === */
//硬件原因导致左上舵机接在了 TIM3 上，所以单独写函数控制
#define  Servo_UL_Channel   TIM_CHANNEL_2   /* PB5 - UP_LEFT    */
#define  Servo_UR_Channel   TIM_CHANNEL_2   /* PB7 - UP_RIGHT   */
#define  Servo_DR_Channel   TIM_CHANNEL_3   /* PB8 - DOWN_LEFT  */
#define  Servo_DL_Channel   TIM_CHANNEL_4   /* PB9 - DOWN_RIGHT */

#define  Servo_UL_ZERO      1570
#define  Servo_UR_ZERO      1660
#define  Servo_DR_ZERO      1570
#define  Servo_DL_ZERO      1400

#define  Servo_PWM_MIN      1000
#define  Servo_PWM_MAX      2000

/* X 翼方向系数:台架联调时单轴阶跃,反向了翻号(不要动公式) */
#define  SIGN_UL  (-1.0f)
#define  SIGN_UR  (+1.0f)
#define  SIGN_DR  (+1.0f)
#define  SIGN_DL  (-1.0f)

 enum
{
    Wing_left,
    Wing_right,
    Vertical_fin
};

enum
{
    FIXED_WING = 0,
    VECTOR_NOZZLE = 1
};

/* X 翼舵机索引,值与 Wing_left/Wing_right/Vertical_fin 重合但语义不同,
 * 二者通过 DART_TYPE 分支区别使用,数据底仓共享 Surface.output_angle_Servo[3][4] */
enum
{
    UP_LEFT     = 0,
    UP_RIGHT    = 1,
    DOWN_RIGHT  = 2,
    DOWN_LEFT   = 3,
    SERVO_COUNT_X = 4
};

enum
{
    Self_Text_State,
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
    float output_angle_Servo [3][4];
    float current_angle_Euler[3][3];
    float target_angle_Euler [3][3];
    float current_gyro_Euler [3][3];
    float output_gyro_Euler  [3][3];
    float Finally_Angle      [3][4];
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
extern uint16_t Guidance_cnt[3];
void surface_control_task(void);


#endif //SURFACE_CONTROL_TSAK_H
