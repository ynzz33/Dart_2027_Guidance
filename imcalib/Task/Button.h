//
// Created by ynz on 2025/12/3.
//

#ifndef BUTTON_H
#define BUTTON_H
#include <stdint.h>

#define Debounce_Limit 30                       //消抖时间
#define SHORT_Press_Time_Recognize 50          //短按判断时间
#define LONG_Press_Time_Recognize 500           //长按判断时间
#define LONG_Press_Cnt_Update 750              //长按计数更新
#define MAX_Press_Long_Cnt 10                    //最长长按时间，秒，在计次那边做处理，用秒计算
#define MAX_Press_Short_Cnt 10                    //最多短按次数
#define MAX_SHORT_Press_Time_END 600
#define MAX_Press_Time_END 1500
enum
{
    NONE            = 0,            //空状态，初始态
    STATE_PRESS     = 1,            //按下状态
    STATE_RELEASE   = 2,            //未按下状态
    Event_PRESS     = 3,            //按下事件处理
    Event_RELEASE   = 4,            //松手事件处理
    TYPE_SHORT      = 5,            //短按态
    TYPE_LONG       = 6,            //长按态
    TYPE_STOP       = 7,            //停止态
};
typedef struct
{
    uint8_t TIM_INIT_FLAG;
    uint8_t Press_State[2];                  //当前gpio状态
    uint8_t Press_Stable_State[2];           //消抖稳定后的状态
    uint8_t Continuous_FLag;                 //是否松手标志位
    uint8_t Press_Type;                      //按键输入类型，长短按
    uint16_t Debounce_Time;                  //消抖时间

    uint16_t Press_Short_Cnt;                //短按计数数
    uint16_t Press_Long_Cnt[2];                 //长按计数
    //======短按判断及短按次数
    uint8_t  Short_Press_Temp;                //判断多次短按的状态机标志位
    uint32_t Short_Press_Start_Time;
    //======长短按判断以及长按时间判断
    uint32_t Start_Time;                     //开始按下的时间
    uint32_t Continuous_Time;                //按下持续的时间，判断是否长按
    uint32_t Current_Time;

    uint8_t Remind_Flag;
}Button_Data_t;

extern Button_Data_t Button_Data;

void Button_Detect(void);

#endif //BUTTON_H
