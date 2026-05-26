//
// Created by ynz on 2025/12/3.
//

#include "Button.h"

#include "buzzer.h"
#include "CallBack_Task.h"
#include "IMU.h"
#include "main.h"
#include "tim.h"
#include "Vofa_send.h"
#include "FreeRTOS.h"
#include "surface_control_task.h"
#include "task.h"
#include "usart.h"

Button_Data_t Button_Data = {0};
Buzzer_note_t Long_notes[10] = {{M2,50},{M3b,50},{M3,50},{M4,50},{M5b,50},{M5,50},{M6b,50},{M6,50},{M7b,50},{M7,50},};
Buzzer_note_t Short_notes[10] = {{M2,50},{M3b,50},{M3,50},{M4,50},{M5b,50},{M5,50},{M6b,50},{M6,50},{M7b,50},{M7,50},};


void Press_Remind(void)
{
        switch (Button_Data.Press_Type)
        {
                case TYPE_SHORT:
                {
                        if (Button_Data.Press_Short_Cnt!=0&&Button_Data.Press_Short_Cnt<=10)
                        {
                                Buzzer_message.note = Short_notes[Button_Data.Press_Short_Cnt-1];
                                Buzzer_message.cmd = BUZZER_CMD_PLAY_NOTE;
                        }
                }break;
                case TYPE_LONG:
                {
                        if (Button_Data.Press_Long_Cnt!=0&&Button_Data.Press_Long_Cnt[NOW]<=10)
                        {
                                Buzzer_message.note = Long_notes[Button_Data.Press_Long_Cnt[NOW]-1];
                                Buzzer_message.cmd = BUZZER_CMD_PLAY_NOTE;
                        }
                }break;
                default:
                {
                                // Buzzer_message.cmd = BUZZER_CMD_STOP;
                }break;
        }

}
void Short_Press_Process(void)
{
        switch(Button_Data.Press_Short_Cnt)
        {
                case 1:
                {
                        // ==================电池上电==============
                        // Total_Power_Control(Power_ON);
                        // ==================视觉自检，内录==============
                        Vision_Self_Text();
                        Guidance_State = Start;
                        // Vision_Transmit(Vision_Cmd_Work);
                        // Vision_Transmit( Vision_Cmd_Record_Start );
                        // ==================阵营切换-红==============
                        // Dart_Trriger_Color_Set(Team_RED);

                }break;
                case 2:
                {
                        // ==================视觉内录==============
                        // Vision_Transmit(Vision_Cmd_Record_Stop);
                        // Guidance_State = End;
                        // ==================阵营切换-蓝==============
                        // Dart_Trriger_Color_Set(Team_Blue);

                }break;
                case 3:
                {
                        // ==================电池掉电==============
                        Total_Power_Control(Power_OFF);
                }break;
                case 4:
                {
                        // ==================放歌==============
                        Buzzer_play_song(song_ni);
                }break;
                case 8:
                {
                        // Buzzer_message.cmd = Buzzer_CMD_PAUSE;
                        // Buzzer_stop();
                        // Buzzer_message.note.frequency = 0;
                        // Buzzer_message.note.period = 0;
                        // Buzzer_message.song.name = 0;
                        // Buzzer_message.song.note_count = 0;
                        // Buzzer_message.song.notes = 0;
                        // Buzzer_message.song.tempo = 0;
                }break;
        }
        Button_Data.Short_Press_Start_Time      = NONE;
        Button_Data.Continuous_FLag             = NONE;
        Button_Data.Press_Long_Cnt[NOW]         = NONE;
        Button_Data.Press_Long_Cnt[LAST]        = NONE;
        Button_Data.Press_Short_Cnt             = NONE;
        Button_Data.Short_Press_Temp            = NONE;
        Button_Data.Press_Type                  = TYPE_STOP;
        // HAL_GPIO_TogglePin( LED_PORT,LED_PIN );

}
void Long_Press_Process(void)
{
        switch (Button_Data.Press_Long_Cnt[NOW])
        {
                case 1:
                {
                        //==============查询镖头状态=================
                        Dart_Trigger_State_Check();
                        // Buzzer_play_song(song_ni);
                        // Buzzer_message.cmd = BUZZER_CMD_PLAY_SONG;
                }break;
                case 2:
                {
                        //==============设为红色=================
                        Dart_Trriger_Color_Set(Team_RED);
                }break;
                case 3:
                {
                        //==============设为蓝色=================
                        Dart_Trriger_Color_Set(Team_Blue);
                }break;
        }
        Button_Data.Continuous_FLag             = NONE;
        Button_Data.Press_Long_Cnt[NOW]         = NONE;
        Button_Data.Press_Long_Cnt[LAST]        = NONE;
        Button_Data.Press_Short_Cnt             = NONE;
        Button_Data.Press_Type                  = TYPE_STOP;
        Button_Data.Short_Press_Temp            = NONE;
        // HAL_GPIO_TogglePin( LED_PORT,LED_PIN );
}
void Event_Process(void)
{
        switch(Button_Data.Continuous_FLag)
        {
                case Event_PRESS:
                {
                        //通过按下的时间判断按下类型
                        if (Button_Data.Continuous_Time >=SHORT_Press_Time_Recognize&&Button_Data.Continuous_Time <=LONG_Press_Time_Recognize)
                        {
                                Button_Data.Press_Type = TYPE_SHORT;
                        }
                        else if (Button_Data.Continuous_Time >=LONG_Press_Time_Recognize&&Button_Data.Continuous_Time <LONG_Press_Cnt_Update)
                        {
                                Button_Data.Press_Type = TYPE_LONG;
                        }
                        else if (Button_Data.Continuous_Time >=LONG_Press_Cnt_Update)
                        {
                                Button_Data.Press_Long_Cnt[LAST] = Button_Data.Press_Long_Cnt[NOW];
                                Button_Data.Press_Long_Cnt[NOW]  = Button_Data.Continuous_Time/LONG_Press_Cnt_Update;
                                if (Button_Data.Press_Long_Cnt[LAST] != Button_Data.Press_Long_Cnt[NOW])
                                Button_Data.Remind_Flag = 1;
                                if (Button_Data.Remind_Flag == 1)
                                {
                                        Button_Data.Remind_Flag = 0;
                                        Press_Remind();
                                }
                                if (Button_Data.Press_Long_Cnt[NOW] >= MAX_Press_Long_Cnt)
                                {
                                        Button_Data.Press_Long_Cnt[NOW] = MAX_Press_Long_Cnt;
                                }
                        }
                }break;
                case Event_RELEASE:
                {
                        switch(Button_Data.Press_Type)
                        {
                                case TYPE_SHORT:
                                {
                                        if (Button_Data.Short_Press_Temp==0)
                                        {
                                                Button_Data.Short_Press_Temp = 1;
                                        }
                                        if (Button_Data.Short_Press_Temp==1)
                                        {
                                                Button_Data.Short_Press_Start_Time = Button_Data.Current_Time;
                                                Button_Data.Press_Short_Cnt++;
                                                Button_Data.Remind_Flag = 1;
                                                if (Button_Data.Remind_Flag == 1)
                                                {
                                                        Button_Data.Remind_Flag = 0;
                                                        Press_Remind();
                                                }
                                                if (Button_Data.Press_Short_Cnt >= MAX_Press_Short_Cnt)
                                                {
                                                        Button_Data.Press_Short_Cnt = MAX_Press_Short_Cnt;
                                                }
                                                Button_Data.Continuous_FLag = NONE;

                                        }
                                }break;
                                case TYPE_LONG:
                                {
                                        Long_Press_Process();
                                }break;
                                default:break;
                        }
                }break;
                default:break;

        }
        if (Button_Data.Current_Time - Button_Data.Short_Press_Start_Time>=MAX_SHORT_Press_Time_END&&Button_Data.Short_Press_Temp==1)
        {
                Short_Press_Process();
        }
}
void STOP_Process(void)
{
        Button_Data.Press_Type=NONE;
        HAL_TIM_Base_Stop_IT(&htim15);
        Button_Data.TIM_INIT_FLAG = 0;
}
void Debounce_and_Judge(void)
{
        //消抖,长时间维持某一个状态才进cnt++
        if (Button_Data.Debounce_Time <=Debounce_Limit)
        {
                if (Button_Data.Press_State[NOW]==Button_Data.Press_State[LAST])
                {
                        Button_Data.Debounce_Time++;
                }
                else if (Button_Data.Press_State[NOW]!=Button_Data.Press_State[LAST])
                {
                        Button_Data.Debounce_Time = 0;
                }
        }
        //消抖后，计次到某个程度，则更新稳定态为当前态
        if (Button_Data.Debounce_Time ==Debounce_Limit)
        {
                Button_Data.Debounce_Time = 0;
                Button_Data.Press_Stable_State[LAST] = Button_Data.Press_Stable_State[NOW];
                Button_Data.Press_Stable_State[NOW] = Button_Data.Press_State[NOW];
                //松手或按下检测，按下则进到短按或长按，松手进松手检测
                if (Button_Data.Press_Stable_State[NOW]==STATE_PRESS && Button_Data.Press_Stable_State[LAST]==STATE_RELEASE)
                {
                        Button_Data.Continuous_FLag = Event_PRESS;//按下
                        Button_Data.Start_Time = xTaskGetTickCount();
                        Button_Data.Current_Time = xTaskGetTickCount();
                }
                else if (Button_Data.Press_Stable_State[NOW]==STATE_RELEASE && Button_Data.Press_Stable_State[LAST]==STATE_PRESS)
                {
                        Button_Data.Continuous_FLag = Event_RELEASE;//松手
                }
        }
}
void Button_Detect(void)
{
        Button_Data.Current_Time = HAL_GetTick();
        Button_Data.Press_State[LAST] = Button_Data.Press_State[NOW];
        Button_Data.Press_State[NOW] = HAL_GPIO_ReadPin(Button_PORT, Button_PIN)+1;
        Debounce_and_Judge();
        Button_Data.Continuous_Time = Button_Data.Current_Time-Button_Data.Start_Time;
        Event_Process();
        if (Button_Data.Press_Type==TYPE_STOP)
        {
                STOP_Process();
        }
}

