/*******************************************************************************
*** ------------------ ██╗███╗   ███╗ ██████╗ █████╗ --------------------------/
*** ------------------ ██║████╗ ████║██╔════╝██╔══██╗--------------------------/
*** ------------------ ██║██╔████╔██║██║     ███████║--------------------------/
*** ------------------ ██║██║╚██╔╝██║██║     ██╔══██║--------------------------/
*** ------------------ ██║██║ ╚═╝ ██║╚██████╗██║  ██║--------------------------/
*** ------------------ ╚═╝╚═╝     ╚═╝ ╚═════╝╚═╝  ╚═╝--------------------------/
*** @File           : buzzer.h
*** @Description    : 蜂鸣器驱动 - 完整的音乐播放系统
*** @Attention      : 使用TIM2_CH2 PWM驱动无源蜂鸣器，基于FreeRTOS
*** @Author         : Lmy
*** @Date           : 2025/10/29
*** @Copyright      : Copyright (c) 2025 IMCA. ALL right reserved
*******************************************************************************/
#ifndef DART_G431_DEMO_BUZZER_H
#define DART_G431_DEMO_BUZZER_H
/*----------------------------------------------------------------------------*/
/* 头文件包含 BEGIN */
#include "stm32g4xx.h"
/* 头文件包含 END */
/*----------------------------------------------------------------------------*/
/* 音符频率定义 BEGIN */

#define P0     0    // 休止符频率

#define LL1    131  // 低低音频率
#define LL2b   138
#define LL2    147
#define LL3b   156
#define LL3    165
#define LL4    175
#define LL5b   185
#define LL5    196
#define LL6b   208
#define LL6    220
#define LL7b   233
#define LL7    247

#define L1     262  // 低音频率
#define L2b    277
#define L2     294
#define L3b    311
#define L3     330
#define L4     349
#define L5b    370
#define L5     392
#define L6b    415
#define L6     440
#define L7b    466
#define L7     494

#define M1     523  // 中音频率
#define M2b    544
#define M2     587
#define M3b    622
#define M3     659
#define M4     698
#define M5b    740
#define M5     784
#define M6b    831
#define M6     880
#define M7b    932
#define M7     988

#define H1     1046 // 高音频率
#define H2b    1109
#define H2     1175
#define H3b    1245
#define H3     1318
#define H4     1397
#define H5b    1480
#define H5     1568
#define H6b    1661
#define H6     1760
#define H7b    1865
#define H7     1976

#define HH1    2092 // 高高音频率
#define HH2b   2218
#define HH2    2350
#define HH3b   2490
#define HH3    2636
#define HH4    2794
#define HH5b   2960
#define HH5    3136
#define HH6b   3322
#define HH6    3520
#define HH7b   3730
#define HH7    3952

/* 音符频率定义 END */
// 系统时钟配置
#define BUZZER_SYS_CLOCK         170000000  // HCLK = 170MHz
#define BUZZER_TIM_PRESCALER     170         // 预分频值

// 频率范围
#define BUZZER_MIN_FREQ         0
#define BUZZER_MAX_FREQ         5000

enum
{
  BUZZER_CMD_PLAY_NOTE,
  BUZZER_CMD_PLAY_SONG,
  Buzzer_CMD_PAUSE,
  BUZZER_CMD_STOP
};

typedef struct {
  uint16_t frequency;         // 音符频率
  float period;               // 音符持续时间，单位为拍
} Buzzer_note_t;

typedef struct {
  Buzzer_note_t* notes;      // 音符数组指针
  uint32_t note_count;      // 音符数量
  uint16_t tempo;           // 节拍速度(BPM)
  char name;                // 歌曲名称
} Buzzer_song_t;

typedef struct {
  uint8_t cmd;
  Buzzer_note_t note;
  Buzzer_song_t song;
} Buzzer_message_t;


/*----------------------------------------------------------------------------*/
/* 函数原型声明 BEGIN */
void Buzzer_play_note(uint16_t freq, uint16_t duration_ms);
void Buzzer_stop(void);
void Buzzer_play_song(Buzzer_song_t song);
uint8_t Buzzer_is_playing(void);
void Buzzer_play_task(Buzzer_message_t *Buzzer_message);

// 内置歌曲声明
extern const Buzzer_song_t song_ni;
extern Buzzer_message_t Buzzer_message;
extern uint32_t current_count ;
/* 函数原型声明 END */
/*----------------------------------------------------------------------------*/
#endif // DART_G431_DEMO_BUZZER_H

