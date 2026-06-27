//
// Created by ynz on 2026/1/10.
//

#ifndef PNG_TASK_H
#define PNG_TASK_H

#define SENSOR_FOV 70
#define SENSOR_TOTAL_PIXEL_WIDTH 320
#define SAMPLE_RATE 20
#define N_rate 5.0f
#define MAX_PNG_OUT 45
#define Vc_min 1.0
#define K_Dyn 20.0

/* === 速度比例导引超前(PNG_Apply_Lead)调参 === */
/* Mode0(Vc缩放,稳健):超前角 corr_deg = PNG_K_VC·Vc·λ̇(deg/s)。设计点:Vc≈15m/s 时 PNG_K_VC·15≈0.05
 * =现固定增益 PN_LEAD_K → 标称速度下幅度不变、随接近速度 Vc 线性自动缩放。台架按跟手/抖动微调。*/
#define PNG_K_VC           0.01f
#define PNG_VC_MIN         0.50f    /* Vc 下限钳位(m/s):防 Vc≈0 时 PN 消失 */
#define PNG_VC_MAX         8.0f   /* Vc 上限钳位(m/s):防异常大 Vc 放大超前 */
#define PNG_LEAD_LIMIT_DEG 8.0f    /* 单轴 PN 超前角限幅(deg):丢帧/速度异常时防爆冲 */

#include <stdint.h>
#include "CallBack_Task.h"
#include "IMU.h"
#include "surface_control_task.h"

typedef struct
{
	float FOV;
	float los_GYRO[2][3];
	float los_ANGLE[2][3];
	float los_vector[3];
	float V_c;
	float N_R;
	/* === PNG_Apply_Lead 的 Vofa 观测(索引[PITCH,YAW]) === */
	float vc_used;          /* 实际用于 PN 的接近速度(钳位后,m/s) */
	float los_rate_used[3]; /* 实际用的视线率(deg/s) */
	float lead_corr[3];     /* PN 超前角输出(deg,限幅后) */
}PNG_Data_t;


extern PNG_Data_t PNG_Data;
/* 速度PN开关(Vofa/调试器在线改):轴使能 + 接入模式 */
extern uint8_t PNG_Yaw_Flag;    /* 1=YAW 用速度PN超前;0=不加(回到原行为) */
extern uint8_t PNG_Pitch_Flag;  /* 1=PITCH 用速度PN超前;0=用原固定增益超前(PN_LEAD_K) */
extern uint8_t PNG_Mode;        /* 0=Vc缩放(在 vision_los_rate 上,稳健);1=EKF全量p×v真PN(验证后切) */

void PNG_Init(PNG_Data_t* PNG_Data);
/* 末制导:在视觉斜坡跟踪目标角之上,叠加按接近速度 Vc 缩放的 PN 超前(仅识别成功段调用,见 surface_control_task.c) */
void PNG_Apply_Lead(Surface_t* Surface , IMU_DATA_t* IMU_Data);
/* 拆分接口:单独控制 yaw/pitch 的 PN 超前(供 Guidance_Terminal 分别门控) */
void PNG_Apply_Lead_Yaw(Surface_t* Surface , IMU_DATA_t* IMU_Data);
void PNG_Apply_Lead_Pitch(Surface_t* Surface , IMU_DATA_t* IMU_Data);


#endif //PNG_TASK_H
