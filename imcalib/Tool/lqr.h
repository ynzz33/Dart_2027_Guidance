/**
 * @file    lqr.h
 * @brief   飞镖 X 翼姿态 LQR 控制器接口（u = -K_d·x，6态→4舵一步解算）
 * @details 详见 lqr.c 头注与 lqr_czn/MCU_LQR_PORTING_GUIDE.md。
 *          ★ 一步替代 PID + 混控器：K_d 已含 X 翼混控几何，直接写 output_angle_Servo。
 *          ★ K 矩阵 dart_lqr_K[4][6] 与 MATLAB 同名同形，重调后整块粘贴覆盖即可(见 lqr.c 粘贴区)。
 *          ★ 未编译：新增 .c 需手动加入 Keil/eIDE 工程。待台架。
 *
 * @author  ynz (AI 移植)
 * @date     2026/06/27
 */

#ifndef __LQR_H
#define __LQR_H

#include "stm32g4xx_hal.h"

/*============================================================================
 *  维度
 *    状态 x = [roll_err, pitch_err, yaw_err, p, q, r]   (rad, rad/s)
 *    控制 u = [delta1, delta2, delta3, delta4]           (rad，MATLAB 舵号)
 *============================================================================*/
#define DART_LQR_STATE_NUM   6
#define DART_LQR_SERVO_NUM   4

/*============================================================================
 *  LQR 状态/观测仓（变量归进结构体，遵循工程少散落变量风格）
 *============================================================================*/
typedef struct
{
    /* ---- 运行状态(每拍刷新，Vofa 可观测) ---- */
    float err_deg[3];                     /* 姿态误差(度，环绕后)，顺序[roll,pitch,yaw]，对应 x[0..2] 转 rad 前的值 */
    float x[DART_LQR_STATE_NUM];          /* 组好的状态向量[roll,pitch,yaw 误差(rad), p,q,r(rad/s)] */
    float u_rad[DART_LQR_SERVO_NUM];      /* LQR 原始解(rad，已限幅，MATLAB 舵号行序 delta1..4)；仅观测/投影用 */
    float axis_cmd_deg[3];                /* 输出:从 u_rad 反投影的三轴力矩需求(度,顺序[PITCH,ROLL,YAW])，
                                           * 写入 output_gyro_Euler，复用 PID 同一条 Roll反旋+Servo_Mix_* 混控链。*/

    /* ---- 上车前需逐轴台架验证的符号/环绕(指南 §9) ---- */
    float   gyro_sign[3];                 /* 角速度符号校正[roll,pitch,yaw]，默认+1；反则 −1 */
    uint8_t roll_wrap;                    /* 1=roll 误差按 ±180° 环绕(默认 0 与 PID 对齐；yaw 恒环绕) */
} LQR_t;

/*============================================================================
 *  全局
 *============================================================================*/
extern LQR_t lqr_ctrl;                                       /* 控制器实例 */
extern float dart_lqr_K[DART_LQR_SERVO_NUM][DART_LQR_STATE_NUM]; /* K 矩阵(MATLAB 粘贴区，见 lqr.c) */
extern float dart_delta_max_rad;                            /* 舵偏限幅(rad)，默认 ±60° */

/*============================================================================
 *  接口
 *============================================================================*/

/** @brief 初始化(清零状态、默认符号)。TotalInitTask 调一次。 */
void LQR_Init(void);

/**
 * @brief 纯 LQR 解算 u = -K_d·x（与 MATLAB 对拍用，不依赖工程全局）
 * @param x  状态[6]，rad / rad·s⁻¹
 * @param u  输出[4]，rad，已按 ±dart_delta_max_rad 限幅
 */
void LQR_Update(const float x[DART_LQR_STATE_NUM], float u[DART_LQR_SERVO_NUM]);

/**
 * @brief 桥接：从 Surface 取姿态/角速度→组 x→LQR→直接写 4 舵机角(度)
 * @param dt 采样周期(s)，当前未用(静态增益)，保留以便扩展
 * @note  直接覆盖 Surface.output_angle_Servo[NOW][...]，绕过 output_gyro_Euler 与 Servo_Mix_*。
 *        在 surface_control_task.c 中 lqr_mode==1 时调用，并跳过混控分派。
 */
void Euler_LQR_Cale(float dt);

#endif /* __LQR_H */
