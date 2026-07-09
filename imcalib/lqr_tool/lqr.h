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

/* 速度调度范围(m/s)，与 MATLAB V_schedule_ac 对齐；超出 clamp 到边界 */
#define DART_LQR_V_MIN       2.0f
#define DART_LQR_V_MAX       20.0f
#define DART_LQR_V_NOM       4.0f   /* 标称速度(初始 K_d / EKF 不可用时的回退) */

/*============================================================================
 *  LQR 状态/观测仓（变量归进结构体，遵循工程少散落变量风格）
 *============================================================================*/
typedef struct
{
    /* ---- 运行状态(每拍刷新，Vofa 可观测) ---- */
    float err_deg[3];                     /* 姿态误差(度，环绕后)，顺序[roll,pitch,yaw]，对应 x[0..2] 转 rad 前的值 */
    float x[DART_LQR_STATE_NUM];          /* 组好的状态向量[roll,pitch,yaw 误差(rad), p,q,r(rad/s)] */
    float u_rad[DART_LQR_SERVO_NUM];      /* LQR 原始解(rad，已限幅，MATLAB 舵号行序 delta1..4) */
    float u_servo_deg[DART_LQR_SERVO_NUM];/* 最终舵角(度，含 SIGN，按工程索引 UL/UR/DR/DL) */
    float axis_cmd_deg[3];                /* Vofa观测:从 u_rad 反解的等效三轴指令(度,顺序[PITCH,ROLL,YAW])，
                                           * 按理想 X 翼 G 模式投影回来的等效量，类比 PID 的 output_gyro_Euler。*/

    /* ---- 速度调度 K_d(V)(50Hz 中断更新，Vofa 可观测) ---- */
    float K_d[DART_LQR_SERVO_NUM][DART_LQR_STATE_NUM]; /* 当前拍 K 矩阵(由 LQR_Gain_Update50Hz 写入) */
    float V_lqr;                          /* 当前拍用于算 K 的速度(m/s，clamp 后) */

    /* ---- 上车前需逐轴台架验证的符号/环绕(指南 §9) ---- */
    float   axis_sign[3];                 /* ★整轴极性[roll,pitch,yaw]，±1，默认+1。某轴打反/发散(如 roll 一直旋)
                                           * 就把对应项翻 −1：在调试器 Watch 在线改、立即生效，不用重跑 MATLAB。
                                           * 它同时翻该轴的角度项+角速度项(=等效翻 MATLAB G 那一行)，是修符号的主旋钮。*/
    float   gyro_sign[3];                 /* 仅角速度(阻尼)符号校正[roll,pitch,yaw]，默认+1。角度方向已对、只是
                                           * 该轴抖/震(阻尼变负)时翻它；整轴反向请用 axis_sign。*/
    uint8_t roll_wrap;                    /* 1=roll 误差按 ±180° 环绕(默认 0 与 PID 对齐；yaw 恒环绕) */

    /* ---- 基于 roll 的机体系补偿(横滚反旋，与 PID 的 Roll_Derotate_PitchYaw 同源) ---- */
    uint8_t roll_comp;                    /* ★1=按当前横滚 Δ 把世界系 pitch/yaw 误差反旋到机体系再进 K；0=直通(旧行为)。
                                           * 角度误差取自世界系欧拉、而 K/B 是机体舵效，横滚后需反旋才对得上舵面。
                                           * 默认 0；台架按指南验 SIGN(共用 ROLL_WORLD_COMP_SIGN)后 Watch 切 1。详见 Euler_LQR_Cale。*/
    float   roll_comp_delta;              /* Vofa/Watch:本拍反旋用的横滚量 Δ=current_roll−Stable_roll(度)；roll_comp=0 时不刷新 */
} LQR_t;

/*============================================================================
 *  全局
 *============================================================================*/
extern LQR_t lqr_ctrl;                                       /* 控制器实例 */
extern float dart_lqr_K[DART_LQR_SERVO_NUM][DART_LQR_STATE_NUM]; /* 旧单点 K 矩阵(对照存根，见 lqr.c) */
extern float dart_delta_max_rad;                            /* 舵偏限幅(rad)，默认 ±60° */
extern uint8_t lqr_pitch_terminal_only;                    /* 1=只在制导段(Terminal)+检测到目标时控 pitch(补偿)；0=全程控。见 lqr.c */
extern uint8_t lqr_use_scheduled_K;                        /* 1=速度调度 K_d(V)(默认)；0=退回静态 dart_lqr_K */

/*============================================================================
 *  接口
 *============================================================================*/

/** @brief 初始化(清零状态、默认符号、初始化 MATLAB Coder 运行时)。TotalInitTask 调一次。 */
void LQR_Init(void);

/**
 * @brief 50Hz 中断调用：从 EKF 速度算 K_d(V)，写入 lqr_ctrl.K_d/V_lqr
 * @note  TIM7 回调(CallBack_Task.c)里调。lqr_use_scheduled_K=0 时直接返回。
 */
void LQR_Gain_Update50Hz(void);

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
