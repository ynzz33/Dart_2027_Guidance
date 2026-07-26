/**
 * @file    lqi_torque.h
 * @brief   飞镖 LQI 力矩控制器接口（9态→3轴力矩，N·m）
 * @details 与旧 LQR 的本质区别：
 *          - LQI 输出三轴物理力矩 [Mx, My, Mz] (N·m)，而非四舵角
 *          - K_lqi[3][9] 通过 dlqr 计算，不含舵面混控
 *          - 力矩→舵面的转换由 torque_allocator 独立完成
 *          - 舵面顺序全线统一为 [UL, UR, DR, DL]
 *
 *          状态: xa = [e_roll, e_pitch, e_yaw, p, q, r, I_roll, I_pitch, I_yaw]
 *                单位: rad, rad, rad, rad/s, rad/s, rad/s, rad·s, rad·s, rad·s
 *          控制: tau = [Mx, My, Mz] (N·m)
 *
 *          ⚠ 未编译：新增 .c/.h 需手动加入 Keil/eIDE 工程编译列表。待台架。
 *
 * @author  ynz (AI 移植)
 * @date    2026/07/23
 */

#ifndef __LQI_TORQUE_H
#define __LQI_TORQUE_H

#include "stm32g4xx_hal.h"

/*============================================================================
 *  维度
 *============================================================================*/
#define LQI_STATE_DIM       9    /* 增广状态: 6 原始 + 3 积分 */
#define LQI_TORQUE_DIM      3    /* 三轴力矩: [Mx, My, Mz] */
#define LQI_SERVO_COUNT     4    /* 四片舵面: [UL, UR, DR, DL] */

/*============================================================================
 *  积分分离阈值
 *  姿态误差 ≥ 此值 → 清零并冻结积分（P/D 主导拉回）
 *  姿态误差 <  此值 → 开启积分（消静差）
 *============================================================================*/
#define LQI_INTEG_THRESHOLD_RAD  0.008726646f*3.0f   /* 0.5° → rad */

/*============================================================================
 *  积分限幅（防 deep windup）
 *  积分超过此值 → clamp，防止目标丢失/舵效过低时积分发散
 *  Pitch 上限极小（不追踪角度，仅阻尼），Roll/Yaw 使用有限上限
 *============================================================================*/
#define LQI_INTEG_LIMIT_ROLL   0.174532925f   /* 10°·s */
#define LQI_INTEG_LIMIT_PITCH  0.0f           /* 0：Pitch 不做积分追踪 */
#define LQI_INTEG_LIMIT_YAW    0.0174532925f*2.0f   /* 10°·s */

/* 积分力矩限幅：积分贡献的力矩上限 N·m（防止积分过强导致舵面饱和/振荡） */
#define LQI_INTEG_TORQUE_LIMIT_ROLL   0.0f
#define LQI_INTEG_TORQUE_LIMIT_PITCH  0.0f    /* Pitch 积分关闭 */
#define LQI_INTEG_TORQUE_LIMIT_YAW    0.3f

/*============================================================================
 *  NaN/Inf 安全兜底
 *============================================================================*/
#define LQI_IS_FINITE(x)  (((x) * 0.0f) == 0.0f)  /* IEEE754: NaN*0=NaN, Inf*0=NaN */

/*============================================================================
 *  LQI 控制状态（变量归进结构体，遵循工程少散落变量风格）
 *============================================================================*/
typedef struct
{
    /* ---- 输入（每拍由桥接函数填入） ---- */
    float attitude_error_rad[3];      /* 姿态角误差(测量−目标), rad, 顺序[roll,pitch,yaw] */
    float body_rate_rad_s[3];         /* 机体角速度, rad/s, 顺序[p,q,r] */
    float integral_error[3];          /* 积分误差累加, rad·s, 顺序[roll,pitch,yaw] */

    /* ---- LQI 状态向量 ---- */
    float xa[LQI_STATE_DIM];          /* 增广状态 [e_roll,e_pitch,e_yaw,p,q,r,I_roll,I_pitch,I_yaw] */

    /* ---- 输出 ---- */
    /* LQI 三类状态反馈对三轴力矩的实际贡献(N·m)，已包含 LQI_GAIN_SCALAR */
    float torque_angle_Nm[LQI_TORQUE_DIM];         /* 角度误差项: xa[0..2] */
    float torque_rate_Nm[LQI_TORQUE_DIM];          /* 角速度项:   xa[3..5] */
    float torque_integral_raw_Nm[LQI_TORQUE_DIM];  /* 积分项，限幅前 */
    float torque_integral_Nm[LQI_TORQUE_DIM];      /* 积分项，限幅后/实际参与合成 */
    float torque_cmd_Nm[LQI_TORQUE_DIM];      /* 指令力矩 [Mx,My,Mz], N·m */
    float torque_achieved_Nm[LQI_TORQUE_DIM]; /* 实际力矩(限幅/不可达后), N·m */
    float torque_error_Nm[LQI_TORQUE_DIM];    /* 力矩误差, N·m */
    float servo_cmd_deg[LQI_SERVO_COUNT];     /* 最终舵角, °, [UL,UR,DR,DL] */
    float pitch_moment_Nm;                    /* 当前拍 Pitch 实际力矩, N·m */

    /* ---- K_lqi[3][9]（LQI_Init 加载单矩阵，与速度无关） ---- */
    float K_lqi[LQI_TORQUE_DIM][LQI_STATE_DIM];

    /* ---- 运行时 ---- */
    float cached_V;                   /* 缓存速度标量 m/s（50Hz 更新），用于 H_tau(V) 计算 */

    /* ---- 标志 ---- */
    uint8_t servo_sat_mask;           /* bit i = 第 i 片舵面饱和 */
    uint8_t allocator_infeasible;     /* 分配器不可达（力矩需求超过舵效） */
    uint8_t state_valid;              /* 状态有效标志 */

    /* ---- 积分抗饱和 ---- */
    uint8_t freeze_integrator[3];     /* 冻结积分标志 [roll,pitch,yaw] */

} LQI_Control_t;

/*============================================================================
 *  全局
 *============================================================================*/
extern LQI_Control_t lqi_ctrl;
extern uint8_t lqi_mode;             /* 0=关(LQR) 1=LQI力矩+零空间分配；优先级高于 lqr_mode */
extern float V_DART_Lqi;

/*============================================================================
 *  接口
 *============================================================================*/

/**
 * @brief 初始化（清零状态、加载 K_lqi）。TotalInitTask 调一次。
 */
void LQI_Init(void);

/**
 * @brief 50Hz 中断调用：读取 EKF 速度，缓存供 1kHz 计算 H_tau(V) 用。
 * @note  TIM7 回调(CallBack_Task.c)里调。替代原 K 表插值逻辑。
 */
void LQI_Velocity_Update50Hz(void);

/**
 * @brief LQI 控制解算：tau = -K_lqi * xa
 * @param dt  采样周期(s)
 * @note  调用前需先填入 attitude_error_rad / body_rate_rad_s
 *        调用后 torque_cmd_Nm 更新
 */
void LQI_Update(float dt);

/**
 * @brief 桥接：从 Surface 取姿态/角速度→组状态→LQI→零空间分配→写 4 舵机角(度)
 * @param dt  采样周期(s)
 * @note  直接写 Surface.output_angle_Servo[NOW][...]
 *        在 surface_control_task.c 中 lqi_mode==1 时调用
 */
void Euler_LQI_Cale(float dt);

extern LQI_Control_t lqi_ctrl;

#endif /* __LQI_TORQUE_H */
