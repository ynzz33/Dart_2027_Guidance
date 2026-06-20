/**
 * @file    adrc.h
 * @brief   LADRC 线性自抗扰控制器 - 单环二阶实现（替代原非线性 ADRC）
 * @details 针对制导飞镖 X 翼构型。从原"非线性 ADRC(TD/ESO/NLSEF + fal/fst)"改为
 *          高志强(Gao)带宽法 LADRC：线性扩张状态观测器(LESO) + 线性状态误差反馈(LSEF)。
 *
 *          ★ 文件名保持 adrc.c/.h 不变(避免改动 eide/MDK 构建工程)，内部已全部换成 LADRC。
 *
 * @author  ynz
 * @date    2026/06/21
 *
 * ============================================================================
 *  为什么从 ADRC 换 LADRC？
 *    原 ADRC 用了一堆非线性函数(fal/fst)和 α/δ 参数，每个轴 2 环 ×(TD r + ESO ω₀ +
 *    NLSEF ωc + b0 + α1/α2/δ ...)十几个旋钮，根本没法系统地调。
 *
 *    LADRC 把它全线性化，整轴只剩 3 个旋钮：
 *      • wc  控制带宽：决定"跟得多快"。越大越快越冲。
 *      • wo  观测带宽：决定"看得多准多快"，一般取 wc 的 3~5 倍。
 *      • b0  控制增益估计：u=(u0−z3)/b0。最关键，决定整体"力度"。
 *    其余 kp/kd/β1/β2/β3 全部由 wc/wo 自动算出，不用手碰。
 *
 *  单环二阶 LADRC 原理（被控对象当作二阶：θ̈ = f + b0·u，f=总扰动）：
 *    1. LESO：只用"测得的角度 y"和"上拍输出 u"，就估出
 *         z1≈角度、z2≈角速度(天生干净，不靠微分)、z3≈总扰动(气动+耦合+模型误差+一切)
 *    2. LSEF：u0 = kp·(目标−z1) − kd·z2           (比例 + 纯阻尼)
 *    3. 扰动补偿：u = (u0 − z3)/b0                  (把估出来的扰动直接减掉)
 *    结果：不要精确模型，自动补偿一切扰动；只调 3 个带宽就行。
 * ============================================================================
 */

#ifndef __LADRC_H
#define __LADRC_H

#include "stm32g4xx_hal.h"

/*============================================================================
 *  通道索引（必须与 Surface 的 [PITCH=0, ROLL=1, YAW=2] 完全一致）
 *============================================================================*/
enum {
    LADRC_PITCH = 0,
    LADRC_ROLL  = 1,
    LADRC_YAW   = 2,
    LADRC_CH_COUNT = 3
};

/*============================================================================
 *  单环二阶 LADRC 控制器
 *  一个轴 = 一个三阶 LESO + 一个 LSEF。整轴只暴露 wc/wo/b0 三个旋钮。
 *============================================================================*/
typedef struct {
    /* ---- 3 个旋钮（调试器 Watch 在线改，改完下一拍即生效） ---- */
    float wc;          /* 控制器带宽 rad/s：越大跟踪越快、越激进 */
    float wo;          /* 观测器带宽 rad/s：一般取 3~5×wc；越大估计越快但越吃噪声 */
    float b0;          /* 控制增益估计：u=(u0−z3)/b0。太小→输出过猛甚至发散；太大→绵软无力。最关键 */

    /* ---- 阻尼源选择（本次改动：阻尼项 −kd·ω 的角速度 ω 用实测陀螺，而非 LESO 估的 z2） ----
     * 纯单环 LADRC 拿 LESO 的 z2 当角速度，z2 准不准全押在 wo/b0 上；本板有现成高质量陀螺，
     * 故默认改用实测角速度做阻尼：相位准、不依赖 b0，roll 不易高频抖。需在 LADRC_Calc 喂入实测值。*/
    uint8_t use_gyro_damp; /* 1=阻尼用实测陀螺(默认)；0=回到纯单环用 z2(对比/兜底) */
    float   gyro_sign;     /* 实测陀螺符号校正：须与"角度增大方向"一致。台架若一动就反向猛打/发散→翻成 −1 */

    /* ---- 由 wc/wo 自动算出的线性增益（每拍按当前 wc/wo 重算，故直接改 wc/wo 即可） ---- */
    float kp;          /* = wc²    (LSEF 比例) */
    float kd;          /* = 2·wc   (LSEF 阻尼) */
    float beta1;       /* = 3·wo   (LESO 对 z1 校正) */
    float beta2;       /* = 3·wo²  (LESO 对 z2 校正) */
    float beta3;       /* = wo³    (LESO 对 z3 校正) */

    /* ---- LESO 状态估计（也是最有用的 Vofa 观测量） ---- */
    float z1;          /* 角度估计 */
    float z2;          /* 角速度估计（不靠微分，天生干净，可代替陀螺反馈） */
    float z3;          /* 总扰动估计（气动力矩+舵面耦合+重心偏移+模型误差+外扰，全在这一项里） */

    /* ---- 限幅 / 死区 / 周期角 ---- */
    float   max_output;/* 输出限幅(±)，与原 PID 内环 MaxOut/AXIS_LIMIT 对齐 */
    float   deadband;  /* 角度误差死区半宽(度)，0=不启用；只软化比例项，扰动补偿 z3 仍守稳态不下垂 */
    uint8_t angle_wrap;/* 1=误差/新息按 ±180° 环绕(roll/yaw 是 atan2 周期角)；roll 默认 0 与原 PID 对齐 */

    /* ---- 运行状态 ---- */
    float   u;         /* 本拍输出(=送混控的力矩需求)，下一拍喂回 LESO（用实际限幅后的值，自带抗饱和） */
    uint8_t inited;    /* 0=未初始化(首拍用当前反馈对齐 z1，避免初始冲击) */

    /* ---- 仅供 Vofa 观测 ---- */
    float target;      /* 当前目标 */
    float feedback;    /* 当前反馈 */
    float error;       /* 当前误差(目标−反馈，已按需环绕) */
    float u0;          /* 补偿前(LSEF)输出 */
    float gyro;        /* 本拍实际用于阻尼的实测角速度°/s(=gyro_sign×传入测量；use_gyro_damp=1 时生效) */
} LADRC_t;

/*============================================================================
 *  全局实例：3 通道，每轴一个单环 LADRC
 *============================================================================*/
extern LADRC_t ladrc_ctrl[LADRC_CH_COUNT];

/*============================================================================
 *  接口
 *============================================================================*/

/**
 * @brief 初始化单轴 LADRC（按通道给默认 wc/wo/b0/限幅/死区）
 * @param c       LADRC 实例
 * @param channel LADRC_PITCH / LADRC_ROLL / LADRC_YAW
 */
void LADRC_Init(LADRC_t *c, uint8_t channel);

/**
 * @brief 初始化全部 3 轴 LADRC（在 TotalInitTask 中调用一次）
 */
void LADRC_Init_All(void);

/**
 * @brief 复位单轴内部状态（切模式 / 重新进入控制时调用）
 */
void LADRC_Reset(LADRC_t *c);

/**
 * @brief 单拍 LADRC 计算
 * @param c        LADRC 实例
 * @param target   目标角度(度)
 * @param feedback 当前角度(度)
 * @param gyro_meas 实测角速度(°/s，同轴陀螺；符号须与角度增大方向一致，use_gyro_damp=1 时用它做阻尼)
 * @param dt       采样周期(s)
 * @return 控制输出(=送混控的力矩需求)，已限幅
 */
float LADRC_Calc(LADRC_t *c, float target, float feedback, float gyro_meas, float dt);

/**
 * @brief 三轴 LADRC 计算（角度→力矩），输出写 Surface.output_gyro_Euler[NOW][i]
 * @note  替代原 Euler_pid_Cale；当 ladrc_mode==1(三轴全 LADRC) 时调用
 */
void Euler_LADRC_Cale(float delta_time);

/**
 * @brief 在线设置带宽（也可直接改 c->wc / c->wo，LADRC_Calc 内每拍会重算增益）
 */
static inline void LADRC_SetBandwidth(LADRC_t *c, float wc, float wo)
{
    c->wc = wc;
    c->wo = wo;
}

#endif /* __LADRC_H */
