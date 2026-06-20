/**
 * @file    adrc.c
 * @brief   LADRC 线性自抗扰控制器实现（替代原非线性 ADRC）
 * @details 单环二阶 LADRC = 三阶线性扩张状态观测器(LESO) + 线性状态误差反馈(LSEF) + 扰动补偿。
 *
 *          ★ 文件名保持 adrc.c/.h 不变(避免改动 eide/MDK 构建工程)，内部已全部换成 LADRC。
 *
 * @author  ynz
 * @date    2026/06/21
 *
 * ============================================================================
 *  单环二阶 LADRC 一拍流程（被控对象当作二阶：θ̈ = f + b0·u）：
 *
 *   1. LESO（线性扩张状态观测器，只吃"角度测量 y"和"上拍输出 u"）：
 *        e  = z1 − y                              (观测新息)
 *        z1 += dt·(z2 − β1·e)                      β1 = 3·wo
 *        z2 += dt·(z3 − β2·e + b0·u)               β2 = 3·wo²
 *        z3 += dt·(   − β3·e)                       β3 = wo³
 *      → z1≈角度, z2≈角速度, z3≈总扰动(一切未建模量)
 *
 *   2. LSEF（线性状态误差反馈）：
 *        u0 = kp·(目标 − z1) − kd·z2               kp = wc², kd = 2·wc
 *      （角速度参考=0 → −kd·z2 是纯阻尼；自稳/定点正合适）
 *
 *   3. 扰动补偿 + 限幅：
 *        u = (u0 − z3) / b0,  再 |u| ≤ max_output
 *
 *  调参就 3 个旋钮：wc(跟多快) / wo(看多快,≈3~5×wc) / b0(整体力度)。详见 LADRC_Init。
 * ============================================================================
 */

#include "adrc.h"
#include <math.h>
#include "surface_control_task.h"
#include "IMU.h"
#include "common_defs.h"

/*============================================================================
 *  全局变量
 *============================================================================*/

/* LADRC 控制器实例：3 通道，每轴一个单环二阶 LADRC */
LADRC_t ladrc_ctrl[LADRC_CH_COUNT];

/*============================================================================
 *  辅助函数
 *============================================================================*/

/* 角度环绕到 (−180,180]：周期角(roll/yaw 由 atan2 得出)跨 ±180° 会出现 ~360° 假跳变，
 * 必须先归一，否则 LESO 新息/LSEF 误差被假跳激励 → 反向猛打致自旋。 */
static float ladrc_wrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* 对称限幅 */
static float ladrc_clamp(float v, float lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* 由 wc/wo 计算全部线性增益（带宽法）：
 *   LSEF 期望闭环特征多项式 (s+wc)²       → kp = wc², kd = 2·wc
 *   LESO 期望观测器特征多项式 (s+wo)³      → β1 = 3·wo, β2 = 3·wo², β3 = wo³
 * 每拍都重算，所以在线改 wc/wo 立即生效。 */
static void ladrc_update_gains(LADRC_t *c)
{
    c->kp    = c->wc * c->wc;
    c->kd    = 2.0f  * c->wc;
    c->beta1 = 3.0f  * c->wo;
    c->beta2 = 3.0f  * c->wo * c->wo;
    c->beta3 = c->wo * c->wo * c->wo;
}

/*============================================================================
 *  初始化
 *============================================================================*/

/**
 * @brief 初始化单轴 LADRC
 * @note  默认参数说明（roll 已按"从温和起步、与原 PID 同力度"标定，可直接上台架试）：
 *
 *   b0 的来历(roll)：原 PID 在 10° 误差时大约输出 12(角度环 Kp=3→30°/s，
 *   内环 Kp=0.4→0.4×30≈12)；LADRC 在 wc=5 时 u0=kp·10=25×10=250，
 *   令 u=250/b0≈12 → b0≈20。故 roll 取 b0=20 起步，力度与原 PID 相当。
 *
 *   调参顺序见文末 LADRC 调参速查：先定 wc，再令 wo=4×wc，最后调 b0。
 */
void LADRC_Init(LADRC_t *c, uint8_t channel)
{
    /* 公共状态清零 */
    c->z1 = c->z2 = c->z3 = 0.0f;
    c->u  = c->u0 = 0.0f;
    c->target = c->feedback = c->error = c->gyro = 0.0f;
    c->inited = 0;
    c->angle_wrap = 0;

    /* 阻尼源默认：用实测陀螺(本次改动)；符号默认 +1(与原串级内环 current_gyro_Euler 同源同向，  
     * 台架万一反了在 Watch 里翻 −1) */
    c->use_gyro_damp = 0;
    c->gyro_sign     = 1.0f;

    switch (channel)
    {
        case LADRC_ROLL:
            /* ★ 本次先试的轴：roll 自稳到 Stable 角，最简单最安全 */
            c->wc = 3.00f;                    /* 控制带宽：温和起步(原 PID 角度环 Kp=3，等效带宽≈3) */
            c->wo = 5*c->wc;                   /* 观测带宽 = 4×wc */
            c->b0 = 60.0f;                   /* 控制增益估计：与原 PID 同力度(见上注释)，最关键的旋钮 */
            c->max_output = AXIS_LIMIT_ROLL; /* ±15，与原 PID 内环/混控限幅一致 */
            c->deadband   = 1.0f;            /* 与原 PID roll 一致，防舵机静态抖 */
            c->angle_wrap = 0;               /* 与原 PID roll 对齐(原 surface_control_pid[Angle][ROLL].angle_wrap=0) */
            break;

        case LADRC_PITCH:
            /* 占位默认(本次未启用，仅初始化好备用) */
            c->wc = 6.0f;
            c->wo = 24.0f;
            c->b0 = 20.0f;
            c->max_output = AXIS_LIMIT_PITCH;/* ±40 */
            c->deadband   = 1.0f;
            c->angle_wrap = 0;               /* pitch 是 asin∈[−90,90]，不需环绕 */
            break;

        case LADRC_YAW:
            /* 占位默认(本次未启用，仅初始化好备用) */
            c->wc = 6.0f;
            c->wo = 24.0f;
            c->b0 = 20.0f;
            c->max_output = AXIS_LIMIT_YAW;  /* ±40 */
            c->deadband   = 0.2f;
            c->angle_wrap = 0;               /* yaw 是周期角，必要时置 1；先与原 PID 对齐置 0 */
            break;

        default:
            c->wc = 5.0f; c->wo = 20.0f; c->b0 = 20.0f;
            c->max_output = 15.0f; c->deadband = 0.0f;
            break;
    }
    ladrc_update_gains(c);
}

/**
 * @brief 初始化全部 3 轴 LADRC
 */
void LADRC_Init_All(void)
{
    LADRC_Init(&ladrc_ctrl[LADRC_PITCH], LADRC_PITCH);
    LADRC_Init(&ladrc_ctrl[LADRC_ROLL],  LADRC_ROLL);
    LADRC_Init(&ladrc_ctrl[LADRC_YAW],   LADRC_YAW);
}

/**
 * @brief 复位单轴内部状态（保留 wc/wo/b0 等参数，只清状态）
 */
void LADRC_Reset(LADRC_t *c)
{
    c->z1 = c->z2 = c->z3 = 0.0f;
    c->u  = c->u0 = 0.0f;
    c->inited = 0;
}

/*============================================================================
 *  核心计算
 *============================================================================*/

/**
 * @brief 单拍 LADRC 计算
 */
float LADRC_Calc(LADRC_t *c, float target, float feedback, float gyro_meas, float dt)
{
    if (dt < 1e-6f) dt = (float)CTRL_PERIOD_MS * 0.001f;

    /* 每拍按当前 wc/wo 重算增益 → 在线改 wc/wo 立即生效 */
    ladrc_update_gains(c);

    /* 记录目标/反馈/误差（供 Vofa） */
    c->target   = target;
    c->feedback = feedback;
    c->gyro     = c->gyro_sign * gyro_meas;   /* 实测角速度(符号校正后)，供阻尼/Vofa */
    float raw_err = target - feedback;
    if (c->angle_wrap) raw_err = ladrc_wrap180(raw_err);
    c->error = raw_err;

    /* 首拍对齐：z1=当前角度，避免初始大新息冲击 */
    if (!c->inited)
    {
        c->z1 = feedback;
        c->z2 = 0.0f;
        c->z3 = 0.0f;
        c->u  = 0.0f;
        c->inited = 1;
    }

    /* ---- 1) LESO：用上一拍输出 c->u 估计 [角度 z1, 角速度 z2, 总扰动 z3] ----
     * 喂回的是"实际限幅后"的 u(=c->u)，故 LESO 看到的就是真正施加的控制 → 自带抗饱和(无 ESO 积分风up)。*/
    float e = c->z1 - feedback;
    if (c->angle_wrap) e = ladrc_wrap180(e);

    float z1n = c->z1 + dt * (c->z2 - c->beta1 * e);
    float z2n = c->z2 + dt * (c->z3 - c->beta2 * e + c->b0 * c->u);
    float z3n = c->z3 + dt * (      - c->beta3 * e);

    /* NaN/Inf 防护：异常则保留上一拍估计 */
    if (!(isnan(z1n) || isinf(z1n))) c->z1 = z1n;
    if (!(isnan(z2n) || isinf(z2n))) c->z2 = z2n;
    if (!(isnan(z3n) || isinf(z3n))) c->z3 = z3n;

    /* ---- 2) LSEF：用 LESO 估计的角度/角速度做误差反馈（比直接用测量更干净） ----
     * 比例误差 ep = 目标 − z1；角速度参考=0 → −kd·z2 为纯阻尼。 */
    float ep = target - c->z1;
    if (c->angle_wrap) ep = ladrc_wrap180(ep);

    /* 死区软化：|ep|≤deadband 当作 0(防静态抖)；只软化比例项，
     * 扰动补偿 z3 仍守住稳态 → 不会像普通比例死区那样下垂。 */
    if (c->deadband > 0.0f)
    {
        if      (ep >  c->deadband) ep -= c->deadband;
        else if (ep < -c->deadband) ep += c->deadband;
        else                         ep  = 0.0f;
    }

    /* 阻尼项角速度来源：默认用实测陀螺(相位准、不依赖 b0)，可切回 LESO 的 z2 做纯单环对比。
     * 注意 LESO 仍照常更新 z2/z3(z3 扰动估计依赖完整观测器)，这里只换"用谁做阻尼"。*/
    float omega = c->use_gyro_damp ? c->gyro : c->z2;
    float u0 = c->kp * ep - c->kd * omega;
    c->u0 = u0;

    /* ---- 3) 扰动补偿：u = (u0 − z3) / b0 ---- */
    float b0 = c->b0;
    if (fabsf(b0) < 1e-6f) b0 = (b0 < 0.0f) ? -1e-6f : 1e-6f;  /* 防除零，保号 */
    float u = (u0 - c->z3) / b0;

    /* ---- 4) 限幅 ---- */
    u = ladrc_clamp(u, c->max_output);
    if (isnan(u) || isinf(u)) u = 0.0f;

    c->u = u;   /* 存给下一拍 LESO 用 */
    return u;
}

/**
 * @brief 三轴 LADRC 计算（角度→力矩），输出写 Surface.output_gyro_Euler[NOW][i]
 * @note  替代原 Euler_pid_Cale；ladrc_mode==1 时整机三轴全用 LADRC。
 *        通道下标 i 直接复用 PITCH=0/ROLL=1/YAW=2，与 ladrc_ctrl[] 一致。
 */
void Euler_LADRC_Cale(float delta_time)
{
    for (int i = 0; i < 3; i++)
    {
        Surface.output_gyro_Euler[NOW][i] = LADRC_Calc(
            &ladrc_ctrl[i],
            Surface.target_angle_Euler[NOW][i],   /* 目标角度 */
            Surface.current_angle_Euler[NOW][i],  /* 当前角度 */
            Surface.current_gyro_Euler[NOW][i],   /* 实测角速度(阻尼用) */
            delta_time);
    }
}
