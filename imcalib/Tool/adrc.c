/**
 * @file    adrc.c
 * @brief   ADRC自抗扰控制器实现
 * @details 针对制导飞镖X翼构型优化
 *
 * @author  ynz
 * @date    2026/06/15
 *
 * ============================================================================
 *  ADRC工作原理（通俗解释）：
 *
 *  传统PID的问题：
 *  - Kp大了抖，小了慢 → 线性增益的固有矛盾
 *  - 不知道"扰动有多大" → 只能被动响应
 *  - 目标跳变时误差突变 → 微分冲击
 *
 *  ADRC怎么解决：
 *  1. TD：不让目标跳变，安排一个光滑过渡（"你慢慢来，我跟得上"）
 *  2. ESO：实时估计"总扰动"有多大（"我知道风在吹我"）
 *  3. NLSEF：误差大时猛打，误差小时轻柔（"该出手时出手，该收手时收手"）
 *  4. 扰动补偿：知道扰动多大，直接减掉（"风往左吹，我往右推"）
 *
 *  结果：不需要精确模型，自动补偿一切扰动，大误差快、小误差稳
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

/* ADRC控制器实例：3通道 × 2环 */
ADRC_t adrc_ctrl[ADRC_CH_COUNT][ADRC_LOOP_COUNT];


/*============================================================================
 *  辅助函数实现
 *============================================================================*/

/**
 * @brief fal函数 - ADRC核心非线性函数
 * @note  特性：
 *        - |e| < delta 时：近似线性（防抖振）
 *        - |e| > delta 时：非线性（大误差小增益，小误差大增益）
 *
 *  数学表达式：
 *        fal(e, α, δ) = { e / δ^(1-α)          , |e| ≤ δ
 *                       { |e|^α · sign(e)       , |e| > δ
 *
 *  当α<1时：
 *        - 误差大时增益小（不猛打，防超调）
 *        - 误差小时增益大（精细调节，消除稳态误差）
 *        - 这就是ADRC能"又快又稳"的秘密
 */
float fal(float e, float alpha, float delta)
{
    if (delta < 1e-6f) delta = 1e-6f;  /* 防除零 */

    float abs_e = fabsf(e);

    if (abs_e <= delta)
    {
        /* 线性区：近似线性，防抖振 */
        return e / powf(delta, 1.0f - alpha);
    }
    else
    {
        /* 非线性区：大误差小增益，小误差大增益 */
        float sign = (e > 0.0f) ? 1.0f : -1.0f;
        float result = sign * powf(abs_e, alpha);

        /* NaN/Inf 防护 */
        if (isnan(result) || isinf(result))
            return sign * 100.0f;  /* 限幅保护 */

        return result;
    }
}

/**
 * @brief fst函数 - 最速综合函数（韩志刚最速控制）
 * @note  用于跟踪微分器(TD)，实现"有限时间收敛"
 *        比简单的积分跟踪更快、更平滑
 *
 *  原理：
 *        在相平面(x1, x2)上，fst是"最速到达原点"的控制量
 *        比bang-bang控制更平滑，比线性控制更快
 */
float fst(float x1, float x2, float r, float h)
{
    if (h < 1e-6f) h = 1e-6f;  /* 防除零 */

    float d = r * h;
    float d0 = d * h;
    float y = x1 + h * x2;

    /* sqrtf 参数防护 */
    float sqrt_arg = d * d + 8.0f * r * fabsf(y);
    if (sqrt_arg < 0.0f) sqrt_arg = 0.0f;
    float a0 = sqrtf(sqrt_arg);

    float a;
    if (fabsf(y) > d0)
    {
        /* 在抛物线外侧 */
        a = x2 + (a0 - d) * ((y > 0.0f) ? 0.5f : -0.5f);
    }
    else
    {
        /* 在抛物线内侧 */
        a = x2 + y / h;
    }

    /* 输出限幅 */
    if (fabsf(a) > d)
    {
        return -r * ((a > 0.0f) ? 1.0f : -1.0f);
    }
    else
    {
        return -r * a / d;
    }
}


/*============================================================================
 *  跟踪微分器(TD)实现
 *============================================================================*/

/**
 * @brief TD初始化
 * @param td TD结构体
 * @param h 采样周期(s)
 */
static void TD_Init(TD_t *td, float h)
{
    td->h = h;
    td->x1 = 0.0f;
    td->x2 = 0.0f;
    td->x1_last = 0.0f;
    td->x2_last = 0.0f;
}

/**
 * @brief TD计算（一拍）
 * @param td TD结构体
 * @param target 目标值
 * @note  输出：td->x1（平滑目标），td->x2（目标微分）
 *
 *  离散算法：
 *        x1(k+1) = x1(k) + h · x2(k)
 *        x2(k+1) = x2(k) + h · fst(x1(k) - target, x2(k), r, h0)
 *
 *  效果：
 *        - 目标阶跃时，x1光滑过渡到目标（不会突变）
 *        - x2是目标变化率，可用作前馈
 */
static void TD_Update(TD_t *td, float target)
{
    /* 最速跟踪 */
    float f = fst(td->x1_last - target, td->x2_last, td->r, td->h0);

    /* 离散积分 */
    td->x1 = td->x1_last + td->h * td->x2_last;
    td->x2 = td->x2_last + td->h * f;

    /* 更新历史 */
    td->x1_last = td->x1;
    td->x2_last = td->x2;
}


/*============================================================================
 *  扩张状态观测器(ESO)实现
 *============================================================================*/

/**
 * @brief ESO初始化
 * @param eso ESO结构体
 * @param h 采样周期(s)
 */
static void ESO_Init(ESO_t *eso, float h)
{
    eso->z1 = 0.0f;
    eso->z2 = 0.0f;
    eso->z3 = 0.0f;
    eso->z1_last = 0.0f;
    eso->z2_last = 0.0f;
    eso->z3_last = 0.0f;
}

/**
 * @brief ESO计算（一拍）
 * @param eso ESO结构体
 * @param y 系统输出（测量值）
 * @param u 控制输入
 * @param h 采样周期
 * @note  这是ADRC的核心！
 *
 *  三阶ESO离散算法：
 *        e = z1 - y
 *        z1(k+1) = z1(k) + h · (z2(k) - β1 · e)
 *        z2(k+1) = z2(k) + h · (z3(k) - β2 · fal(e, α1, δ) + b0 · u)
 *        z3(k+1) = z3(k) + h · (-β3 · fal(e, α2, δ))
 *
 *  含义：
 *        z1 → 角度估计（比测量更平滑）
 *        z2 → 角速度估计（不需要微分）
 *        z3 → 总扰动估计（模型误差+气动+耦合+一切）
 *
 *  关键洞察：
 *        如果系统是 ẍ = f(x,ẋ,t) + b·u
 *        那么z3估计的就是f（总扰动）
 *        控制时只需 u = (u0 - z3) / b0 就能抵消扰动
 */
static void ESO_Update_Internal(ESO_t *eso, float y, float u, float h)
{
    /* 观测误差 */
    float e = eso->z1_last - y;

    /* 状态更新（梯形法离散，比欧拉法更精确） */
    float dz1 = eso->z2_last - eso->beta1 * e;
    float dz2 = eso->z3_last - eso->beta2 * fal(e, eso->alpha1, eso->delta) + eso->b0 * u;
    float dz3 = -eso->beta3 * fal(e, eso->alpha2, eso->delta);

    eso->z1 = eso->z1_last + h * dz1;
    eso->z2 = eso->z2_last + h * dz2;
    eso->z3 = eso->z3_last + h * dz3;

    /* NaN/Inf 防护 */
    if (isnan(eso->z1) || isinf(eso->z1)) eso->z1 = eso->z1_last;
    if (isnan(eso->z2) || isinf(eso->z2)) eso->z2 = eso->z2_last;
    if (isnan(eso->z3) || isinf(eso->z3)) eso->z3 = eso->z3_last;

    /* 更新历史 */
    eso->z1_last = eso->z1;
    eso->z2_last = eso->z2;
    eso->z3_last = eso->z3;
}

/* 外部接口 */
void ESO_Update(ESO_t *eso, float y, float u, float dt)
{
    ESO_Update_Internal(eso, y, u, dt);
}


/*============================================================================
 *  非线性状态误差反馈(NLSEF)实现
 *============================================================================*/

/**
 * @brief NLSEF计算
 * @param nlsef NLSEF结构体
 * @param e1 角度误差（TD输出 - ESO估计）
 * @param e2 角速度误差（TD微分 - ESO估计）
 * @param z3 ESO估计的总扰动
 * @param b0 控制增益估计
 * @return 补偿后的控制量
 *
 *  算法：
 *        u0 = β1 · fal(e1, α1, δ) + β2 · fal(e2, α2, δ)
 *        u = (u0 - z3) / b0
 *
 *  关键：
 *        - fal的非线性增益：误差大时猛打，误差小时轻柔
 *        - z3/b0补偿：直接减掉估计的扰动，实现"模型无关"控制
 */
static float NLSEF_Calc(NLSEF_t *nlsef, float e1, float e2, float z3, float b0)
{
    /* 非线性组合 */
    nlsef->u0 = nlsef->beta1 * fal(e1, nlsef->alpha1, nlsef->delta)
              + nlsef->beta2 * fal(e2, nlsef->alpha2, nlsef->delta);

    /* 扰动补偿 */
    nlsef->disturbance = z3;
    if (fabsf(b0) < 1e-6f) b0 = 1e-6f;  /* 防除零 */
    nlsef->u_compensated = (nlsef->u0 - z3) / b0;

    /* NaN/Inf 防护 */
    if (isnan(nlsef->u_compensated) || isinf(nlsef->u_compensated))
        nlsef->u_compensated = 0.0f;

    return nlsef->u_compensated;
}


/*============================================================================
 *  完整ADRC计算
 *============================================================================*/

/**
 * @brief ADRC完整计算（一拍）
 * @param adrc  ADRC结构体
 * @param target 目标值（角度或角速度）
 * @param feedback 反馈值（当前角度或角速度）
 * @param dt 采样周期(s)
 * @return 控制输出
 *
 *  计算流程：
 *        1. TD：目标 → 平滑目标 + 目标微分
 *        2. ESO：测量 + 上一拍控制 → 状态估计 + 扰动估计
 *        3. NLSEF：(TD输出 - ESO状态) → 控制量 - 扰动补偿
 *        4. 限幅输出
 */
float ADRC_Calc(ADRC_t *adrc, float target, float feedback, float dt)
{
    /* 保存用于观测 */
    adrc->target = target;
    adrc->feedback = feedback;
    adrc->error = target - feedback;

    /* 首次调用初始化：用当前目标初始化TD，避免初始冲击 */
    if (!adrc->td_inited)
    {
        adrc->td.x1 = target;
        adrc->td.x1_last = target;
        adrc->td.x2 = 0.0f;
        adrc->td.x2_last = 0.0f;
        adrc->eso.z1 = feedback;  /* 用当前反馈初始化ESO */
        adrc->eso.z1_last = feedback;
        adrc->td_inited = 1;
    }

    /* 死区处理（如果启用） */
    float err_for_calc = adrc->error;
    if (adrc->deadband > 0.0f)
    {
        if (fabsf(err_for_calc) < adrc->deadband)
            err_for_calc = 0.0f;
        else
            err_for_calc -= (err_for_calc > 0.0f) ? adrc->deadband : -adrc->deadband;
    }

    /* 计算死区处理后的目标（用于TD跟踪） */
    float target_for_td = feedback + err_for_calc;

    /* 1. 跟踪微分器(TD)：安排过渡过程 */
    /*    使用死区处理后的目标，确保死区生效 */
    TD_Update(&adrc->td, target_for_td);

    /* 2. 扩张状态观测器(ESO)：估计状态和扰动 */
    /*    注意：ESO用上一拍的控制量，避免代数环 */
    ESO_Update_Internal(&adrc->eso, feedback, adrc->control_out, dt);

    /* 3. 非线性状态误差反馈(NLSEF)：产生控制量 */
    float e1 = adrc->td.x1 - adrc->eso.z1;     /* 角度误差 */
    float e2 = adrc->td.x2 - adrc->eso.z2;     /* 角速度误差 */
    float u = NLSEF_Calc(&adrc->nlsef, e1, e2, adrc->eso.z3, adrc->eso.b0);

    /* 4. 限幅 */
    u = adrc_limit(u, adrc->min_output, adrc->max_output);

    /* NaN/Inf 防护 */
    if (isnan(u) || isinf(u))
        u = 0.0f;

    /* 保存控制输出（下一拍ESO用） */
    adrc->control_out = u;

    return u;
}


/*============================================================================
 *  参数调整接口
 *============================================================================*/

/**
 * @brief 设置ESO观测器带宽
 * @note  根据带宽法自动计算β1, β2, β3
 *
 *  带宽法原理：
 *        期望观测器特征多项式为 (s + ω₀)³
 *        展开得：s³ + 3ω₀s² + 3ω₀²s + ω₀³
 *        对比系数：β1=3ω₀, β2=3ω₀², β3=ω₀³
 *
 *  调参指南：
 *        - ω₀越大：跟踪越快，但对噪声敏感
 *        - 角度环：10~30（响应不需要太快）
 *        - 角速度环：50~100（需要快速跟踪）
 */
void ESO_SetBandwidth(ESO_t *eso, float w0)
{
    eso->beta1 = 3.0f * w0;
    eso->beta2 = 3.0f * w0 * w0;
    eso->beta3 = w0 * w0 * w0;
}

/**
 * @brief 设置TD跟踪速度
 * @note  r越大跟踪越快，但对噪声敏感
 *        角度环：50~200
 *        角速度环：200~500
 */
void TD_SetSpeed(TD_t *td, float r)
{
    td->r = r;
}

/**
 * @brief 设置NLSEF控制器带宽
 * @note  根据带宽法自动计算β1, β2
 *
 *  带宽法原理：
 *        期望闭环特征多项式为 (s + ωc)²
 *        展开得：s² + 2ωcs + ωc²
 *        对比系数：比例增益=ωc²，微分增益=2ωc
 */
void NLSEF_SetBandwidth(NLSEF_t *nlsef, float wc)
{
    nlsef->beta1 = wc * wc;
    nlsef->beta2 = 2.0f * wc;
}


/*============================================================================
 *  初始化函数实现
 *============================================================================*/

/**
 * @brief 初始化角度环ADRC（外环）
 * @note  基于当前PID参数映射配置
 *
 *  PID → ADRC 参数映射原则：
 *        - ESO带宽 ω₀ ≈ 3~5 × Kp（观测器要比闭环快）
 *        - NLSEF带宽 ωc ≈ Kp（保持相似的响应速度）
 *        - TD速度 r：根据目标变化率需求
 *        - b0：控制增益，一般从1.0开始调
 *
 *  当前PID参数（镖体1）：
 *        PITCH: Kp=8.0,  Ki=1.0, Kd=0.5,  MaxOut=200, deadband=1.0°
 *        ROLL:  Kp=0.9,  Ki=0,   Kd=0.02, MaxOut=1200, deadband=1.0°
 *        YAW:   Kp=6.0,  Ki=0.5, Kd=0.02, MaxOut=500, deadband=0.2°
 */
void ADRC_Init_AngleLoop(ADRC_t *adrc, uint8_t channel)
{
    adrc->channel = channel;
    adrc->loop_type = ADRC_ANGLE_LOOP;

    /* TD参数 */
    TD_Init(&adrc->td, 0.001f);      /* 1kHz采样 */
    adrc->td.h0 = 0.005f;            /* 滤波因子：5ms */

    /* ESO参数（通用） */
    ESO_Init(&adrc->eso, 0.001f);
    adrc->eso.alpha1 = 0.75f;        /* z2通道：0.5~1 */
    adrc->eso.alpha2 = 0.5f;         /* z3通道：0.25~0.5 */
    adrc->eso.delta = 0.01f;         /* 线性区宽度：约0.6° */

    /* NLSEF参数（通用） */
    adrc->nlsef.alpha1 = 0.75f;
    adrc->nlsef.alpha2 = 0.75f;      /* 微分通道指数：0.5~1 */
    adrc->nlsef.delta = 0.01f;

    /* 根据通道设置参数（映射自PID） */
    switch (channel)
    {
        case ADRC_PITCH:
            /* PID: Kp=8.0, Ki=1.0, Kd=0.5 → 闭环带宽约8~12 rad/s */
            adrc->td.r = 100.0f;                    /* 跟踪速度：中等偏快 */
            adrc->eso.b0 = 1.0f;                    /* 控制增益估计 */
            ESO_SetBandwidth(&adrc->eso, 40.0f);    /* ω₀=40 ≈ 5×Kp，观测器快 */
            NLSEF_SetBandwidth(&adrc->nlsef, 10.0f); /* ωc=10 ≈ Kp，保持响应速度 */
            adrc->max_output = 200.0f;              /* 与PID MaxOut一致 */
            adrc->min_output = -200.0f;
            adrc->deadband = 1.0f;                  /* 与PID一致 */
            break;

        case ADRC_ROLL:
            /* PID: Kp=0.9, Ki=0, Kd=0.02 → 闭环带宽约2~4 rad/s */
            adrc->td.r = 50.0f;                     /* 跟踪速度：较慢 */
            adrc->eso.b0 = 1.0f;
            ESO_SetBandwidth(&adrc->eso, 15.0f);    /* ω₀=15 ≈ 5×Kp */
            NLSEF_SetBandwidth(&adrc->nlsef, 3.0f);  /* ωc=3 ≈ Kp×3 */
            adrc->max_output = 1200.0f;             /* 与PID MaxOut一致 */
            adrc->min_output = -1200.0f;
            adrc->deadband = 1.0f;                  /* 与PID一致 */
            break;

        case ADRC_YAW:
            /* PID: Kp=6.0, Ki=0.5, Kd=0.02 → 闭环带宽约6~10 rad/s */
            adrc->td.r = 80.0f;                     /* 跟踪速度：中等 */
            adrc->eso.b0 = 1.0f;
            ESO_SetBandwidth(&adrc->eso, 30.0f);    /* ω₀=30 = 5×Kp */
            NLSEF_SetBandwidth(&adrc->nlsef, 8.0f);  /* ωc=8 ≈ Kp */
            adrc->max_output = 500.0f;              /* 与PID MaxOut一致 */
            adrc->min_output = -500.0f;
            adrc->deadband = 0.2f;                  /* 与PID一致 */
            break;
    }
}

/**
 * @brief 初始化角速度环ADRC（内环）
 * @note  基于当前PID参数映射配置
 *
 *  当前PID参数（镖体1）：
 *        PITCH: Kp=0.8, MaxOut=20.0, deadband=1.0°
 *        ROLL:  Kp=0.2, MaxOut=15.0, deadband=0.5°
 *        YAW:   Kp=0.4, MaxOut=40.0, deadband=0.0°
 */
void ADRC_Init_GyroLoop(ADRC_t *adrc, uint8_t channel)
{
    adrc->channel = channel;
    adrc->loop_type = ADRC_GYRO_LOOP;

    /* TD参数 */
    TD_Init(&adrc->td, 0.001f);
    adrc->td.h0 = 0.002f;            /* 滤波因子更小：2ms */

    /* ESO参数（通用） */
    ESO_Init(&adrc->eso, 0.001f);
    adrc->eso.alpha1 = 0.5f;         /* 内环用更激进的非线性 */
    adrc->eso.alpha2 = 0.25f;
    adrc->eso.delta = 0.1f;          /* 线性区：约5.7°/s */

    /* NLSEF参数（通用） */
    adrc->nlsef.alpha1 = 0.5f;
    adrc->nlsef.alpha2 = 0.75f;
    adrc->nlsef.delta = 0.1f;

    /* 根据通道设置参数（映射自PID） */
    switch (channel)
    {
        case ADRC_PITCH:
            /* PID: Kp=0.8, MaxOut=20.0 → 内环带宽约5~10 rad/s */
            adrc->td.r = 200.0f;                    /* 内环跟踪更快 */
            adrc->eso.b0 = 1.0f;
            ESO_SetBandwidth(&adrc->eso, 40.0f);    /* ω₀=40 = 50×Kp，内环观测要快 */
            NLSEF_SetBandwidth(&adrc->nlsef, 6.0f);  /* ωc=6 ≈ 8×Kp */
            adrc->max_output = 20.0f;               /* 与PID MaxOut一致 */
            adrc->min_output = -20.0f;
            adrc->deadband = 1.0f;                  /* 与PID一致 */
            break;

        case ADRC_ROLL:
            /* PID: Kp=0.2, MaxOut=15.0 → 内环带宽约2~5 rad/s */
            adrc->td.r = 150.0f;
            adrc->eso.b0 = 1.0f;
            ESO_SetBandwidth(&adrc->eso, 20.0f);    /* ω₀=20 = 100×Kp */
            NLSEF_SetBandwidth(&adrc->nlsef, 3.0f);  /* ωc=3 = 15×Kp */
            adrc->max_output = 15.0f;               /* 与PID MaxOut一致 */
            adrc->min_output = -15.0f;
            adrc->deadband = 0.5f;                  /* 与PID一致 */
            break;

        case ADRC_YAW:
            /* PID: Kp=0.4, MaxOut=40.0 → 内环带宽约3~8 rad/s */
            adrc->td.r = 180.0f;
            adrc->eso.b0 = 1.0f;
            ESO_SetBandwidth(&adrc->eso, 30.0f);    /* ω₀=30 = 75×Kp */
            NLSEF_SetBandwidth(&adrc->nlsef, 5.0f);  /* ωc=5 = 12.5×Kp */
            adrc->max_output = 40.0f;               /* 与PID MaxOut一致 */
            adrc->min_output = -40.0f;
            adrc->deadband = 0.0f;                  /* 与PID一致 */
            break;
    }
}

/**
 * @brief 重置ADRC内部状态
 */
void ADRC_Reset(ADRC_t *adrc)
{
    adrc->td.x1 = 0.0f;
    adrc->td.x2 = 0.0f;
    adrc->td.x1_last = 0.0f;
    adrc->td.x2_last = 0.0f;

    adrc->eso.z1 = 0.0f;
    adrc->eso.z2 = 0.0f;
    adrc->eso.z3 = 0.0f;
    adrc->eso.z1_last = 0.0f;
    adrc->eso.z2_last = 0.0f;
    adrc->eso.z3_last = 0.0f;

    adrc->control_out = 0.0f;
    adrc->nlsef.u0 = 0.0f;
    adrc->nlsef.u_compensated = 0.0f;
    adrc->nlsef.disturbance = 0.0f;

    /* 重置初始化标志 */
    adrc->td_inited = 0;
}


/*============================================================================
 *  串级ADRC接口（替代原有Euler_pid_Cale）
 *============================================================================*/

/**
 * @brief 初始化所有ADRC控制器
 */
void ADRC_Init_All(void)
{
    /* 角度环（外环） */
    ADRC_Init_AngleLoop(&adrc_ctrl[ADRC_PITCH][ADRC_ANGLE_LOOP], ADRC_PITCH);
    ADRC_Init_AngleLoop(&adrc_ctrl[ADRC_ROLL][ADRC_ANGLE_LOOP],  ADRC_ROLL);
    ADRC_Init_AngleLoop(&adrc_ctrl[ADRC_YAW][ADRC_ANGLE_LOOP],   ADRC_YAW);

    /* 角速度环（内环） */
    ADRC_Init_GyroLoop(&adrc_ctrl[ADRC_PITCH][ADRC_GYRO_LOOP], ADRC_PITCH);
    ADRC_Init_GyroLoop(&adrc_ctrl[ADRC_ROLL][ADRC_GYRO_LOOP],  ADRC_ROLL);
    ADRC_Init_GyroLoop(&adrc_ctrl[ADRC_YAW][ADRC_GYRO_LOOP],   ADRC_YAW);
}

/**
 * @brief 串级ADRC计算（替代Euler_pid_Cale）
 * @param delta_time 采样周期(s)
 * @details
 *        计算流程（每轴）：
 *        1. 外环ADRC：目标角度 → 角速度指令
 *        2. 内环ADRC：角速度指令 → 舵面输出
 *
 *        输出写入 Surface.output_gyro_Euler[NOW][i]
 * @note  ADRC_Init_All() 已在 TotalInitTask() 中调用，此处不再重复初始化
 */
void Euler_ADRC_Cale(float delta_time)
{
    for (int i = 0; i < 3; i++)
    {
        /* 外环：角度 → 角速度指令 */
        float gyro_cmd = ADRC_Calc(
            &adrc_ctrl[i][ADRC_ANGLE_LOOP],
            Surface.target_angle_Euler[NOW][i],   /* 目标角度 */
            Surface.current_angle_Euler[NOW][i],  /* 当前角度 */
            delta_time
        );

        /* 内环：角速度指令 → 舵面输出 */
        Surface.output_gyro_Euler[NOW][i] = ADRC_Calc(
            &adrc_ctrl[i][ADRC_GYRO_LOOP],
            gyro_cmd,                              /* 外环输出作为内环目标 */
            Surface.current_gyro_Euler[NOW][i],   /* 当前角速度 */
            delta_time
        );
    }
}


/*============================================================================
 *  调试接口
 *============================================================================*/

/**
 * @brief 获取ADRC扰动估计（用于Vofa观测）
 * @param channel 通道
 * @param loop 环类型
 * @return 估计的扰动值
 */
float ADRC_GetDisturbance(uint8_t channel, uint8_t loop)
{
    if (channel < ADRC_CH_COUNT && loop < ADRC_LOOP_COUNT)
        return adrc_ctrl[channel][loop].eso.z3;
    return 0.0f;
}

/**
 * @brief 获取ADRC状态估计（用于Vofa观测）
 * @param channel 通道
 * @param loop 环类型
 * @param z1 角度估计输出
 * @param z2 角速度估计输出
 */
void ADRC_GetStateEstimate(uint8_t channel, uint8_t loop, float *z1, float *z2)
{
    if (channel < ADRC_CH_COUNT && loop < ADRC_LOOP_COUNT)
    {
        *z1 = adrc_ctrl[channel][loop].eso.z1;
        *z2 = adrc_ctrl[channel][loop].eso.z2;
    }
}
