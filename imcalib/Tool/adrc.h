/**
 * @file    adrc.h
 * @brief   ADRC自抗扰控制器 - 完整实现
 * @details 针对制导飞镖X翼构型优化的ADRC控制器
 *          包含：跟踪微分器(TD)、扩张状态观测器(ESO)、非线性状态误差反馈(NLSEF)
 *          支持串级控制：外环(角度) + 内环(角速度)
 *
 * @author  ynz
 * @date    2026/06/15
 *
 * 设计原理：
 *   ADRC不依赖精确模型，通过ESO实时估计"总扰动"（模型误差+外部干扰+耦合），
 *   然后在控制量中直接补偿，从而实现：
 *   - 大误差时快速响应（非线性增益）
 *   - 小误差时稳定不抖（小增益+阻尼）
 *   - 自动补偿气动扰动、耦合、模型误差
 *   - 对参数变化鲁棒
 */

#ifndef __ADRC_H
#define __ADRC_H

#include "stm32g4xx_hal.h"

/*============================================================================
 *  全局配置宏
 *============================================================================*/

/* ADRC通道索引 */
enum {
    ADRC_PITCH = 0,
    ADRC_ROLL  = 1,
    ADRC_YAW   = 2,
    ADRC_CH_COUNT = 3
};

/* ADRC环类型 */
enum {
    ADRC_ANGLE_LOOP = 0,    /* 外环：角度环 */
    ADRC_GYRO_LOOP  = 1,    /* 内环：角速度环 */
    ADRC_LOOP_COUNT = 2
};

/*============================================================================
 *  核心数据结构
 *============================================================================*/

/**
 * @brief 跟踪微分器(TD) - 安排过渡过程
 * @note  功能：
 *        1. 给目标信号安排光滑过渡，避免阶跃冲击
 *        2. 同时提取目标的微分信号（前馈）
 *        3. 可独立调节跟踪速度和超调量
 */
typedef struct {
    /* 输出 */
    float x1;              /* 跟踪信号（平滑后的目标） */
    float x2;              /* 微分信号（目标变化率） */

    /* 参数 */
    float r;               /* 速度因子：越大跟踪越快，但噪声敏感 */
    float h;               /* 采样周期(s) */
    float h0;              /* 滤波因子：越大滤波效果越好，但跟踪滞后 */

    /* 内部状态 */
    float x1_last;
    float x2_last;
} TD_t;

/**
 * @brief 扩张状态观测器(ESO) - ADRC核心
 * @note  功能：
 *        1. 估计系统状态（角度、角速度）
 *        2. 估计"总扰动"（模型误差+气动干扰+耦合+一切未建模动态）
 *        3. 不需要精确模型，只需要控制增益b0的大致范围
 * @note  总扰动 z3 包含：
 *        - 气动阻力矩
 *        - 舵面耦合
 *        - 重心偏移
 *        - 模型不确定
 *        - 外部干扰
 */
typedef struct {
    /* 输出（状态估计） */
    float z1;              /* 角度估计 */
    float z2;              /* 角速度估计 */
    float z3;              /* 总扰动估计 */

    /* 参数 */
    float beta1;           /* 观测器增益1（对z1的校正强度） */
    float beta2;           /* 观测器增益2（对z2的校正强度） */
    float beta3;           /* 观测器增益3（对z3的校正强度） */
    float b0;              /* 控制增益估计（系统b的标称值） */
    float alpha1;          /* fal函数指数1（z2通道，0.5~1） */
    float alpha2;          /* fal函数指数2（z3通道，0.25~0.5） */
    float delta;           /* fal函数线性区宽度（防抖振） */

    /* 内部状态 */
    float z1_last;
    float z2_last;
    float z3_last;
} ESO_t;

/**
 * @brief 非线性状态误差反馈(NLSEF) - 控制律
 * @note  功能：
 *        1. 组合TD和ESO输出产生控制量
 *        2. 非线性增益：误差大时增益大，误差小时增益小
 *        3. 加上扰动补偿，实现"模型无关"控制
 */
typedef struct {
    /* 参数 */
    float beta1;           /* 比例增益（对角度误差的响应） */
    float beta2;           /* 微分增益（对角速度误差的阻尼） */
    float alpha1;          /* fal函数指数1（比例通道） */
    float alpha2;          /* fal函数指数2（微分通道） */
    float delta;           /* fal函数线性区宽度 */

    /* 输出（用于观测） */
    float u0;              /* 补偿前控制量 */
    float u_compensated;   /* 补偿后控制量（最终输出） */
    float disturbance;     /* 估计的扰动 */
} NLSEF_t;

/**
 * @brief 完整ADRC控制器
 * @note  包含TD + ESO + NLSEF三个组件
 *        支持角度环和角速度环两种配置
 */
typedef struct {
    /* 三个核心组件 */
    TD_t    td;            /* 跟踪微分器 */
    ESO_t   eso;           /* 扩张状态观测器 */
    NLSEF_t nlsef;         /* 非线性状态误差反馈 */

    /* 配置 */
    uint8_t loop_type;     /* ADRC_ANGLE_LOOP 或 ADRC_GYRO_LOOP */
    uint8_t channel;       /* ADRC_PITCH / ADRC_ROLL / ADRC_YAW */

    /* 输出限幅 */
    float max_output;      /* 最大输出 */
    float min_output;      /* 最小输出（通常为-max_output） */

    /* 死区（可选） */
    float deadband;        /* 死区半宽，0=不启用 */

    /* 初始化标志 */
    uint8_t td_inited;     /* TD是否已初始化，0=未初始化，1=已初始化 */

    /* 用于观测的中间变量 */
    float target;          /* 当前目标 */
    float feedback;        /* 当前反馈 */
    float error;           /* 当前误差 */
    float control_out;     /* 最终控制输出 */
} ADRC_t;


/*============================================================================
 *  初始化函数
 *============================================================================*/

/**
 * @brief 初始化角度环ADRC参数（外环）
 * @param adrc ADRC结构体指针
 * @param channel 通道：ADRC_PITCH / ADRC_ROLL / ADRC_YAW
 */
void ADRC_Init_AngleLoop(ADRC_t *adrc, uint8_t channel);

/**
 * @brief 初始化角速度环ADRC参数（内环）
 * @param adrc ADRC结构体指针
 * @param channel 通道：ADRC_PITCH / ADRC_ROLL / ADRC_YAW
 */
void ADRC_Init_GyroLoop(ADRC_t *adrc, uint8_t channel);

/**
 * @brief 重置ADRC内部状态（切换模式时调用）
 */
void ADRC_Reset(ADRC_t *adrc);


/*============================================================================
 *  核心计算函数
 *============================================================================*/

/**
 * @brief ADRC完整计算（一拍）
 * @param adrc  ADRC结构体
 * @param target 目标值（角度或角速度）
 * @param feedback 反馈值（当前角度或角速度）
 * @param dt 采样周期(s)
 * @return 控制输出
 */
float ADRC_Calc(ADRC_t *adrc, float target, float feedback, float dt);

/**
 * @brief 仅运行ESO（用于观测扰动，不输出控制量）
 * @param eso ESO结构体
 * @param y 系统输出（测量值）
 * @param u 控制输入
 * @param dt 采样周期
 */
void ESO_Update(ESO_t *eso, float y, float u, float dt);


/*============================================================================
 *  串级ADRC接口（替代原有Euler_pid_Cale）
 *============================================================================*/

/**
 * @brief 初始化所有ADRC控制器（3通道 × 2环）
 * @note  在首次调用 Euler_ADRC_Cale 前调用，或在切换模式时调用
 */
void ADRC_Init_All(void);

/**
 * @brief 串级ADRC计算（角度外环 + 角速度内环）
 * @param delta_time 采样周期(s)
 * @details 替代原有的Euler_pid_Cale函数
 *          输出写入 Surface.output_gyro_Euler[NOW][i]
 */
void Euler_ADRC_Cale(float delta_time);

/**
 * @brief ADRC全局实例（3通道 × 2环）
 */
extern ADRC_t adrc_ctrl[ADRC_CH_COUNT][ADRC_LOOP_COUNT];


/*============================================================================
 *  调试接口
 *============================================================================*/

/**
 * @brief 获取ADRC扰动估计（用于Vofa观测）
 * @param channel 通道：ADRC_PITCH / ADRC_ROLL / ADRC_YAW
 * @param loop 环类型：ADRC_ANGLE_LOOP / ADRC_GYRO_LOOP
 * @return 估计的扰动值
 */
float ADRC_GetDisturbance(uint8_t channel, uint8_t loop);

/**
 * @brief 获取ADRC状态估计（用于Vofa观测）
 * @param channel 通道
 * @param loop 环类型
 * @param z1 角度估计输出
 * @param z2 角速度估计输出
 */
void ADRC_GetStateEstimate(uint8_t channel, uint8_t loop, float *z1, float *z2);


/*============================================================================
 *  参数在线调整接口（调试用）
 *============================================================================*/

/**
 * @brief 设置ESO观测器带宽（核心参数）
 * @param eso ESO结构体
 * @param w0 观测器带宽(rad/s)：越大跟踪越快，但噪声敏感
 * @note  推荐值：角度环 10~30，角速度环 50~100
 */
void ESO_SetBandwidth(ESO_t *eso, float w0);

/**
 * @brief 设置TD跟踪速度
 * @param td TD结构体
 * @param r 速度因子：越大跟踪越快
 * @note  推荐值：角度环 50~200，角速度环 200~500
 */
void TD_SetSpeed(TD_t *td, float r);

/**
 * @brief 设置NLSEF控制器带宽
 * @param nlsef NLSEF结构体
 * @param wc 控制器带宽(rad/s)：越大响应越快
 * @note  推荐值：角度环 5~15，角速度环 20~50
 */
void NLSEF_SetBandwidth(NLSEF_t *nlsef, float wc);


/*============================================================================
 *  辅助函数
 *============================================================================*/

/**
 * @brief fal函数 - ADRC核心非线性函数
 * @param e 误差
 * @param alpha 指数（0<alpha<1时有"大误差小增益，小误差大增益"特性）
 * @param delta 线性区宽度（防抖振）
 * @return fal(e, alpha, delta)
 */
float fal(float e, float alpha, float delta);

/**
 * @brief fst函数 - 最速综合函数（TD用）
 * @param x1 状态1
 * @param x2 状态2
 * @param r 速度因子
 * @param h 滤波因子
 * @return fst值
 */
float fst(float x1, float x2, float r, float h);

/**
 * @brief 限幅函数
 */
static inline float adrc_limit(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

static inline float adrc_abs_limit(float val, float limit)
{
    return adrc_limit(val, -limit, limit);
}


#endif /* __ADRC_H */
