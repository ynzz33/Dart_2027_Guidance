//
// vision_ekf.h —— 6态 bearing-only 非线性EKF
//
// 只估计世界系位置(p)和速度(v),共6个误差状态。
// 姿态固定使用 Mahony 的 R_matrix_T,不估计姿态误差和陀螺零偏。
// 加速度零偏靠第一阶段标定(A_Offset),不在滤波器内估计。
// 量测:方位角+俯仰角(不需要距离),适合视觉距离不可靠时使用。
//
// 输出:ekf_out 与旧KF的 VinsOut_t 同构,通过 ekf_mode 切换。
//
#ifndef VISION_EKF_H
#define VISION_EKF_H

#include <stdint.h>
#include "filter.h"

/* ---- 可调参数(调试时按需修改) ---- */

/* 量测噪声标准差(rad):角度量测不确定性。增大→滤波器更信任IMU预测,对量测更新更保守 */
#define EKF_SIGMA_BEARING       0.015f

/* 过程噪声标准差:加速度白噪声(m/s²)。增大→允许更大加速度变化,位置不确定度增长更快 */
#define EKF_SIGMA_ACC           2.0f

/* 初始不确定度(对角线标准差) */
#define EKF_P0_POS              10.0f    /* 位置(m):发射前目标位置未知,给大值 */
#define EKF_P0_VEL_VAR          5.0f     /* 速度先验方差(m/s)²:不是速度值也不是标准差。
                                            当前临时值,待由实际 V_NOM_MS 离线测速标准差确定:
                                            若测得 sigma_v0 m/s,则设 EKF_P0_VEL_VAR = sigma_v0*sigma_v0 */

/* ZUPT量测噪声标准差(m/s):静止时速度=0的量测不确定性。越小→ZUPT越强 */
#define EKF_SIGMA_ZUPT          0.05f

/* bearing-only 初始距离先验(m):第一次识别目标时的参考距离,不是真实距离 */
#define EKF_RANGE_PRIOR         5.0f

/* 新息门控卡方阈值(自由度=2):超过此值丢弃本帧量测。20对应约p<1e-4 */
#define EKF_INNO_GATE_CHI2      20.0f

/* ---- 输出结构 ---- */
typedef struct {
    float p_world[3];   /* 世界系位置(m) */
    float v_world[3];   /* 世界系速度(m/s) */
    float range_m;      /* 距目标距离(m) = |p_world|,仅作名义参考 */
    float vc;           /* 接近速度(m/s) = -p·v/|p|,仅作名义参考 */
    uint8_t locked;     /* 1=至少一次量测更新过 */
} EkfOut_t;

extern EkfOut_t ekf_out;

/* ---- 矩阵工作缓冲区(CMSIS-DSP) ----
 * 每个函数使用独立缓冲区,同函数内可复用,不同函数间不复用。
 * Pm 绑定到 vision_ekf.c 中的 static float P[6][6]。 */
typedef struct {
    /* ── 全局 ── */
    float _pm[36];       mat   Pm;       /* 6×6  协方差 */

    /* ── Predict 专属 ── */
    float _f[36];        mat   F;        /* 6×6  状态转移 */
    float _q[36];        mat   Q;        /* 6×6  过程噪声 */
    float _fp[36];       mat   FP;       /* 6×6  F·P / FPFᵀ */

    /* ── UpdateBearing 专属 ── */
    float _hp[12];       mat   HP;       /* 2×6  H·P */
    float _ht[12];       mat   Ht;       /* 6×2  Hᵀ */
    float _s2[4];        mat   S2;       /* 2×2  新息协方差 */
    float _sinv2[4];     mat   Sinv2;    /* 2×2  S⁻¹ */
    float _ph[12];       mat   PHt;      /* 6×2  P·Hᵀ */
    float _k2[12];       mat   K2;       /* 6×2  增益 */
    float _ks[12];       mat   KS;       /* 6×2  K·S */
    float _ksk[36];      mat   KSK;      /* 6×6  K·S·Kᵀ */
    float _y6[6];        mat   y6;       /* 6×1  状态修正向量 */

    /* ── UpdateZeroVel 专属 ── */
    float _s3[9];        mat   S3;       /* 3×3  新息协方差 */
    float _sinv3[9];     mat   Sinv3;    /* 3×3  S⁻¹ */
    float _ppv[9];       mat   P_pv;     /* 3×3  位置-速度协方差块 */
    float _kp[9];        mat   Kp;       /* 3×3  位置增益 */
    float _kv[9];        mat   Kv;       /* 3×3  速度增益 */
    float _k63[18];      mat   K63;      /* 6×3  组合增益[Kp;Kv] */
    float _d_state[6];   mat   d_state;  /* 6×1  状态修正向量 */
    float _ksk_z[36];    mat   KSK_z;    /* 6×6  K·S·Kᵀ */
} EkfMat;

/* ---- 接口函数 ---- */

/* 上电初始化(在 Init_Config.c 中调用, VisInsEKF_Init 之后) */
void EKF_Init(void);

/* 发射前复位:清零状态+协方差+位置初始化标志,供重复发射时调用 */
void EKF_Reset(void);

/* 查询位置是否已初始化(首帧视觉后为1) */
uint8_t EKF_PosInited(void);

/* 位置初始化:第一次识别目标时调用,用首帧视觉角度+先验距离构造初始位置 */
void EKF_InitPos(float az_rad, float el_rad, float range_prior);

/* IMU预测:每拍(1kHz)调用,输入世界系去重力加速度(m/s²)和采样周期 */
void EKF_Predict(const float acc_world[3], float dt);

/* 方位/俯仰量测更新:视觉识别成功时调用,输入弧度 */
void EKF_UpdateBearing(float az_rad, float el_rad);

/* 速度锚定:俯冲入段时调用,直接设速度+收紧协方差 */
void EKF_SetVel(float vx, float vy, float vz);

/* 零速更新:物理静止时调用,速度归零+协方差收紧 */
void EKF_UpdateZeroVel(void);

#endif /* VISION_BEARING_EKF_H */
