/**
 * @file    lqi_torque.c
 * @brief   飞镖 LQI 力矩控制器实现（9态→3轴力矩）
 * @details 控制律: tau_cmd = -K_lqi * xa
 *
 *          ★ K_lqi 与速度无关（B_tau = I⁻¹ 不含气动），存单矩阵。
 *          ★ H_tau(V_DART_Lqi) = (V_DART_Lqi/V_ref)² * H_tau_Vref，每拍解析计算。
 *          ★ 力矩→舵面由 torque_allocator 独立完成。
 *
 *          [扩展] 启用气动阻尼(DART_LQI_ENABLE_AERO_A=true)后：
 *            - K_lqi 随速变化 → 需恢复速度表(lqi_K_table)、
 *              find_speed_index()、LQI_InterpolateK() 架构
 *            - MATLAB 重新生成 lqi_gain_table.h（多速点表）
 *            - LQI_Init 改为加载标称速 K，LQI_Gain_Update50Hz
 *              恢复为 LQI_InterpolateK(V_DART_Lqi, lqi_ctrl.K_lqi)
 *
 *          ★ 未编译：需手动加入 Keil/eIDE 工程编译列表。待台架。
 *
 * @author  ynz (AI 移植)
 * @date    2026/07/23
 */

#include "lqi_torque.h"
#include "lqi_gain_table.h"              /* lqi_K[3][9] 单矩阵 */
#include "lqi_geometry_table.h"          /* lqi_H_tau_Vref + lqi_N_ry */
#include "torque_allocator.h"            /* 零空间分配器 */
#include "../Task/surface_control_task.h"  /* Surface / PITCH,ROLL,YAW / Guidance_State */
#include "../Task/IMU.h"                   /* IMU_Data.Velocity */
#include "../User/common_defs.h"           /* DEG2RAD / RAD2DEG */
#include <math.h>                          /* sqrtf, fabsf */
#include <string.h>                        /* memset, memcpy */

/*============================================================================
 *  全局实例 + 运行时旋钮
 *============================================================================*/
LQI_Control_t lqi_ctrl;
uint8_t lqi_mode = 1;       /* 0=关(LQR) 1=LQI力矩分配 */
uint8_t lqi_alloc_mode = 0; /* 0=简单pinv(H_tau) 1=零空间Pitch保护 */
float V_DART_Lqi = 0;

/*============================================================================
 *  LQI 控制更新
 *============================================================================*/

/**
 * @brief 组装增广状态，执行 tau = -K_lqi * xa，更新积分
 * @param dt  采样周期 (s)
 */
void LQI_Update(float dt)
{
    float (*K)[LQI_STATE_DIM] = lqi_ctrl.K_lqi;

    /* ---- 1) 组装增广状态 xa ---- */
    lqi_ctrl.xa[0] = lqi_ctrl.attitude_error_rad[0];   /* e_roll */
    lqi_ctrl.xa[1] = lqi_ctrl.attitude_error_rad[1];   /* e_pitch */
    lqi_ctrl.xa[2] = lqi_ctrl.attitude_error_rad[2];   /* e_yaw */
    lqi_ctrl.xa[3] = lqi_ctrl.body_rate_rad_s[0];      /* p */
    lqi_ctrl.xa[4] = lqi_ctrl.body_rate_rad_s[1];      /* q */
    lqi_ctrl.xa[5] = lqi_ctrl.body_rate_rad_s[2];      /* r */
    lqi_ctrl.xa[6] = lqi_ctrl.integral_error[0];
    lqi_ctrl.xa[7] = lqi_ctrl.integral_error[1];
    lqi_ctrl.xa[8] = lqi_ctrl.integral_error[2];

    /* ---- 2) 力矩指令: tau = -K_lqi * xa × LQI_GAIN_SCALAR ---- */
    /* 分两段算：非积分部分 + 积分部分（积分部分单独限幅） */
    {
        static const float integ_torque_limits[3] = {
            LQI_INTEG_TORQUE_LIMIT_ROLL,
            LQI_INTEG_TORQUE_LIMIT_PITCH,
            LQI_INTEG_TORQUE_LIMIT_YAW
        };
        for (uint8_t i = 0; i < LQI_TORQUE_DIM; i++)
        {
            float tau_noI = 0.0f;  /* 非积分贡献：角度误差 + 角速度 */
            float tau_I   = 0.0f;  /* 积分贡献 */
            for (uint8_t j = 0; j < LQI_STATE_DIM; j++)
            {
                float term = -K[i][j] * lqi_ctrl.xa[j];
                if (j >= 6)  /* xa[6..8] = ∫e_roll, ∫e_pitch, ∫e_yaw */
                    tau_I += term;
                else
                    tau_noI += term;
            }
            /* 积分部分限幅 */
            float lim = integ_torque_limits[i];
            if (tau_I >  lim) tau_I =  lim;
            if (tau_I < -lim) tau_I = -lim;
            lqi_ctrl.torque_cmd_Nm[i] = (tau_noI + tau_I) * LQI_GAIN_SCALAR;
        }
    }

    /* ---- 3) 积分更新（积分分离 + 抗饱和 + 限幅，逐轴独立） ---- */
    {
        static const float integ_limits[3] = {
            LQI_INTEG_LIMIT_ROLL,
            LQI_INTEG_LIMIT_PITCH,
            LQI_INTEG_LIMIT_YAW
        };
        for (uint8_t i = 0; i < 3; i++)
        {
            /* 积分分离：大误差 → 清零积分（靠 P/D 拉回）；小误差 → 开启积分（消静差） */
            if (fabsf(lqi_ctrl.attitude_error_rad[i]) >= LQI_INTEG_THRESHOLD_RAD)
            {
                lqi_ctrl.integral_error[i] = 0.0f;
                continue;
            }
            if (lqi_ctrl.freeze_integrator[i])
                continue;
            lqi_ctrl.integral_error[i] += lqi_ctrl.attitude_error_rad[i] * dt;

            /* clamp 防止 deep windup */
            float lim = integ_limits[i];
            if (lqi_ctrl.integral_error[i] >  lim) lqi_ctrl.integral_error[i] =  lim;
            if (lqi_ctrl.integral_error[i] < -lim) lqi_ctrl.integral_error[i] = -lim;
        }
    }
}

/*============================================================================
 *  50Hz 速度更新（仅缓存 V，供 1kHz 计算 H_tau(V_DART_Lqi) 用）
 *============================================================================*/

void LQI_Velocity_Update50Hz(void)
{
    if (lqi_mode == 0) return;

    float vx = IMU_Data.Velocity[World][NOW][X];
    float vy = IMU_Data.Velocity[World][NOW][Y];
    float vz = IMU_Data.Velocity[World][NOW][Z];
    V_DART_Lqi = sqrtf(vx * vx + vy * vy + vz * vz);

    /* clamp 防止 V_DART_Lqi=0 导致除零 */
    if (V_DART_Lqi < LQI_V_MIN) V_DART_Lqi = LQI_V_MIN;
    if (V_DART_Lqi > LQI_V_MAX) V_DART_Lqi = LQI_V_MAX;

    lqi_ctrl.cached_V = V_DART_Lqi;
}

/*============================================================================
 *  初始化
 *============================================================================*/

void LQI_Init(void)
{
    memset(&lqi_ctrl, 0, sizeof(lqi_ctrl));

    /* K_lqi 与速度无关，直接拷贝单矩阵 */
    memcpy(lqi_ctrl.K_lqi, lqi_K, sizeof(lqi_K));

    lqi_ctrl.cached_V = LQI_V_REF;  /* 初始默认 V_ref */
    lqi_ctrl.state_valid = 1;
}

/*============================================================================
 *  桥接：Euler_LQI_Cale
 *  从 Surface 全局取姿态/角速度 → 组 LQI 状态 → LQI_Update → 零空间分配 → 写舵面
 *============================================================================*/

void Euler_LQI_Cale(float dt)
{
    /* ---- 0) NaN/Inf 安全兜底 ---- */
    {
        float att_check = Surface.current_angle_Euler[NOW][ROLL]
                        + Surface.current_angle_Euler[NOW][PITCH]
                        + Surface.current_angle_Euler[NOW][YAW]
                        + Surface.target_angle_Euler[NOW][ROLL]
                        + Surface.target_angle_Euler[NOW][PITCH]
                        + Surface.target_angle_Euler[NOW][YAW]
                        + Surface.current_gyro_Euler[NOW][ROLL]
                        + Surface.current_gyro_Euler[NOW][PITCH]
                        + Surface.current_gyro_Euler[NOW][YAW]
                        + lqi_ctrl.cached_V;
        if (!LQI_IS_FINITE(att_check))
        {
            /* 输入异常：冻结积分、舵面回中、标记无效 */
            lqi_ctrl.freeze_integrator[0] = 1;
            lqi_ctrl.freeze_integrator[1] = 1;
            lqi_ctrl.freeze_integrator[2] = 1;
            lqi_ctrl.state_valid = 0;
            for (uint8_t i = 0; i < LQI_SERVO_COUNT; i++)
                Surface.output_angle_Servo[NOW][i] = 0.0f;
            return;
        }
        lqi_ctrl.state_valid = 1;
    }

    /* ---- 1) 姿态误差：测量 − 目标 ---- */
    lqi_ctrl.attitude_error_rad[0] = DEG2RAD(Surface.current_angle_Euler[NOW][ROLL]
                                           - Surface.target_angle_Euler[NOW][ROLL]);
    // lqi_ctrl.attitude_error_rad[1] = DEG2RAD(Surface.current_angle_Euler[NOW][PITCH]
    //                                        - Surface.target_angle_Euler[NOW][PITCH]);
    lqi_ctrl.attitude_error_rad[1] = 0;
    lqi_ctrl.attitude_error_rad[2] = DEG2RAD(Surface.current_angle_Euler[NOW][YAW]
                                           - Surface.target_angle_Euler[NOW][YAW]);

    /* ---- 2) 机体角速度 (rad/s) ---- */
    lqi_ctrl.body_rate_rad_s[0] = DEG2RAD(Surface.current_gyro_Euler[NOW][ROLL]);
    lqi_ctrl.body_rate_rad_s[1] = DEG2RAD(Surface.current_gyro_Euler[NOW][PITCH]);
    lqi_ctrl.body_rate_rad_s[2] = DEG2RAD(Surface.current_gyro_Euler[NOW][YAW]);
    // if (IMU_Data.Euler[NOW][PITCH]<=10.0)
    // {
    //     lqi_ctrl.body_rate_rad_s[1] = 0;                                                                    
    // }
    /* ---- 3) Pitch 门控：非 Terminal 段不追 Pitch 角度，但保留角速度阻尼 ---- */
    if (Guidance_State < Terminal)
    {
        // lqi_ctrl.attitude_error_rad[2] = 0.0f;   /* 不追     YAW 角度 */
        lqi_ctrl.integral_error[2]     = 0.0f;   /* 强制清零 YAW 积分 */
        lqi_ctrl.freeze_integrator[2]  = 1;      /* 永久冻结 YAW 积分 */
    }

        // lqi_ctrl.attitude_error_rad[0] = 0.0f;   /* 不追     ROLL 角度 */
        lqi_ctrl.integral_error[0]     = 0.0f;   /* 强制清零 ROLL 积分 */
        lqi_ctrl.freeze_integrator[0]  = 1;      /* 永久冻结 ROLL 积分 */

        lqi_ctrl.integral_error[1]     = 0.0f;   /* 强制清零 Pitch 积分 */
        lqi_ctrl.freeze_integrator[1]  = 1;      /* 永久冻结 Pitch 积分 */
    /* ---- 4) LQI 解算力矩 ---- */
    LQI_Update(dt);

    /* ---- 5) 计算 H_tau(V_DART_Lqi) = (V_DART_Lqi/V_ref)² * H_tau_Vref ---- */
    float V_DART_Lqi  = lqi_ctrl.cached_V;
    // float Vs = (V_DART_Lqi / LQI_V_REF) * (V_DART_Lqi / LQI_V_REF);   /* (V_DART_Lqi/V_ref)² */
    float Vs = 6;
    float H_tau[3][4];
    for (uint8_t row = 0; row < 3; row++)
        for (uint8_t col = 0; col < 4; col++)
            H_tau[row][col] = Vs * lqi_H_tau_Vref[row][col];

    /* ---- 6) 力矩→舵面分配 ---- */
    uint8_t sat_mask, infeasible;
    if (lqi_alloc_mode == 0)
    {
        /* 简单伪逆：pinv(H_tau) × tau → 三轴全满足，最小舵量（等价旧 G 矩阵分配） */
        Torque_Allocate_Simple(
            lqi_ctrl.torque_cmd_Nm,
            H_tau,
            LQI_DELTA_MAX_RAD,
            lqi_ctrl.servo_cmd_deg,
            lqi_ctrl.torque_achieved_Nm,
            &sat_mask,
            &infeasible);
        lqi_ctrl.pitch_moment_Nm = 0.0f;  /* Simple 模式不单独算 Pitch */
    }
    else
    {
        /* 零空间 Pitch 保护：先满足 Roll+Yaw，零空间压低 Pitch */
        Torque_Allocate_PitchProtected(
            lqi_ctrl.torque_cmd_Nm,
            H_tau,
            lqi_N_ry,
            H_tau[LQI_PITCH_ROW],
            LQI_DELTA_MAX_RAD,
            LQI_LAMBDA_PITCH,
            LQI_LAMBDA_SERVO,
            lqi_ctrl.servo_cmd_deg,
            lqi_ctrl.torque_achieved_Nm,
            &lqi_ctrl.pitch_moment_Nm,
            &sat_mask,
            &infeasible);
    }

    lqi_ctrl.servo_sat_mask       = sat_mask;
    lqi_ctrl.allocator_infeasible = infeasible;

    lqi_ctrl.torque_error_Nm[0] = lqi_ctrl.torque_cmd_Nm[0] - lqi_ctrl.torque_achieved_Nm[0];
    lqi_ctrl.torque_error_Nm[1] = lqi_ctrl.torque_cmd_Nm[1] - lqi_ctrl.torque_achieved_Nm[1];
    lqi_ctrl.torque_error_Nm[2] = lqi_ctrl.torque_cmd_Nm[2] - lqi_ctrl.torque_achieved_Nm[2];

    /* ---- 7) 写舵面输出 ---- */
    for (uint8_t i = 0; i < LQI_SERVO_COUNT; i++)
        Surface.output_angle_Servo[NOW][i] = lqi_ctrl.servo_cmd_deg[i];

    /* ---- 8) 积分管理：积分分离 + 逐轴抗饱和 ---- */
    /* 积分分离：大误差 → 冻结积分（P/D 主导）；小误差 → 开启积分（消静差） */
    for (uint8_t i = 0; i < 3; i++)
    {
        if (fabsf(lqi_ctrl.attitude_error_rad[i]) >= LQI_INTEG_THRESHOLD_RAD)
            lqi_ctrl.freeze_integrator[i] = 1;
        else
            lqi_ctrl.freeze_integrator[i] = 0;
    }

    /*  积分冻结兜底 */
    if (Guidance_State < Terminal)
    {
        lqi_ctrl.freeze_integrator[0] = 1;
        lqi_ctrl.freeze_integrator[1] = 1;
        lqi_ctrl.freeze_integrator[2] = 1;
    }

    // /* 逐轴抗饱和：力矩误差大 → 该轴积分冻结（替代全轴一刀切） */
    // {
    //     static const float torque_error_thresh_Nm[3] = {0.02f, 0.02f, 0.02f};
    //     for (uint8_t i = 0; i < 3; i++)
    //     {
    //         if (fabsf(lqi_ctrl.torque_error_Nm[i]) > torque_error_thresh_Nm[i])
    //             lqi_ctrl.freeze_integrator[i] = 1;
    //     }
    // }

    /* 全局不可达兜底：分配器完全失败 → 全轴冻结 */
    if (infeasible)
    {
        lqi_ctrl.freeze_integrator[0] = 1;
        lqi_ctrl.freeze_integrator[1] = 1;
        lqi_ctrl.freeze_integrator[2] = 1;
    }
}
