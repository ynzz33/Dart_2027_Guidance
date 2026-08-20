/**
 * @file    lqi_torque.c
 * @brief   飞镖 LQI 力矩控制器实现（9态→3轴力矩）
 * @details 控制律: tau_cmd = -K_lqi * xa
 *
 *          ★ K_lqi 与速度无关（B_tau = I⁻¹ 不含气动），存单矩阵。
 *          ★ H_tau = Vs * H_tau_Vref；Vs 固定 = 6.0f（2026-08-11：EKF 速度幅度不准，
 *            暂不喂动压调度，Vs 作舵效标定系数，待台架；动态 Vs 方案见 Euler_LQI_Cale #if 0）。
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
#include "CallBack_Task.h"

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
void  LQI_Update(float dt)
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
            float tau_angle  = 0.0f; /* 角度误差贡献，限幅前 */
            float tau_rate   = 0.0f; /* 角速度贡献，限幅前 */
            float tau_I_raw  = 0.0f; /* 积分贡献，限幅前 */
            float tau_I_used = 0.0f; /* 积分贡献，限幅后 */
            for (uint8_t j = 0; j < 3; j++)
            {
                tau_angle += -K[i][j]     * lqi_ctrl.xa[j];
                tau_rate  += -K[i][j + 3] * lqi_ctrl.xa[j + 3];
                tau_I_raw += -K[i][j + 6] * lqi_ctrl.xa[j + 6];
            }
            /* 积分部分限幅 */
            float lim = integ_torque_limits[i];
            tau_I_used = tau_I_raw;
            if (tau_I_used >  lim) tau_I_used =  lim;
            if (tau_I_used < -lim) tau_I_used = -lim;

            /* 保存实际三类力矩贡献；积分项同时保留限幅前/后的值 */
            lqi_ctrl.torque_integral_raw_Nm[i] = tau_I_raw  * LQI_GAIN_SCALAR;
            lqi_ctrl.torque_angle_Nm[i]        = tau_angle  * LQI_GAIN_SCALAR;
            lqi_ctrl.torque_rate_Nm[i]         = tau_rate   * LQI_GAIN_SCALAR;
            lqi_ctrl.torque_integral_Nm[i]     = tau_I_used * LQI_GAIN_SCALAR;
            lqi_ctrl.torque_cmd_Nm[i] =
                lqi_ctrl.torque_angle_Nm[i]
              + lqi_ctrl.torque_rate_Nm[i]
              + lqi_ctrl.torque_integral_Nm[i];
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
        /* (2026-08-11 删除了无效语句 RAD2DEG(integral_error[2])：宏调用不赋值、无效果) */
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

    /* ---- 1) 姿态误差：测量 − 目标 ----
     * PITCH 误差恒置 0（方案 A，2026-08-11）：Pitch 依托镖架初始动力、不追踪目标角度，
     * 仅保留角速度阻尼（见 2)）。这样 pitch 通道只做阻尼、不主动改俯仰姿态，
     * 把舵面资源让给 yaw/roll。roll 自稳、yaw 视觉制导正常参与。 */
    lqi_ctrl.attitude_error_rad[0] = DEG2RAD(Surface.current_angle_Euler[NOW][ROLL]
                                           - Surface.target_angle_Euler[NOW][ROLL]);
    lqi_ctrl.attitude_error_rad[1] = 0.0f;   /* PITCH 不追角度（依托初始动力） */
    lqi_ctrl.attitude_error_rad[2] = DEG2RAD(Surface.current_angle_Euler[NOW][YAW]
                                           - Surface.target_angle_Euler[NOW][YAW]);

    /* ---- 2) 机体角速度 (rad/s) ---- */
    lqi_ctrl.body_rate_rad_s[0] = DEG2RAD(Surface.current_gyro_Euler[NOW][ROLL]);
    lqi_ctrl.body_rate_rad_s[1] = DEG2RAD(Surface.current_gyro_Euler[NOW][PITCH]);
    lqi_ctrl.body_rate_rad_s[2] = DEG2RAD(Surface.current_gyro_Euler[NOW][YAW]);
    /* ⚠ 2026-08-11 删除调试残留：原 PITCH≤0 时 `q /= cnt` 把 pitch 角速度阻尼逐拍除到 1/1000，
     * 俯冲段 pitch 等于无阻尼裸奔（发散根因）。现 pitch 阻尼全量保留（方案 A 的"保留阻尼"部分）。 */

    /* ---- 3) 丢目标时削弱 YAW 误差（防丢目标瞬间猛打；原注释误写"Pitch 门控"，修正为真实 YAW 削弱） ---- */
    if (Guidance_State < Terminal && Vision_Rx_Data.Vision_recognize_flag==RECOGNIZE_FAILURE)
    {
        lqi_ctrl.attitude_error_rad[2] *= 0.1f;
    }

    /* ---- 3.5) 积分门控（2026-08-11 重写）：ROLL/PITCH 恒不积分，仅 YAW 在 Terminal 段放行积分。
     * 原 227-234 每拍"强制清零 + 冻结全部积分"把积分通道整个短路（"误差积不上去"根因）。
     * 现：roll/pitch 恒清零冻结；yaw 非 Terminal 段清零冻结、Terminal 段放行，由 LQI_Update 内
     * 的积分分离（阈值 0.5°）与限幅（5°·s）接管。 */
    lqi_ctrl.integral_error[0]    = 0.0f;
    lqi_ctrl.freeze_integrator[0] = 1;                 /* ROLL 恒不积分 */
    lqi_ctrl.integral_error[1]    = 0.0f;
    lqi_ctrl.freeze_integrator[1] = 1;                 /* PITCH 恒不积分 */
    if (Guidance_State < Terminal)
    {
        lqi_ctrl.integral_error[2]    = 0.0f;
        lqi_ctrl.freeze_integrator[2] = 1;             /* 未入制导段：YAW 不积分 */
    }
    else
    {
        lqi_ctrl.freeze_integrator[2] = 0;             /* Terminal 段：YAW 积分放行（消静差） */
    }

    /* ---- 4) LQI 解算力矩 ---- */
    LQI_Update(dt);

    /* ---- 5) 计算 H_tau = Vs * H_tau_Vref（动压缩放；H_tau 是 力矩→舵角 换算矩阵） ----
     * 理想：Vs = (V_DART_Lqi / V_ref)² —— 气动力矩 ∝ 动压 q = ½ρV²。
     * 现状(2026-08-11)：EKF 速度不准（视觉 dist_cm 由 blob 像素反算、极粗 → V 抖），
     *  直接喂 V² 会让 H_tau/舵角抖；故固定 Vs=6 换取稳定（当前调试基线）。
     * ⚠ 注意：Vs=6 ≠ "速度 6"。真按 V=V_NOM=6 的动压缩放应为 (6/6)² = 1，
     *  此处 =6 是把 H_tau 整体放大 6 倍（≈等效舵效/动压的台架标定系数），
     *  若 V 真≈6，则舵角会被 pinv(H_tau) 缩小 6 倍——台架务必验证舵效量级。
     * 根治方向(见 PROGRESS TODO)：① EKF 速度先低通(~0.4s)再平方；
     *  ② 改"初速 V_NOM + 气动阻力衰减"弹道模型 V(t)；③ 台架标真实 V 衰减曲线。 */
    float Vs;
#if 0  /* 实时动压平方调度（EKF 速度验证准后改 1 启用） */
    {
        float v_cur = lqi_ctrl.cached_V;
        if (v_cur < LQI_V_MIN) v_cur = LQI_V_MIN;
        if (v_cur > LQI_V_MAX) v_cur = LQI_V_MAX;
        Vs = (v_cur / LQI_V_REF) * (v_cur / LQI_V_REF);
    }
#else  /* 固定动压（当前调试基线：H_tau 放大 6 倍，V 不再参与） */
    Vs = 6.0f;
#endif
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

    /* ---- 8) 积分兜底（2026-08-11 精简）：积分门控已在上面 §3.5 统一设置
     *    （ROLL/PITCH 恒冻结、YAW 按状态机放行；积分分离/限幅由 LQI_Update 内部处理）。
     *    这里只保留"分配器全局不可达 → 全轴冻结积分"防 windup 兜底。 */
    if (infeasible)
    {
        lqi_ctrl.freeze_integrator[0] = 1;
        lqi_ctrl.freeze_integrator[1] = 1;
        lqi_ctrl.freeze_integrator[2] = 1;
    }
}
