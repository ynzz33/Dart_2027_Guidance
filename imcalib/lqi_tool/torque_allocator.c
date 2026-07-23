/**
 * @file    torque_allocator.c
 * @brief   飞镖 Pitch 保护型零空间舵面分配器实现
 * @details 核心算法：
 *
 *          第 1 步：解 Roll/Yaw 子问题
 *            H_ry = H_tau 的 roll+yaw 行 (2×4)
 *            tau_ry = [Mx_cmd; Mz_cmd]
 *            delta0 = pinv(H_ry) * tau_ry   （最小范数解）
 *
 *          第 2 步：零空间优化（2×2 解析求解）
 *            delta = delta0 + N_ry * eta
 *            优化: J = λ_p·(h_pitch·δ)² + λ_s·(δ'·δ)
 *            → 解析化为 2×2 对称正定线性系统 H·eta = -g
 *            → 使用 Cramer 法则直接求解，避免调用 CMSIS-DSP
 *
 *          第 3 步：限幅 + 统一缩放
 *          第 4 步：回算实际力矩 + 设置标志
 *
 *          ⚠ 未编译。待台架。
 *
 * @author  ynz (AI 移植)
 * @date    2026/07/23
 */

#include "torque_allocator.h"
#include "../User/common_defs.h"   /* RAD2DEG */
#include <math.h>                  /* fabsf */
#include <string.h>                /* memset */

/*============================================================================
 *  内部辅助：2×2 矩阵求逆（Cramer 法则）
 *    A = [a, b; c, d],  det = a*d - b*c
 *    inv(A) = 1/det * [d, -b; -c, a]
 *  返回 0 = 成功, 非 0 = 奇异
 *============================================================================*/
static int invert_2x2(const float A[4], float invA[4])
{
    float det = A[0] * A[3] - A[1] * A[2];   /* a*d - b*c */
    float abs_det = fabsf(det);

    /* 奇异阈值：行列式过小视为不可逆 */
    if (abs_det < 1e-12f)
        return -1;

    float inv_det = 1.0f / det;
    invA[0] =  A[3] * inv_det;    /*  d / det */
    invA[1] = -A[1] * inv_det;    /* -b / det */
    invA[2] = -A[2] * inv_det;    /* -c / det */
    invA[3] =  A[0] * inv_det;    /*  a / det */
    return 0;
}

/*============================================================================
 *  内部辅助：2×4 伪逆
 *    输入:  H (2×4, 按行存储: row0[4], row1[4])
 *    输出:  pinvH (4×2, 按列存储: col0[4], col1[4])
 *
 *    pinv(H) = H' * inv(H * H')
 *    H * H' 是 2×2 对称矩阵，直接算：
 *      HHt[0] = r0²+r1²+r2²+r3²
 *      HHt[1] = HHt[2] = r0*y0+r1*y1+r2*y2+r3*y3
 *      HHt[3] = y0²+y1²+y2²+y3²
 *============================================================================*/
static int pinv_2x4(const float H_row0[4], const float H_row1[4], float pinv_col0[4], float pinv_col1[4])
{
    /* H * H' = [a, b; b, c] (2×2 对称) */
    float a = H_row0[0]*H_row0[0] + H_row0[1]*H_row0[1] + H_row0[2]*H_row0[2] + H_row0[3]*H_row0[3];
    float b = H_row0[0]*H_row1[0] + H_row0[1]*H_row1[1] + H_row0[2]*H_row1[2] + H_row0[3]*H_row1[3];
    float c = H_row1[0]*H_row1[0] + H_row1[1]*H_row1[1] + H_row1[2]*H_row1[2] + H_row1[3]*H_row1[3];

    float HHt[4] = {a, b, b, c};
    float invHHt[4];
    if (invert_2x2(HHt, invHHt) != 0)
        return -1;   /* 奇异 */

    /* pinv = H' * inv(H*H') */
    for (uint8_t i = 0; i < 4; i++)
    {
        /* H'[i,:] = [H_row0[i], H_row1[i]] */
        /* pinv[i,k] = sum_j H'[i,j] * invHHt[j,k] */
        pinv_col0[i] = H_row0[i] * invHHt[0] + H_row1[i] * invHHt[1];
        pinv_col1[i] = H_row0[i] * invHHt[2] + H_row1[i] * invHHt[3];
    }
    return 0;
}

/*============================================================================
 *  内部辅助：4 维向量点积
 *============================================================================*/
static float dot4(const float a[4], const float b[4])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

/*============================================================================
 *  Pitch 保护型零空间分配
 *============================================================================*/
void Torque_Allocate_PitchProtected(
    const float torque_cmd_Nm[3],
    const float H_tau[3][4],
    const float N_ry[4][2],
    const float h_pitch[4],
    float delta_max_rad,
    float lambda_pitch,
    float lambda_servo,
    float servo_cmd_deg[4],
    float torque_achieved[3],
    float *pitch_moment,
    uint8_t *sat_mask,
    uint8_t *infeasible)
{
    float delta[4];
    *sat_mask = 0;
    *infeasible = 0;

    /* ---- Step 1: 提取 Roll/Yaw 子矩阵和指令 ---- */
    /* H_ry: H_tau 的第 0(roll) 行和第 2(yaw) 行 */
    const float *H_roll = H_tau[0];    /* roll 行 */
    const float *H_yaw  = H_tau[2];    /* yaw 行 */
    float tau_ry[2] = {torque_cmd_Nm[0], torque_cmd_Nm[2]};  /* [Mx_cmd, Mz_cmd] */

    /* ---- Step 2: delta0 = pinv(H_ry) * tau_ry ---- */
    float pinv_col0[4], pinv_col1[4];
    if (pinv_2x4(H_roll, H_yaw, pinv_col0, pinv_col1) != 0)
    {
        /* 奇异：回退到舵面回中 */
        *infeasible = 1;
        for (uint8_t i = 0; i < 4; i++) delta[i] = 0.0f;
        goto clamp_and_exit;
    }

    float delta0[4];
    for (uint8_t i = 0; i < 4; i++)
        delta0[i] = pinv_col0[i] * tau_ry[0] + pinv_col1[i] * tau_ry[1];

    /* ---- Step 3: 零空间优化 ---- */
    /* a = h_pitch * N_ry  (1×2), a[k] = Σ_i h_pitch[i] * N_ry[i][k] */
    float a[2];
    a[0] = 0.0f; a[1] = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        a[0] += h_pitch[i] * N_ry[i][0];
        a[1] += h_pitch[i] * N_ry[i][1];
    }

    /* b = h_pitch * delta0 (scalar) */
    float b = dot4(h_pitch, delta0);

    /* Hessian H = λ_p*(a'*a) + λ_s*(N_ry'*N_ry)  (2×2) */
    /* a'*a = [a0², a0*a1; a0*a1, a1²] */
    float H_hess[4];
    H_hess[0] = lambda_pitch * a[0] * a[0];
    H_hess[1] = lambda_pitch * a[0] * a[1];
    H_hess[2] = H_hess[1];   /* 对称 */
    H_hess[3] = lambda_pitch * a[1] * a[1];

    /* N_ry'*N_ry = sum over rows: N_ry[i,:]' * N_ry[i,:] */
    /* = [Σn_i0²,  Σn_i0*n_i1; Σn_i0*n_i1, Σn_i1²] */
    float NtN[4] = {0};
    for (uint8_t i = 0; i < 4; i++)
    {
        NtN[0] += N_ry[i][0] * N_ry[i][0];
        NtN[1] += N_ry[i][0] * N_ry[i][1];
        NtN[3] += N_ry[i][1] * N_ry[i][1];
    }
    NtN[2] = NtN[1];   /* 对称 */

    H_hess[0] += lambda_servo * NtN[0];
    H_hess[1] += lambda_servo * NtN[1];
    H_hess[2] += lambda_servo * NtN[2];
    H_hess[3] += lambda_servo * NtN[3];

    /* RHS: g = -(λ_p*b*a' + λ_s*N_ry'*delta0)  (2×1) */
    float g[2];
    g[0] = -(lambda_pitch * b * a[0]);
    g[1] = -(lambda_pitch * b * a[1]);

    /* N_ry' * delta0: (2×4) × (4×1) = 2×1 */
    for (uint8_t k = 0; k < 2; k++)
    {
        float ntd = 0.0f;
        for (uint8_t i = 0; i < 4; i++)
            ntd += N_ry[i][k] * delta0[i];
        g[k] -= lambda_servo * ntd;
    }

    /* 解 H_hess * eta = g (2×2 Cramer 法则) */
    float invH[4];
    float eta[2];
    if (invert_2x2(H_hess, invH) == 0)
    {
        eta[0] = invH[0] * g[0] + invH[1] * g[1];
        eta[1] = invH[2] * g[0] + invH[3] * g[1];
    }
    else
    {
        /* 病态 Hessian：不优化，eta = 0 */
        eta[0] = 0.0f;
        eta[1] = 0.0f;
    }

    /* ---- Step 4: delta = delta0 + N_ry * eta ---- */
    for (uint8_t i = 0; i < 4; i++)
        delta[i] = delta0[i] + N_ry[i][0] * eta[0] + N_ry[i][1] * eta[1];

    /* ---- Step 5: 限幅 + 统一缩放 ---- */
clamp_and_exit:
    {
        /* 找最大舵偏 */
        float max_abs = 0.0f;
        for (uint8_t i = 0; i < 4; i++)
        {
            float abs_d = fabsf(delta[i]);
            if (abs_d > max_abs) max_abs = abs_d;
        }

        if (max_abs > delta_max_rad)
        {
            *infeasible = 1;
            float scale = delta_max_rad / max_abs;
            for (uint8_t i = 0; i < 4; i++)
                delta[i] *= scale;
        }

        /* 检测各片饱和 */
        for (uint8_t i = 0; i < 4; i++)
        {
            if (fabsf(delta[i]) >= delta_max_rad - 1e-6f)
                *sat_mask |= (1u << i);
        }
    }

    /* ---- Step 6: 回算实际力矩 ---- */
    for (uint8_t row = 0; row < 3; row++)
    {
        float tau = 0.0f;
        for (uint8_t col = 0; col < 4; col++)
            tau += H_tau[row][col] * delta[col];
        torque_achieved[row] = tau;
    }

    /* Pitch 实际力矩 */
    *pitch_moment = dot4(h_pitch, delta);

    /* ---- Step 7: rad → deg 输出 ---- */
    for (uint8_t i = 0; i < 4; i++)
    {
        float deg = RAD2DEG(delta[i]);
        /* 安全兜底：二次限幅 */
        float limit_deg = RAD2DEG(delta_max_rad);
        if (deg >  limit_deg) deg =  limit_deg;
        if (deg < -limit_deg) deg = -limit_deg;
        servo_cmd_deg[i] = deg;
    }
}

/*============================================================================
 *  简单伪逆分配器（三轴全满足，最小舵量，无零空间优化）
 *  delta = pinv(H_tau) × tau_cmd
 *        = H_tau' × inv(H_tau×H_tau') × tau_cmd
 *
 *  与 Torque_Allocate_PitchProtected 的区别：
 *    - 本函数直接 pinv 整个 3×4 的 H_tau，三轴力矩都满足
 *    - PitchProtected 只 pinv Roll+Yaw (2×4)，Pitch 在零空间优化
 *    - 本函数等价于旧 LQR 的 "G 矩阵最小范数分配"
 *============================================================================*/
void Torque_Allocate_Simple(
    const float torque_cmd_Nm[3],
    const float H_tau[3][4],
    float delta_max_rad,
    float servo_cmd_deg[4],
    float torque_achieved[3],
    uint8_t *sat_mask,
    uint8_t *infeasible)
{
    float delta[4];
    *sat_mask = 0;
    *infeasible = 0;

    /* ---- 1) S = H_tau × H_tau' (3×3 对称) ---- */
    float S[9];  /* row-major: S[row*3+col] */
    for (uint8_t r = 0; r < 3; r++)
    {
        for (uint8_t c = r; c < 3; c++)
        {
            float sum = 0.0f;
            for (uint8_t j = 0; j < 4; j++)
                sum += H_tau[r][j] * H_tau[c][j];
            S[r * 3 + c] = sum;
            S[c * 3 + r] = sum;   /* 对称 */
        }
    }

    /* ---- 2) 3×3 行列式 (Cramer 法则) ---- */
    float det = S[0] * (S[4] * S[8] - S[5] * S[7])
              - S[1] * (S[3] * S[8] - S[5] * S[6])
              + S[2] * (S[3] * S[7] - S[4] * S[6]);

    if (fabsf(det) < 1e-12f)
    {
        *infeasible = 1;
        for (uint8_t i = 0; i < 4; i++) delta[i] = 0.0f;
        goto clamp_and_exit_simple;
    }

    {
        float inv_det = 1.0f / det;

        /* inv(S) 上三角（对称矩阵的逆也对称） */
        float iS00 =  (S[4] * S[8] - S[5] * S[7]) * inv_det;
        float iS01 = -(S[1] * S[8] - S[2] * S[7]) * inv_det;  /* = -(S[3]*S[8]-S[5]*S[6])? 不对... */

        /* 辅助余子式 */
        float iS01_ = (S[5] * S[6] - S[3] * S[8]) * inv_det;  /* = invS[0][1] */
        float iS02_ = (S[3] * S[7] - S[4] * S[6]) * inv_det;
        float iS12_ = (S[6] * S[1] - S[0] * S[7]) * inv_det;  /* wait, let me be more careful */

        /* 3×3 伴随矩阵 / det，逐元素 */
        /* invS[r][c] = C[c][r] / det where C[c][r] = (-1)^(c+r) * minor(S, c, r) */
        /* 因为 inv(S) = adj(S) / det = C' / det */

        /* 余子式矩阵 C（未转置） */
        float C00 =  (S[4] * S[8] - S[5] * S[7]);
        float C01 = -(S[3] * S[8] - S[5] * S[6]);
        float C02 =  (S[3] * S[7] - S[4] * S[6]);
        float C10 = -(S[1] * S[8] - S[2] * S[7]);
        float C11 =  (S[0] * S[8] - S[2] * S[6]);
        float C12 = -(S[0] * S[7] - S[1] * S[6]);
        float C20 =  (S[1] * S[5] - S[2] * S[4]);
        float C21 = -(S[0] * S[5] - S[2] * S[3]);
        float C22 =  (S[0] * S[4] - S[1] * S[3]);

        /* inv(S) = C' / det（C 转置再除以 det） */
        float iS0[3] = { C00 * inv_det, C10 * inv_det, C20 * inv_det };
        float iS1[3] = { C01 * inv_det, C11 * inv_det, C21 * inv_det };
        float iS2[3] = { C02 * inv_det, C12 * inv_det, C22 * inv_det };

        /* ---- 3) x = inv(S) × tau (3×1) ---- */
        float x0 = iS0[0] * torque_cmd_Nm[0] + iS0[1] * torque_cmd_Nm[1] + iS0[2] * torque_cmd_Nm[2];
        float x1 = iS1[0] * torque_cmd_Nm[0] + iS1[1] * torque_cmd_Nm[1] + iS1[2] * torque_cmd_Nm[2];
        float x2 = iS2[0] * torque_cmd_Nm[0] + iS2[1] * torque_cmd_Nm[1] + iS2[2] * torque_cmd_Nm[2];

        /* ---- 4) delta = H_tau' × x (4×1) ---- */
        for (uint8_t i = 0; i < 4; i++)
            delta[i] = H_tau[0][i] * x0 + H_tau[1][i] * x1 + H_tau[2][i] * x2;
    }

    /* ---- 5) 限幅 + 统一缩放 ---- */
clamp_and_exit_simple:
    {
        float max_abs = 0.0f;
        for (uint8_t i = 0; i < 4; i++)
        {
            float abs_d = fabsf(delta[i]);
            if (abs_d > max_abs) max_abs = abs_d;
        }

        if (max_abs > delta_max_rad)
        {
            *infeasible = 1;
            float scale = delta_max_rad / max_abs;
            for (uint8_t i = 0; i < 4; i++)
                delta[i] *= scale;
        }

        for (uint8_t i = 0; i < 4; i++)
        {
            if (fabsf(delta[i]) >= delta_max_rad - 1e-6f)
                *sat_mask |= (1u << i);
        }
    }

    /* ---- 6) 回算实际力矩 ---- */
    for (uint8_t row = 0; row < 3; row++)
    {
        float tau_sum = 0.0f;
        for (uint8_t col = 0; col < 4; col++)
            tau_sum += H_tau[row][col] * delta[col];
        torque_achieved[row] = tau_sum;
    }

    /* ---- 7) rad → deg ---- */
    for (uint8_t i = 0; i < 4; i++)
    {
        float deg = RAD2DEG(delta[i]);
        float limit_deg = RAD2DEG(delta_max_rad);
        if (deg >  limit_deg) deg =  limit_deg;
        if (deg < -limit_deg) deg = -limit_deg;
        servo_cmd_deg[i] = deg;
    }
}
