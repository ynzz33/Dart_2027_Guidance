//
// vision_ekf.c —— 6态 bearing-only 非线性EKF(位置/速度)
//
// 姿态从 Mahony 的 IMU_Data.R_matrix_T 读取,不自己估计姿态误差。
// 加速度零偏靠第一阶段标定(A_Offset),不在滤波器内估计。
// 量测:方位角+俯仰角(不需要距离)。
// 位置定义:nom_p = 镖−靶(靶为原点,镖的位置),与旧 vision_ins.c 一致。
// 注意:range_m 和 vc 是名义推导量,仅作参考,不能用于控制决策。
//
#include "vision_ekf.h"
#include "pid.h"
#include "IMU.h"
#include <math.h>
#include <string.h>

/* ===== 名义状态(位置/速度;姿态从 IMU_Data 读) ===== */
static float nom_p[3];
static float nom_v[3];

/* ===== 误差状态与协方差(6维:dp,dv) ===== */
static float dx[6];
static float P[6][6];

/* ===== 对外输出 ===== */
EkfOut_t ekf_out = {0};

/* ===== 内部标志 ===== */
static uint8_t pos_inited = 0;  /* 首帧位置初始化完成标志 */
static EkfMat M;

static void publish(void)
{
    int state_finite;
    float v_out, r2;

    /* NaN/Inf 兜底:内部状态异常时输出安全值,不掩盖(ekf_out.locked 标记供上层检查) */
    state_finite = 1;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(nom_p[i]) || !isfinite(nom_v[i])) { state_finite = 0; break; }
    }
    if (!state_finite) {
        for (int i = 0; i < 3; i++) { ekf_out.p_world[i] = 0; ekf_out.v_world[i] = 0; }
        ekf_out.range_m = 0; ekf_out.vc = 0;
        return;
    }
    for (int i = 0; i < 3; i++) {
        ekf_out.p_world[i] = nom_p[i];
        /* 仅输出限幅,不修改内部 nom_v:保持滤波器数学连续性 */
        v_out = nom_v[i];
        abs_limit(&v_out, VEL_MAX_MS);
        ekf_out.v_world[i] = v_out;
    }
    r2 = nom_p[0]*nom_p[0]+nom_p[1]*nom_p[1]+nom_p[2]*nom_p[2];
    ekf_out.range_m = sqrtf(r2);
    ekf_out.vc = (ekf_out.range_m > 0.01f)
            ? -(nom_p[0]*nom_v[0]+nom_p[1]*nom_v[1]+nom_p[2]*nom_v[2])/ekf_out.range_m
            : 0.0f;
}


void EKF_Init(void)
{
    memset(nom_p, 0, sizeof(nom_p));
    memset(nom_v, 0, sizeof(nom_v));
    memset(dx, 0, sizeof(dx));
    memset(P, 0, sizeof(P));
    for (int i = 0; i < 3; i++) {
        P[i][i]     = EKF_P0_POS;      /* dp */
        P[3+i][3+i] = EKF_P0_VEL_VAR;      /* dv */
    }
    ekf_out.locked = 0;
    pos_inited = 0;
    /* 矩阵运算缓冲 mat_init (只需一次) */
    mat_init(&M.Pm,      6, 6, (float*)P);
    mat_init(&M.F,       6, 6, M._f);
    mat_init(&M.Q,       6, 6, M._q);
    mat_init(&M.FP,      6, 6, M._fp);
    mat_init(&M.HP,      2, 6, M._hp);
    mat_init(&M.Ht,      6, 2, M._ht);
    mat_init(&M.S2,      2, 2, M._s2);
    mat_init(&M.Sinv2,   2, 2, M._sinv2);
    mat_init(&M.PHt,     6, 2, M._ph);
    mat_init(&M.K2,      6, 2, M._k2);
    mat_init(&M.KS,      6, 2, M._ks);
    mat_init(&M.KSK,     6, 6, M._ksk);
    mat_init(&M.y6,      6, 1, M._y6);
    mat_init(&M.S3,      3, 3, M._s3);
    mat_init(&M.Sinv3,   3, 3, M._sinv3);
    mat_init(&M.P_pv,    3, 3, M._ppv);
    mat_init(&M.Kp,      3, 3, M._kp);
    mat_init(&M.Kv,      3, 3, M._kv);
    mat_init(&M.K63,     6, 3, M._k63);  /* K63 = [Kp;Kv] 组合增益 */
    mat_init(&M.d_state, 6, 1, M._d_state);
    mat_init(&M.KSK_z,   6, 6, M._ksk_z);
    publish();
}

/* 发射前复位:清零状态+协方差+位置初始化标志。
 * 当前为单次发射模型(飞镖上电→发射→结束),仅上电时 Init_Config 调用 Init 一次。
 * 若项目改为重复发射,需在每次新发射流程开始前调用此函数,
 * 并同步复位:旧 VisInsEKF(VisInsEKF_Init)、IMU.c 的 vins_last_recog/vel_Reanchor_Flag。*/
void EKF_Reset(void)
{
    EKF_Init();
}

uint8_t EKF_PosInited(void)
{
    return pos_inited;
}

/* 位置初始化:第一次识别目标时调用,用首帧视觉角度+先验距离构造初始位置。
 * az_rad/el_rad:首帧视觉方位角/俯仰角(rad)
 * range_prior:仅作为 bearing-only 的尺度参考,不是真实距离。
 * 位置定义:nom_p = 镖−靶。镖→靶方向 = −nom_p/range。*/
void EKF_InitPos(float az_rad, float el_rad, float range_prior)
{
    const float (*R)[3];
    float cb, u_body[3], u_world[3], Rt_buf[9], pos_var;
    mat R_m, Rt_m, ub_m, uw_m;

    R = IMU_Data.R_matrix_T;
    /* 根据视觉角度构造机体系镖→靶单位向量 */
    cb = cosf(el_rad);
    u_body[0] = sinf(az_rad)*cb; u_body[1] = cosf(az_rad)*cb; u_body[2] = sinf(el_rad);
    /* 转世界系:u_world = Rᵀ · u_body (机体系→世界系) */
    arm_mat_init_f32(&R_m,  3, 3, (float*)R);
    arm_mat_init_f32(&Rt_m, 3, 3, Rt_buf);
    mat_trans(&Rt_m, &R_m);              /* Rᵀ(世界←机体) */
    arm_mat_init_f32(&ub_m, 3, 1, u_body);
    arm_mat_init_f32(&uw_m, 3, 1, u_world);
    mat_mult(&uw_m, &Rt_m, &ub_m);      /* u_world = Rᵀ · u_body */
    /* nom_p = 镖−靶 = −range·u_world (镖→靶方向取负) */
    nom_p[0] = -range_prior * u_world[0];
    nom_p[1] = -range_prior * u_world[1];
    nom_p[2] = -range_prior * u_world[2];
    /* 位置不确定性:距离方向大(不可观),角度方向小(已知) */
    pos_var = (range_prior * 0.5f) * (range_prior * 0.5f);
    for (int i = 0; i < 3; i++) P[i][i] = pos_var;
    pos_inited = 1;
    publish();
}

/* ===== IMU预测(1kHz) =====
 * 误差状态6维: [dp(3), dv(3)]
 * F矩阵(6×6):
 *   dp' = dp + dv*dt
 *   dv' = dv
 * 姿态 R 从 IMU_Data.R_matrix_T 读取(Mahony 输出),固定使用。
 * acc_world = IMU_Data.A_World(已去重力、已去偏、世界系)。
 * 注意:不再重复扣除 A_Offset。*/
void EKF_Predict(const float acc_world[3], float dt)
{
    float Ft_buf[36], fp_buf[36];
    float s2, dt2, dt3, dt4;
    mat Ft_local, FP_local;

    if (dt <= 0.0f || dt > 0.1f) return;

    /* 名义状态传播:先用旧速度更新位置,再更新速度。
     * 不在内部硬限幅 nom_v:限幅会破坏 P 与状态的一致性(位置连续但速度跳变)。
     * 仅在 publish() 输出时限幅 ekf_out.v_world。*/
    for (int i = 0; i < 3; i++) {
        nom_p[i] += nom_v[i] * dt + 0.5f * acc_world[i] * dt * dt;
        nom_v[i] += acc_world[i] * dt;
    }

    /* --- P = F·P·Fᵀ + Q (完整 6×6 矩阵形式) ---
     * F = [[I, dt·I],[0, I]] */
    memset(M._f, 0, 36*sizeof(float));
    for (int i = 0; i < 3; i++) {
        M._f[i*6+i] = 1.0f;         /* 左上 I */
        M._f[i*6+i+3] = dt;         /* 右上 dt·I */
        M._f[(i+3)*6+i+3] = 1.0f;   /* 右下 I */
    }

    /* FP = F · P */
    mat_mult(&M.FP, &M.F, &M.Pm);

    /* FPFᵀ = FP · Fᵀ  (复用 FP 缓冲存结果) */
    arm_mat_init_f32(&Ft_local, 6, 6, Ft_buf);
    mat_trans(&Ft_local, &M.F);
    arm_mat_init_f32(&FP_local, 6, 6, fp_buf);
    memcpy(fp_buf, M._fp, 36*sizeof(float));
    mat_mult(&M.FP, &FP_local, &Ft_local);  /* FP = FPFᵀ */

    /* Q: 白噪声加速度模型 */
    s2  = EKF_SIGMA_ACC * EKF_SIGMA_ACC;
    dt2 = dt*dt; dt3 = dt2*dt; dt4 = dt3*dt;
    memset(M._q, 0, 36*sizeof(float));
    for (int i = 0; i < 3; i++) {
        M._q[i*6+i]         = s2*dt4*0.25f;  /* qpp */
        M._q[i*6+i+3]       = s2*dt3*0.5f;   /* qpv */
        M._q[(i+3)*6+i]     = s2*dt3*0.5f;   /* qvp */
        M._q[(i+3)*6+i+3]   = s2*dt2;        /* qvv */
    }

    /* P = FPFᵀ + Q */
    mat_add(&M.Pm, &M.FP, &M.Q);

    publish();
}

/* ===== 方位/俯仰量测更新(~30Hz) =====
 * 量测: az_rad, el_rad (从 Vision_Rx_Data.Euler 换算)
 * 姿态 R 固定为 Mahony 输出,不估计姿态误差。
 * H矩阵(2×6): 只有位置项 ∂z/∂dp = (∂z/∂u)·(−R)。
 * 位置定义:nom_p = 镖−靶(靶为原点)。*/
void EKF_UpdateBearing(float az_rad, float el_rad)
{
    /* --- 变量定义(全部在函数最前) --- */
    const float (*R)[3];
    float u_b[3], norm_xy, az_hat, el_hat, y[2];
    float u0, u1, u2, u_sq, u_xy_sq;
    float daz_du[3], inv_norm, del_du[3];
    float neg_R[3][3], Jac[2][3], H[2][6];
    float k2_t_buf[12], y_buf[2], Rm, chi2, s;
    mat Jac_m, negR_m, Hblk, H_local, y_col, K2_trans;

    R = IMU_Data.R_matrix_T;

    /* 机体系视线方向(镖→靶):u_b = R × (−nom_p)。R是世界→机体,−nom_p是世界系镖→靶方向 */
    for (int i = 0; i < 3; i++) {
        u_b[i] = 0.0f;
        for (int j = 0; j < 3; j++) u_b[i] += R[i][j] * (-nom_p[j]);
    }
    norm_xy = sqrtf(u_b[0]*u_b[0] + u_b[1]*u_b[1]);
    if (norm_xy < 1e-6f) return;

    /* 预测量测:az=atan2(水平横向,前向), el=atan2(竖直,水平投影) */
    az_hat = atan2f(u_b[0], u_b[1]);
    el_hat = atan2f(u_b[2], norm_xy);

    /* 新息(角度环绕) */
    y[0] = az_rad - az_hat;
    y[1] = el_rad - el_hat;
    if (y[0] >  M_PI) y[0] -= 2.0f*M_PI;
    if (y[0] < -M_PI) y[0] += 2.0f*M_PI;
    if (y[1] >  M_PI) y[1] -= 2.0f*M_PI;
    if (y[1] < -M_PI) y[1] += 2.0f*M_PI;

    /* --- H矩阵(2×6): 只有位置项 ∂z/∂dp --- */
    u0 = u_b[0]; u1 = u_b[1]; u2 = u_b[2];
    u_sq = u0*u0+u1*u1+u2*u2;
    u_xy_sq = u0*u0+u1*u1;
    if (u_sq < 1e-8f || u_xy_sq < 1e-8f) return;

    daz_du[0] = u1/u_xy_sq; daz_du[1] = -u0/u_xy_sq; daz_du[2] = 0.0f;
    inv_norm = 1.0f / (u_sq * norm_xy);
    del_du[0] = -u0*u2*inv_norm; del_du[1] = -u1*u2*inv_norm; del_du[2] = u_xy_sq*inv_norm;

    /* H = ∂z/∂u_b · (−R) (矩阵乘法,Jacobian 2×3 乘 −R 3×3) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            neg_R[i][j] = -R[i][j];
    Jac[0][0] = daz_du[0]; Jac[0][1] = daz_du[1]; Jac[0][2] = daz_du[2];
    Jac[1][0] = del_du[0]; Jac[1][1] = del_du[1]; Jac[1][2] = del_du[2];
    memset(H, 0, sizeof(H));
    arm_mat_init_f32(&Jac_m,  2, 3, (float*)Jac);
    arm_mat_init_f32(&negR_m, 3, 3, (float*)neg_R);
    arm_mat_init_f32(&Hblk,   2, 3, (float*)H);
    mat_mult(&Hblk, &Jac_m, &negR_m);              /* H[0..1][0..2] = Jac · (−R) */

    /* --- S = H·P·Hᵀ + R (矩阵乘法) --- */
    arm_mat_init_f32(&H_local, 2, 6, (float*)H);
    mat_mult(&M.HP, &H_local, &M.Pm);         /* HP(2×6) = H(2×6) · P(6×6) */
    mat_trans(&M.Ht, &H_local);               /* Hᵀ(6×2) = H(2×6)ᵀ */
    mat_mult(&M.S2, &M.HP, &M.Ht);            /* S(2×2) = HP(2×6) · Hᵀ(6×2) */
    Rm = EKF_SIGMA_BEARING * EKF_SIGMA_BEARING;
    M._s2[0] += Rm;  M._s2[3] += Rm;

    /* S⁻¹(2×2) */
    if (mat_inv(&M.Sinv2, &M.S2) != ARM_MATH_SUCCESS) return;
    for (int k = 0; k < 4; k++) if (!isfinite(M._sinv2[k])) return;

    /* 新息门控: yᵀ·S⁻¹·y */
    chi2 = y[0]*(M._sinv2[0]*y[0]+M._sinv2[1]*y[1])
         + y[1]*(M._sinv2[2]*y[0]+M._sinv2[3]*y[1]);
    if (!isfinite(chi2) || chi2 > EKF_INNO_GATE_CHI2) return;

    /* --- K = P·Hᵀ·S⁻¹ (矩阵乘法) --- */
    mat_mult(&M.PHt, &M.Pm, &M.Ht);           /* PHt(6×2) = P(6×6) · Hᵀ(6×2) */
    mat_mult(&M.K2, &M.PHt, &M.Sinv2);        /* K(6×2) = PHt(6×2) · S⁻¹(2×2) */

    /* --- dx += K·y --- */
    y_buf[0] = y[0]; y_buf[1] = y[1];
    arm_mat_init_f32(&y_col, 2, 1, y_buf);
    mat_mult(&M.y6, &M.K2, &y_col);           /* y6(6×1) = K(6×2) · y(2×1) */
    for (int k = 0; k < 6; k++) if (!isfinite(M._y6[k])) return;
    for (int r = 0; r < 6; r++) dx[r] += M._y6[r];

    /* --- P -= K·S·Kᵀ (矩阵乘法,强制对称) --- */
    mat_mult(&M.KS, &M.K2, &M.S2);            /* KS(6×2) = K(6×2) · S(2×2) */
    arm_mat_init_f32(&K2_trans, 2, 6, k2_t_buf);
    mat_trans(&K2_trans, &M.K2);               /* Kᵀ(2×6) */
    mat_mult(&M.KSK, &M.KS, &K2_trans);       /* KSK(6×6) = KS(6×2) · Kᵀ(2×6) */
    mat_sub(&M.Pm, &M.Pm, &M.KSK);            /* P -= KSK */
    for (int r = 0; r < 6; r++)                    /* 强制对称 */
        for (int c = r+1; c < 6; c++) {
            s = 0.5f*(P[r][c]+P[c][r]);
            P[r][c] = s; P[c][r] = s;
        }

    /* --- 注入误差到名义状态(dp,dv):矩阵加法 --- */
    {
        mat nom_p_m, nom_v_m, dx_p, dx_v;
        arm_mat_init_f32(&nom_p_m, 3, 1, nom_p);
        arm_mat_init_f32(&dx_p,    3, 1, dx);
        arm_mat_init_f32(&nom_v_m, 3, 1, nom_v);
        arm_mat_init_f32(&dx_v,    3, 1, &dx[3]);
        mat_add(&nom_p_m, &nom_p_m, &dx_p);   /* nom_p += dx[0..2] */
        mat_add(&nom_v_m, &nom_v_m, &dx_v);   /* nom_v += dx[3..5] */
    }

    memset(dx, 0, sizeof(dx));
    ekf_out.locked = 1;
    publish();
}

/* 速度锚定:俯冲入段时调用,直接设速度+收紧速度协方差。
 * V_NOM_MS 是初速度先验(标称值),不是真实速度量测。
 * 姿态前向 = R_matrix_T 第1行(机体Y轴=前向),ENU 分量顺序 fwd_x/fwd_y/fwd_z。*/
void EKF_SetVel(float vx, float vy, float vz)
{
    nom_v[0]=vx; nom_v[1]=vy; nom_v[2]=vz;
    /* P_vv = EKF_P0_VEL_VAR · I */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[3+i][3+j] = (i==j) ? EKF_P0_VEL_VAR : 0.0f;
    /* P_pv = 0, P_vp = 0 (旧相关失效) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            P[i][3+j] = 0.0f;
            P[3+i][j] = 0.0f;
        }
    publish();
}

/* 零速更新:ZUPT 作为速度=0 的量测更新(非直接写零)。
 * 只在发射前(Self_Text/Start)且静止判据满足时由 IMU.c 调用。
 * 量测:z=[0,0,0], H=[0,I](只作用于速度), R=σ_zupt²·I。
 * 更新只影响速度和速度相关协方差,不直接改位置。
 * 异常保护:S 奇异或含 NaN/Inf 时跳过本次 ZUPT,保持状态不变。*/
void EKF_UpdateZeroVel(void)
{
    /* --- 变量定义(全部在函数最前) --- */
    float Rz, Rz_I[9], y_buf[3], ks63_buf[18], k63_t_buf[18], s;
    int s_finite;
    mat Pvv, RzI, y_col, KS63, K63_trans;

    Rz = EKF_SIGMA_ZUPT * EKF_SIGMA_ZUPT;

    /* S = P_vv + R_zupt·I (矩阵加法) */
    arm_mat_init_f32(&Pvv, 3, 3, (float*)&P[3][3]);
    Rz_I[0] = Rz; Rz_I[1] = 0;  Rz_I[2] = 0;
    Rz_I[3] = 0;  Rz_I[4] = Rz; Rz_I[5] = 0;
    Rz_I[6] = 0;  Rz_I[7] = 0;  Rz_I[8] = Rz;
    arm_mat_init_f32(&RzI, 3, 3, Rz_I);
    mat_add(&M.S3, &Pvv, &RzI);           /* S(3×3) = P_vv + Rz·I */

    /* NaN/Inf 检查:S 含非有限值→跳过本次 ZUPT */
    s_finite = 1;
    for (int i = 0; i < 9 && s_finite; i++)
        if (!isfinite(M._s3[i])) { s_finite = 0; }
    if (!s_finite) { publish(); return; }

    /* S⁻¹(3×3) */
    if (mat_inv(&M.Sinv3, &M.S3) != ARM_MATH_SUCCESS) {
        publish();
        return;
    }
    for (int k = 0; k < 9; k++) if (!isfinite(M._sinv3[k])) { publish(); return; }

    /* P_pv: 从 P 提取位置-速度交叉块 */
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            M._ppv[r*3+c] = P[r][3+c];

    /* K = [P_pv; P_vv] · S⁻¹ → 分别算 Kp(3×3) 和 Kv(3×3) */
    mat_mult(&M.Kp, &M.P_pv, &M.Sinv3);    /* Kp(3×3) = P_pv · S⁻¹ */
    mat_mult(&M.Kv, &Pvv, &M.Sinv3);        /* Kv(3×3) = P_vv · S⁻¹ */
    /* K63 = [Kp; Kv]: 复制到 _k63 连续缓冲 */
    memcpy(M._k63,     M._kp, 9*sizeof(float));
    memcpy(M._k63 + 9, M._kv, 9*sizeof(float));

    /* 状态修正: [dp, dv] = K · y */
    y_buf[0] = -nom_v[0]; y_buf[1] = -nom_v[1]; y_buf[2] = -nom_v[2];
    arm_mat_init_f32(&y_col, 3, 1, y_buf);
    mat_mult(&M.d_state, &M.K63, &y_col);   /* d_state(6×1) = K(6×3) · y(3×1) */
    for (int k = 0; k < 6; k++) if (!isfinite(M._d_state[k])) { publish(); return; }
    for (int i = 0; i < 3; i++) {
        nom_p[i] += M._d_state[i];
        nom_v[i] += M._d_state[3+i];
    }

    /* 协方差更新: P -= K·S·Kᵀ (矩阵乘法,强制对称) */
    arm_mat_init_f32(&KS63, 6, 3, ks63_buf);
    mat_mult(&KS63, &M.K63, &M.S3);         /* KS(6×3) = K(6×3) · S(3×3) */
    arm_mat_init_f32(&K63_trans, 3, 6, k63_t_buf);
    mat_trans(&K63_trans, &M.K63);           /* Kᵀ(3×6) */
    mat_mult(&M.KSK_z, &KS63, &K63_trans);  /* KSK(6×6) = KS(6×3) · Kᵀ(3×6) */
    mat_sub(&M.Pm, &M.Pm, &M.KSK_z);        /* P -= KSK */
    for (int r = 0; r < 6; r++)                 /* 强制对称 */
        for (int c = r+1; c < 6; c++) {
            s = 0.5f*(P[r][c]+P[c][r]);
            P[r][c] = s; P[c][r] = s;
        }

    memset(dx, 0, sizeof(dx));
    publish();
}
