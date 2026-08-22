//
// vision_bearing_eskf.c —— 6态 bearing-only 非线性EKF(位置/速度)
//
// 姿态从 Mahony 的 IMU_Data.R_matrix_T 读取,不自己估计姿态误差。
// 加速度零偏靠第一阶段标定(A_Offset),不在滤波器内估计。
// 量测:方位角+俯仰角(不需要距离)。
// 位置定义:nom_p = 镖−靶(靶为原点,镖的位置),与旧 vision_ins.c 一致。
// 注意:range_m 和 vc 是名义推导量,仅作参考,不能用于控制决策。
//
#include "vision_bearing_eskf.h"
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
EskfOut_t eskf_out = {0};

/* ===== 内部标志 ===== */
static uint8_t pos_inited = 0;  /* 首帧位置初始化完成标志 */

static void publish(void)
{
    /* NaN/Inf 兜底:内部状态异常时输出安全值,不掩盖(eskf_out.locked 标记供上层检查) */
    int state_finite = 1;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(nom_p[i]) || !isfinite(nom_v[i])) { state_finite = 0; break; }
    }
    if (!state_finite) {
        for (int i = 0; i < 3; i++) { eskf_out.p_world[i] = 0; eskf_out.v_world[i] = 0; }
        eskf_out.range_m = 0; eskf_out.vc = 0;
        return;
    }
    for (int i = 0; i < 3; i++) {
        eskf_out.p_world[i] = nom_p[i];
        /* 仅输出限幅,不修改内部 nom_v:保持滤波器数学连续性 */
        float v_out = nom_v[i];
        abs_limit(&v_out, VEL_MAX_MS);
        eskf_out.v_world[i] = v_out;
    }
    float r2 = nom_p[0]*nom_p[0]+nom_p[1]*nom_p[1]+nom_p[2]*nom_p[2];
    eskf_out.range_m = sqrtf(r2);
    eskf_out.vc = (eskf_out.range_m > 0.01f)
            ? -(nom_p[0]*nom_v[0]+nom_p[1]*nom_v[1]+nom_p[2]*nom_v[2])/eskf_out.range_m
            : 0.0f;
}

void BearingESKF_Init(void)
{
    memset(nom_p, 0, sizeof(nom_p));
    memset(nom_v, 0, sizeof(nom_v));
    memset(dx, 0, sizeof(dx));
    memset(P, 0, sizeof(P));
    for (int i = 0; i < 3; i++) {
        P[i][i]     = ESKF_P0_POS;      /* dp */
        P[3+i][3+i] = ESKF_P0_VEL_VAR;      /* dv */
    }
    eskf_out.locked = 0;
    pos_inited = 0;
    publish();
}

/* 发射前复位:清零状态+协方差+位置初始化标志。
 * 当前为单次发射模型(飞镖上电→发射→结束),仅上电时 Init_Config 调用 Init 一次。
 * 若项目改为重复发射,需在每次新发射流程开始前调用此函数,
 * 并同步复位:旧 VisInsEKF(VisInsEKF_Init)、IMU.c 的 vins_last_recog/vel_Reanchor_Flag。*/
void BearingESKF_Reset(void)
{
    BearingESKF_Init();
}

uint8_t BearingESKF_PosInited(void)
{
    return pos_inited;
}

/* 位置初始化:第一次识别目标时调用,用首帧视觉角度+先验距离构造初始位置。
 * az_rad/el_rad:首帧视觉方位角/俯仰角(rad)
 * range_prior:仅作为 bearing-only 的尺度参考,不是真实距离。
 * 位置定义:nom_p = 镖−靶。镖→靶方向 = −nom_p/range。*/
void BearingESKF_InitPos(float az_rad, float el_rad, float range_prior)
{
    const float (*R)[3] = IMU_Data.R_matrix_T;
    /* 根据视觉角度构造机体系镖→靶单位向量 */
    float cb = cosf(el_rad);
    float u_body[3] = { sinf(az_rad)*cb, cosf(az_rad)*cb, sinf(el_rad) };
    /* 转世界系:u_world = R × u_body (机体系→世界系) */
    float u_world[3];
    for (int i = 0; i < 3; i++) {
        u_world[i] = 0.0f;
        for (int j = 0; j < 3; j++) u_world[i] += R[j][i] * u_body[j];
    }
    /* nom_p = 镖−靶 = −range·u_world (镖→靶方向取负) */
    nom_p[0] = -range_prior * u_world[0];
    nom_p[1] = -range_prior * u_world[1];
    nom_p[2] = -range_prior * u_world[2];
    /* 位置不确定性:距离方向大(不可观),角度方向小(已知) */
    float pos_var = (range_prior * 0.5f) * (range_prior * 0.5f);
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
void BearingESKF_Predict(const float acc_world[3], float dt)
{
    if (dt <= 0.0f || dt > 0.1f) return;

    /* 名义状态传播:先用旧速度更新位置,再更新速度。
     * 不在内部硬限幅 nom_v:限幅会破坏 P 与状态的一致性(位置连续但速度跳变)。
     * 仅在 publish() 输出时限幅 eskf_out.v_world。*/
    for (int i = 0; i < 3; i++) {
        nom_p[i] += nom_v[i] * dt + 0.5f * acc_world[i] * dt * dt;
        nom_v[i] += acc_world[i] * dt;
    }

    /* --- P = F·P·Fᵀ + Q (6×6闭式,与旧KF同结构) --- */
    /* F = [[I, dt·I],[0, I]]; 按3×3分块: A=pp, B=pv, C=vp, D=vv */
    float newA[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            newA[i][j] = P[i][j] + dt*(P[i][3+j] + P[3+i][j]) + dt*dt*P[3+i][3+j];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            P[i][3+j] = P[i][3+j] + dt*P[3+i][3+j];
            P[3+i][j] = P[3+i][j] + dt*P[3+i][3+j];
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = newA[i][j];

    /* Q:白噪声加速度模型 */
    float s2  = ESKF_SIGMA_ACC * ESKF_SIGMA_ACC;
    float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt;
    for (int i = 0; i < 3; i++) {
        P[i][i]     += s2*dt4*0.25f;
        P[i][3+i]   += s2*dt3*0.5f;
        P[3+i][i]   += s2*dt3*0.5f;
        P[3+i][3+i] += s2*dt2;
    }

    publish();
}

/* ===== 方位/俯仰量测更新(~30Hz) =====
 * 量测: az_rad, el_rad (从 Vision_Rx_Data.Euler 换算)
 * 姿态 R 固定为 Mahony 输出,不估计姿态误差。
 * H矩阵(2×6): 只有位置项 ∂z/∂dp = (∂z/∂u)·(−R)。
 * 位置定义:nom_p = 镖−靶(靶为原点)。*/
void BearingESKF_UpdateBearing(float az_rad, float el_rad)
{
    /* 读 Mahony 旋转矩阵 R(世界→机体) */
    const float (*R)[3] = IMU_Data.R_matrix_T;

    /* 机体系视线方向(镖→靶):u_b = R × (−nom_p)。R是世界→机体,−nom_p是世界系镖→靶方向 */
    float u_b[3];
    for (int i = 0; i < 3; i++) {
        u_b[i] = 0.0f;
        for (int j = 0; j < 3; j++) u_b[i] += R[i][j] * (-nom_p[j]);
    }
    float norm_xy = sqrtf(u_b[0]*u_b[0] + u_b[1]*u_b[1]);
    if (norm_xy < 1e-6f) return;

    /* 预测量测:az=atan2(水平横向,前向), el=atan2(竖直,水平投影) */
    float az_hat = atan2f(u_b[0], u_b[1]);
    float el_hat = atan2f(u_b[2], norm_xy);

    /* 新息(角度环绕) */
    float y[2];
    y[0] = az_rad - az_hat;
    y[1] = el_rad - el_hat;
    if (y[0] >  M_PI) y[0] -= 2.0f*M_PI;
    if (y[0] < -M_PI) y[0] += 2.0f*M_PI;
    if (y[1] >  M_PI) y[1] -= 2.0f*M_PI;
    if (y[1] < -M_PI) y[1] += 2.0f*M_PI;

    /* --- H矩阵(2×6): 只有位置项 ∂z/∂dp --- */
    /* 雅可比推导:
     *   u_b = R · (−p_world), R是世界→机体旋转矩阵  →  ∂u_b/∂p = −R
     *   az = atan2(u_b[0], u_b[1])
     *     ∂az/∂u_b = [+u1/(u0²+u1²), -u0/(u0²+u1²), 0]
     *   el = atan2(u_b[2], √(u0²+u1²))
     *     ∂el/∂u_b = [-u0·u2/norm³, -u1·u2/norm³, (u0²+u1²)/norm³]
     *   其中 norm = |u_b|, norm_xy = √(u0²+u1²)
     *   H = ∂z/∂u_b · ∂u_b/∂p = ∂z/∂u_b · (−R)*/
    float u0=u_b[0], u1=u_b[1], u2=u_b[2];
    float u_sq = u0*u0+u1*u1+u2*u2;
    float u_xy_sq = u0*u0+u1*u1;
    if (u_sq < 1e-8f || u_xy_sq < 1e-8f) return;

    /* ∂az/∂u_b = [+u1/u_xy_sq, -u0/u_xy_sq, 0] (真梯度,非负梯度) */
    float daz_du[3] = {u1/u_xy_sq, -u0/u_xy_sq, 0.0f};
    float inv_norm = 1.0f / (u_sq * norm_xy);
    float del_du[3] = {-u0*u2*inv_norm, -u1*u2*inv_norm, u_xy_sq*inv_norm};

    /* H = ∂z/∂u_b · ∂u_b/∂p = ∂z/∂u_b · (−R)
     * u_b = R·(−p) ⇒ ∂u_b/∂p = −R
     * 注意:−R ≠ Rᵀ(除非R=I),必须显式取负 */
    float H[2][6];
    memset(H, 0, sizeof(H));
    for (int j = 0; j < 3; j++) {
        float s_az=0, s_el=0;
        for (int k = 0; k < 3; k++) {
            s_az += daz_du[k] * (-R[k][j]);
            s_el += del_du[k] * (-R[k][j]);
        }
        H[0][j] = s_az;
        H[1][j] = s_el;
    }

    /* --- S = H·P·Hᵀ + R (2×2) --- */
    float HP[2][6];
    for (int r = 0; r < 2; r++)
        for (int c = 0; c < 6; c++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += H[r][k] * P[k][c];
            HP[r][c] = s;
        }
    float S[4];
    float Rm = ESKF_SIGMA_BEARING * ESKF_SIGMA_BEARING;
    S[0]=S[1]=S[2]=S[3]=0.0f;
    for (int k = 0; k < 6; k++) {
        S[0] += HP[0][k]*H[0][k];
        S[1] += HP[0][k]*H[1][k];
        S[2] += HP[1][k]*H[0][k];
        S[3] += HP[1][k]*H[1][k];
    }
    S[0]+=Rm; S[3]+=Rm;

    float det = S[0]*S[3]-S[1]*S[2];
    if (fabsf(det) < 1e-12f) return;
    float inv_det = 1.0f/det;
    float Sinv[4] = {S[3]*inv_det, -S[1]*inv_det, -S[2]*inv_det, S[0]*inv_det};

    /* 新息门控 */
    float chi2 = y[0]*(Sinv[0]*y[0]+Sinv[1]*y[1])+y[1]*(Sinv[2]*y[0]+Sinv[3]*y[1]);
    if (chi2 > ESKF_INNO_GATE_CHI2) return;

    /* --- K = P·Hᵀ·S⁻¹ (6×2) --- */
    float PHt[6][2];
    for (int r = 0; r < 6; r++) {
        PHt[r][0]=PHt[r][1]=0.0f;
        for (int k = 0; k < 6; k++) {
            PHt[r][0] += P[r][k]*H[0][k];
            PHt[r][1] += P[r][k]*H[1][k];
        }
    }
    float K[6][2];
    for (int r = 0; r < 6; r++) {
        K[r][0] = PHt[r][0]*Sinv[0]+PHt[r][1]*Sinv[2];
        K[r][1] = PHt[r][0]*Sinv[1]+PHt[r][1]*Sinv[3];
    }

    /* --- dx += K·y --- */
    for (int r = 0; r < 6; r++)
        dx[r] += K[r][0]*y[0]+K[r][1]*y[1];

    /* --- P -= K·S·Kᵀ (强制对称) --- */
    for (int r = 0; r < 6; r++)
        for (int c = r; c < 6; c++) {
            float ks = K[r][0]*(S[0]*K[c][0]+S[1]*K[c][1])
                      +K[r][1]*(S[2]*K[c][0]+S[3]*K[c][1]);
            P[r][c] = P[c][r] = P[r][c] - ks;
        }

    /* --- 注入误差到名义状态(dp,dv) --- */
    for (int i = 0; i < 3; i++) {
        nom_p[i] += dx[i];      /* dp */
        nom_v[i] += dx[3+i];    /* dv */
    }

    /* 重置误差状态 */
    memset(dx, 0, sizeof(dx));

    eskf_out.locked = 1;
    publish();
}

/* 速度锚定:俯冲入段时调用,直接设速度+收紧速度协方差。
 * V_NOM_MS 是初速度先验(标称值),不是真实速度量测。
 * 姿态前向 = R_matrix_T 第1行(机体Y轴=前向),ENU 分量顺序 fwd_x/fwd_y/fwd_z。*/
void BearingESKF_SetVel(float vx, float vy, float vz)
{
    nom_v[0]=vx; nom_v[1]=vy; nom_v[2]=vz;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[3+i][3+j] = (i==j) ? ESKF_P0_VEL_VAR : 0.0f;
    /* 速度被外部锚定,清位置↔速度互协方差(旧相关失效) */
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
void BearingESKF_UpdateZeroVel(void)
{
    float Rz = ESKF_SIGMA_ZUPT * ESKF_SIGMA_ZUPT;

    /* S = P_vv + R_zupt·I (3×3) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P[3+i][3+j] + ((i==j) ? Rz : 0.0f);

    /* NaN/Inf 检查:S 含非有限值→跳过本次 ZUPT */
    int s_finite = 1;
    for (int i = 0; i < 3 && s_finite; i++)
        for (int j = 0; j < 3; j++)
            if (!isfinite(S[i][j])) { s_finite = 0; break; }
    if (!s_finite) { publish(); return; }

    /* S⁻¹ (3×3解析求逆) */
    float a=S[0][0], b=S[0][1], c=S[0][2];
    float d=S[1][0], e=S[1][1], f=S[1][2];
    float g=S[2][0], h=S[2][1], k=S[2][2];
    float det = a*(e*k-f*h) - b*(d*k-f*g) + c*(d*h-e*g);
    if (fabsf(det) < 1e-18f) {
        /* S 奇异→跳过本次 ZUPT,保持 nom_p/nom_v/P 不变 */
        publish();
        return;
    }
    float inv_det = 1.0f/det;
    float Si[3][3];
    Si[0][0] = (e*k-f*h)*inv_det;  Si[0][1] = (c*h-b*k)*inv_det;  Si[0][2] = (b*f-c*e)*inv_det;
    Si[1][0] = (f*g-d*k)*inv_det;  Si[1][1] = (a*k-c*g)*inv_det;  Si[1][2] = (c*d-a*f)*inv_det;
    Si[2][0] = (d*h-e*g)*inv_det;  Si[2][1] = (b*g-a*h)*inv_det;  Si[2][2] = (a*e-b*d)*inv_det;

    /* 新息 y = 0 - nom_v */
    float y[3] = {-nom_v[0], -nom_v[1], -nom_v[2]};

    /* K = P_pv · S⁻¹ (3×3, P_pv 是 P 的位置-速度交叉块) */
    /* 更新: v += K·y, P_vv -= K·S·Kᵀ, P_pv -= K·S·(K_pv)ᵀ 等 */
    /* 简化:因为 H=[0,I], K 的行对应位置和速度:
     *   K[0..2] = P_pv · S⁻¹, K[3..5] = P_vv · S⁻¹ */
    float Kv[3][3]; /* K 的速度行 */
    float Kp[3][3]; /* K 的位置行 */
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float sp=0, sv=0;
            for (int m = 0; m < 3; m++) {
                sp += P[r][3+m] * Si[m][c];
                sv += P[3+r][3+m] * Si[m][c];
            }
            Kp[r][c] = sp;
            Kv[r][c] = sv;
        }

    /* 状态更新: p += Kp·y, v += Kv·y */
    for (int r = 0; r < 3; r++) {
        float sp=0, sv=0;
        for (int c = 0; c < 3; c++) { sp += Kp[r][c]*y[c]; sv += Kv[r][c]*y[c]; }
        nom_p[r] += sp;
        nom_v[r] += sv;
    }

    /* 协方差更新: P -= K·S·Kᵀ */
    /* KSKᵀ 的位置-位置块: Kp·S·Kpᵀ */
    float KSKpp[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = r; c < 3; c++) {
            float s = 0;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    s += Kp[r][i] * S[i][j] * Kp[c][j];
            KSKpp[r][c] = KSKpp[c][r] = s;
        }
    /* KSKᵀ 的速度-速度块: Kv·S·Kvᵀ */
    float KSKvv[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = r; c < 3; c++) {
            float s = 0;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    s += Kv[r][i] * S[i][j] * Kv[c][j];
            KSKvv[r][c] = KSKvv[c][r] = s;
        }
    /* KSKᵀ 的位置-速度块: Kp·S·Kvᵀ */
    float KSKpv[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float s = 0;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    s += Kp[r][i] * S[i][j] * Kv[c][j];
            KSKpv[r][c] = s;
        }

    /* P_pp -= KSKpp (强制对称) */
    for (int r = 0; r < 3; r++)
        for (int c = r; c < 3; c++) {
            P[r][c] -= KSKpp[r][c];
            if (r != c) P[c][r] = P[r][c];
        }
    /* P_vv -= KSKvv (强制对称) */
    for (int r = 0; r < 3; r++)
        for (int c = r; c < 3; c++) {
            P[3+r][3+c] -= KSKvv[r][c];
            if (r != c) P[3+c][3+r] = P[3+r][3+c];
        }
    /* P_pv -= KSKpv, P_vp = P_pvᵀ */
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            P[r][3+c] -= KSKpv[r][c];
            P[3+c][r] = P[r][3+c];
        }

    /* 强制协方差对称:P = 0.5*(P + Pᵀ) */
    for (int r = 0; r < 6; r++)
        for (int c = r+1; c < 6; c++) {
            float sym = 0.5f * (P[r][c] + P[c][r]);
            P[r][c] = sym;
            P[c][r] = sym;
        }

    memset(dx, 0, sizeof(dx));
    publish();
}
