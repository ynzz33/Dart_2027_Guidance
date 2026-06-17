//
// vision_ins.c —— 见 vision_ins.h 顶部设计说明。6 态线性 KF(世界系 p,v,相对固定靶),
// IMU 加速度做输入预测(1kHz)、视觉笛卡尔位置量测更新(~30Hz)、静止零速更新。
// 结构固定、手写显式矩阵运算(只有 3x3 求逆借 arm_math),单实例静态、单任务调用、零竞争。
//
#include "vision_ins.h"
#include <math.h>

/* ===== 模块内部状态(单实例) ===== */
static float X[6];        /* [px,py,pz, vx,vy,vz] 世界系 */
static float P[6][6];     /* 协方差 */

/* ===== 对外输出 ===== */
float   vins_p_world[3] = {0};
float   vins_v_world[3] = {0};
float   vins_range_m    = 0.0f;
float   vins_vc         = 0.0f;
uint8_t vins_locked     = 0;

static void publish(void)
{
    for (int i = 0; i < 3; i++) { vins_p_world[i] = X[i]; vins_v_world[i] = X[3+i]; }
    float r2 = X[0]*X[0] + X[1]*X[1] + X[2]*X[2];
    vins_range_m = sqrtf(r2);
    /* 接近速度 V_c = −d|p|/dt = −(p·v)/|p|;靠近(|p|减小)时为正 */
    vins_vc = (vins_range_m > 0.01f)
            ? -(X[0]*X[3] + X[1]*X[4] + X[2]*X[5]) / vins_range_m
            : 0.0f;
}

void VisInsEKF_Init(void)
{
    for (int i = 0; i < 6; i++) { X[i] = 0.0f; for (int j = 0; j < 6; j++) P[i][j] = 0.0f; }
    for (int i = 0; i < 3; i++) { P[i][i] = VINS_P0_POS; P[3+i][3+i] = VINS_P0_VEL; }
    vins_locked = 0;
    publish();
}

/* 通用量测更新:量测的是状态从 m0 起的 3 个分量(m0=0→位置, m0=3→速度)。
 * H = 在 m0 处的 3x3 单位、其余 0。标准 KF:S=HPHᵀ+R; K=PHᵀS⁻¹; x+=K(z−Hx); P=(I−KH)P。*/
static void ekf_update(int m0, const float z[3], const float R[3][3])
{
    /* S = P[m0..][m0..] + R  (3x3) */
    float Sd[9], Sinvd[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Sd[i*3+j] = P[m0+i][m0+j] + R[i][j];

    arm_matrix_instance_f32 Sm, Sim;
    arm_mat_init_f32(&Sm,  3, 3, Sd);
    arm_mat_init_f32(&Sim, 3, 3, Sinvd);
    if (arm_mat_inverse_f32(&Sm, &Sim) != ARM_MATH_SUCCESS) return;  /* 奇异则跳过本次更新 */

    /* K = P·Hᵀ·S⁻¹,P·Hᵀ = P 的第 m0..m0+2 列 (6x3) */
    float K[6][3];
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 3; c++) {
            float s = 0.0f;
            for (int m = 0; m < 3; m++) s += P[r][m0+m] * Sinvd[m*3+c];
            K[r][c] = s;
        }

    /* x += K·(z − x[m0..]) */
    float y[3];
    for (int i = 0; i < 3; i++) y[i] = z[i] - X[m0+i];
    for (int r = 0; r < 6; r++) {
        float s = 0.0f;
        for (int c = 0; c < 3; c++) s += K[r][c] * y[c];
        X[r] += s;
    }

    /* P −= K·(H·P),H·P = P 的第 m0..m0+2 行 (3x6);全部用更新前的 P 计算 */
    float dP[6][6];
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++) {
            float s = 0.0f;
            for (int m = 0; m < 3; m++) s += K[r][m] * P[m0+m][c];
            dP[r][c] = s;
        }
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++)
            P[r][c] -= dP[r][c];

    /* 数值对称化,防止协方差因舍入失对称发散 */
    for (int r = 0; r < 6; r++)
        for (int c = r+1; c < 6; c++) {
            float a = 0.5f * (P[r][c] + P[c][r]);
            P[r][c] = P[c][r] = a;
        }
    publish();
}

void VisInsEKF_Predict(const float a_world[3], float dt)
{
    /* 状态:p += v·dt + ½a·dt²; v += a·dt */
    for (int i = 0; i < 3; i++) {
        X[i]   += X[3+i]*dt + 0.5f*a_world[i]*dt*dt;
        X[3+i] += a_world[i]*dt;
    }
    /* 协方差 P = F·P·Fᵀ + Q,F=[[I, dt·I],[0, I]];按 3x3 分块闭式更新(A=pp,B=pv,C=vp,D=vv):
     *   A' = A + dt(B+C) + dt²D ;  B' = B + dt·D ;  C' = C + dt·D ;  D' = D    (均用旧值) */
    float newA[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            newA[i][j] = P[i][j] + dt*(P[i][3+j] + P[3+i][j]) + dt*dt*P[3+i][3+j];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            P[i][3+j] = P[i][3+j] + dt*P[3+i][3+j];   /* B' (旧 D) */
            P[3+i][j] = P[3+i][j] + dt*P[3+i][3+j];   /* C' (旧 D) */
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = newA[i][j];
    /* Q:白噪声加速度模型,σ_a 注入。Qpp=σ²dt⁴/4, Qpv=σ²dt³/2, Qvv=σ²dt²(各轴独立,只加块对角) */
    float s2  = VINS_SIGMA_ACC*VINS_SIGMA_ACC;
    float qpp = s2*dt*dt*dt*dt*0.25f, qpv = s2*dt*dt*dt*0.5f, qvv = s2*dt*dt;
    for (int i = 0; i < 3; i++) {
        P[i][i]     += qpp;
        P[i][3+i]   += qpv;
        P[3+i][i]   += qpv;
        P[3+i][3+i] += qvv;
    }
    publish();
}

void VisInsEKF_UpdateVision(float x_px, float y_px, float dist_cm, const float R_b2w[3][3])
{
    /* 像素 → 机体系视线单位向量(机体 X=右,Y=前=镜头光轴,Z=上;居中→指向前) */
    float az = VINS_AZ_SIGN * x_px * VINS_RAD_PER_PIXEL;   /* 水平方位(绕上轴) */
    float el = VINS_EL_SIGN * y_px * VINS_RAD_PER_PIXEL;   /* 垂直俯仰(绕右轴) */
    float cb = cosf(el);
    float u_body[3];
    u_body[0] = sinf(az) * cb;   /* 右 */
    u_body[1] = cosf(az) * cb;   /* 前 */
    u_body[2] = sinf(el);        /* 上 */

    float range = dist_cm * 0.01f;          /* cm → m */
    if (range < VINS_RANGE_MIN) range = VINS_RANGE_MIN;

    /* 机体→世界:world[i] = Σ_k R_b2w[k][i]·body[k](与 IMU.c A_World 同一约定) */
    float u_world[3];
    for (int i = 0; i < 3; i++)
        u_world[i] = R_b2w[0][i]*u_body[0] + R_b2w[1][i]*u_body[1] + R_b2w[2][i]*u_body[2];

    /* 量测 z = p = 镖−靶 = −range·u_world(u_world 指镖→靶) */
    float z[3];
    for (int i = 0; i < 3; i++) z[i] = -range * u_world[i];

    /* 各向异性量测噪声:R = σ⊥²·I + (σr²−σ⊥²)·u·uᵀ。σ⊥=range·σ_bearing(方位准),σr=σ_range(测距粗) */
    float sp = range * VINS_SIGMA_BEARING;
    float sp2 = sp*sp, sr2 = VINS_SIGMA_RANGE*VINS_SIGMA_RANGE;
    float R[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R[i][j] = (i==j ? sp2 : 0.0f) + (sr2 - sp2)*u_world[i]*u_world[j];

    ekf_update(0, z, R);
    vins_locked = 1;
}

void VisInsEKF_UpdateZeroVel(void)
{
    float z[3] = {0.0f, 0.0f, 0.0f};
    float s = VINS_SIGMA_ZUPT*VINS_SIGMA_ZUPT;
    float R[3][3] = {{s,0,0},{0,s,0},{0,0,s}};
    ekf_update(3, z, R);   /* 量测速度=0 */
}

void VisInsEKF_SetVel(float vx, float vy, float vz)
{
    X[3] = vx; X[4] = vy; X[5] = vz;
    /* 重置速度协方差、清位置↔速度互协方差(初速被外部锚定,旧相关失效) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            P[3+i][3+j] = (i==j) ? VINS_P0_VEL : 0.0f;
            P[i][3+j]   = 0.0f;
            P[3+i][j]   = 0.0f;
        }
    publish();
}
