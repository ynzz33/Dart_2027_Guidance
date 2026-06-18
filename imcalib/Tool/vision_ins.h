//
// vision_ins.h —— 视觉(方位+测距) / IMU 紧耦合松姿态 EKF,给飞镖末制导提供不漂的速度。
//
// 为什么这么做(而不是纯积分 + ZUPT):
//   纯积分速度无外部观测必线性发散;ZUPT 只在"静止"时成立,出膛后飞行段没有零速时刻,救不了飞行漂移。
//   唯一能在飞行中把速度钉住的外部观测就是视觉——它看着固定靶,既给方位(像素→角,准)又给距离(dist_cm,粗)。
//   视觉 ~30fps 低速、IMU 1kHz 高速:用 IMU 在两视觉帧之间做高频预测,视觉帧到了再做一次校正,这正是紧耦合惯导。
//
// 状态(6 维,世界系 ENU,相对固定靶):
//   x = [ p(3), v(3) ]
//     p = 镖位置 − 靶位置 (m,世界系);  v = 镖速度 (m/s,世界系)
//   靶固定 → dp/dt = v, dv/dt = a_world。a_world(IMU 去重力后的世界系线加速度)当"输入"而非状态 → 模型线性 → 标准 KF,数值稳。
//   姿态不进状态:沿用现成 Mahony 的 R_matrix_T(松耦合姿态),只紧耦合位置/速度——在 G4 上正确、鲁棒、算得动。
//   加速度零偏不进状态:靠地面静态标定 + 静止在线 refine(已有);飞行段视觉位置量测会直接吸收残余零偏导致的速度误差。
//
// 量测:
//   视觉帧(识别成功):像素 x/y → 机体系视线单位向量 u_body(前向为镜头光轴);dist_cm → 距离 range。
//     合成"镖相对靶的世界系笛卡尔位置" z = −range·u_world 作 3 维位置量测,H=[I 0]。
//     量测噪声各向异性:R = σ⊥²·I + (σr²−σ⊥²)·u_world·u_worldᵀ
//       —— 垂直视线方向(方位)给小噪声 σ⊥=range·σ_bearing;沿视线方向(测距)给大噪声 σr。把"准方位 + 粗测距"用到极致。
//   零速更新:物理静止(|‖a‖−1g| 与陀螺双小,判据脱离 Guidance_State)时量测 v=0,H=[0 I],小噪声 → 钉死速度并间接收敛位置。
//
// 线程模型:predict 与所有 update 必须在同一个任务里调用(本工程放 IMUTask),单实例静态缓冲、零跨任务竞争。
//

#ifndef VISION_INS_H
#define VISION_INS_H

#include "arm_math.h"
#include "common_defs.h"

/* ===== 可调参数(全部待台架实测) ===== */
/* 过程噪声:把 IMU 世界加速度的不确定度(零偏残差+噪声,m/s²)注入 v,再经 dt 传到 p。越大越信视觉、越小越信 IMU 积分。*/
#define VINS_SIGMA_ACC        2.0f      /* m/s²,1σ。BMX055 去重力后残余等效加速度噪声,先给 2,漂得快就调大 */
/* 视觉量测噪声 */
#define VINS_SIGMA_BEARING    0.015f    /* rad,1σ。≈像素噪声×每像素角(70°/320≈0.0038rad/px,给几px余量) */
#define VINS_SIGMA_RANGE      1.5f      /* m,1σ。dist_cm 由 blob 反算,很粗 → 给大,主要靠方位收敛、测距只做弱约束 */
#define VINS_RANGE_MIN        0.30f     /* m。range 下限,防 dist_cm=0/异常 */
/* 零速更新噪声 */
#define VINS_SIGMA_ZUPT       0.02f     /* m/s,1σ。静止时速度量测=0 的可信度,给很小=强约束 */
/* 初始协方差 */
#define VINS_P0_POS           100.0f    /* m²。锁定首帧视觉前位置完全未知 → 大 */
#define VINS_P0_VEL           25.0f     /* (m/s)²。初速不确定度 */
/* 像素→角(与 PNG 一致:SENSOR_FOV/SENSOR_TOTAL_PIXEL_WIDTH);此处直接用 rad/像素 */
#define VINS_RAD_PER_PIXEL    (DEG2RAD(70.0f) / 320.0f)
/* 相机像素符号 → 机体系视线角(台架按"目标在右/在上时 u_body 指向是否正确"翻号) */
#define VINS_AZ_SIGN          (+1.0f)   /* 像素 x(水平)→ 绕上轴方位:+ = 目标偏右 */
#define VINS_EL_SIGN          (+1.0f)   /* 像素 y(垂直)→ 绕右轴俯仰:+ = 目标偏上 */

/* EKF 输出(世界系),也回写到 IMU_Data.Velocity 供 PNG/遥测 */
typedef struct {
    float   p_world[3];   /* 镖相对靶位置 (m) */
    float   v_world[3];   /* 镖速度 (m/s) */
    float   range_m;      /* |p| 估计距离 (m),融合后的,比裸 dist_cm 稳 */
    float   vc;           /* 接近速度 V_c = −(p·v)/|p| (m/s),供 PNG */
    uint8_t locked;       /* 1=已被视觉锁定过(p 可信) */
} VinsOut_t;

extern VinsOut_t vins_out;

void VisInsEKF_Init(void);
/* 1kHz:用 IMU 世界加速度推进一步。a_world 单位 m/s²,dt 单位 s */
void VisInsEKF_Predict(const float a_world[3], float dt);
/* 视觉新帧(识别成功)时调用一次。x_px/y_px=目标像素偏移,dist_cm=视觉距离,R_b2w=R_matrix_T(机体↔世界) */
void VisInsEKF_UpdateVision(float x_px, float y_px, float dist_cm, const float R_b2w[3][3]);
/* 物理静止时调用:零速量测 */
void VisInsEKF_UpdateZeroVel(void);
/* 俯冲入段锚定初速(世界系),清不确定度到 P0_VEL */
void VisInsEKF_SetVel(float vx, float vy, float vz);

#endif //VISION_INS_H
