//
// Created by ynz on 2025/11/17.
//
#include "mytype.h"
#include "common_defs.h"
#ifndef IMU_H
#define IMU_H


#define ACC_CS_PORT GPIOB
#define ACC_CS_PIN GPIO_PIN_12

#define GYR_CS_PORT GPIOA
#define GYR_CS_PIN GPIO_PIN_8

#define HIGH_ACC_CS_PORT GPIOB
#define HIGH_ACC_CS_PIN GPIO_PIN_11

#define MAG_CS_PORT GPIOA
#define MAG_CS_PIN GPIO_PIN_11

/* Mahony 互补滤波:预测(陀螺积分)-校正(加速度拉回重力方向),即"更聪明的输入滤波" */
#define mahony_MAXOUT   30.00f   /* 修正量限幅,防加速度突变把姿态拉飞 */
#define mahony_i_maxout  5.00f   /* 积分限幅 */
#define mahony_Kp        10.0f    /* 加速度校正强度:大→快速消陀螺漂移但易被振动带歪;小→抗扰好但收敛慢 */
#define mahony_Ki        0.1f  /* 陀螺零偏在线估计:静止时 acc_trust≈1→自动学习,高机动时 acc_trust≈0→冻结。
                                 * 值小=收敛慢但稳,0.01 约 100s 时间常数;mahony_i_maxout 防积分饱和。旧值 0=关。 */
#define mahony_Kd        0.0f    /* 必须为 0:标准 Mahony 只有 PI,D 项会放大噪声 */

/* === 加速度可信度门控:高g/机动(发射推力/气动减速/冲击)期间加速度计测的是比力而非重力,
 * 用它做 Mahony 校正会把姿态拽歪且事后回不来;按 |a| 偏离 1g 的程度线性降低校正权重,偏离大时
 * 纯靠(已去零偏的)陀螺积分 coast。另叠状态机硬门控(发射后全程不信),见 IMU.c IMU_Attitude_Algorithm。*/
#define ACC_TRUST_FULL_DEV  0.10f   /* ||a|-1g| <= 此值:完全信任加速度(权重=1) */
#define ACC_TRUST_ZERO_DEV  0.50f   /* ||a|-1g| >= 此值:完全不信任(权重=0,纯陀螺) */

#define GYR_KF_Q 1.0f
#define GYR_KF_R 10.0f
#define ACC_KF_Q 1.0f
#define ACC_KF_R 10.0f

/* === 传感器→机体系符号 (机体 X=右/东, Y=前/北, Z=上/天, 右手 ENU) ===
 * 每轴正方向只能台架实测锁定(BMX055 加速度/陀螺两片封装轴向本就不同);默认按当前硬件,
 * 按「验证表」做单轴动作,某轴方向反了就把对应符号翻 ±1。
 *
 * 加速度物理映射(寄存器序 rx2,1=accX / rx4,3=accY / rx6,5=accZ):
 *   A[X](右)=accY寄存器, A[Y](前)=accX寄存器, A[Z](上)=accZ寄存器 —— 见 BMX055_Read,静止 A[Z]≈+g(上为正)。 */
#define ACC_SIGN_X  (+1.0f)   /* 机体X 右/东:  + = 向右加速度为正 */
#define ACC_SIGN_Y  (+1.0f)   /* 机体Y 前/北:  + = 向前(发射方向)加速度为正 */
#define ACC_SIGN_Z  (-1.0f)   /* 机体Z 上/天:  使静止读数 = +g(上为正) */

/* 陀螺喂四元数的机体角速度(右手 rad/s): gx(绕右)=G[PITCH]=chipGyrY, gy(绕前)=G[ROLL]=chipGyrX(纵轴),
 * gz(绕上)=G[YAW]=chipGyrZ。右手系下 +gx=抬头、+gy=右滚、+gz=左偏(故上报 yaw 取 −gz)。 */
#define GYR_SIGN_X  (+1.0f)   /* 绕机体X(右): + 应 = 抬头(右手) */
#define GYR_SIGN_Y  (+1.0f)   /* 绕机体Y(前): + 应 = 右滚(右手) */
#define GYR_SIGN_Z  (-1.0f)   /* 绕机体Z(上): + 应 = 左偏(右手);上报 yaw 右+ = −此值 */

/* 俯冲入段锚定世界速度用的标称滑翔速度(m/s),待台架实测。
 * 只决定弹道角 γ 的演化速率(γ̇≈−g·cosγ/V),不决定初始 γ(初始 γ 只由姿态前向方向定),故粗略即可。*/
#define V_NOM_MS    9.0f
/* VEL_MAX_MS 见 common_defs.h */

/* === ZUPT 零速更新 + 地面零偏对准:纯积分速度无外部速度观测,任何加速度零偏/姿态残差都被无限积分→漂
 * (实测"漂移远大于真实运动"即此)。发射前(状态机 Self_Text/Start、地面静止)用零速观测把速度钉回0
 * (ZUPT=融合、非低通),并把"静止残差 a_raw−R_col3"慢速喂给机体系零偏 A_Offset 在线对准(不依赖标定姿态水平);
 * 发射后冻结零偏、停 ZUPT(防匀速飞行 ‖a‖≈1g 被误判静止而错误归零),靠对准好的零偏 + V_NOM 锚定撑过短俯冲段。*/
#define ZUPT_ACC_DEV_G   0.3f   /* 静止判据:加速度模长偏离 1g 的容差(g),超出视为运动。0.05→0.08:未在线对准时
                                  * ‖a‖含零偏/安装倾角可达≈1.05g,旧 0.05 卡死→静止永远判不出、ZUPT 不触发、零偏不对准 */
#define ZUPT_GYR_DPS     10.0f    /* 静止判据:角速度模长阈值(°/s),超出视为运动 */
#define ZUPT_HOLD_CNT    100     /* 连续满足判据多少拍(=ms@1kHz)才确认静止,防抖 */
#define ACC_BIAS_LPF_K   0.002f  /* 静止期零偏在线 refine 的 LPF 系数(时间常数≈1/K ms);初值=标定 A_Offset */

enum
{
    ACC = 0,
    GYR = 1,
    MAG = 2,
};

enum
{
    PITCH,
    ROLL,
    YAW,
};

enum
{
    X  ,
    Y  ,
    Z  ,
};

enum
{
    NOW,
    LAST,
    LLAST
};

enum
{
    World,
    Body,
};
/* BMX055 全部姿态状态。除特别说明外第一维=[NOW,LAST] 历史槽;陀螺量程 ±2000°/s、加速度 ±16g。
 * 机体系 ENU:X=右/东, Y=前/北(纵轴), Z=上/天。*/
typedef struct
{
    float G[2][3];          /* 陀螺角速度 °/s, 索引[PITCH,ROLL,YAW];已去零偏+标量卡尔曼。注:G[ROLL]=chipGyrX(前/纵轴)、G[PITCH]=chipGyrY(右) */
    float A[2][3];          /* 加速度 g, 索引[X右,Y前,Z上];去饱和/NaN+标量卡尔曼,静止 A[Z]≈+1 */
    float M[2][3];          /* 磁力计原始值;MAG 当前不启用(场景磁干扰大) */
    float G_Rad[2][3];      /* 陀螺角速度 rad/s = DEG2RAD(G);喂四元数积分 / PNG 视线角速度 */
    float Q[2][4];          /* 姿态四元数,顺序 [w,x,y,z] */
    float Euler[2][3];      /* 欧拉角 °, 索引[PITCH,ROLL,YAW];PITCH 绕X抬头+, ROLL 绕Y右滚+, YAW 绕Z右偏+ */
    float R_matrix_T[3][3]; /* 姿态旋转矩阵(机体↔世界);第3列=重力在机体系投影,供 Mahony 校正与 A_World 解算 */
    float A_Normed[2][3];   /* 归一化加速度(单位向量),Mahony 误差叉乘用 */
    float A_theory[2][3];   /* 理论重力方向(R_matrix_T 第3列 tx/ty/tz),Mahony 校正基准 */
    float A_World[2][3];    /* 世界系线加速度(已扣重力);→ 速度积分/PNG(当前未接主环) */
    float Velocity[2][2][3];/* 速度 [World/Body][NOW/LAST][X/Y/Z];当前未启用(速度卡尔曼注释掉) */
    float Vel_Dir[3];       /* 速度方向角(世界系)索引[PITCH,ROLL(未用),YAW] (°);EKF 输出后解算,供速度外环 PID。
                             * 注:必须 [3] 不能 [2]——枚举 YAW=2,Vel_Dir[YAW] 即下标2;原 [2] 会越界踩到下面 temp[0][0]。*/
    float temp[2][3];       /* 暂存(3维加速度卡尔曼用,当前 #if0 禁用) */
    float G_Offset[3];      /* 陀螺零偏(上电2s静态标定均值),IMU_Data_Read 中扣除 */
    float A_Offset[3];      /* 加速度零偏(标定均值,Z 已扣 1g);当前未回扣主环 */
    uint8_t calib_done;     /* 0=标定中(不减偏) 1=标定完成 2=已锁 Stable 姿态角 */
}
IMU_DATA_t;


extern IMU_DATA_t IMU_Data;
void IMU_Init(void);
void IMU_Calibrate(void);
void BMX055_Write(uint8_t Sensor,uint8_t Reg_Addr,uint8_t data);
void BMX055_Read(uint8_t Sensor,uint8_t Reg_Addr);
void IMU_Data_Read(void);
void IMU_Attitude_Algorithm(void);
void ALL_CS_Free(void);
#if USE_BMI088
void BMI088_Init_Acc(void);
void BMI088_Read_Acc(void);
#endif
extern uint32_t IMU_Cnt,control_cnt;
extern float acc_trust_obs;   /* Vofa 可观测:Mahony 加速度校正权重(1=全信任 / 0=纯陀螺coast) */
extern float gamma_pitch_deg;     /* 弹道角(速度积分版)°。速度链路 #if 0 后不再更新,消费端改用 gamma_pitch_fwd_deg */
extern float gamma_pitch_fwd_deg; /* 弹道角γ姿态前向估计°(不漂):取代会漂的速度版,末制导俯冲限幅/Vofa 用 */
extern uint8_t Vel_Reanchor_Flag; /* 俯冲入段置1,IMU 下一拍用姿态前向×V_NOM 锚定世界速度后清0 */
extern uint8_t imu_is_static;     /* Vofa 可观测:1=发射前判定静止、正在 ZUPT 归零速度+对准零偏,0=运动 */
extern uint8_t Yaw_Zero_Req;      /* 置1请求 IMU 下一拍以当前绝对航向为新原点(yaw 归零),IMU 捕获后清0 */
extern float   Yaw_Zero_Offset;   /* 绝对航向归零偏移°,可 Vofa 观测/调试器查看 */
#endif //IMU_H
