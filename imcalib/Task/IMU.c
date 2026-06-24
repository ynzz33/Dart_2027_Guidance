//
// Created by ynz on 2025/11/17.
//

#include "IMU.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "mytype.h"
#include "spi.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "filter.h"
#include "math.h"
#include "pid.h"
#include "surface_control_task.h"
#include "CallBack_Task.h"
#include "vision_ins.h"
u8 Current_Sensor,Current_Use_Flag,receiveflag = 0;
IMU_DATA_t IMU_Data = {0};
uint32_t IMU_Cnt = 0,control_cnt = 0;
float   gamma_pitch_deg = 0.0f;   /* 弹道角(速度方向俯仰角,速度积分版)°。注:速度链路已 #if 0 停用→此变量不再更新,消费端改用下面 gamma_pitch_fwd_deg;保留定义便于日后接观测器复活 */
float   gamma_pitch_fwd_deg = 0.0f; /* 弹道角γ的姿态前向估计°(机体纵轴前向仰角,不漂):取代会漂的速度版,供末制导俯冲限幅/Vofa 用 */
uint8_t Vel_Reanchor_Flag = 0;    /* 俯冲入段置1:本拍 IMU 用"姿态前向×V_NOM"锚定世界速度后清0(见下) */
uint8_t imu_is_static = 0;        /* Vofa:1=发射前判定静止、正 ZUPT 归零速度+对准零偏,0=运动(见下 ZUPT 段) */
void IMU_Attitude_Algorithm(void)
{
#if 0 /*对加速度的一阶三维卡尔曼滤波*/
    float *filtered_acc = kalman_filter3_imu_calc(&IMU_Kalman_Filter_3, IMU_Data.A[NOW][X], IMU_Data.A[NOW][Y], IMU_Data.A[NOW][Z], IMU_Data.G_Rad[NOW][PITCH], IMU_Data.G_Rad[NOW][ROLL], IMU_Data.G_Rad[NOW][YAW]);

    // 更新滤波后的加速度值
    IMU_Data.temp[NOW][X] = filtered_acc[0];
    IMU_Data.temp[NOW][Y] = filtered_acc[1];
    IMU_Data.temp[NOW][Z] = filtered_acc[2];
    float acc_norm_t = sqrt(IMU_Data.temp[NOW][X]*IMU_Data.temp[NOW][X]+
                          IMU_Data.temp[NOW][Y]*IMU_Data.temp[NOW][Y]+
                          IMU_Data.temp[NOW][Z]*IMU_Data.temp[NOW][Z]);
    if (acc_norm_t<0.00001)
    {
        IMU_Data.temp[NOW][X] = 0;
        IMU_Data.temp[NOW][Y] = 0;
        IMU_Data.temp[NOW][Z] = 0;
    }
    else
    {
        IMU_Data.temp[NOW][X]/=acc_norm_t;
        IMU_Data.temp[NOW][Y]/=acc_norm_t;
        IMU_Data.temp[NOW][Z]/=acc_norm_t;
    }
#endif
#if 0 /*对速度的二阶卡尔曼*/ //速度改用ekf，

#endif
#if 1   /*局部变量定义*/
    float
    /* Q_theory_W       */  q0 = IMU_Data.Q[NOW][0],
    /* Q_theory_X       */  q1 = IMU_Data.Q[NOW][1],
    /* Q_theory_Y       */  q2 = IMU_Data.Q[NOW][2],
    /* Q_theory_Z       */  q3 = IMU_Data.Q[NOW][3],
    /* A_Real_X         */  a_raw_x = IMU_Data.A[NOW][X],
    /* A_Real_Y         */  a_raw_y = IMU_Data.A[NOW][Y],
    /* A_Real_Z         */  a_raw_z = IMU_Data.A[NOW][Z],
    /*A_NORMED          */  ax_normed = IMU_Data.A_Normed[NOW][X],
    /*A_NORMED          */  ay_normed = IMU_Data.A_Normed[NOW][Y],
    /*A_NORMED          */  az_normed = IMU_Data.A_Normed[NOW][Z],
    /* G_theory_X       */  tx,
    /* G_theory_Y       */  ty,
    /* G_theory_Z       */  tz,
    /* 统一机体系 X=右/Y=前/Z=上 (右手 ENU),陀螺与加速度同系——这是修复关键:
       加速度 a_raw=(A[X]=右,A[Y]=前,A[Z]=上);陀螺必须取同系角速度,否则 Mahony 把
       两个差 X↔Y 对调的系做叉乘、修正算错致姿态发散。
       绕右(机体X)=chipGyrY=G_Rad[PITCH]; 绕前(机体Y,纵轴)=chipGyrX=G_Rad[ROLL]; 绕上(机体Z)=G_Rad[YAW]。*/
    /* G 机体X(右)=chipY */  gx = GYR_SIGN_X * IMU_Data.G_Rad[NOW][PITCH],
    /* G 机体Y(前)=chipX */  gy = GYR_SIGN_Y * IMU_Data.G_Rad[NOW][ROLL ],
    /* G 机体Z(上)=chipZ */  gz = GYR_SIGN_Z * IMU_Data.G_Rad[NOW][YAW  ],
    /* 重力加速度,方便归一*/  gravity = GRAVITY_MS2;
#endif
#if 1   /*内环角速度反馈(deg):用原始测量(未叠Mahony),用户极性 抬头+/右滚+/右偏+*/
    /* 串级内环要求"角速度 = 对应欧拉角的导数"。pitch(绕右)/roll(绕前)与右手机体角速度同号直通;
       yaw 右偏+ = −绕上(右手) → 取 −gz,与新欧拉角 YAW(右+)一致。gx/gy/gz 此刻是原始测量。*/
    Surface.current_gyro_Euler[NOW][PITCH] =  RAD2DEG(gx);
    Surface.current_gyro_Euler[NOW][ROLL ] =  RAD2DEG(gy);
    Surface.current_gyro_Euler[NOW][YAW  ] = -RAD2DEG(gz);
#endif
#if 1   /*mahony补偿*/
    /*逆旋转矩阵的转化*/
    IMU_Data.R_matrix_T[0][0] = 1-2*(q2*q2+q3*q3);      IMU_Data.R_matrix_T[0][1] = 2*(q1*q2+q0*q3);        IMU_Data.R_matrix_T[0][2] = 2*(q1*q3-q0*q2);
    IMU_Data.R_matrix_T[1][0] = 2*(q1*q2-q0*q3);        IMU_Data.R_matrix_T[1][1] = 1-2*(q1*q1+q3*q3);      IMU_Data.R_matrix_T[1][2] = 2*(q2*q3+q0*q1);
    IMU_Data.R_matrix_T[2][0] = 2*(q1*q3+q0*q2);        IMU_Data.R_matrix_T[2][1] = 2*(q2*q3-q0*q1);        IMU_Data.R_matrix_T[2][2] = 1-2*(q1*q1+q2*q2);
    tx = IMU_Data.R_matrix_T[0][2] ;
    ty = IMU_Data.R_matrix_T[1][2] ;
    tz = IMU_Data.R_matrix_T[2][2] ;
    /*实际加速度归一化*/
    float mahony_temp[3] = {0};
    float acc_norm = sqrtf(a_raw_x*a_raw_x+a_raw_y*a_raw_y+a_raw_z*a_raw_z);
    if (acc_norm<0.001f)
    {
        ax_normed = 0;
        ay_normed = 0;
        az_normed = 0;
    }
    else
    {
        /* 归一化：保留符号！fabs会丢失方向信息，导致Mahony补偿错误 */
        ax_normed = a_raw_x / acc_norm;
        ay_normed = a_raw_y / acc_norm;
        az_normed = a_raw_z / acc_norm;
    }
    /* === 加速度可信度门控(核心修复) ===
     * acc_norm 以 g 为单位、静止≈1.0。偏离 1g 越多→越可能掺入线加速度(发射推力/气动减速/冲击),
     * 此时加速度方向不是重力,用它做 Mahony 校正会把姿态拉飞且回不来 → 按偏离程度线性降权。*/
    float acc_dev = fabsf(fabs(acc_norm) - 1.0f);
    float acc_trust;
    if      (acc_dev <= ACC_TRUST_FULL_DEV) acc_trust = 1.0f;
    else if (acc_dev >= ACC_TRUST_ZERO_DEV) acc_trust = 0.0f;
    else    acc_trust = (ACC_TRUST_ZERO_DEV - acc_dev) / (ACC_TRUST_ZERO_DEV - ACC_TRUST_FULL_DEV);
    /* 方案B:发射后整个飞行段无"干净重力相"(推力→气动减速→冲击),气动减速幅度有时≈1g却方向朝后
     * 会骗过幅度门控;故状态机一旦判出已发射(Start→Stable 用 A_Normed[Y]≥0.8),全程硬置0、纯靠
     * (已去零偏的)陀螺 coast。Guidance_State/枚举见 surface_control_task.h(IMU.c 已 include)。*/
    if ((Guidance_State == Stable || Guidance_State == Terminal || Guidance_State == End)&&imu_is_static==0)
        acc_trust = 0.0f;
    // acc_trust = 1.0f;
    /* 计算误差(误差先乘可信度:trust=0 时 err=0 → 比例项=0 且 pid 积分停止累加=冻结零偏估计不被污染,
     * 同时 pid 内 iout 保留发射前学到的好零偏值继续补偿陀螺,正是 coast 想要的)*/
        mahony_temp[X] = pid_calc(&mahony_pid[X], acc_trust*(az_normed*ty - ay_normed*tz), 0, dT);
        mahony_temp[Y] = pid_calc(&mahony_pid[Y], acc_trust*(ax_normed*tz - az_normed*tx), 0, dT);
        mahony_temp[Z] = pid_calc(&mahony_pid[Z], acc_trust*(ay_normed*tx - ax_normed*ty), 0, dT);
        
        // mahony_temp[X] = pid_calc(&mahony_pid[X], az_normed*ty - ay_normed*tz, 0, dT);
        // mahony_temp[Y] = pid_calc(&mahony_pid[Y], ax_normed*tz - az_normed*tx, 0, dT);
        // mahony_temp[Z] = pid_calc(&mahony_pid[Z], ay_normed*tx - ax_normed*ty, 0, dT);
    gx += mahony_temp[X];
    gy += mahony_temp[Y];
    gz += mahony_temp[Z];

    /*计算修正后的陀螺仪数据（比例+积分补偿）*/
#endif

#if 1   
   /*世界加速度与
      世界速度与
      机体速度*/
    /* === ZUPT 零速更新 + 地面零偏对准(仅发射前) ===
     * 纯积分速度无外部速度观测→任何加速度零偏/姿态残差都被无限积分而漂(实测"漂移远大于真实运动"即此)。
     * 发射前(状态机未发射 且 静止)用零速观测把速度钉回0,并把"静止残差 a_raw−R_col3"(姿态此刻被 Mahony
     * 校正,该残差≈机体系真零偏)慢速喂给 A_Offset 在线对准(不依赖标定时是否水平);发射后冻结零偏、停 ZUPT
     * (防匀速飞行 ‖a‖≈1g 被误判静止而错误归零)。判据:|‖a‖−1g| 与 角速度 双小、持续 HOLD 拍。宏见 IMU.h。*/
    float g_norm_dps = sqrtf(IMU_Data.G[NOW][PITCH]*IMU_Data.G[NOW][PITCH] +
                             IMU_Data.G[NOW][ROLL ]*IMU_Data.G[NOW][ROLL ] +
                             IMU_Data.G[NOW][YAW  ]*IMU_Data.G[NOW][YAW  ]);
    /* ZUPT 静止判据要用"去零偏后"的加速度模长,不能用裸 acc_dev:
     * BMX055 加速度零位偏移可达 ~0.07-0.15g,静止时裸 ‖a_raw‖ 未必落在 1g±ZUPT_ACC_DEV_G,
     * 该固定偏移会把 acc_dev 永久顶在阈值外→"静止且状态对仍判不出静止"(实测根因)。
     * 扣掉标定/在线对准的 A_Offset 后,静止 ≈1g → acc_dev_zupt≈0。A_Offset 上电即由 IMU_Calibrate 填好,无死锁。*/
    float axc = a_raw_x - IMU_Data.A_Offset[X];
    float ayc = a_raw_y - IMU_Data.A_Offset[Y];
    float azc = a_raw_z - IMU_Data.A_Offset[Z];
    float acc_dev_zupt = fabsf(sqrtf(axc*axc + ayc*ayc + azc*azc) - 1.0f);
    static uint16_t zupt_cnt = 0;
    uint8_t pre_launch = (Guidance_State == Self_Text_State || Guidance_State == Start);
    if (pre_launch && acc_dev_zupt < ZUPT_ACC_DEV_G && g_norm_dps < ZUPT_GYR_DPS)  /* 用去零偏模长,见上 */
    {
        if (zupt_cnt < ZUPT_HOLD_CNT) zupt_cnt++;
    }
    else zupt_cnt = 0;
    imu_is_static = (zupt_cnt >= ZUPT_HOLD_CNT) ? 1 : 0;
    if (imu_is_static)   /* 静止确认:在线 refine 机体系零偏(运动/发射后不更新=冻结) */
    {
        IMU_Data.A_Offset[X] += ACC_BIAS_LPF_K * ((a_raw_x - IMU_Data.R_matrix_T[0][2]) - IMU_Data.A_Offset[X]);
        IMU_Data.A_Offset[Y] += ACC_BIAS_LPF_K * ((a_raw_y - IMU_Data.R_matrix_T[1][2]) - IMU_Data.A_Offset[Y]);
        IMU_Data.A_Offset[Z] += ACC_BIAS_LPF_K * ((a_raw_z - IMU_Data.R_matrix_T[2][2]) - IMU_Data.A_Offset[Z]);
    }
    /* 去重力得机体系真实线加速度,单位 m/s²。先扣机体系零偏 A_Offset、再扣重力投影。a_raw 单位 g(静止|a|≈1),
     * 速度积分(v+=dT·a)与锚定 V_NOM_MS 按 m/s,故 ×gravity(=GRAVITY_MS2)把 (a_raw−bias) 换成 m/s²、再扣
     * 机体系重力投影 gravity·R_col3(R_matrix_T 第3列)。合并即 gravity·((a_raw−bias) − R_col3):静止且零偏
     * 对准时 a_raw−bias=R_col3 → 线加速度=0(速度不漂),飞行时得真实 m/s²。原写法 a_raw − gravity·R 是
     * g 减 m/s²、量纲不一致(静止误出 ≈−8.8)且未扣零偏,使 A_World/速度/弹道角 γ 全错——此即 γ 不准的根因。*/
    float ax_no_gravity = gravity * ((a_raw_x - IMU_Data.A_Offset[X]) - IMU_Data.R_matrix_T[0][2]);
    float ay_no_gravity = gravity * ((a_raw_y - IMU_Data.A_Offset[Y]) - IMU_Data.R_matrix_T[1][2]);
    float az_no_gravity = gravity * ((a_raw_z - IMU_Data.A_Offset[Z]) - IMU_Data.R_matrix_T[2][2]);
    for (int i = 0; i<3 ;i++)
    {
        IMU_Data.A_World[NOW] [i]  =  ( IMU_Data.R_matrix_T[0][i]*ax_no_gravity +
                                        IMU_Data.R_matrix_T[1][i]*ay_no_gravity +
                                        IMU_Data.R_matrix_T[2][i]*az_no_gravity );
        IMU_Data.A_World[NOW] [i] = KalmanFilter( &ACC_WORLD_Kalman_Filter[i],IMU_Data.A_World[NOW][i],0.1f,5.0f );
    }
    /* === 视觉/IMU 紧耦合 EKF:取代纯积分,给不漂的世界/机体速度(见 Tool/vision_ins.c) ===
     * 全部在本任务(IMUTask)内调用:predict 每拍 + 视觉新帧位置更新 + 静止零速更新,单任务零竞争。*/
    {
        /* 1) 预测:本拍世界加速度推进一步(1kHz) */
        float a_w[3] = { IMU_Data.A_World[NOW][X], IMU_Data.A_World[NOW][Y], IMU_Data.A_World[NOW][Z] };
        VisInsEKF_Predict(a_w, dT);
    }
    /* 2) 俯冲入段锚定初速(姿态前向×V_NOM) */
    if (Vel_Reanchor_Flag)
    {
        float fwd_x = IMU_Data.R_matrix_T[1][0];
        float fwd_y = IMU_Data.R_matrix_T[1][1];
        float fwd_z = IMU_Data.R_matrix_T[1][2];
        VisInsEKF_SetVel(V_NOM_MS * fwd_x, V_NOM_MS * fwd_y, V_NOM_MS * fwd_z);
        Vel_Reanchor_Flag = 0;
    }
    /* 3) 视觉新帧(识别成功且有距离包)→ 笛卡尔位置量测更新。用 Vision_Recog_Cnt 跳变判新帧,
     *    不抢 TotalControlTask 的 Vision_New_Data_flag;本任务读 Vision_Rx_Data(ISR 写,字段小). */
    {
        static uint32_t vins_last_recog = 0;
        uint32_t rc = Vision_Rx_Data.Vision_Recog_Cnt;
        if (rc != vins_last_recog)
        {
            if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_SUCCESS && Vision_Rx_Data.dist_cm > 0)
                VisInsEKF_UpdateVision((float)Vision_Rx_Data.x[NOW], (float)Vision_Rx_Data.y[NOW],
                                       (float)Vision_Rx_Data.dist_cm, IMU_Data.R_matrix_T);
            vins_last_recog = rc;
        }
    }
    /* 4) 物理静止 → 零速更新(脱离 Guidance_State,见上 ZUPT 修复) */
    if (imu_is_static) VisInsEKF_UpdateZeroVel();
    /* 5) EKF 世界速度回写,供下面机体速度映射 + PNG V_c + Vofa */
    IMU_Data.Velocity[World][NOW][X] = vins_out.v_world[X];
    IMU_Data.Velocity[World][NOW][Y] = vins_out.v_world[Y];
    IMU_Data.Velocity[World][NOW][Z] = vins_out.v_world[Z];
    /*机体速度*/
    IMU_Data.Velocity[Body][NOW][X] = IMU_Data.R_matrix_T[0][0] * IMU_Data.Velocity[World][NOW][X] +
                                      IMU_Data.R_matrix_T[0][1] * IMU_Data.Velocity[World][NOW][Y] +
                                      IMU_Data.R_matrix_T[0][2] * IMU_Data.Velocity[World][NOW][Z];
    IMU_Data.Velocity[Body][NOW][Y] = IMU_Data.R_matrix_T[1][0] * IMU_Data.Velocity[World][NOW][X] +
                                      IMU_Data.R_matrix_T[1][1] * IMU_Data.Velocity[World][NOW][Y] +
                                      IMU_Data.R_matrix_T[1][2] * IMU_Data.Velocity[World][NOW][Z];
    IMU_Data.Velocity[Body][NOW][Z] = IMU_Data.R_matrix_T[2][0] * IMU_Data.Velocity[World][NOW][X] +
                                      IMU_Data.R_matrix_T[2][1] * IMU_Data.Velocity[World][NOW][Y] +
                                      IMU_Data.R_matrix_T[2][2] * IMU_Data.Velocity[World][NOW][Z];
    /* 6) 速度方向角(世界系):供速度外环 PID 使用 */
    {
        float vx = vins_out.v_world[X], vy = vins_out.v_world[Y], vz = vins_out.v_world[Z];
        float v_horiz = sqrtf(vx*vx + vy*vy);
        IMU_Data.Vel_Dir[PITCH] = RAD2DEG(atan2f(vz, v_horiz));   /* 仰角 */
        IMU_Data.Vel_Dir[YAW]   = RAD2DEG(atan2f(vx, vy));        /* 方位角 */
    }

#endif
    /* === 弹道角 γ:姿态前向估计(新变量 gamma_pitch_fwd_deg,不动旧速度版 gamma_pitch_deg) ===
     * 无动力俯冲弹速度矢量≈机体纵轴前向;纯积分世界速度无外部观测、飞行段(ZUPT 停)线性漂不可用,故弃用速度版。
     * 改用姿态前向(机体前向[0,1,0]映到世界系=R_matrix_T 第1行)的仰角:只依赖姿态(陀螺 coast,几秒漂<1°),
     * 不漂、地面可验证。俯冲时 fwd_z<0→γ<0,与 PITCH 同号。注:此 γ 恒等于 Euler[PITCH](都是前向轴仰角),
     * 不含迎角信息→末制导迎角补偿退化为常值 AOA_TRIM。本段独立于上面已 #if 0 的速度链路,始终编译执行。*/
    {
        float fwd_x = IMU_Data.R_matrix_T[1][0];
        float fwd_y = IMU_Data.R_matrix_T[1][1];
        float fwd_z = IMU_Data.R_matrix_T[1][2];
        gamma_pitch_fwd_deg = RAD2DEG(atan2f(fwd_z, sqrtf(fwd_x*fwd_x + fwd_y*fwd_y)));
    }


#if 1   /*四元数解算及欧拉角转换*/

    /*四元数积分*/
    q0 -= (0.5f*dT*(q1*gx + q2*gy + q3*gz));
    q1 += (0.5f*dT*(q0*gx + q2*gz - q3*gy));
    q2 += (0.5f*dT*(q0*gy - q1*gz + q3*gx));
    q3 += (0.5f*dT*(q0*gz + q1*gy - q2*gx));
    /*四元数更归一化*/
    float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (q_norm<0.001f)
    {
        q0 = 1;
        q1 = 0;
        q2 = 0;
        q3 = 0;
    }
    else
    {
        q0 /=q_norm;
        q1 /=q_norm;
        q2 /=q_norm;
        q3 /=q_norm;
    }
    /*欧拉角转换 (机体系 X=右/Y=前/Z=上, q=[q0..q3]=[w,x,y,z]):
      PITCH 绕X(右) 抬头+; ROLL 绕Y(前) 右滚+; YAW 绕Z(上) 右偏+。已数值核验三轴极性。*/
    IMU_Data.Euler[NOW][PITCH]  = asinf (2.0f*(q2*q3 + q0*q1));
    IMU_Data.Euler[NOW][ROLL]   = atan2f(2.0f*(q0*q2 - q1*q3), 1.0f-2.0f*(q1*q1+q2*q2));
    IMU_Data.Euler[NOW][YAW]    = atan2f(2.0f*(q1*q2 - q0*q3), 1.0f-2.0f*(q1*q1+q3*q3));

    IMU_Data.Euler[NOW][PITCH]  =   RAD2DEG(IMU_Data.Euler[NOW][PITCH]);
    IMU_Data.Euler[NOW][ROLL]   =   RAD2DEG(IMU_Data.Euler[NOW][ROLL]);
    IMU_Data.Euler[NOW][YAW]    =   RAD2DEG(IMU_Data.Euler[NOW][YAW]);
#endif
#if 1   /*历史值记录，数据更新*/
    // if (IMU_Data.Euler[NOW][PITCH] < 0)
    // {
    //     IMU_Data.Euler[NOW][PITCH] += 180;
    // }
    // if (IMU_Data.Euler[NOW][ROLL] < 0)
    // {
    //     IMU_Data.Euler[NOW][ROLL]  += 360;
    // }
    // if (IMU_Data.Euler[NOW][YAW] < 0 )
    // {
    //     IMU_Data.Euler[NOW][YAW]   += 360;
    // }
    IMU_Data.Q[NOW][0]  = q0 ;
    IMU_Data.Q[NOW][1]  = q1 ;
    IMU_Data.Q[NOW][2]  = q2 ;
    IMU_Data.Q[NOW][3]  = q3 ;
    IMU_Data.A_Normed[NOW][X]  = ax_normed  ;
    IMU_Data.A_Normed[NOW][Y]  = ay_normed  ;
    IMU_Data.A_Normed[NOW][Z]  = az_normed  ;
    /*更新*/
    IMU_Data.A_theory[NOW][X] = tx;
    IMU_Data.A_theory[NOW][Y] = ty;
    IMU_Data.A_theory[NOW][Z] = tz;
    for (int i = 0;i<4;i++)
    {
        IMU_Data.Q[LAST][i]=IMU_Data.Q[NOW][i];
    }
    for (int i = 0;i<3;i++)
    {
        Surface.current_angle_Euler[NOW][i]=IMU_Data.Euler[NOW][i];
        IMU_Data.Euler   [LAST][i]=IMU_Data.Euler[NOW][i];
        IMU_Data.A_theory[LAST][i] = IMU_Data.A_theory[NOW][i];
        // IMU_Data.Velocity[World][LAST][i] = IMU_Data.Velocity[World][NOW][i];   /* 速度链路 #if 0 停用,历史拷贝一并注释省算力 */
        // IMU_Data.Velocity[Body][LAST][i] = IMU_Data.Velocity[Body][NOW][i];
        // IMU_Data.A_World [LAST][i] = IMU_Data.A_World[NOW][i];
    }
#endif
}
#if 1
/*imu底层通信*/
void BMX055_CS_ACC_Select(void)
{
    HAL_GPIO_WritePin( ACC_CS_PORT ,ACC_CS_PIN, GPIO_PIN_RESET);
}

void BMX055_CS_GYR_Select(void)
{
    HAL_GPIO_WritePin( GYR_CS_PORT ,GYR_CS_PIN, GPIO_PIN_RESET);
}

void BMX055_CS_MAG_Select(void)
{
    HAL_GPIO_WritePin( MAG_CS_PORT ,MAG_CS_PIN, GPIO_PIN_RESET);
}

void BMX055_CS_Select(uint8_t sensor)
{
    switch(sensor)
    {
        case ACC:
        {
            BMX055_CS_ACC_Select();
        }break;

        case GYR:
        {
            BMX055_CS_GYR_Select();
        }break;

        case MAG:
        {
            BMX055_CS_MAG_Select();
        }break;
    }
}

void BMX055_CS_ACC_Free(void)
{
    HAL_GPIO_WritePin( ACC_CS_PORT ,ACC_CS_PIN, GPIO_PIN_SET);
}

void BMX055_CS_GYR_Free(void)
{
    HAL_GPIO_WritePin( GYR_CS_PORT ,GYR_CS_PIN, GPIO_PIN_SET);
}

void BMX055_CS_MAG_Free(void)
{
    HAL_GPIO_WritePin( MAG_CS_PORT ,MAG_CS_PIN, GPIO_PIN_SET);
}
void BMX055_CS_HIGH_ACC_Free(void)
{
    HAL_GPIO_WritePin( HIGH_ACC_CS_PORT ,HIGH_ACC_CS_PIN, GPIO_PIN_SET);
}
void ALL_CS_Free(void)
{
    BMX055_CS_ACC_Free();
    BMX055_CS_GYR_Free();
    BMX055_CS_MAG_Free();
	  BMX055_CS_HIGH_ACC_Free();
}
void BMX055_CS_Free(uint8_t sensor)
{
    switch(sensor)
    {
        case ACC:
        {
            BMX055_CS_ACC_Free();
        }break;

        case GYR:
        {
            BMX055_CS_GYR_Free();
        }break;

        case MAG:
        {
            BMX055_CS_MAG_Free();
        }break;
    }
}
void BMX055_Read(uint8_t Sensor,uint8_t Reg_Addr)
{

        BMX055_CS_Select(Sensor);

        uint8_t tx_buf[7] = {0};
        uint8_t rx_buf[7] = {0};
        tx_buf[0] = Reg_Addr|0x80;


        HAL_SPI_TransmitReceive( &hspi2,tx_buf,rx_buf,7 ,5000);


        BMX055_CS_Free(Sensor);

          switch(Sensor)
            {
                case ACC:
                {
                    /* 寄存器 rx(2,1)=accX rx(4,3)=accY rx(6,5)=accZ → 机体系 X=右/Y=前/Z=上;
                       符号见 IMU.h ACC_SIGN_*(默认: 前=+chipX, 右=+chipY, 上=−chipZ 使静止+g)。*/
                    IMU_Data.A[NOW][Y] = ACC_SIGN_Y * (int16_t)(rx_buf[2]<<8|rx_buf[1]) * ACC_LSB_16G;
                    IMU_Data.A[NOW][X] = ACC_SIGN_X * (int16_t)(rx_buf[4]<<8|rx_buf[3]) * ACC_LSB_16G;
                    IMU_Data.A[NOW][Z] = ACC_SIGN_Z * (int16_t)(rx_buf[6]<<8|rx_buf[5]) * ACC_LSB_16G;
                    for (int k = 0; k < 3; k++)
                    {
                        if (isnan(IMU_Data.A[NOW][k]) || fabsf(IMU_Data.A[NOW][k]) > ACC_SAT_G)
                            IMU_Data.A[NOW][k] = IMU_Data.A[LAST][k];
                    }
                    IMU_Data.A[NOW][X] = KalmanFilter(&IMU_Kalman_Filter[ACC][X],IMU_Data.A[NOW][X],ACC_KF_Q,ACC_KF_R);
                    IMU_Data.A[NOW][Y] = KalmanFilter(&IMU_Kalman_Filter[ACC][Y],IMU_Data.A[NOW][Y],ACC_KF_Q,ACC_KF_R);
                    IMU_Data.A[NOW][Z] = KalmanFilter(&IMU_Kalman_Filter[ACC][Z],IMU_Data.A[NOW][Z],ACC_KF_Q,ACC_KF_R);
                    for (int k = 0; k < 3; k++) IMU_Data.A[LAST][k] = IMU_Data.A[NOW][k];

                }break;
                case GYR:
                {
                    IMU_Data.G[NOW][ROLL ] = ((int16_t)(rx_buf[2]<<8|rx_buf[1])) / GYRO_LSB_2000DPS;
                    IMU_Data.G[NOW][PITCH] = ((int16_t)(rx_buf[4]<<8|rx_buf[3])) / GYRO_LSB_2000DPS;
                    IMU_Data.G[NOW][YAW  ] = ((int16_t)(rx_buf[6]<<8|rx_buf[5])) / GYRO_LSB_2000DPS;
                    for (int k = 0; k < 3; k++)
                    {
                        if (isnan(IMU_Data.G[NOW][k]) || fabsf(IMU_Data.G[NOW][k]) > GYRO_SAT_DPS)
                            IMU_Data.G[NOW][k] = IMU_Data.G[LAST][k];
                    }
                    if (IMU_Data.calib_done!=0)
                    {
                        for (int k = 0; k < 3; k++)
                            IMU_Data.G[NOW][k] -= IMU_Data.G_Offset[k];
                    }
                    IMU_Data.G[NOW][PITCH] = KalmanFilter(&IMU_Kalman_Filter[GYR][PITCH],IMU_Data.G[NOW][PITCH],GYR_KF_Q,GYR_KF_R);
                    IMU_Data.G[NOW][ROLL ] = KalmanFilter(&IMU_Kalman_Filter[GYR][ROLL ],IMU_Data.G[NOW][ROLL ],GYR_KF_Q,GYR_KF_R);
                    IMU_Data.G[NOW][YAW  ] = KalmanFilter(&IMU_Kalman_Filter[GYR][YAW  ],IMU_Data.G[NOW][YAW  ],GYR_KF_Q,GYR_KF_R);
                    /* current_gyro_Euler 改到 IMU_Attitude_Algorithm 设置(按机体系/用户极性,yaw 取负);
                       此处只更新原始量(deg)与 G_Rad(rad),供四元数/PNG/遥测使用。*/
                    for (int i = 0;i<3;i++)
                    {
                        IMU_Data.G_Rad[NOW][i] = DEG2RAD(IMU_Data.G[NOW][i]);
                        IMU_Data.G[LAST][i] = IMU_Data.G[NOW][i];
                    }
                    receiveflag++;
                }break;
                case MAG:
                {
                    IMU_Data.M[NOW][X] = -(int16_t)((rx_buf[2] << 5) | (rx_buf[1]>>3));
                    IMU_Data.M[NOW][Y] = -(int16_t)((rx_buf[4] << 5) | (rx_buf[3]>>3));
                    IMU_Data.M[NOW][Z] = -(int16_t)((rx_buf[6] << 7) | (rx_buf[5]>>1))*0.3125f;
                }break;

            }

}

void BMX055_Write(uint8_t Sensor,uint8_t Reg_Addr,uint8_t data)
{

        BMX055_CS_Select(Sensor);

        for (volatile int i = 0; i < 10; i++);

        uint8_t tx_buf[2] = {Reg_Addr,data};
        uint8_t rx_buf[2] = {0,0};

        HAL_SPI_TransmitReceive( &hspi2,tx_buf,rx_buf,2 ,5000);

        BMX055_CS_Free(Sensor);

}

void BMX055_Init_Acc_Gyr(void)
{
     //配置范围与ODR+-16g,250Hz
     BMX055_Write( ACC,0x11,0x00 );
     BMX055_Write( ACC,0X0F,0X0C );
     BMX055_Write( ACC,0x10,0x1F );
     //+-2000.400hz
     BMX055_Write( GYR,0x11,0x00 );
     BMX055_Write( GYR,0x0F,0x00 );
     BMX055_Write( GYR,0x10,0x02 );

}
void BMX055_Init_Mag(void)
{
    BMX055_Write(MAG, 0x4B, 0x01);
    // HAL_Delay(10); // 手册要求：NVM数据加载延迟≥5ms
    BMX055_Write(MAG, 0x4C, 0x38);
    // 预期返回值：0x40，如果不是则通信有问题
}

void IMU_Data_Read(void)
{
    BMX055_Read(ACC,0X02);
    BMX055_Read(GYR,0X02);
     
    // BMX055_Read(MAG,0X42);
    // 应用三维卡尔曼滤波 (使用修改后的函数)
}

void IMU_Init(void)
{
    BMX055_Init_Acc_Gyr();

    // BMX055_Init_Mag();
}

/*
 * 静态零偏校准:上电后保持飞镖静止 3 秒,采样均值作为 gyro/acc 零偏。
 * 校准期间 calib_done=0,IMU_Data_Read 不减偏,保证采样的是真实零位。
 *
 * 关键改进:校准期间同步运行姿态算法,让 R_matrix_T 收敛后再计算零偏,
 * 确保零偏与重力投影自洽,静止时 A_World≈0。
 */
void IMU_Calibrate(void)
{
    const uint16_t WARMUP_MS = 1000;   /* 姿态收敛等待 */
    const uint16_t SAMPLE_MS = 2000;   /* 零偏采样 */
    float gsum[3] = {0}, asum[3] = {0};

    IMU_Data.calib_done = 0;

    /* Phase 1: 姿态收敛期 — 持续读传感器+解算姿态,不采零偏 */
    for (uint16_t n = 0; n < WARMUP_MS; n++)
    {
        IMU_Data_Read();
        IMU_Attitude_Algorithm();
        osDelay(1);
    }

    /* Phase 2: 零偏采样期 — 姿态已收敛,R_col3≈真重力方向,此时采零偏与去重力自洽 */
    for (uint16_t n = 0; n < SAMPLE_MS; n++)
    {
        IMU_Data_Read();
        IMU_Attitude_Algorithm();
        for (int i = 0; i < 3; i++)
        {
            gsum[i] += IMU_Data.G[NOW][i];
            asum[i] += IMU_Data.A[NOW][i];
        }
        osDelay(1);
    }

    /* 零偏 = 均值 − R_col3(姿态已收敛,此即真重力方向)
     * 与去重力公式 a_no_grav = gravity*((a_raw−A_Offset)−R_col3) 完全自洽:
     * 静止时 a_raw=A_Offset+R_col3 → a_no_grav=0 → A_World=0 → 速度不漂。*/
    float amean[3];
    for (int i = 0; i < 3; i++)
    {
        IMU_Data.G_Offset[i] = gsum[i] / (float)SAMPLE_MS;
        amean[i] = asum[i] / (float)SAMPLE_MS;
    }
    /* R_col3 = R_matrix_T 第3列(姿态已收敛,取最新值) */
    IMU_Data.A_Offset[X] = amean[X] - IMU_Data.R_matrix_T[0][2];
    IMU_Data.A_Offset[Y] = amean[Y] - IMU_Data.R_matrix_T[1][2];
    IMU_Data.A_Offset[Z] = amean[Z] - IMU_Data.R_matrix_T[2][2];

    IMU_Data.calib_done = 1;
}



#endif
