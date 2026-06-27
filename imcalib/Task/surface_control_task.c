/******************************************************************************
*** @File           : surface_control_task.c
*** @Description    : None
*** @Attention      : None
*** @Author         : ynzz33
*** @Date           : 2025/3/2
*** @版权归属:
***                  ██╗███╗   ███╗ ██████╗ █████╗
***                  ██║████╗ ████║██╔════╝██╔══██╗
***                  ██║██╔████╔██║██║     ███████║
***                  ██║██║╚██╔╝██║██║     ██╔══██║
***                  ██║██║ ╚═╝ ██║╚██████╗██║  ██║
***                  ╚═╝╚═╝     ╚═╝ ╚═════╝╚═╝  ╚═╝
******************************************************************************/
/* 头文件 include(s) BEGIN */
#include "surface_control_task.h"

#include "buzzer.h"
#include "CallBack_Task.h"
#include "pid.h"
#include "adrc.h"           /* LADRC 线性自抗扰控制器(文件名仍 adrc.*) */
#include "lqr.h"            /* LQR 姿态控制器(6态→4舵一步解算，含混控)；未编译需手动加入工程 */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "IMU.h"
#include "tim.h"
#include "filter.h"
#include "PNG_Task.h"
#include "vision_ins.h"

/* 头文件 include(s) END */
/*---------------------------------------------------------------------------*/
/* 静态变量定义 static variable(s) BEGIN */

/* 静态变量定义 static variable(s) END */
/*---------------------------------------------------------------------------*/
/* 全局变量定义 global variable(s) BEGIN */


uint8_t DART_TYPE = VECTOR_NOZZLE;
uint8_t Guidance_State;
Surface_t Surface;
Self_Text_t Self_Text;
uint8_t Wing_Servo_Control_Flag = 1,Stable_Flag = 0;//舵机控制标志位
int8_t Stable_Cnt = 0;
uint16_t current_tick;


/* === 速度矢量追踪制导:三级串级 ===
 * 外环:速度方向追目标方向 → 输出 body 目标角
 * 中环:镖头追 body 目标角 → 输出 rate_cmd (复用角度环 PID)
 * 内环:角速度追 rate_cmd → 输出舵偏 (复用 gyro 环 PID)
 * vel_pursuit_mode=0 时退化为原两级 PID。*/
uint8_t vel_pursuit_mode = 0;
/* LADRC 控制选择：0=全 PID(安全默认), 1=三轴全 LADRC, 3=仅 Roll 用 LADRC(本次先试)
 * LADRC 旋钮在 ladrc_ctrl[轴].{wc,wo,b0}，可用调试器 Watch 在线改(见 adrc.c)。*/
uint8_t ladrc_mode = 0;   /* ★ 先在 roll 上试 LADRC，pitch/yaw 仍走 PID；要回全 PID 就改 0 */

/* LQR 控制选择：0=关(走 PID/LADRC，安全默认), 1=用 LQR 一步解算(6态→4舵，含混控，绕过 PID 与 Servo_Mix_*)
 * K 矩阵在 lqr.c 的 dart_lqr_K[4][6](MATLAB 粘贴区)；符号/换序见 lqr.c。优先级高于 ladrc_mode/vel_pursuit_mode。
 * ★ 未编译/待台架：上车前务必按指南 §9 逐轴阶跃验符号(飞镖一次性，G 符号反=正反馈)。*/
uint8_t lqr_mode = 1;

/* === 控制分配(混控)状态:整合为单一结构体 Alloc(原 Alloc_Mode/Prio/B、alloc_*、servo_lat_scale 散落全局) === */
Alloc_t Alloc = {
    .Mode = 2,                                 /* 默认最小能量;1=三轴限幅 0=旧PitchPriority对照 */
    .Prio = { ROLL, YAW, PITCH },              /* 交付A(Mode1)逐级优先级:[0]最高=roll>yaw>pitch;Mode2 不读、其饱和优先级硬编码 pitch>横侧 */
    /* 舵效矩阵 B (3x4): τ=B·u, 行[pitch,roll,yaw] 列[UL,UR,DR,DL]. 默认理想 X 阵(BBᵀ=4I).
     * 台架辨识替换法:固定其余三舵=0,给第 j 舵单位阶跃 Δu_j,读三轴力矩响应 Δτ_p/r/y,
     * 令 B[0..2][j]=Δτ_{p,r,y}/Δu_j;改这里即可换实测值,无需改算法,奇异自动退回理想阵闭式. */
    .B = {
        { -1.0f, -1.0f, -1.0f, -1.0f },        /* pitch (四片同号,逻辑列=−1, SIGN 翻转后=真俯仰) */
        { +1.0f, -1.0f, -1.0f, +1.0f },        /* roll  */
        { -1.0f, +1.0f, -1.0f, +1.0f },        /* yaw   */
    },
    .v_scale = 1.0f, .p_scale = 1.0f, .lat_scale = 1.0f,   /* 其余(u0/u_out/alpha/u0_span/infeasible/singular_flag)默认 0 */
};


float   vision_los_final[2][3],vision_los_current[3];   /* 末制导世界系视线终点:视觉新帧锁存,target 每 tick 斜坡逼近(方案3) */
float   vision_los_rate[3] = {0};                    /* 末制导世界系惯性视线率λ̇(°/s):视觉帧间差分,PN超前用;丢帧保持/丢目标清0 */
/* ★封存:下列 4 个 Vofa 观测全局随末制导增益调度/俯冲下限函数(本文件 #if 0 块)一并停用,确认无其它消费者 */
// float   pitch_dive_floor = 0.0f, closeness_s = 0.0f; /* Vofa:末制导俯仰俯冲下限θ_floor° / 接近度s∈[0,1] */
// float   yaw_distance_gain = 1.0f;                   /* Vofa:末制导 yaw 距离面积增益 */
// float   pitch_distance_gain = 1.0f;                  /* Vofa:末制导 pitch 距离面积增益(远端小、近端大) */
float pitch_control_limit_deg = -0.0f;//开始放开pitch控制的角度限制
uint8_t pitch_glide_mode = 1;   /* 0=旧 pitch_control_limit 门限;1=主动滑翔→扎(本次新增),可 Watch 在线切回 0 做 A/B */
float   pitch_glide_target = 0.0f, pitch_glide_blend = 0.0f;
/* 全局变量定义 global variable(s) END */
/*---------------------------------------------------------------------------*/
#if 1

void Velocity_Pursuit_Cale(float delta_time)
{
    /* --- 外环:速度方向追目标方向 → body 目标角 --- */
    float body_cmd_pitch = Surface.target_angle_Euler[NOW][PITCH]; /* 默认直通 */
    float body_cmd_yaw   = Surface.target_angle_Euler[NOW][YAW];

    if (Guidance_State > Stable)
    {
        body_cmd_pitch = pid_calc(&vel_pursuit_pid[PITCH],
                                  IMU_Data.Vel_Dir[PITCH],
                                  Surface.target_angle_Euler[NOW][PITCH],
                                  delta_time);
    }
    body_cmd_yaw = pid_calc(&vel_pursuit_pid[YAW],
                            IMU_Data.Vel_Dir[YAW],
                            Surface.target_angle_Euler[NOW][YAW],
                            delta_time);

    /* --- 中环:镖头追外环输出 → rate_cmd --- */
    if (Guidance_State > Stable)
    {
        temp[PITCH] = pid_calc(&surface_control_pid[Angle][PITCH],
                               Surface.current_angle_Euler[NOW][PITCH],
                               body_cmd_pitch,
                               delta_time);
    }
    temp[YAW] = pid_calc(&surface_control_pid[Angle][YAW],
                          Surface.current_angle_Euler[NOW][YAW],
                          body_cmd_yaw,
                          delta_time);
    temp[ROLL] = pid_calc(&surface_control_pid[Angle][ROLL],
                           Surface.current_angle_Euler[NOW][ROLL],
                           Surface.target_angle_Euler[NOW][ROLL],
                           delta_time);

    /* --- 内环:角速度追 rate_cmd → 舵偏 --- */
    Surface.output_gyro_Euler[NOW][PITCH] = pid_calc(&surface_control_pid[Gyro][PITCH],
        Surface.current_gyro_Euler[NOW][PITCH], temp[PITCH], delta_time);
    Surface.output_gyro_Euler[NOW][YAW] = pid_calc(&surface_control_pid[Gyro][YAW],
        Surface.current_gyro_Euler[NOW][YAW], temp[YAW], delta_time);
    Surface.output_gyro_Euler[NOW][ROLL] = pid_calc(&surface_control_pid[Gyro][ROLL],
        Surface.current_gyro_Euler[NOW][ROLL], temp[ROLL], delta_time);
}

void Euler_Updata(void)
{
    //Updata
    for (int i = 0;i<3;i++)
    {
        Surface.target_angle_Euler [LLAST][i] = Surface.target_angle_Euler [LAST][i];
        Surface.target_angle_Euler [LAST ][i] = Surface.target_angle_Euler [NOW ][i];
        Surface.current_angle_Euler[LLAST][i] = Surface.current_angle_Euler[LAST][i];
        Surface.current_angle_Euler[LAST ][i] = Surface.current_angle_Euler[NOW ][i];
    }
}
void Servo_Updata(void)
{
    /* X 翼跑 4 个舵机,十字翼跑 3 个;output_angle_Servo 共用 [3][4] 缓存 */
    int n = (DART_TYPE == VECTOR_NOZZLE) ? 4 : 3;
    for (int i = 0; i < n; i++)
    {
        Surface.output_angle_Servo[LLAST][i] = Surface.output_angle_Servo[LAST][i];
        Surface.output_angle_Servo[LAST ][i] = Surface.output_angle_Servo[NOW ][i];
    }
}                    
void Data_Updata(void)
{
    Euler_Updata();
    Servo_Updata();
    Surface.target_angle_Euler[LAST][YAW] = Surface.target_angle_Euler[NOW][YAW];
}
#if 1 //DART_TYPE == VECTOR_NOZZLE
/* X 翼 4 个舵机 PWM 写入:接 TIM3 CH2 TIM4 CH2-CH4 (PB6/PB7/PB8/PB9)
 * 输入 data 单位是度(±90 内), 内部映射到 PWM 微秒并做 ZERO 偏置 + 限幅
 */
void Wing_UL_Control(float data)//硬件原因导致左上舵机接在了 TIM3 上，所以单独写函数控制
{
    Surface.Finally_Angle[NOW][UP_LEFT]  = Servo_UL_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Surface.Finally_Angle[NOW][UP_LEFT]);
    
    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_UL_ZERO);
}
void Wing_UR_Control(float data)
{ 
    Surface.Finally_Angle[NOW][UP_RIGHT] = Servo_UR_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_UR_Channel, Surface.Finally_Angle[NOW][UP_RIGHT]);
    
    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_UR_ZERO);
}
void Wing_DL_Control(float data)
{
    Surface.Finally_Angle[NOW][DOWN_LEFT]  = Servo_DL_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_DL_Channel, Surface.Finally_Angle[NOW][DOWN_LEFT]);

    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_DR_ZERO);
}
void Wing_DR_Control(float data)
{
    Surface.Finally_Angle[NOW][DOWN_RIGHT] = Servo_DR_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_DR_Channel, Surface.Finally_Angle[NOW][DOWN_RIGHT]);

    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_DL_ZERO);
}

void Wing_Control_VECTOR_NOZZLE(void)
{
    if (Guidance_State == Terminal||Guidance_State == Self_Text_State||Stable_Flag ==1 )
    // if (Guidance_State == Self_Text_State)
    {
        Wing_UL_Control(Surface.output_angle_Servo[NOW][UP_LEFT]    );
        Wing_UR_Control(Surface.output_angle_Servo[NOW][UP_RIGHT]   );
        Wing_DL_Control(Surface.output_angle_Servo[NOW][DOWN_LEFT]  );
        Wing_DR_Control(Surface.output_angle_Servo[NOW][DOWN_RIGHT] );
    }
    else
    {
        // 末端失去目标或制导结束,舵面回中
        Wing_UL_Control(0.0f);
        Wing_UR_Control(0.0f);
        Wing_DL_Control(0.0f);
        Wing_DR_Control(0.0f);
        return;
    }
        // Wing_UL_Control(0.0f);
        // Wing_UR_Control(0.0f);
        // Wing_DL_Control(0.0f);
        // Wing_DR_Control(0.0f);
}

#endif
void Wing_Control(void)
{
    if (Wing_Servo_Control_Flag == 1)
    {      
        if(Guidance_State == Self_Text_State)
        {
            Surface.output_angle_Servo[NOW][UP_LEFT]    = 0;
            Surface.output_angle_Servo[NOW][UP_RIGHT]   = 0;
            Surface.output_angle_Servo[NOW][DOWN_LEFT]  = 0;
            Surface.output_angle_Servo[NOW][DOWN_RIGHT] = 0; 
        }
        else if(Guidance_State == Start&&IMU_Data.calib_done==1)
        {
            Surface.output_angle_Servo[NOW][UP_LEFT]    = 0;
            Surface.output_angle_Servo[NOW][UP_RIGHT]   = 0;
            Surface.output_angle_Servo[NOW][DOWN_LEFT]  = 0;
            Surface.output_angle_Servo[NOW][DOWN_RIGHT] = 0; 
        }
        Wing_Control_VECTOR_NOZZLE();
    }
}

#if 0   /* ★封存:末制导视线角归一化/目标斜坡/俯冲下限/距离增益——LPF段简化为直接用锁存视线后,主路径已不再调用这些函数;保留可回退(改回 #if 1 即恢复) */
/* === 视线角半径归一化 ===
 * 远处引导灯 blob 半径小(~5px),同样像素偏移对应更大实际角度;
 * 近处 blob 半径大(~30px),同样像素偏移对应更小实际角度。
 * 归一化到 REF_RADIUS(15px):normalized = angle × (REF_RADIUS / radius)
 * → 控制增益不再随距离变化,同一像素偏移始终对应同一归一化角。
 *
 * 输入:angle_deg 视觉原始视线角(度), radius_px blob等效半径(像素)
 * 输出:归一化后的视线角(度),radius<REF_RADIUS_MIN 时原样返回(防除零) */
static float Vision_Angle_Normalize(float angle_deg, uint16_t radius_px)
{
    if (radius_px < REF_RADIUS_MIN) return angle_deg;  /* 半径太小/无数据,不补偿 */
    float r = (float)radius_px;
    
    return angle_deg * (REF_RADIUS / r);
}

/* 目标斜坡逼近(速率限制):把 cur 朝 target 每拍最多移动 max_step,把视觉阶跃摊成斜坡(方案3)。
 * wrap=1:差值按角度环绕到最短弧(YAW 是 atan2 周期角);wrap=0:普通(PITCH 是 asin 不环绕)。*/
static float Target_Slew(float cur, float target, float max_step, uint8_t wrap)
{
    float err = target - cur;
    if (wrap) err = Angle_Wrap_180(err);
    if (err >  max_step) err =  max_step;
    if (err < -max_step) err = -max_step;
    return cur + err;
}

/* === 末制导俯仰能量管理:随接近度放开的最陡俯冲下限 θ_floor ===
 * 钳"机体俯仰目标 ≥ θ_floor"=不许比 θ_floor 更陡。两条"≥"下限取较不陡(较大)者:
 *  ① 调度限幅 L_sched:接近度 s∈[0,1] 从远(s=0,浅滑翔保射程)线性放开到近(s=1,期望入射角)。
 *     s 分段合成(面积+距离一起用,数据来自视觉 0x5B 包):远段用距离 dist_cm(标定准、连续,s:0→DIVE_SCHED_SWITCH),
 *     近段用面积 area(blob 大、近场更可靠,s:DIVE_SCHED_SWITCH→1),dist_cm 决定走哪段;两包都无(dist_cm=0)
 *     退化按弹道角 γ 自调度(γ 随重力变陡→逐步放开)。
 *  ② 迎角限幅:θ ≥ γ−AOA_MARGIN,即鼻子不许指到比速度方向(γ)再低 AOA_MARGIN 以上——大负迎角
 *     →掉升力→因重力掉高度损能,正是"过早陡俯冲损耗能量"的物理根因;此条防距离/面积被骗时仍守住。
 * 终端 γ≈θ≈入射角时迎角 θ−γ→0=正向撞击。Vofa 观测 closeness_s / pitch_dive_floor。*/
static float Pitch_Dive_Floor(uint16_t dist_cm, uint16_t area)
{
    float s_dist, s_area, s;
    if (dist_cm == 0)                              /* 无 0x5B 距离/面积包 → 退化按弹道角 γ 自调度 */
        // s = (gamma_pitch_deg - GAMMA_FAR_DEG) / (PITCH_INCIDENT_DEG - GAMMA_FAR_DEG);
        s = 0.0f;   /* 直接退化到最保守的远段限幅,不放开 γ 自调度了(实测 γ 不够准),待实测再改回按 γ 调度 */
    else if ((float)dist_cm > DIST_NEAR_CM)        /* 远段:距离调度,s 由远(s=0)线性升到切换点 */
        s = DIVE_SCHED_SWITCH * (DIST_ACQUIRE_CM - (float)dist_cm) / (DIST_ACQUIRE_CM - DIST_NEAR_CM);
    else                                           /* 近段:面积调度,s 由切换点升到近(s=1) */
        s = DIVE_SCHED_SWITCH + (1.0f - DIVE_SCHED_SWITCH) * ((float)area - AREA_NEAR) / (AREA_IMPACT - AREA_NEAR);
    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;
    closeness_s = s;

    float L_sched = PITCH_DIVE_LIMIT_FAR_DEG + s * (PITCH_INCIDENT_DEG - PITCH_DIVE_LIMIT_FAR_DEG);
    float L_aoa   = gamma_pitch_fwd_deg - AOA_MARGIN_DEG;
    float dive_floor = (L_sched > L_aoa) ? L_sched : L_aoa;   /* 两个"≥"下限取较不陡(较大)者 */
    pitch_dive_floor = dive_floor;
    return dive_floor;
}

/* === [旧函数已注释] 末制导 YAW 距离面积增益 ===
 * 原函数使用距离+面积双段合成,调试中出现异常增益值,注释保留供参考。
 * 替换为纯距离线性插值版 Yaw_Gain_ByDist()。
 *
static float Yaw_Distance_Area_Gain(uint16_t dist_cm, uint16_t area)
{
    ... (原代码已禁用)
}
*/

/* === [旧函数已注释] 末制导 PITCH 距离面积增益 ===
 * 同上,替换为纯距离版 Pitch_Gain_ByDist()。
 *
static float Pitch_Distance_Area_Gain(uint16_t dist_cm, uint16_t area)
{
    ... (原代码已禁用)
}
*/

/* ============================================================================
 * Yaw_Gain_ByDist — 纯距离线性增益(YAW 通道)
 *
 * 原理:同样像素误差,远处对应更大实际角度→需要更大增益补偿;近处角度误差小→减小增益防过冲。
 * 只用距离 dist_cm,不用面积,消除面积异常值导致的增益跳变。
 *
 * 输入:
 *   dist_cm  视觉测距(cm),0=无数据
 *
 * 输出:
 *   增益值 ∈ [YAW_GAIN_NEAR, YAW_GAIN_FAR], 全局 yaw_distance_gain 同步更新
 *
 * 分段:
 *   dist > YAW_GAIN_ENABLE_DIST(800cm): 返回 1.0(远处不调整)
 *   dist == 0:                          返回 1.0(无数据不调整)
 *   DIST_ACQUIRE_CM(700) → DIST_NEAR_CM(470): 远→近线性插值
 *   dist < DIST_NEAR_CM(470):           夹到近端增益
 * ============================================================================ */
static float Yaw_Gain_ByDist(uint16_t dist_cm)
{
    float gain;

    if (dist_cm == 0)
    {
        /* 无距离数据:保持默认增益,不调整 */
        gain = 1.0f;
    }
    else if ((float)dist_cm > YAW_GAIN_ENABLE_DIST)
    {
        /* 超过启用阈值:远处不调整 */
        gain = 1.0f;
    }
    else if ((float)dist_cm >= DIST_ACQUIRE_CM)
    {
        /* 刚识别到但很远(DIST_ACQUIRE ~ ENABLE):用远端增益 */
        gain = YAW_GAIN_FAR;
    }
    else if ((float)dist_cm <= DIST_NEAR_CM)
    {
        /* 已很近:用近端增益 */
        gain = YAW_GAIN_NEAR;
    }
    else
    {
        /* 中间段:距离从 DIST_ACQUIRE 线性降到 DIST_NEAR,增益从 FAR 线性降到 NEAR */
        float t = (DIST_ACQUIRE_CM - (float)dist_cm) / (DIST_ACQUIRE_CM - DIST_NEAR_CM);
        /* t ∈ (0,1),t=0→远端,t=1→近端 */
        gain = YAW_GAIN_FAR + t * (YAW_GAIN_NEAR - YAW_GAIN_FAR);
    }

    /* 最终硬夹:任何浮点异常都不可能越界 */
    if (gain < YAW_GAIN_NEAR) gain = YAW_GAIN_NEAR;
    if (gain > YAW_GAIN_FAR)  gain = YAW_GAIN_FAR;

    yaw_distance_gain = gain;
    return gain;
}

/* ============================================================================
 * Pitch_Gain_ByDist — 纯距离线性增益(PITCH 通道)
 *
 * 与 Yaw_Gain_ByDist 对称,但插值方向相反:
 *   远端(FAR)→小增益(保守,保射程);  近端(NEAR)→大增益(激进,精准命中)。
 *
 * 输入/输出/分段逻辑同 Yaw_Gain_ByDist,仅增益常量换为 PITCH_GAIN_*。
 * ============================================================================ */
static float Pitch_Gain_ByDist(uint16_t dist_cm)
{
    float gain;

    if (dist_cm == 0)
    {
        gain = 1.0f;
    }
    else if ((float)dist_cm > PITCH_GAIN_ENABLE_DIST)
    {
        gain = 1.0f;
    }
    else if ((float)dist_cm >= DIST_ACQUIRE_CM)
    {
        gain = PITCH_GAIN_FAR;
    }
    else if ((float)dist_cm <= DIST_NEAR_CM)
    {
        gain = PITCH_GAIN_NEAR;
    }
    else
    {
        float t = (DIST_ACQUIRE_CM - (float)dist_cm) / (DIST_ACQUIRE_CM - DIST_NEAR_CM);
        gain = PITCH_GAIN_FAR + t * (PITCH_GAIN_NEAR - PITCH_GAIN_FAR);
    }

    if (gain < PITCH_GAIN_FAR)  gain = PITCH_GAIN_FAR;
    if (gain > PITCH_GAIN_NEAR) gain = PITCH_GAIN_NEAR;

    pitch_distance_gain = gain;
    return gain;
}
#endif


#if 1
/* === 世界系 pitch/yaw 解算:roll 反旋(输出端) ===
 * 喂 PID 的 ZYX 欧拉 pitch/yaw 是世界系参考(绕纵轴横滚不改 pitch 读数,已台架确认),
 * 故 PID 输出 = 世界系 pitch/yaw 力矩需求;但 X 翼舵面产生的是机体系力矩。
 * 机身相对参考姿态横滚 Δ=当前roll−Stable_roll 后,把 (Pw,Yw) 反旋 Δ 投到机体系:
 *     Pb =  cosΔ·Pw + sinΔ·Yw
 *     Yb = −sinΔ·Pw + cosΔ·Yw
 * Δ=0(标称 roll)→恒等。roll 通道绕纵轴,不在此处理。
 * Roll_World_Comp_Flag=0 时直通(=旧行为,便于 A/B);
 * ROLL_WORLD_COMP_SIGN 为横滚正向/舵面朝向符号,台架单轴阶跃验证后可翻 ±1。*/
void Roll_Derotate_PitchYaw(float Pw, float Yw, float *Pb, float *Yb)
{
    float delta_roll = Surface.current_angle_Euler[NOW][ROLL] - Surface.Stable_Euler_Angle[ROLL];
    float r = DEG2RAD(ROLL_WORLD_COMP_SIGN * delta_roll);
    float c = cosf(r), s = sinf(r);
    *Pb =  c * Pw + s * Yw;
    *Yb = -s * Pw + c * Yw;
}
/* ===== 控制分配(混控)重写:可调三轴限幅(交付A) + 最小能量分配(交付B) =====
 * 三者输出约定一致:写 Surface.output_angle_Servo[NOW][0..3](度,含SIGN,±SERVO_ANGLE_LIMIT 内),
 * 下标 0/1/2/3 = UP_LEFT/UP_RIGHT/DOWN_RIGHT/DOWN_LEFT。由调用点按 Alloc.Mode 分派。*/

/* 交付A(重写):完全可配置的逐级(字典序)优先级缩放分配。
 * Alloc.Prio[0..2] 指定三轴优先级(轴枚举,[0]最高)。最高优先轴在前置限幅内全额保障;
 * 次级/三级各在每片剩余舵机余量内最大化保留,绝不回削高优先轴 → 高优先轴不被低优先轴污染。
 * 写 Surface.output_angle_Servo[NOW][0..3](度,含SIGN,±SERVO_ANGLE_LIMIT 内) 与 Alloc.u_out[]。*/
void Servo_Mix_AxisLimit(float p, float r, float y)
{
    /* 1) 前置轴级限幅(各轴独立),保证 |d[a]| ≤ AXIS_LIMIT(应≤SERVO_ANGLE_LIMIT) */
    abs_limit(&p, AXIS_LIMIT_PITCH);
    abs_limit(&r, AXIS_LIMIT_ROLL);
    abs_limit(&y, AXIS_LIMIT_YAW);
    float d[3] = { p, r, y };                       /* 按 PITCH/ROLL/YAW 下标 */

    /* 逻辑符号阵 C(未含物理 SIGN),列序 P/R/Y。pitch 列四片同号 [−1,−1,−1,−1] 是 X 翼解算所需:
     * 4 片成 X(45°,从尾看 UL135/UR45/DR315/DL225),每片切向力对力矩贡献 pitch∝cosθ、yaw∝sinθ、roll=常数;
     * 配 SIGN=[−1,+1,+1,−1](左右镜像,见.h)后 pitch 落到舵令 u=[+,−,−,+]=真俯仰、与 roll/yaw 正交解耦。
     * 轴间配对结构由本阵的列决定(SIGN 只修整片装反);同步:Alloc.B pitch 行。
     * 注:pitch 逻辑列用 −1(非 +1)是因为 SIGN 翻转后等效,−1×SIGN=[+1,−1,−1,+1] 与物理舵面对齐。*/
    static const float C[4][3] = {                  /* 列序 P/R/Y */
        { -1.0f, +1.0f, -1.0f },                    /* UP_LEFT    */
        { -1.0f, -1.0f, +1.0f },                    /* UP_RIGHT   */
        { -1.0f, -1.0f, -1.0f },                    /* DOWN_RIGHT */
        { -1.0f, +1.0f, +1.0f },                    /* DOWN_LEFT  */
    };

    /* 2) Alloc.Prio 合法性校验:必须是 {0,1,2} 的排列,否则退回默认 {PITCH,YAW,ROLL} */
    uint8_t prio[3];
    {
        uint8_t seen = 0, ok = 1;
        for (int rank = 0; rank < 3; rank++)
        {
            uint8_t a = Alloc.Prio[rank];
            if (a > 2 || (seen & (1u << a))) { ok = 0; break; }
            seen |= (1u << a);
        }
        if (ok) { prio[0]=Alloc.Prio[0]; prio[1]=Alloc.Prio[1]; prio[2]=Alloc.Prio[2]; }
        else    { prio[0]=PITCH;         prio[1]=YAW;           prio[2]=ROLL;          }
    }

    /* 3) 逐级缩放:base[i]=各已定轴在片 i 的累计逻辑贡献 */
    float base[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float k_last  = 1.0f;
    for (int rank = 0; rank < 3; rank++)
    {
        int   a = prio[rank];
        float k = 1.0f;
        for (int i = 0; i < 4; i++)
        {
            float term = C[i][a] * d[a];
            float at   = (term < 0.0f) ? -term : term;
            if (at < 1e-6f) continue;               /* 无贡献 → 不构成约束 */
            float sgn  = (term < 0.0f) ? -1.0f : 1.0f;
            float ki   = (SERVO_ANGLE_LIMIT - sgn * base[i]) / at;
            if (ki < k) k = ki;
        }
        if (k < 0.0f) k = 0.0f;
        if (k > 1.0f) k = 1.0f;
        for (int i = 0; i < 4; i++)
            base[i] += C[i][a] * k * d[a];          /* 叠加本级,永不回削前级 */
        k_last = k;
    }
    Alloc.lat_scale = k_last;                       /* Vofa:最低优先轴保留比(横侧 k 的泛化) */

    /* 4) ×物理装配 SIGN + 每片兜底限幅(消 FP 误差) → 写出 */
    float SGN[4];
    SGN[UP_LEFT]    = SIGN_UL;  SGN[UP_RIGHT]   = SIGN_UR;
    SGN[DOWN_RIGHT] = SIGN_DR;  SGN[DOWN_LEFT]  = SIGN_DL;
    for (int i = 0; i < 4; i++)
    {
        float v = SGN[i] * base[i];
        abs_limit(&v, SERVO_ANGLE_LIMIT);
        Surface.output_angle_Servo[NOW][i] = v;
        Alloc.u_out[i] = v;
    }
}
/* 1 维零空间 n:n_j=(-1)^j·det(B 删去第 j 列的 3x3 子阵)。理想阵应得 n=[4,4,-4,-4]∝[1,1,-1,-1]。*/
static void alloc_compute_nullspace(const float B[3][4], float n[4])
{
    for (int j = 0; j < 4; j++)
    {
        int c[3], k = 0;
        for (int cc = 0; cc < 4; cc++) if (cc != j) c[k++] = cc;
        float a = B[0][c[0]], b = B[0][c[1]], d = B[0][c[2]];
        float e = B[1][c[0]], f = B[1][c[1]], g = B[1][c[2]];
        float h = B[2][c[0]], m = B[2][c[1]], q = B[2][c[2]];
        float det = a * (f * q - g * m) - b * (e * q - g * h) + d * (e * m - f * h);
        n[j] = ((j & 1) ? -1.0f : 1.0f) * det;
    }
}

/* 最小能量伪逆解 u0 = Bᵀ(BBᵀ)⁻¹ v;成功返回1,(BBᵀ)奇异返回0。CMSIS,数据缓冲静态。*/
static uint8_t alloc_minnorm_solve(float B[3][4], float v[3], float u0[4])
{
    static float Bt_d[12], M_d[9], Minv_d[9], t3_d[3];
    mat Bm, Btm, Mm, Minvm, vm, t3m, u0m;
    mat_init(&Bm,    3, 4, (float *)B);
    mat_init(&Btm,   4, 3, Bt_d);
    mat_trans(&Bm, &Btm);                      /* Bᵀ (4x3) */
    mat_init(&Mm,    3, 3, M_d);
    mat_mult(&Bm, &Btm, &Mm);                  /* M = B·Bᵀ (3x3) */
    mat_init(&Minvm, 3, 3, Minv_d);
    if (mat_inv(&Mm, &Minvm) == ARM_MATH_SINGULAR) return 0;
    mat_init(&vm,    3, 1, v);
    mat_init(&t3m,   3, 1, t3_d);
    mat_mult(&Minvm, &vm, &t3m);               /* t3 = M⁻¹·v */
    mat_init(&u0m,   4, 1, u0);
    mat_mult(&Btm, &t3m, &u0m);                /* u0 = Bᵀ·t3 (4x1) */
    return 1;
}

/* 沿零空间 n 把 u0 投影进约束盒 |u_i|≤u_max(Bn=0 故不改变 Bu=v)。
 * 可行区间非空→就地更新 u0、记录 α、返回1;为空(v 不可达)→返回0。*/
static uint8_t alloc_project_nullspace(float u0[4], const float n[4], float u_max)
{
    float a_lo = -1e30f, a_hi = 1e30f;
    for (int i = 0; i < 4; i++)
    {
        if (fabsf(n[i]) < 1e-6f)
        {
            if (fabsf(u0[i]) > u_max) return 0;        /* 该舵无零空间自由度且已越界 → 不可达 */
            continue;
        }
        float lo = (-u_max - u0[i]) / n[i];
        float hi = ( u_max - u0[i]) / n[i];
        if (lo > hi) { float t = lo; lo = hi; hi = t; }
        if (lo > a_lo) a_lo = lo;
        if (hi < a_hi) a_hi = hi;
    }
    if (a_lo > a_hi) return 0;                          /* 可行区间空 → 不可达 */

    float ndotn = n[0]*n[0] + n[1]*n[1] + n[2]*n[2] + n[3]*n[3];
    float u0dn  = u0[0]*n[0] + u0[1]*n[1] + u0[2]*n[2] + u0[3]*n[3];
    float a = (ndotn > 1e-9f) ? -(u0dn / ndotn) : 0.0f; /* 最小能量 α* = -(u0·n)/(n·n) */
    if (a < a_lo) a = a_lo;
    if (a > a_hi) a = a_hi;
    Alloc.alpha = a;
    for (int i = 0; i < 4; i++) u0[i] += a * n[i];
    return 1;
}

/* 用 v=[p,r,y] 解 minnorm→×ALLOC_GAIN→零空间投影。可达返回1(u0填最终解),(BBᵀ)奇异返回-1,不可达返回0。*/
static int alloc_try(float p, float r, float y, const float n[4], float u0[4])
{
    float v[3] = { p, r, y };
    if (!alloc_minnorm_solve(Alloc.B, v, u0)) return -1;
    for (int i = 0; i < 4; i++) u0[i] *= ALLOC_GAIN;
    return alloc_project_nullspace(u0, n, ALLOC_U_MAX) ? 1 : 0;
}

/* 交付B:真正的最小能量控制分配。min‖u‖² s.t. B·u=v(×ALLOC_GAIN),|u_i|≤u_max(=ALLOC_U_MAX)。
 * 可达→精确实现该力矩的最小能量解;不可达→优先级 pitch>yaw>roll:先保 pitch 二分缩横侧,横侧缩尽仍
 * 不可达(pitch 力矩超单舵极限)再二分缩 pitch(等效 Mode1 饱和截断);(BBᵀ)奇异→退回三轴限幅。
 * B 默认理想阵,可台架辨识替换(见 Alloc.B 注释)。*/
void Servo_Mix_MinEnergy(float p, float r, float y)
{
    float n[4];
    alloc_compute_nullspace(Alloc.B, n);

    float u0[4] = {0};
    float s_lat = 1.0f, s_pit = 1.0f;
    int rc;

    /* 先判 pitch 单独(横侧全关)可达性:决定走"保 pitch 缩横侧"还是"缩 pitch" */
    rc = alloc_try(p, 0.0f, 0.0f, n, u0);
    if (rc < 0) { Alloc.singular_flag = 1; Servo_Mix_AxisLimit(p, r, y); return; }

    if (rc == 1)
    {
        /* 阶段1:pitch 可达 → 保 pitch,二分缩横侧(yaw/roll) */
        for (int it = 0; it < 10; it++)
        {
            rc = alloc_try(p, r * s_lat, y * s_lat, n, u0);
            if (rc != 0) break;                /* 可达(1)或奇异(-1)都跳出 */
            s_lat *= 0.7f;
        }
        if (rc == 0) { rc = alloc_try(p, 0.0f, 0.0f, n, u0); s_lat = 0.0f; }  /* 兜底:退化到纯 pitch(已知可达) */
    }
    else
    {
        /* 阶段2:pitch 力矩超单舵极限 → 横侧置 0,二分缩 pitch(牺牲幅度,等效 Mode1 饱和) */
        s_lat = 0.0f;
        for (int it = 0; it < 12; it++)
        {
            s_pit *= 0.7f;
            rc = alloc_try(p * s_pit, 0.0f, 0.0f, n, u0);
            if (rc != 0) break;
        }
    }
    if (rc < 0) { Alloc.singular_flag = 1; Servo_Mix_AxisLimit(p, r, y); return; }
    Alloc.singular_flag = 0;
    Alloc.v_scale = s_lat;
    Alloc.p_scale = s_pit;
    Alloc.infeasible = (s_pit < 0.999f) ? 2 : ((s_lat < 0.999f) ? 1 : 0);  /* 0可达 1缩横侧 2连pitch也缩 */

    /* 输出 ×SIGN + 兜底限幅,记录 Vofa */
    float SGN[4];
    SGN[UP_LEFT]    = SIGN_UL;  SGN[UP_RIGHT]   = SIGN_UR;
    SGN[DOWN_RIGHT] = SIGN_DR;  SGN[DOWN_LEFT]  = SIGN_DL;
    float lo = 1e30f, hi = -1e30f;
    for (int i = 0; i < 4; i++)
    {
        Alloc.u0[i] = u0[i];
        if (u0[i] < lo) lo = u0[i];
        if (u0[i] > hi) hi = u0[i];
        float out = SGN[i] * u0[i];
        abs_limit(&out, SERVO_ANGLE_LIMIT);
        Surface.output_angle_Servo[NOW][i] = out;
        Alloc.u_out[i] = out;
    }
    Alloc.u0_span = hi - lo;
}
#endif


#endif
void Guidance_Start(void)//自检后的判断
{
            Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
            Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
}
void Guidance_Stable(void)//自稳
{
            // Buzzer_Remind();
        Surface.target_angle_Euler[NOW][PITCH] = 
        Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][YAW]   = 
        Surface.Stable_Euler_Angle[YAW];
}
void Guidance_Terminal(void)//制导段
{
    /* PN 视线率状态(函数级静态,帧间保持):上帧世界系视线终点 / 自上帧起累计的控制拍数 / 首帧标志 */
    static uint16_t vis_dt_cnt    = 0;
    static uint8_t  los_rate_init = 0;
    vis_dt_cnt++;
        // Surface.current_angle_Euler[NOW][ROLL]+((Surface.Stable_Euler_Angle[ROLL]-Surface.current_angle_Euler[NOW][ROLL])/2.0f);

    vision_los_current[PITCH] = Surface.current_angle_Euler[NOW][PITCH];
    vision_los_current[YAW]   = Surface.current_angle_Euler[NOW][YAW];
    /* 视线目标锁存(方向A):视觉~20Hz、控制1kHz。只在"视觉新数据到达"(Vision_New_Data_flag==1)那一刻,用当时
     * 姿态把视线锁存到世界系 los=v+current,写入斜坡终点 vision_los_final(不再直接阶跃写 target);帧间(flag==0)
     * 终点保持不变 → 终点−current 仍随机体转动实时变化(航位推算)。新数据处理结束后置回0,等下一帧再置1。*/
    if (Vision_Rx_Data.Vision_New_Data_flag == 1)
    {
        taskENTER_CRITICAL();
        Vision_Rx_Buf_t v = Vision_Rx_Data;
        taskEXIT_CRITICAL() ;
        if (v.Vision_recognize_flag == RECOGNIZE_SUCCESS) /* 识别到目标:锁存世界系视线终点(全程用快照 v,避免与视觉中断撕裂读) */
        {
            /* YAW:始终视觉制导,锁存世界系视线终点 */
            vision_los_final[NOW][YAW]   = v.Euler[NOW][1] + Surface.current_angle_Euler[NOW][YAW];
            vision_los_final[NOW][PITCH] = v.Euler[NOW][0] + Surface.current_angle_Euler[NOW][PITCH];

                
            if (los_rate_init)
            {
                float dt_vis = (float)vis_dt_cnt * (CTRL_PERIOD_MS * 0.001f);
                if (dt_vis < 1e-3f) dt_vis = 1e-3f;
                /* 帧间差分算原始世界系视线率(PN 超前用) */
                vision_los_rate[YAW]   = (vision_los_final[LAST][YAW]   - vision_los_final[NOW][YAW])   / dt_vis;
                vision_los_rate[PITCH] = (vision_los_final[LAST][PITCH] - vision_los_final[NOW][PITCH]) / dt_vis;
                abs_limit(&vision_los_rate[YAW],   LOS_RATE_LIMIT_DPS);
                abs_limit(&vision_los_rate[PITCH], LOS_RATE_LIMIT_DPS);
            }
            vision_los_final[LAST][YAW]   = vision_los_final[NOW][YAW];
            vision_los_final[LAST][PITCH] = vision_los_final[NOW][PITCH];
            los_rate_init   = 1;
            vis_dt_cnt      = 0;
        }
        Vision_Rx_Data.Vision_New_Data_flag = 0;   /* 新数据处理完毕,置回0 */
    }
    if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE) 
    {
        vision_los_final[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        vision_los_final[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
        vision_los_current[PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        vision_los_current[YAW]   = Surface.current_angle_Euler[NOW][YAW];
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
        vision_los_rate[PITCH] = 0.0f;   
        vision_los_rate[YAW]   = 0.0f;
        los_rate_init = 0;
    }

    /* 视觉目标 = 锁存的世界系视线终点(无额外平滑) */
    {
        /* 视觉目标 = 锁存的世界系视线终点 vision_los_final(新帧阶跃更新,帧间靠航位推算)。
         * 原 LPF / 距离增益缩放(Yaw/Pitch_Gain_ByDist)已停用,此处直接取用,不再有中间滤波量。
         * 丢目标(FAILURE)时 vision_los_final[NOW] 已在上方被置为当前姿态 → 自然就地保持。*/

        /* YAW:直接追锁存视线终点 */
        Surface.target_angle_Euler[NOW][YAW] = vision_los_final[NOW][YAW];
        Surface.target_angle_Euler[NOW][PITCH] = vision_los_final[NOW][PITCH];

        /* PITCH:主动滑翔→扎(pitch_glide_mode=1) / 旧门限逻辑(=0) */
        // if (pitch_glide_mode)
        // {
        //     /* phi=世界系看灯视线俯角(越负=越近越陡)。blend 0→1:
        //      * 远端住 THETA_GLIDE 压平轨迹增程,近了平滑过渡到追视觉目标扎下去。*/
        //     float phi   = vision_los_final[NOW][PITCH];
        //     float blend = (GLIDE_LOS_HI_DEG - phi) / (GLIDE_LOS_HI_DEG - GLIDE_LOS_LO_DEG);
        //     if (blend < 0.0f) blend = 0.0f;
        //     if (blend > 1.0f) blend = 1.0f;
        //     pitch_glide_blend  = blend;
        //     pitch_glide_target = (1.0f - blend) * THETA_GLIDE_DEG + blend * vision_los_final[NOW][PITCH];
        //     Surface.target_angle_Euler[NOW][PITCH] = pitch_glide_target;
        // }
        // else if (Surface.current_angle_Euler[NOW][PITCH] <= pitch_control_limit_deg)
        // {
        //     Surface.target_angle_Euler[NOW][PITCH] = vision_los_final[NOW][PITCH];
        // }
        // else
        // {
        //     Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        // }
    }

    /* PN 超前:暂时关闭,先用纯 PID 跟踪视觉目标验证基础跟踪性能。
     * 后续标定好 K_Dyn 后再打开 Mode1(速度²缩放)。*/
    #if 0
    if (vins_out.locked)
    {
        PNG_Apply_Lead_Yaw(&Surface, &IMU_Data);
        if (Surface.current_angle_Euler[NOW][PITCH] <= pitch_control_limit_deg)
            PNG_Apply_Lead_Pitch(&Surface, &IMU_Data);
    }
    #endif

}
void Guidance_End(void)
{
    // Dart_Trigger_Power_Control( Power_OFF );
    if (Surface.Guidance_cnt[4]++>1000)
    {
        Vision_Transmit(Vision_Cmd_Record_Stop);
        Guidance_State = PROCESS_OK;
        Buzzer_stop();
        Surface.Guidance_cnt[4] = 0;
    }
    Wing_Servo_Control_Flag = 0;
    // Total_Power_Control( Power_OFF )//不能太快掉电，不然openmv保存不了视频等等
}
void Guidance_Process_OK(void)
{
    if (Surface.Guidance_cnt[5]++>3000)
    {
		Total_Power_Control(Power_OFF);
        Surface.Guidance_cnt[5] = 0;
    }
}

void get_current_Target(void)
{
        // Guidance_State = Stable;
        Stable_Flag = 0;
        switch(Guidance_State)
        {
            case Start:
            {
                Guidance_Start();                  
            }break;
            case Stable:
            {
                if(IMU_Data.calib_done==1)
                {
                    static uint16_t cnt = 0 ;
                    if(cnt++>1)
                    {
                        IMU_Data.calib_done = 2;
                        // Surface.Stable_Euler_Angle[PITCH] = IMU_Data.Euler[NOW][PITCH];
                        // Surface.Stable_Euler_Angle[ROLL]  = IMU_Data.Euler[NOW][ROLL]; 
                        // Surface.Stable_Euler_Angle[YAW]   = IMU_Data.Euler[NOW][YAW]; 
                        // Surface.target_angle_Euler[NOW][ROLL]  = Surface.Stable_Euler_Angle[ROLL];
                        // Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
                    }
                }   
                Stable_Flag = 1;
                Guidance_Stable();
            }break;
            case Terminal:
            {
                Guidance_Terminal();
            }break;
            case End:
            {
                Guidance_End();
            }break;
            case PROCESS_OK:
            {
                Guidance_Process_OK();
            }break;
        }
        
        /* ROLL 始终自稳(与视觉新数据无关),每 tick 刷新 */
        // Surface.target_angle_Euler[NOW][ROLL]  =  
        // Surface.Stable_Euler_Angle[ROLL];
        // Surface.target_angle_Euler[NOW][YAW]  =  
        // Surface.Stable_Euler_Angle[YAW];
        // Surface.target_angle_Euler[NOW][PITCH]  =  
        // Surface.current_angle_Euler[NOW][PITCH];

}   
void get_current_State(void)
{
    static uint8_t yaw_zero_latched = 0;   /* yaw 归零一次性闩锁:窗口内只发一次请求,流程重置后重新武装 */
    if(
            Surface.Guidance_flag[1] == 1
    )
    {
            Buzzer_Remind();
    }
    if (Self_Text.Self_Text_Process==Self_Text_OK&&Guidance_State == Self_Text_State)
    {
        if (Surface.Guidance_cnt[0]++>2000)
        
        {
            Buzzer_Remind();
            Guidance_State = Start;
            Vision_Transmit( Vision_Cmd_Work );
            if (Self_Text.Self_Text_Process<5)
            {
                Self_Text.Self_Text_Process = 5; 
            } 
            Surface.Guidance_cnt[0] = 0;
            Surface.Stable_Euler_Angle[PITCH] = IMU_Data.Euler[NOW][PITCH];
            // Shot_Pitch = IMU_Data.Euler[NOW][PITCH];
        }
    }
    else if (Guidance_State == Start )
    {
        if(IMU_Data.Euler[NOW][PITCH]<=Shot_Pitch+2.0f&&IMU_Data.Euler[NOW][PITCH]>=Shot_Pitch- 2.0f&&
           IMU_Data.Euler[NOW][ROLL]<=Shot_Roll+3.0f&&IMU_Data.Euler[NOW][ROLL]>=Shot_Roll- 3.0f)
        {
            Vision_Transmit( Vision_Cmd_Work );
            Surface.Guidance_cnt[1]++;
        }
        if(Surface.Guidance_cnt[1]>=5&&Surface.Guidance_cnt[1]<20)
        {
            /* 进入窗口仅一次:请求 IMU 以当前绝对航向为新原点,使此刻起 yaw=0 */
            if(!yaw_zero_latched){ Yaw_Zero_Req = 1; yaw_zero_latched = 1; }
        }
        if(Surface.Guidance_cnt[1]>=20&&Surface.Guidance_flag[1]==0)
        {
            Surface.Stable_Euler_Angle[ROLL]  = IMU_Data.Euler[NOW][ROLL]; 
            Surface.Stable_Euler_Angle[YAW]   = IMU_Data.Euler[NOW][YAW]; 
            Surface.Guidance_flag[1] = 1;
            Vision_Transmit( Vision_Cmd_Record_Start );
        }
        if(Surface.Guidance_flag[1] == 1&&(IMU_Data.Velocity[Body][NOW][Z]<=-0.7f||(IMU_Data.Velocity[Body][NOW][Y]>=0.7f)))
        {
            Buzzer_Remind();
            Guidance_State = Stable;
            Surface.Guidance_cnt[1] = 0;
            Surface.Guidance_flag[1] = 2;
        }
    }
    else if (Guidance_State == Stable && (IMU_Data.Euler[NOW][PITCH]<=10.0f&&Vision_Rx_Data.Vision_recognize_flag==RECOGNIZE_SUCCESS))
    {
        Vision_Transmit( Vision_Cmd_Work );
        if(Surface.Guidance_cnt[2]++>5)
        {
            Buzzer_Remind();
            Guidance_State = Terminal;
            Surface.Guidance_cnt[2] = 0;
            Vel_Reanchor_Flag = 1;   /* 俯冲入段:请求 IMU 下一拍用"姿态前向×V_NOM"锚定世界速度 → γ起始≈机体俯仰 */
        }
    }
    else if (Guidance_State == Terminal &&fabs(IMU_Data.Velocity[Body][NOW][Z]==0.0f||IMU_Data.A_Normed[NOW][Y])>= 0.90f&& IMU_Data.A[NOW][Y]<= -1.3f)
    {
        if(Surface.Guidance_cnt[3]++>5)
        {
            Buzzer_Remind();
            Guidance_State = End;
            Surface.Guidance_cnt[3] = 0;
        }
        
        // if(fabs(IMU_Data.Velocity[Body][NOW][Z]==0.0f||IMU_Data.A_Normed[NOW][Y])>= 0.80f&& IMU_Data.A[NOW][Y]<= -0.9f)
        // {
        //     Surface.Guidance_cnt[3]++ ;   
        // }
        // if(IMU_Data.Velocity[Body][NOW][X] == 0.0f)
        // {
        //     if(Surface.Guidance_cnt[3]>5)
        //     {
        //         Buzzer_Remind();
        //         Guidance_State = End;
        //         Surface.Guidance_cnt[3] = 0;
        //     }
        // }
    }
    else 
    {
        Surface.Guidance_cnt[0] = 0;
        Surface.Guidance_cnt[1] = 0;
        Surface.Guidance_cnt[2] = 0;
        Surface.Guidance_cnt[3] = 0;
        yaw_zero_latched = 0;   /* 流程退回:重新武装 yaw 归零,下次进窗口再触发一次 */
    }
}

/*---- 线程区 ----*/
void surface_control_task(void) 
{
  /* USER CODE BEGIN surface_control_task */
    static uint32_t prev_tick = 0;
    static uint8_t  first_run = 1;
    current_tick = xTaskGetTickCount();
    float delta_time;
    if (first_run)
    {
        delta_time = (float)CTRL_PERIOD_MS * 0.001f;
        first_run = 0;
    }
    else
    {
        delta_time = (float)(current_tick - prev_tick) * 0.001f;
        if (delta_time < 1e-4f) delta_time = (float)CTRL_PERIOD_MS * 0.001f;
    }
    prev_tick = current_tick;
    /*状态机，拿目标值*/
    get_current_State();
    get_current_Target();
    /*pid/adrc，算输出值*/
    Surface.pid_cale_flag = 0;
    if ((Guidance_State==Stable||Guidance_State==Terminal)&&(imu_is_static==0))
    {
        Surface.pid_cale_flag = 1;

        /* ========== LQR / LADRC / PID 控制选择 ==========
         * lqr_mode==1：LQR 一步 6态→4舵(含混控)，直接写 output_angle_Servo，绕过下面的 Servo_Mix_*。
         *   其余分支(LADRC/PID)仍是两步：先算 output_gyro_Euler，再由混控器分配。*/
        if (lqr_mode == 1)
        {
            Euler_LQR_Cale(delta_time);   /* 直接写 Surface.output_angle_Servo[NOW][...]，下方混控分派会跳过 */
        }
        else if (ladrc_mode == 1)
        {
            /* 三轴全 LADRC */
            Euler_LADRC_Cale(delta_time);
        }
        else if (ladrc_mode == 3)
        {
            /* 仅 Roll 用 LADRC，Pitch/Yaw 仍 PID：先按 PID 算三轴，再用 LADRC 覆盖 Roll 通道 */
            Euler_pid_Cale(delta_time);
            Surface.output_gyro_Euler[NOW][ROLL] = LADRC_Calc(
                &ladrc_ctrl[LADRC_ROLL],
                Surface.target_angle_Euler[NOW][ROLL],   /* roll 自稳目标 = Stable 角 */
                Surface.current_angle_Euler[NOW][ROLL],  /* 当前 roll 角 */
                Surface.current_gyro_Euler[NOW][ROLL],   /* 实测 roll 角速度(阻尼用，替代 LESO z2) */
                delta_time);
        }
        else
        {
            /* 原 PID（默认安全档） 或 速度矢量追踪 */
            if (vel_pursuit_mode)
                Velocity_Pursuit_Cale(delta_time);
            else
                Euler_pid_Cale(delta_time);
        }
        /* ===================================== */
    }
    // else if(imu_is_static==1)
    // {
    //     Surface.output_gyro_Euler[NOW][PITCH] = 0; 
    //     Surface.output_gyro_Euler[NOW][YAW] = 0;
    //     Surface.output_gyro_Euler[NOW][ROLL] = 0;
    //         LADRC_Reset(&ladrc_ctrl[LADRC_ROLL]);
    // }
    /*解算到舵面*/
    if(DART_TYPE == VECTOR_NOZZLE   )//x翼
    {
        if ((Guidance_State==Stable||Guidance_State==Terminal) && lqr_mode != 1)
        // if (Guidance_State==Terminal)
        {
            /* lqr_mode==1 时 Euler_LQR_Cale 已直接写好 4 舵角(含混控)，跳过本反旋+混控分派。
             * 先把世界系 pitch/yaw 输出按当前 roll 反旋到机体系,再做 Pitch 优先最小能量分配:
             * pitch 全保,roll/yaw 等比缩进限幅,不污染 pitch。SIGN_xx 仍在函数内逐片乘上,标定流程不变。 */
            float 
            p_body = Surface.output_gyro_Euler[NOW][PITCH], 
            y_body = Surface.output_gyro_Euler[NOW][YAW],
            r_body = Surface.output_gyro_Euler[NOW][ROLL];
        // if (Guidance_State==Terminal&&IMU_Data.Euler[NOW][PITCH]<5.0F)
        // {
        //     // ladrc_ctrl[LADRC_ROLL].max_output = 10;
        //     // y_body *= 1.5f;
        //     // p_body = 0.0f;  
        // }
            if (Guidance_State==Terminal)
            {
            Roll_Derotate_PitchYaw(Surface.output_gyro_Euler[NOW][PITCH],
                                   Surface.output_gyro_Euler[NOW][YAW],
                                   &p_body, &y_body);           
            }

            // p_body = 0; 
            // y_body = 0; 
            // r_body = 15;
            /* 三轴统一接 PID 内环输出(roll 绕纵轴不反旋,直接用);按 Alloc.Mode 分派分配器。*/
            Surface.output_Body_Euler[NOW][PITCH] = p_body;
            Surface.output_Body_Euler[NOW][YAW]   = y_body; 
            Surface.output_Body_Euler[NOW][ROLL]  = r_body;
            switch (Alloc.Mode)
            {
                // case 0:  Servo_Mix_PitchPriority(p_body, r_body, y_body); break;  /* 旧对照(roll 入参已改 PID 输出) */
                case 1:  Servo_Mix_AxisLimit    (p_body, r_body, y_body); break;
                case 2:  Servo_Mix_MinEnergy    (p_body, r_body, y_body); break;
            }
        }
        for (int i = 0; i < 4; i++)                    /* 安全网:分配已保证在限内,此处仅兜底 FP 误差 */
            abs_limit(&Surface.output_angle_Servo[NOW][i], SERVO_ANGLE_LIMIT);
    }
    if (Guidance_State == End || Guidance_State == PROCESS_OK)
    {
        for (int i = 0; i < 4; i++) Surface.output_angle_Servo[NOW][i] = 0;
    }
    Wing_Control();
    Data_Updata();
    if (Self_Text.Self_Text_Process == Self_Text_Dart_Trigeer&&Self_Text.Dart_Trigger_Self_Text_flag == Self_Text_Failure && Self_Text.Vision_Self_Text_flag == Self_Text_Success)
    {
        osDelay( 300 );
    }
  /* USER CODE END surface_control_task */
}
