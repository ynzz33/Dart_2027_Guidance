 /**
 * @file    lqr.c
 * @brief   飞镖 X 翼姿态 LQR 控制器（移植自 lqr_czn/dart_attitude_LQR_v1.m）
 * @details 一步从"姿态误差+角速度(6态)"直接解出"4 片舵面偏角(4控)"：u = -K_d·x。
 *
 *          ★ 与 PID/LADRC 的本质区别：
 *            PID/LADRC 链路是两步——先算三轴力矩需求(output_gyro_Euler)，再过混控器
 *            (Servo_Mix_AxisLimit/MinEnergy)分配到 4 舵。
 *            LQR 的 K_d[4][6] 已经把"X 翼混控几何 G"烘焙进模型 B 里，dlqr 求逆得到的 K
 *            直接是 6态→4舵。所以本模块一步替代 PID + 混控器两步，直接写
 *            Surface.output_angle_Servo[NOW][...]，不经过 output_gyro_Euler、不经过 Servo_Mix_*。
 *
 *          ★ 符号约定：整机三轴极性在 MATLAB 的 G 矩阵(各轴行)里调；本文件的换序/×SIGN 见 §3 与下方。
 *
 *          ★ K 矩阵以 MATLAB 同名同形 dart_lqr_K[4][6] 存放(见下方"粘贴区")：
 *            MATLAB 脚本 Step5 会打印 `static const float dart_lqr_K[4][6] = { ... };`，
 *            每次重调 Q/R/惯量/速度后重跑 MATLAB，把打印出来的 4 行数值直接覆盖粘贴到粘贴区即可。
 *
 *          ★ 未编译：Keil/eIDE 工程 AI 编不了；新增本 .c 需手动加入工程编译列表。待台架。
 *
 * @author  ynz (AI 移植)
 * @date     2026/06/27
 *
 * ============================================================================
 *  状态/控制约定（必须与 MCU_LQR_PORTING_GUIDE 一致）
 *    x = [roll_err, pitch_err, yaw_err, p, q, r]   角 rad、角速度 rad/s，右手定则
 *        err = 测量 − 期望(配平/目标)；u = -K_d·x 把 err 打回 0
 *    u = [delta1, delta2, delta3, delta4]           4 舵目标偏角 rad
 *        MATLAB 舵号: delta1=右上 delta2=左上 delta3=左下 delta4=右下
 *
 *  ⚠ MATLAB 舵号顺序 ≠ 本工程索引(UL=0,UR=1,DR=2,DL=3)。
 *    本文件在 K_ROW_TO_SERVO[] 处换序；上车前务必按指南 §9 逐轴阶跃验符号。
 * ============================================================================
 */

#include "lqr.h"
#include "surface_control_task.h"   /* Surface / 舵机索引 UP_LEFT.. / SIGN_xx / SERVO_ANGLE_LIMIT / PITCH,ROLL,YAW */
#include "IMU.h"                     /* NOW 等历史槽枚举 / IMU_Data.Velocity */
#include "pid.h"                     /* abs_limit / Angle_Wrap_180 复用 */
#include "common_defs.h"            /* DEG2RAD / RAD2DEG */
#include "CallBack_Task.h"

/* MATLAB Coder 生成的速度方程 K_d(V) */
#include "LQR_K_Dart_d.h"
#include "LQR_K_Dart_d_initialize.h"

#include <math.h>                    /* sqrtf */


float V_DART = 0.0f;   /* 当前拍速度(m/s，clamp 后) */

/*============================================================================
 *  PID-for-LQR 积分器(P=D=0 纯积分，对姿态误差 current−target 积分，iout 叠加到 err_deg)
 *  索引 [PITCH,ROLL,YAW]，由 LQR_Init 初始化。详见 lqr.h 宏 + Euler_LQR_Cale。
 *============================================================================*/
pid_t pid_i_for_lqr[3];

/*============================================================================
 *  全局实例 + 运行时旋钮
 *============================================================================*/
LQR_t lqr_ctrl;   /* Vofa/Watch 可观测：状态 x、原始/限幅后舵偏、K_d(V) */

/* 舵偏限幅(rad)，与 MATLAB delta_max_ac=deg2rad(60) 及工程 SERVO_ANGLE_LIMIT(60°) 对齐 */
float dart_delta_max_rad = 1.0471975511965976f;   /* ±60° */

/* pitch 轴门控：1=只在制导段(Terminal)+检测到目标时控俯仰(补偿)，其他阶段/丢目标时 pitch 不受控、
 *   飞镖俯仰主要靠镖架初始动力；0=全程控 pitch(旧行为)。调试器 Watch 在线切换做 A/B。详见 Euler_LQR_Cale 内说明。*/
uint8_t lqr_pitch_terminal_only = 1;


/* MATLAB 舵号(行序 delta1..4) → 本工程舵机索引(列序 UL/UR/DR/DL)··
 *   delta1=右上=UP_RIGHT, delta2=左上=UP_LEFT, delta3=左下=DOWN_LEFT, delta4=右下=DOWN_RIGHT */
static const uint8_t K_ROW_TO_SERVO[DART_LQR_SERVO_NUM] = {
    UP_RIGHT,    /* delta1 右上 */
    UP_LEFT,     /* delta2 左上 */
    DOWN_LEFT,   /* delta3 左下 */
    DOWN_RIGHT   /* delta4 右下 */
};

/*============================================================================
 *  核心：纯 LQR 解算 u = -K_d·x
 *  x 单位 rad / rad·s⁻¹；u 单位 rad，已按 ±dart_delta_max_rad 限幅。
 *  ⚠ 控制律带负号；不要把 K 预先取负(否则需改名 minus_K)。
 *
 *  K 来源由 lqr_use_scheduled_K 控制：
 *    1 = lqr_ctrl.K_d（50Hz 中断由 LQR_Gain_Update50Hz 更新，速度调度版，默认）
 *    0 = dart_lqr_K（旧单点静态版，A/B 对照存根）
 *============================================================================*/
void LQR_Update(const float x[DART_LQR_STATE_NUM], float u[DART_LQR_SERVO_NUM])
{
    float (*K)[DART_LQR_STATE_NUM] = lqr_ctrl.K_d;

    for (uint8_t i = 0; i < DART_LQR_SERVO_NUM; i++)
    {
        float ui = 0.0f;
        for (uint8_t j = 0; j < DART_LQR_STATE_NUM; j++)
            ui -= K[i][j] * x[j];

        if (ui >  dart_delta_max_rad)
        {
            ui = dart_delta_max_rad;
        }
        if (ui < -dart_delta_max_rad)
        {
            ui = -dart_delta_max_rad;
        }
        u[i] = ui;
    }
}

/*============================================================================
 *  桥接：从 Surface 取姿态/角速度 → 组状态 x → LQR → 写 4 舵机角(度)
 *  直接覆盖 Surface.output_angle_Servo[NOW][UL/UR/DR/DL]，绕过 output_gyro_Euler 与混控器。
 *  调用点见 surface_control_task.c：lqr_mode==1 时调用本函数，并跳过混控分派。
 *
 *  @note dt 当前未参与计算(纯静态增益 LQR，无积分/状态记忆)，保留入参以便后续扩展。
 *============================================================================*/
void Euler_LQR_Cale(float dt)
{
    /* 1) 姿态误差 err = 测量 − 期望(配平/目标)，存入 lqr_ctrl.err_deg[roll,pitch,yaw](度，可观测)。
     *    yaw 为周期角须环绕到 ±180°；roll 按 roll_wrap 选择是否环绕。
     *    工程欧拉/角速度索引为 [PITCH,ROLL,YAW]；状态 x 顺序为 [roll,pitch,yaw,p,q,r]。*/
    lqr_ctrl.err_deg[0] = Surface.current_angle_Euler[NOW][ROLL]  - Surface.target_angle_Euler[NOW][ROLL];
    // lqr_ctrl.err_deg[1] = Surface.current_angle_Euler[NOW][PITCH] - Surface.target_angle_Euler[NOW][PITCH];
    lqr_ctrl.err_deg[1] = 0;
    lqr_ctrl.err_deg[2] = Surface.current_angle_Euler[NOW][YAW]   - Surface.target_angle_Euler[NOW][YAW];
    if (lqr_ctrl.roll_wrap) lqr_ctrl.err_deg[0] = Angle_Wrap_180(lqr_ctrl.err_deg[0]);
    lqr_ctrl.err_deg[2] = Angle_Wrap_180(lqr_ctrl.err_deg[2]);   /* yaw 始终环绕，防跨 ±180° 爆冲 */

    /* gyro 状态 x[3..5](机体角速度 rad/s)，不进积分 */
    lqr_ctrl.x[3] = DEG2RAD(Surface.current_gyro_Euler[NOW][ROLL]) ;
    lqr_ctrl.x[4] = DEG2RAD(Surface.current_gyro_Euler[NOW][PITCH])  ;
    lqr_ctrl.x[5] = DEG2RAD(Surface.current_gyro_Euler[NOW][YAW])   ;


    if(Guidance_State>Terminal)
    { 
        lqr_ctrl.x[4]   = 0.0f ;
    }    

    /* ---- PID-for-LQR 积分(yaw only)：用 pid_i_for_lqr[YAW] 对 yaw 误差(current−target)纯积分，
     *    叠加到 yaw err_deg 后再 DEG2RAD 进 LQR 状态 x[2]。积分分离：|yaw_err|≥阈值→清零 iout；
     *    |yaw_err|<阈值→pid_calc 累积。分离阈值是唯一门控(PID deadband=0)。
     *    舵面饱和→回退本拍 iout(抗饱和)。roll/pitch 不积分，直通。---- */
    {
        /* yaw 积分分离 + PID 累积 */
        if(Guidance_State >= Terminal)
        {
            if (fabsf(lqr_ctrl.err_deg[2]) >= LQR_I_SEPARATION_DEG_DEFAULT)
            {
                pid_i_for_lqr[YAW].iout = 0.0f;            /* 分离区外：清零积分 */
            }
            else
            {
                /* set=当前yaw, get=目标yaw → PID err = current−target = LQR err_deg(同号) */
                pid_calc(&pid_i_for_lqr[YAW],
                        Surface.target_angle_Euler[NOW][YAW],
                        Surface.current_angle_Euler[NOW][YAW],
                        dt);
            }
        }



        /* roll/pitch: err_deg 直通；yaw: 增强误差 = err_deg + 积分(度) → rad */
        lqr_ctrl.x[0] = DEG2RAD(lqr_ctrl.err_deg[0]);         /* roll  直通 rad */
        // lqr_ctrl.x[1] = DEG2RAD(lqr_ctrl.err_deg[1]);         /* pitch 直通 rad */
        lqr_ctrl.x[1] = 0.0f;         /* pitch 直通 rad */
        lqr_ctrl.x[2] = DEG2RAD(lqr_ctrl.err_deg[2] + pid_i_for_lqr[YAW].iout); /* yaw 增强 rad */

        /* yaw 死区：|误差| < 0.15° 不做 yaw 控制，清零 x[2] */
        // if (fabsf(lqr_ctrl.err_deg[2]) < 0.05f)
        // {
        //     lqr_ctrl.x[2] = 0.0f;
        //     lqr_ctrl.x[5] = 0.0f;
        // }
        LQR_Update(lqr_ctrl.x, lqr_ctrl.u_rad);
    }
    /* 3) 按 MATLAB 舵号→工程索引换序，转度，写舵机角(±SERVO_ANGLE_LIMIT)。
     *    ★ 不乘工程 SIGN_xx：那套是为 PID 的"共模逻辑列"设计的(C 阵 pitch 列四片同号)，
     *      而 MATLAB G 的 pitch/roll 已是差动；再乘 ×SIGN 会把 pitch/roll 打成共模(产生不了力矩)。
     *      u_rad 即物理舵偏，换序后直接写。整机三轴极性在 MATLAB G 矩阵(roll/pitch/yaw 各行)里翻，重跑重粘 K。
     *    若某片舵机机械装反(单片)，再在此处按 idx 单独翻号；勿动 K 行顺序(指南 §9)。*/
    for (uint8_t row = 0; row < DART_LQR_SERVO_NUM; row++)
    {
        uint8_t idx = K_ROW_TO_SERVO[row];
        float   deg = RAD2DEG(lqr_ctrl.u_rad[row]);
        abs_limit(&deg, SERVO_ANGLE_LIMIT);
        lqr_ctrl.u_servo_deg[idx] = deg;                 /* Vofa：最终舵角(按工程索引 UL/UR/DR/DL) */
        Surface.output_angle_Servo[NOW][idx] = deg;      /* 直接写舵面，等同 Servo_Mix_* 的产物 */
    }
}

/*============================================================================
 *  50Hz 中断调用：从 EKF 速度算 K_d(V)，写入 lqr_ctrl.K_d/V_lqr
 *  lqr_use_scheduled_K=0 时直接返回(退回静态 K)。
 *============================================================================*/
void LQR_Gain_Update50Hz(void)
{

    /* 1) 取当前飞行速度标量(m/s)，从 EKF 世界速度算模长 */
    float vx = IMU_Data.Velocity[World][NOW][X];
    float vy = IMU_Data.Velocity[World][NOW][Y];
    float vz = IMU_Data.Velocity[World][NOW][Z];
    V_DART = sqrtf(vx * vx + vy * vy + vz * vz);

    /* 2) clamp 到拟合范围 */
    if (V_DART < DART_LQR_V_MIN) V_DART = DART_LQR_V_MIN;
    if (V_DART > DART_LQR_V_MAX) V_DART = DART_LQR_V_MAX;
    lqr_ctrl.V_lqr = V_DART;


    /* 3) 调 MATLAB Coder 生成的方程（double 精度） */
    
    double K_flat[24];
    LQR_K_Dart_d((double)6, K_flat);
    for (uint8_t col = 0; col < DART_LQR_STATE_NUM; col++)
        for (uint8_t row = 0; row < DART_LQR_SERVO_NUM; row++)
            lqr_ctrl.K_d[row][col] = (float)K_flat[col * DART_LQR_SERVO_NUM + row];
}

/*============================================================================
 *  初始化：清零状态、默认符号、初始化 MATLAB Coder 运行时(在 TotalInitTask 调一次)
 *============================================================================*/
void LQR_Init(void)
{
    for (uint8_t i = 0; i < DART_LQR_STATE_NUM; i++) lqr_ctrl.x[i] = 0.0f;
    for (uint8_t i = 0; i < DART_LQR_SERVO_NUM; i++) lqr_ctrl.u_rad[i] = 0.0f;
    for (uint8_t i = 0; i < SERVO_COUNT_X     ; i++) lqr_ctrl.u_servo_deg[i] = 0.0f;
    lqr_ctrl.axis_cmd_deg[0] = lqr_ctrl.axis_cmd_deg[1] = lqr_ctrl.axis_cmd_deg[2] = 0.0f;
    lqr_ctrl.roll_wrap = 0;         /* 与 PID roll 默认对齐(roll 不环绕)；yaw 恒环绕 */
    lqr_ctrl.roll_comp = 0;         /* 默认关(直通=旧 LQR)；台架验 SIGN 后再 Watch 切 1 启用 roll 补偿(见 Euler_LQR_Cale) */
    lqr_ctrl.roll_comp_delta = 0.0f; 

    /* ---- PID-for-LQR 积分器初始化(仅 yaw) ---- */
    PID_struct_init(&pid_i_for_lqr[YAW], POSITION_PID,
                    LQR_I_LIMIT_DEG_DEFAULT,  /* MaxOutput */
                    LQR_I_LIMIT_DEG_DEFAULT,          /* IntegralLimit: iout 限幅(度) */
                    0.0f,                             /* kp=0 */
                    LQR_I_KI_DEFAULT ,                             /* ki: 台架可调 */
                    0.0f,                             /* kd=0 */
                    0.0f, 0.0f);                      /* 前馈关闭 */
    pid_i_for_lqr[YAW].deadband   = 0.0f;             /* ★死区=0：分离阈值是唯一门控 */
    pid_i_for_lqr[YAW].max_err    = 0.0f;
    pid_i_for_lqr[YAW].angle_wrap = 0;                /* yaw 周期角环绕 */
    /* 初始化 MATLAB Coder 运行时（rt_InitInfAndNaN + 标志位） */
    LQR_K_Dart_d_initialize();

    /* 用标称速度算初始 K_d，50Hz 中断会持续覆盖 */
    LQR_Gain_Update50Hz();
    lqr_ctrl.V_lqr = V_DART;
}
