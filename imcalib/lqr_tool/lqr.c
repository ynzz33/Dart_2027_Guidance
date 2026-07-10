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

/*============================================================================
 *  ★★★ K 矩阵粘贴区 ★★★
 *  从 MATLAB(dart_attitude_LQR_v1.m) Step5 输出复制 4 行数值，整体覆盖到这里。
 *  行 = 4 个舵(MATLAB delta1..delta4)，列 = 6 个状态[roll_err,pitch_err,yaw_err,p,q,r]。
 *  当前为脚本占位参数(占位惯量/气动/QR)导出的值，台架前必须用真实参数重跑 MATLAB 更新。
 *  注意：保留 float 精度，别手动四舍五入到太少位数。
 *============================================================================*/
float dart_lqr_K[DART_LQR_SERVO_NUM][DART_LQR_STATE_NUM] = {




    {-0.60756979845502246, -0.27200540115909788, 0.29379300472243813, -0.32361332019090711, -0.37186397456477854, 0.37349076858548441},
    {-0.60756979845503545, 0.27200540115900668, 0.29379300472245867, -0.32361332019090738, 0.37186397456476578, 0.3734907685854868},
    {-0.60756979845504633, 0.27200540115906036, -0.29379300472243208, -0.32361332019090755, 0.37186397456477249, -0.37349076858548397},
    {-0.60756979845503345, -0.27200540115904892, -0.29379300472245262, -0.32361332019090788, -0.37186397456477427, -0.37349076858548641}




};

/*============================================================================
 *  全局实例 + 运行时旋钮
 *============================================================================*/
LQR_t lqr_ctrl;   /* Vofa/Watch 可观测：状态 x、原始/限幅后舵偏、K_d(V) */

/* 舵偏限幅(rad)，与 MATLAB delta_max_ac=deg2rad(60) 及工程 SERVO_ANGLE_LIMIT(60°) 对齐 */
float dart_delta_max_rad = 1.0471975511965976f;   /* ±60° */

/* pitch 轴门控：1=只在制导段(Terminal)+检测到目标时控俯仰(补偿)，其他阶段/丢目标时 pitch 不受控、
 *   飞镖俯仰主要靠镖架初始动力；0=全程控 pitch(旧行为)。调试器 Watch 在线切换做 A/B。详见 Euler_LQR_Cale 内说明。*/
uint8_t lqr_pitch_terminal_only = 1;

/* 速度调度开关：1=用 lqr_ctrl.K_d(由 50Hz 中断更新，默认)；0=退回旧静态 dart_lqr_K */
uint8_t lqr_use_scheduled_K = 1;

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
    const float (*K)[DART_LQR_STATE_NUM] =
        lqr_use_scheduled_K ? lqr_ctrl.K_d : dart_lqr_K;

    for (uint8_t i = 0; i < DART_LQR_SERVO_NUM; i++)
    {
        float ui = 0.0f;
        for (uint8_t j = 0; j < DART_LQR_STATE_NUM; j++)
            ui -= K[i][j] * x[j];

        if (ui >  dart_delta_max_rad) ui =  dart_delta_max_rad;
        if (ui < -dart_delta_max_rad) ui = -dart_delta_max_rad;
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
    (void)dt;

    /* 1) 姿态误差 err = 测量 − 期望(配平/目标)，存入 lqr_ctrl.err_deg[roll,pitch,yaw](度，可观测)，
     *    再转 rad 填 x[0..2]。yaw 为周期角须环绕到 ±180°；roll 按 roll_wrap 选择是否环绕。
     *    工程欧拉/角速度索引为 [PITCH,ROLL,YAW]；状态 x 顺序为 [roll,pitch,yaw,p,q,r]。*/
    lqr_ctrl.err_deg[0] = Surface.current_angle_Euler[NOW][PITCH] - Surface.target_angle_Euler[NOW][PITCH];
    lqr_ctrl.err_deg[1] = Surface.current_angle_Euler[NOW][ROLL]  - Surface.target_angle_Euler[NOW][ROLL];
    lqr_ctrl.err_deg[2] = Surface.current_angle_Euler[NOW][YAW]   - Surface.target_angle_Euler[NOW][YAW];
    if (lqr_ctrl.roll_wrap) lqr_ctrl.err_deg[0] = Angle_Wrap_180(lqr_ctrl.err_deg[0]);
    lqr_ctrl.err_deg[2] = Angle_Wrap_180(lqr_ctrl.err_deg[2]);   /* yaw 始终环绕，防跨 ±180° 爆冲 */

    /* 1.1) 基于 roll 的机体系补偿(roll_comp=1)：把世界系 pitch/yaw 误差按当前横滚反旋到机体系。
     *   依据：角度误差取自世界系 ZYX 欧拉(current_angle_Euler)，但 LQR 的 K/B 是机体舵效；机身横滚
     *   Δ=current_roll−Stable_roll 后，世界系 (pitch_err,yaw_err) 要在机体系里旋一个 Δ 才对得上舵面。
     *   只反旋角度误差的 pitch/yaw 这一对：roll_err 绕纵轴不变、p/q/r(x[3..5]) 已是机体陀螺(IMU.c) → 都不旋。
     *   复用 Roll_Derotate_PitchYaw(与 PID 同一旋转、同一 ROLL_WORLD_COMP_SIGN)；★用独立临时量收结果，
     *   绝不传同一变量当输入兼输出——该函数内部 *Pb=…Pw…; *Yb=…Pw… 会被 in-place 别名覆盖算错。
     *   roll_comp=0 时直通=旧 LQR 行为(A/B 对照)。Stable 段 Δ≈0 自然恒等，主要在 Terminal 横滚时生效。*/
    float err_pitch = lqr_ctrl.err_deg[0];
    float err_roll  = lqr_ctrl.err_deg[1];
    float err_yaw   = lqr_ctrl.err_deg[2]; 

    lqr_ctrl.x[0] = DEG2RAD(err_roll) ;               /* roll 误差(绕纵轴，不反旋) */
    lqr_ctrl.x[1] = DEG2RAD(err_pitch) ;             /* pitch 误差(roll_comp=1 时为反旋后机体系；err_deg[1] 仍存反旋前世界值供对照) */
    lqr_ctrl.x[2] = DEG2RAD(err_yaw) ;               /* yaw   误差(同上) */

    lqr_ctrl.x[3] = DEG2RAD(Surface.current_gyro_Euler[NOW][ROLL]) ;
    lqr_ctrl.x[4] = DEG2RAD(Surface.current_gyro_Euler[NOW][PITCH])  ;
    lqr_ctrl.x[5] = DEG2RAD(Surface.current_gyro_Euler[NOW][YAW])   ;

    /* pitch 仅制导段(Terminal)受控:发射前/稳定段 pitch 不打舵(靠镖架初动力),进入 Terminal 就全程放开;
     * pitch 目标由 Guidance_Terminal 的主动滑翔→扎给出(远段住 THETA_GLIDE 压平增程、看灯视线变陡再平滑扎下)。
     * ★原 dist_cm>400 硬开关(4m 处 pitch 从"不控"突跳到"全控追视觉")已去掉——那个跳变改由 blend 平滑过渡承担。
     *   lqr_pitch_terminal_only=0 → 全程控 pitch(旧行为)做 A/B 对照。*/
    if (lqr_pitch_terminal_only && Guidance_State < Terminal&&Surface.current_angle_Euler[NOW][PITCH] > -5.0f)
    {
        lqr_ctrl.x[1] = 0.0f;   /* pitch 误差不进控制 */
        lqr_ctrl.x[4] = 0.0f;   /* q(俯仰角速度阻尼)不进控制 */
    }

    /* 2) LQR 解算(rad)，已限幅 */
    LQR_Update(lqr_ctrl.x, lqr_ctrl.u_rad);

    /* 2.5) 仅观测：从 u_rad 反解等效三轴指令(度)，类比 PID 的 output_gyro_Euler。
     *      u_rad 为 MATLAB 舵号 delta1..4=[右上UR,左上UL,左下DL,右下DR]，按理想 X 翼 G 行模式投影：
     *        roll  ∝ 四片同向和；pitch ∝ (δ1−δ2−δ3+δ4)；yaw ∝ (δ1+δ2−δ3−δ4)。
     *      ×0.25 归一成"等效对称舵偏度数"。不参与控制，仅供观测对照。*/
    {
        float d1 = lqr_ctrl.u_rad[0], d2 = lqr_ctrl.u_rad[1];
        float d3 = lqr_ctrl.u_rad[2], d4 = lqr_ctrl.u_rad[3];
        lqr_ctrl.axis_cmd_deg[ROLL]  = RAD2DEG((d1 + d2 + d3 + d4) * 0.25f);
        lqr_ctrl.axis_cmd_deg[PITCH] = RAD2DEG((d1 - d2 - d3 + d4) * 0.25f);
        lqr_ctrl.axis_cmd_deg[YAW]   = RAD2DEG((d1 + d2 - d3 - d4) * 0.25f);
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
    if (!lqr_use_scheduled_K) return;

    /* 1) 取当前飞行速度标量(m/s)，从 EKF 世界速度算模长 */
    float vx = IMU_Data.Velocity[World][NOW][X];
    float vy = IMU_Data.Velocity[World][NOW][Y];
    float vz = IMU_Data.Velocity[World][NOW][Z];
    float V = sqrtf(vx * vx + vy * vy + vz * vz);

    /* 2) clamp 到拟合范围 */
    if (V < DART_LQR_V_MIN) V = DART_LQR_V_MIN;
    if (V > DART_LQR_V_MAX) V = DART_LQR_V_MAX;
    lqr_ctrl.V_lqr = V;

    /* 3) 调 MATLAB Coder 生成的方程（double 精度） */
    double K_flat[24];
    LQR_K_Dart_d((double)V, K_flat);

    /* 4) 列优先→行优先，写入 lqr_ctrl.K_d
     *    MATLAB 按列排：K_flat[0..3]=col0(roll_err), K_flat[4..7]=col1(pitch_err), ... */
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
    for (uint8_t i = 0; i < SERVO_COUNT_X;      i++) lqr_ctrl.u_servo_deg[i] = 0.0f;
    lqr_ctrl.axis_cmd_deg[0] = lqr_ctrl.axis_cmd_deg[1] = lqr_ctrl.axis_cmd_deg[2] = 0.0f;
    lqr_ctrl.roll_wrap = 0;         /* 与 PID roll 默认对齐(roll 不环绕)；yaw 恒环绕 */
    lqr_ctrl.roll_comp = 0;         /* 默认关(直通=旧 LQR)；台架验 SIGN 后再 Watch 切 1 启用 roll 补偿(见 Euler_LQR_Cale) */
    lqr_ctrl.roll_comp_delta = 0.0f;

    /* 初始化 MATLAB Coder 运行时（rt_InitInfAndNaN + 标志位） */
    LQR_K_Dart_d_initialize();

    /* 用标称速度算初始 K_d，50Hz 中断会持续覆盖 */
    LQR_Gain_Update50Hz();
    lqr_ctrl.V_lqr = DART_LQR_V_NOM;   /* 初始值覆盖为标称速度(首次 Update 可能用 clamp 后的值) */
}
