/**
 * @file    lqr.c
 * @brief   飞镖 X 翼姿态 LQR 控制器（移植自 lqr_czn/dart_attitude_LQR_v1.m）
 * @details 一步从"姿态误差+角速度(6态)"解出 u = -K_d·x，再把 4 舵解反投影成三轴力矩需求，
 *          写 output_gyro_Euler，复用工程已台架验证的 Roll反旋 + Servo_Mix_* 混控链分配到 4 舵。
 *
 *          ★ 与 PID/LADRC 的关系：三者输出口径一致(都写 output_gyro_Euler 三轴力矩需求)，
 *            LQR 只是把"角度环+角速度阻尼"用单步状态反馈一次算完，混控/符号/装配复用同一条链。
 *
 *          ★ 历史：曾试过"LQR 直接出 4 舵 ×SIGN"，但 MATLAB G 的混控/符号约定与工程 C×SIGN 冲突，
 *            会把 pitch/roll 打成共模且 roll/pitch 反号(台架实测)。故改为只出三轴需求、复用现成混控。
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
#include "IMU.h"                     /* NOW 等历史槽枚举 */
#include "pid.h"                     /* abs_limit / Angle_Wrap_180 复用 */
#include "common_defs.h"            /* DEG2RAD / RAD2DEG */

/*============================================================================
 *  ★★★ K 矩阵粘贴区 ★★★
 *  从 MATLAB(dart_attitude_LQR_v1.m) Step5 输出复制 4 行数值，整体覆盖到这里。
 *  行 = 4 个舵(MATLAB delta1..delta4)，列 = 6 个状态[roll_err,pitch_err,yaw_err,p,q,r]。
 *  当前为脚本占位参数(占位惯量/气动/QR)导出的值，台架前必须用真实参数重跑 MATLAB 更新。
 *  注意：保留 float 精度，别手动四舍五入到太少位数。
 *============================================================================*/
float dart_lqr_K[DART_LQR_SERVO_NUM][DART_LQR_STATE_NUM] = {



    {0.47777069977510783, 0.49792246672720919, 1.5715597790783886, 0.15667095274685194, 0.22846983891393896, 0.33361239150210648},
    {0.47777069977503522, -0.49792246672725388, 1.5715597790783933, 0.15667095274685117, -0.22846983891394126, 0.33361239150211958},
    {0.47777069977470149, -0.49792246672712648, -1.571559779078191, 0.15667095274684809, -0.22846983891392941, -0.33361239150208238},
    {0.47777069977477338, 0.49792246672733587, -1.5715597790781963, 0.15667095274684883, 0.22846983891395087, -0.33361239150209587}



};

/*============================================================================
 *  全局实例 + 运行时旋钮
 *============================================================================*/
LQR_t lqr_ctrl;   /* Vofa/Watch 可观测：状态 x、原始/限幅后舵偏 */

/* 舵偏限幅(rad)，与 MATLAB delta_max_ac=deg2rad(60) 及工程 SERVO_ANGLE_LIMIT(60°) 对齐 */
float dart_delta_max_rad = 1.0471975511965976f;   /* ±60° */

/*============================================================================
 *  核心：纯 LQR 解算 u = -K_d·x（与 MATLAB / C 对拍用，不依赖工程任何全局）
 *  x 单位 rad / rad·s⁻¹；u 单位 rad，已按 ±dart_delta_max_rad 限幅。
 *  ⚠ 控制律带负号；不要把 K 预先取负(否则需改名 minus_K)。
 *============================================================================*/
void LQR_Update(const float x[DART_LQR_STATE_NUM], float u[DART_LQR_SERVO_NUM])
{
    for (uint8_t i = 0; i < DART_LQR_SERVO_NUM; i++)
    {
        float ui = 0.0f;
        for (uint8_t j = 0; j < DART_LQR_STATE_NUM; j++)
            ui -= dart_lqr_K[i][j] * x[j];

        if (ui >  dart_delta_max_rad) ui =  dart_delta_max_rad;
        if (ui < -dart_delta_max_rad) ui = -dart_delta_max_rad;
        u[i] = ui;
    }
}

/*============================================================================
 *  桥接：从 Surface 取姿态/角速度 → 组状态 x → LQR → 反投影成三轴力矩需求写 output_gyro_Euler。
 *  之后由 surface_control_task.c 的 Roll反旋 + Servo_Mix_*(按 Alloc.Mode)分配到 4 舵，与 PID/LADRC 同链。
 *  调用点：lqr_mode==1 时调用本函数(替代 Euler_pid_Cale/Euler_LADRC_Cale)，混控分派照常执行。
 *
 *  @note dt 当前未参与计算(纯静态增益 LQR，无积分/状态记忆)，保留入参以便后续扩展。
 *============================================================================*/
void Euler_LQR_Cale(float dt)
{
    (void)dt;

    /* 1) 姿态误差 err = 测量 − 期望(配平/目标)，存入 lqr_ctrl.err_deg[roll,pitch,yaw](度，可观测)，
     *    再转 rad 填 x[0..2]。yaw 为周期角须环绕到 ±180°；roll 按 roll_wrap 选择是否环绕。
     *    工程欧拉/角速度索引为 [PITCH,ROLL,YAW]；状态 x 顺序为 [roll,pitch,yaw,p,q,r]。*/
    lqr_ctrl.err_deg[0] = Surface.current_angle_Euler[NOW][ROLL]  - Surface.target_angle_Euler[NOW][ROLL];
    lqr_ctrl.err_deg[1] = Surface.current_angle_Euler[NOW][PITCH] - Surface.target_angle_Euler[NOW][PITCH];
    lqr_ctrl.err_deg[2] = Surface.current_angle_Euler[NOW][YAW]   - Surface.target_angle_Euler[NOW][YAW];
    if (lqr_ctrl.roll_wrap) lqr_ctrl.err_deg[0] = Angle_Wrap_180(lqr_ctrl.err_deg[0]);
    lqr_ctrl.err_deg[2] = Angle_Wrap_180(lqr_ctrl.err_deg[2]);   /* yaw 始终环绕，防跨 ±180° 爆冲 */

    lqr_ctrl.x[0] = DEG2RAD(lqr_ctrl.err_deg[0]);
    lqr_ctrl.x[1] = DEG2RAD(lqr_ctrl.err_deg[1]);
    lqr_ctrl.x[2] = DEG2RAD(lqr_ctrl.err_deg[2]);

    /* 角速度 p/q/r：用与 PID 内环同源的 current_gyro_Euler(°/s)，乘 gyro_sign 校正后转 rad/s。
     * gyro_sign 默认全 +1；台架若某轴阻尼反向(一动就发散)→把对应符号翻 −1(指南 §9 第4步)。*/
    lqr_ctrl.x[3] = DEG2RAD(lqr_ctrl.gyro_sign[0] * Surface.current_gyro_Euler[NOW][ROLL]);
    lqr_ctrl.x[4] = DEG2RAD(lqr_ctrl.gyro_sign[1] * Surface.current_gyro_Euler[NOW][PITCH]);
    lqr_ctrl.x[5] = DEG2RAD(lqr_ctrl.gyro_sign[2] * Surface.current_gyro_Euler[NOW][YAW]);

    /* 2) LQR 解算(rad)，已限幅 */
    LQR_Update(lqr_ctrl.x, lqr_ctrl.u_rad);

    /* 3) 把 4 舵解反投影成"三轴力矩需求(度)"，写 output_gyro_Euler —— 与 PID/LADRC 完全同一接口，
     *    之后复用工程已台架验证的 Roll 反旋 + Servo_Mix_* 混控链，由那条链统一处理舵面分配/符号/装配 SIGN。
     *
     *    ★ 为什么不再"LQR 直接出 4 舵 ×SIGN"(旧实现)：
     *      MATLAB G 的混控/符号约定与工程 C×SIGN 不一致——直接 ×SIGN 会把 pitch/roll 打成共模、
     *      且 roll/pitch 整体反号。改走"只出三轴需求、复用现成混控"从根上消除两套约定冲突。
     *
     *    投影：u_rad 为 MATLAB 舵号 δ1..δ4=[右上UR,左上UL,左下DL,右下DR]，按理想 X 翼 G 行模式取：
     *      roll ∝ 四片同向和；pitch ∝ (δ1−δ2−δ3+δ4)；yaw ∝ (δ1+δ2−δ3−δ4)；×0.25 归一成等效舵偏度数。
     *    其符号 ∝ (目标−当前)，与 PID 的 output_gyro_Euler 同号，故可直接喂同一条混控链。*/
    float d1 = lqr_ctrl.u_rad[0], d2 = lqr_ctrl.u_rad[1];
    float d3 = lqr_ctrl.u_rad[2], d4 = lqr_ctrl.u_rad[3];
    lqr_ctrl.axis_cmd_deg[ROLL]  = RAD2DEG((d1 + d2 + d3 + d4) * 0.25f);
    lqr_ctrl.axis_cmd_deg[PITCH] = RAD2DEG((d1 - d2 - d3 + d4) * 0.25f);
    lqr_ctrl.axis_cmd_deg[YAW]   = RAD2DEG((d1 + d2 - d3 - d4) * 0.25f);

    /* 写三轴力矩需求 → 交给后续 Roll_Derotate + Servo_Mix_*(按 Alloc.Mode 分派)分配到 4 舵。
     * 与 Euler_pid_Cale / Euler_LADRC_Cale 输出口径一致。*/
    Surface.output_gyro_Euler[NOW][PITCH] = lqr_ctrl.axis_cmd_deg[PITCH];
    Surface.output_gyro_Euler[NOW][ROLL]  = lqr_ctrl.axis_cmd_deg[ROLL];
    Surface.output_gyro_Euler[NOW][YAW]   = lqr_ctrl.axis_cmd_deg[YAW];
}

/*============================================================================
 *  初始化：清零状态、默认符号、限幅(在 TotalInitTask 调一次即可)
 *============================================================================*/
void LQR_Init(void)
{
    for (uint8_t i = 0; i < DART_LQR_STATE_NUM; i++) lqr_ctrl.x[i] = 0.0f;
    for (uint8_t i = 0; i < DART_LQR_SERVO_NUM; i++) lqr_ctrl.u_rad[i] = 0.0f;
    lqr_ctrl.axis_cmd_deg[0] = lqr_ctrl.axis_cmd_deg[1] = lqr_ctrl.axis_cmd_deg[2] = 0.0f;
    lqr_ctrl.gyro_sign[0] = 1.0f;   /* roll  */
    lqr_ctrl.gyro_sign[1] = 1.0f;   /* pitch */
    lqr_ctrl.gyro_sign[2] = 1.0f;   /* yaw   */
    lqr_ctrl.roll_wrap = 0;         /* 与 PID roll 默认对齐(roll 不环绕)；yaw 恒环绕 */
}
