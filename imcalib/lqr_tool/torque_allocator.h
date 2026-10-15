/**
 * @file    torque_allocator.h
 * @brief   飞镖 Pitch 保护型零空间舵面分配器接口
 * @details 输入三轴指令力矩 tau_cmd (N·m)，输出四片舵角 delta (°)。
 *
 *          算法：
 *            1. 先满足 Roll/Yaw 力矩（2×4 子矩阵求最小范数解）
 *            2. 在 Roll/Yaw 零空间内优化：
 *               - 最小化 Pitch 力矩
 *               - 最小化总舵面动作
 *            3. 舵面限幅 + 统一缩放
 *            4. 回算实际力矩
 *
 *          舵面顺序：[UL, UR, DR, DL]
 *          力矩顺序：[Mx(roll), My(pitch), Mz(yaw)]
 *
 *          ⚠ 未编译。待台架。
 *
 * @author  ynz (AI 移植)
 * @date    2026/07/23
 */

#ifndef __TORQUE_ALLOCATOR_H
#define __TORQUE_ALLOCATOR_H

#include "stm32g4xx_hal.h"

/**
 * @brief Pitch 保护型零空间舵面分配器
 * @param torque_cmd_Nm    [in]  3×1 指令力矩 [Mx,My,Mz], N·m
 * @param H_tau            [in]  3×4 力矩矩阵 (N·m/rad)
 * @param N_ry             [in]  4×2 Roll/Yaw 零空间矩阵
 * @param h_pitch          [in]  1×4 H_tau 的 pitch 行
 * @param delta_max_rad    [in]  单舵偏转限幅, rad
 * @param lambda_pitch     [in]  Pitch 力矩惩罚权重
 * @param lambda_servo     [in]  舵面动作惩罚权重
 * @param servo_cmd_deg    [out] 4×1 舵面指令 [UL,UR,DR,DL], °
 * @param torque_achieved  [out] 3×1 实际力矩, N·m
 * @param pitch_moment     [out] 实际 Pitch 力矩, N·m
 * @param sat_mask         [out] bit i = 第 i 片舵面饱和
 * @param infeasible       [out] 分配不可达标志
 */
void Torque_Allocate_PitchProtected(
    const float torque_cmd_Nm[3],
    const float H_tau[3][4],
    const float N_ry[4][2],
    const float h_pitch[4],
    float delta_max_rad,
    float lambda_pitch,
    float lambda_servo,
    float servo_cmd_deg[4],
    float torque_achieved[3],
    float *pitch_moment,
    uint8_t *sat_mask,
    uint8_t *infeasible);

/**
 * @brief 简单伪逆分配器（三轴全满足，最小舵量）
 * @param torque_cmd_Nm    [in]  3×1 指令力矩 [Mx,My,Mz], N·m
 * @param H_tau            [in]  3×4 力矩矩阵 (N·m/rad)
 * @param delta_max_rad    [in]  单舵偏转限幅, rad
 * @param servo_cmd_deg    [out] 4×1 舵面指令 [UL,UR,DR,DL], °
 * @param torque_achieved  [out] 3×1 实际力矩, N·m
 * @param sat_mask         [out] bit i = 第 i 片舵面饱和
 * @param infeasible       [out] 分配不可达标志
 */
void Torque_Allocate_Simple(
    const float torque_cmd_Nm[3],
    const float H_tau[3][4],
    float delta_max_rad,
    float servo_cmd_deg[4],
    float torque_achieved[3],
    uint8_t *sat_mask,
    uint8_t *infeasible);

#endif /* __TORQUE_ALLOCATOR_H */
