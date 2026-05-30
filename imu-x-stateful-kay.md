# Dart_2027 飞控三模块优化方案

## Context

本项目是一个基于 STM32G431 + BMX055 + FreeRTOS 的 Dart 飞镖型飞行器飞控。最近的 commit `f78f964` 已说明 "基础硬件功能都正常，解算需要改为x翼"。三大核心模块（IMU 解算 / 控制算法 / X翼输出）经全量调研发现存在 **若干会让控制环失效的硬性 Bug**，以及大量精度/效率问题。本次优化目标：

1. **修关键 Bug，让 PID 真正工作**：当前 `delta_time` 恒为 0，I/D 项实质失效；四元数积分系数错误，gyro 等效缩到 1/4；舵机部分通道被写死成中位
2. **重构 IMU 解算**：去 double 软浮点 / 加 gyro 零偏校准 / 标准化 Mahony / 修复重力与 M_PI 精度
3. **重写舵面输出为 4 舵 X 翼布局**：用 TIM4 CH1-CH4 (PB6/PB7/PB8/PB9) 接 4 个舵机，按 X 翼标准混控
4. **约束**：不动 SPI 通信、不启用磁力计（用户场景磁干扰大）、PNG 暂不接入主控制环

整个方案分 4 个 Phase，每 Phase 可独立验证、可回滚。

---

## Phase 0: 公共宏统一

### 0.1 新建 [imcalib/User/common_defs.h](imcalib/User/common_defs.h)

```c
#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

#ifndef dT
#define dT          0.001f
#endif

#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif

#define RAD2DEG(x)  ((x) * 57.29577951308232f)
#define DEG2RAD(x)  ((x) * 0.017453292519943295f)

#define IMU_SAMPLE_HZ        1000.0f
#define CTRL_PERIOD_MS       1
#define GYRO_LSB_2000DPS     16.4f
#define ACC_LSB_16G          (1.0f/(16.0f*128.0f))
#define GRAVITY_MS2          9.80665f

#define GYRO_SAT_DPS         1900.0f
#define ACC_SAT_G            15.5f

#endif
```

### 0.2 删除重复宏，三处文件改为 include 公共头
- [imcalib/Task/IMU.h:21](imcalib/Task/IMU.h#L21) 删 `#define dT 0.001f`
- [imcalib/Task/IMU.h:33-34](imcalib/Task/IMU.h#L33-L34) 删两行 `#define M_PI`
- [imcalib/Tool/filter.h:20](imcalib/Tool/filter.h#L20) 删 `#define dT 0.001f`

---

## Phase 1: 关键 Bug 修复（算法核心不动）

### 1.1 ⚠️ prev_tick 重置 Bug（最严重，让 PID 的 I/D 项失效）
[imcalib/Task/surface_control_task.c:296-302](imcalib/Task/surface_control_task.c#L296-L302)

改前：
```c
static uint32_t prev_tick;
prev_tick = xTaskGetTickCount();          // <-- 立即重置！
uint32_t current_tick = xTaskGetTickCount();
float delta_time = (float)(current_tick - prev_tick) * 0.001f;   // 恒为 0
prev_tick = current_tick;
```

改后：
```c
static uint32_t prev_tick = 0;
static uint8_t  first_run = 1;
uint32_t current_tick = xTaskGetTickCount();
float delta_time;
if (first_run) {
    delta_time = (float)CTRL_PERIOD_MS * 0.001f;
    first_run = 0;
} else {
    delta_time = (float)(current_tick - prev_tick) * 0.001f;
    if (delta_time < 1e-4f) delta_time = (float)CTRL_PERIOD_MS * 0.001f;
}
prev_tick = current_tick;
```

### 1.2 ⚠️ 四元数积分系数错误（让 gyro 等效缩 4 倍）
[imcalib/Task/IMU.c:138-141](imcalib/Task/IMU.c#L138-L141)：4 行的 `0.125f*dT` → `0.5f*dT`

### 1.3 ⚠️ PID FFC 空指针写入（UB）
- [imcalib/Tool/pid.h:43](imcalib/Tool/pid.h#L43)：`FFC_t * xFeedForward;` → `FFC_t xFeedForward;`
- [imcalib/Tool/pid.c:144](imcalib/Tool/pid.c#L144)：`FeedForwardParamInit(pid->xFeedForward, ...)` → `FeedForwardParamInit(&pid->xFeedForward, ...)`

### 1.4 重力常数 / Mahony 误差不应乘 gravity
[imcalib/Task/IMU.c:65,109-111](imcalib/Task/IMU.c#L65-L111)：`gravity = 8.886f` → `gravity = GRAVITY_MS2`。Mahony 误差用归一化加速度向量，不参与；`A_World` 计算单位由 g 转 m/s²。

### 1.5 Mahony 改回标准 Kp+Ki，去掉 D 项
[imcalib/Task/IMU.h:23-27](imcalib/Task/IMU.h#L23-L27)：
```c
#define mahony_MAXOUT    10.00f
#define mahony_i_maxout   1.00f
#define mahony_Kp         2.0f
#define mahony_Ki         0.01f
#define mahony_Kd         0.0f
```

[imcalib/Tool/pid.c:85-89](imcalib/Tool/pid.c#L85-L89) D 项前增加守卫：
```c
if (pid->d == 0.0f) {
    pid->dout = 0.0f;
} else {
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]) / delta_time;
    if (delta_time < 1e-6f) pid->dout = 0.0f;
}
```

### 1.6 float 全面 f 后缀（用单精度 FPU 替代软浮点）
[imcalib/Task/IMU.c](imcalib/Task/IMU.c) 全文件改：

| 行号 | 改前 | 改后 |
|---|---|---|
| 28,30,77,143 | `sqrt(...)` | `sqrtf(...)` |
| 159 | `asin(...)` | `asinf(...)` |
| 160-161 | `atan2(...)` | `atan2f(...)` |
| 163-165 | `/M_PI*180` | `RAD2DEG(...)` |
| 318-320 | `/16.4` | `/ GYRO_LSB_2000DPS` |
| 326 | `/180*M_PI` | `DEG2RAD(...)` |
| 335 | `*0.3125` | `*0.3125f` |

### 1.7 数据有效性守卫
[imcalib/Task/IMU.c:308-313, 318-320](imcalib/Task/IMU.c#L308-L320) 加饱和/NaN 守卫：
```c
for (int k = 0; k < 3; k++) {
    if (isnan(IMU_Data.A[NOW][k]) || fabsf(IMU_Data.A[NOW][k]) > ACC_SAT_G)
        IMU_Data.A[NOW][k] = IMU_Data.A[LAST][k];
    if (isnan(IMU_Data.G[NOW][k]) || fabsf(IMU_Data.G[NOW][k]) > GYRO_SAT_DPS)
        IMU_Data.G[NOW][k] = IMU_Data.G[LAST][k];
}
```

### 1.8 Vision_Rx_Data 临界区保护
[imcalib/Task/surface_control_task.c:85,98,103-114](imcalib/Task/surface_control_task.c#L85-L114) 在 task 端读取前 snapshot：
```c
taskENTER_CRITICAL();
Vision_Rx_t v = Vision_Rx_Data;
taskEXIT_CRITICAL();
```
后续 `Guidance_Stable/Terminal` 内全部用 `v.xxx` 替换。

### 1.9 Phase 1 验证
- Vofa 同时画 `IMU_Data.Euler[NOW][0..2]` 与 `IMU_Data.G_Rad[NOW][0..2]`
- 单独绕一轴匀速翻 360°，欧拉角应连续不再有 4 倍突变
- 进入 Stable 后，Vofa 看 `temp[]` 与 `output_gyro_Euler[NOW][]`，改前因 dt=0 全为纯 P，改后应明显出现 I 积累与 D 微分

---

## Phase 2: IMU 解算重构 + Gyro 零偏校准

### 2.1 扩展 IMU_DATA_t
[imcalib/Task/IMU.h:69-85](imcalib/Task/IMU.h#L69-L85) 在 struct 内增加：
```c
float G_Offset[3];
float A_Offset[3];
uint8_t calib_done;
```

### 2.2 新增 IMU_Calibrate（2 秒静态采样）
在 [imcalib/Task/IMU.c](imcalib/Task/IMU.c) 增加：
```c
void IMU_Calibrate(void)
{
    const uint16_t N = 2000;
    float gsum[3] = {0}, asum[3] = {0};
    for (uint16_t n = 0; n < N; n++) {
        IMU_Data_Read();
        for (int i = 0; i < 3; i++) {
            gsum[i] += IMU_Data.G[NOW][i];
            asum[i] += IMU_Data.A[NOW][i];
        }
        HAL_Delay(1);
    }
    for (int i = 0; i < 3; i++) {
        IMU_Data.G_Offset[i] = gsum[i] / N;
        IMU_Data.A_Offset[i] = asum[i] / N;
    }
    IMU_Data.A_Offset[Z] -= 1.0f;
    IMU_Data.calib_done = 1;
}
```
在 `IMU_Init` 完成后、主控制 task 进入循环之前调用一次（建议放 [Core/Src/app_freertos.c](Core/Src/app_freertos.c) IMU task 入口）。校准期间蜂鸣器响一短声。

### 2.3 IMU_Data_Read 减偏
[imcalib/Task/IMU.c:318-326](imcalib/Task/IMU.c#L318-L326)：
```c
IMU_Data.G[NOW][PITCH] = ((int16_t)(rx_buf[2]<<8|rx_buf[1])) / GYRO_LSB_2000DPS;
IMU_Data.G[NOW][ROLL ] = ((int16_t)(rx_buf[4]<<8|rx_buf[3])) / GYRO_LSB_2000DPS;
IMU_Data.G[NOW][YAW  ] = ((int16_t)(rx_buf[6]<<8|rx_buf[5])) / GYRO_LSB_2000DPS;
if (IMU_Data.calib_done) {
    for (int i = 0; i < 3; i++)
        IMU_Data.G[NOW][i] -= IMU_Data.G_Offset[i];
}
for (int i = 0; i < 3; i++) {
    IMU_Data.G_Rad[NOW][i] = DEG2RAD(IMU_Data.G[NOW][i]);
    Surface.current_gyro_Euler[NOW][i] = IMU_Data.G[NOW][i];
}
```

### 2.4 删死代码
[imcalib/Task/IMU.c:200-201](imcalib/Task/IMU.c#L200-L201)：删除 `Velocity[World/Body][LAST][i]` 搬移两行（`Kalman_Vel_Calc` 已注释，搬移无意义）。`Velocity[2][2][3]` 字段保留以减少 struct 波及面。

### 2.5 Phase 2 验证
- 校准结束 LED/蜂鸣器 1 短鸣
- Vofa 看静置 30 秒 yaw 漂移，目标 < 1°/min
- ACC 静置应近似 (0,0,1g)

---

## Phase 3: X 翼输出改造（核心改造）

### 3.1 重写 surface_control_task.h 枚举与维度
[imcalib/Task/surface_control_task.h:9-28](imcalib/Task/surface_control_task.h#L9-L28) 整段替换：

```c
#define Servo_UL_Channel   TIM_CHANNEL_1   /* PB6 - UP_LEFT    */
#define Servo_UR_Channel   TIM_CHANNEL_2   /* PB7 - UP_RIGHT   */
#define Servo_DL_Channel   TIM_CHANNEL_3   /* PB8 - DOWN_LEFT  */
#define Servo_DR_Channel   TIM_CHANNEL_4   /* PB9 - DOWN_RIGHT */

#define Servo_UL_ZERO      1500
#define Servo_UR_ZERO      1500
#define Servo_DL_ZERO      1500
#define Servo_DR_ZERO      1500
#define Servo_PWM_MIN      1000
#define Servo_PWM_MAX      2000

#define SIGN_UL  (+1.0f)
#define SIGN_UR  (+1.0f)
#define SIGN_DL  (+1.0f)
#define SIGN_DR  (+1.0f)

enum { UP_LEFT, UP_RIGHT, DOWN_LEFT, DOWN_RIGHT, SERVO_COUNT };
```

[imcalib/Task/surface_control_task.h:66-76](imcalib/Task/surface_control_task.h#L66-L76) Surface_t 结构：
```c
typedef struct {
    float output_angle_Servo [3][4];     // 3 改 4
    float current_angle_Euler[3][3];
    float target_angle_Euler [3][3];
    float current_gyro_Euler [3][3];
    float output_gyro_Euler  [3][3];
    float Finally_Angle      [3][4];     // 3 改 4
    uint8_t pid_cale_flag;
    uint8_t Text_Flag;
} Surface_t;
```

旧的 `Wing_left_ZERO_POINT / Wing_right_ZERO_POINT / Vertical_fin_ZERO_POINT / Wing_up_Change_Angle / Wing_down_Change_Angle / Vertical_fin_Change_Angle / Wing_left_Channel / Wing_right_Channel / Vertical_fin_Channel` 与 `enum { Wing_left, Wing_right, Vertical_fin }` 全部删除。

### 3.2 受牵连引用点全清单（必改）
- [imcalib/Task/surface_control_task.c:60-63](imcalib/Task/surface_control_task.c#L60-L63) `Servo_Updata` 内层循环 `i<3` → `i<4`
- [imcalib/Task/surface_control_task.c:213-272](imcalib/Task/surface_control_task.c#L213-L272) 三个旧函数 `Wing_left_Control / Wing_right_Control / Vertical_fin_Control` 整体删除（被 3.4 的 `Servo_Output_All` 替代）
- [imcalib/Task/surface_control_task.c:273-290](imcalib/Task/surface_control_task.c#L273-L290) `Wing_Control()` 重写为 `Servo_Output_All`
- [imcalib/Task/surface_control_task.c:314-345](imcalib/Task/surface_control_task.c#L314-L345) `#if DART_TYPE` 整段替换为 X 翼混控（见 3.3）
- [imcalib/Task/surface_control_task.c:346-356](imcalib/Task/surface_control_task.c#L346-L356) Start/End 状态三句 `Wing_xxx` 改为 4 元循环置 0
- [imcalib/Task/TotalControl.c](imcalib/Task/TotalControl.c) 所有 `Surface.Finally_Angle[NOW][Wing_xxx]` 改为 `[NOW][UP_LEFT..DOWN_RIGHT]`
- [imcalib/User/user_lib.c](imcalib/User/user_lib.c) 同枚举改名连带

### 3.3 X 翼标准混控公式（替换原 314-345 段）
```c
if (Guidance_State == Stable || Guidance_State == Terminal) {
    float r = Surface.output_gyro_Euler[NOW][ROLL];
    float p = Surface.output_gyro_Euler[NOW][PITCH];
    float y = Surface.output_gyro_Euler[NOW][YAW];

    Surface.output_angle_Servo[NOW][UP_LEFT]    = SIGN_UL * ( +p + r - y );
    Surface.output_angle_Servo[NOW][UP_RIGHT]   = SIGN_UR * ( +p - r + y );
    Surface.output_angle_Servo[NOW][DOWN_LEFT]  = SIGN_DL * ( -p + r + y );
    Surface.output_angle_Servo[NOW][DOWN_RIGHT] = SIGN_DR * ( -p - r - y );
}
for (int i = 0; i < 4; i++) {
    Surface.output_angle_Servo[NOW][i] = Low_Pass_Filter(
        Surface.output_angle_Servo[NOW][i],
        Surface.output_angle_Servo[LAST][i], 0.7f);     // 修复返回值丢弃 bug
    abs_limit(&Surface.output_angle_Servo[NOW][i], 60.0f);
}
```
**符号原则**：`SIGN_xx` 在台架联调时单独打 1 个轴的阶跃，Vofa 看哪两个舵反向了再翻号；不要直接动公式。

### 3.4 Servo_Output_All（统一输出函数，替代旧 273-290）
```c
static inline uint16_t pwm_clip(float us) {
    if (us < Servo_PWM_MIN) return Servo_PWM_MIN;
    if (us > Servo_PWM_MAX) return Servo_PWM_MAX;
    return (uint16_t)us;
}
void Servo_Output_All(void)
{
    Surface.Finally_Angle[NOW][UP_LEFT]    = Servo_UL_ZERO + Surface.output_angle_Servo[NOW][UP_LEFT]    / 90.0f * 1000.0f;
    Surface.Finally_Angle[NOW][UP_RIGHT]   = Servo_UR_ZERO + Surface.output_angle_Servo[NOW][UP_RIGHT]   / 90.0f * 1000.0f;
    Surface.Finally_Angle[NOW][DOWN_LEFT]  = Servo_DL_ZERO + Surface.output_angle_Servo[NOW][DOWN_LEFT]  / 90.0f * 1000.0f;
    Surface.Finally_Angle[NOW][DOWN_RIGHT] = Servo_DR_ZERO + Surface.output_angle_Servo[NOW][DOWN_RIGHT] / 90.0f * 1000.0f;
    __HAL_TIM_SET_COMPARE(&htim4, Servo_UL_Channel, pwm_clip(Surface.Finally_Angle[NOW][UP_LEFT]));
    __HAL_TIM_SET_COMPARE(&htim4, Servo_UR_Channel, pwm_clip(Surface.Finally_Angle[NOW][UP_RIGHT]));
    __HAL_TIM_SET_COMPARE(&htim4, Servo_DL_Channel, pwm_clip(Surface.Finally_Angle[NOW][DOWN_LEFT]));
    __HAL_TIM_SET_COMPARE(&htim4, Servo_DR_Channel, pwm_clip(Surface.Finally_Angle[NOW][DOWN_RIGHT]));
}
```

### 3.5 补全 TIM4 CH1 (PB6)
[Core/Src/tim.c:206](Core/Src/tim.c#L206) `USER CODE BEGIN TIM4_Init 2` 区域新增：
```c
HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
```

[Core/Src/tim.c:455-465](Core/Src/tim.c#L455-L465) `HAL_TIM_MspPostInit` 的 TIM4 段：`GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9` 改为 `GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9`，注释加 `PB6 ------> TIM4_CH1`。

[imcalib/User/user_lib.c:35](imcalib/User/user_lib.c#L35) `HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1)` 已存在，无需新增。

### 3.6 Phase 3 验证
- 单舵测试：临时把 `output_angle_Servo[NOW][i] = sinf(t)*30.0f` 逐路灌入，确认 CH1-CH4 全活、4 个舵都能独立摆动
- Vofa 显示 4 路 `Finally_Angle[NOW][0..3]`，所有值必须在 [1000,2000] 内
- 手摇飞镖产生姿态扰动，4 舵反应方向必须满足：
  - **抬头** → UL/UR 同向，DL/DR 反向
  - **左滚** → UL/DL 同向、UR/DR 反向
  - **左偏航** → UL/DR 同向、UR/DL 反向
  - 任一不符立刻翻该舵 `SIGN_xx`，不要改公式

---

## Phase 4: 控制算法精修

### 4.1 D 项低通（消噪）
[imcalib/Tool/pid.h](imcalib/Tool/pid.h) struct 加：
```c
float d_lpf_alpha;
float dout_last;
```

[imcalib/Tool/pid.c:85](imcalib/Tool/pid.c#L85) D 项计算改为：
```c
float d_raw = pid->d * (pid->err[NOW] - pid->err[LAST]) / delta_time;
float alpha = (pid->d_lpf_alpha > 0.0f) ? pid->d_lpf_alpha : 0.2f;
pid->dout = alpha * d_raw + (1.0f - alpha) * pid->dout_last;
pid->dout_last = pid->dout;
```
[imcalib/Tool/pid.c:153-164](imcalib/Tool/pid.c#L153-L164) `pid_param_init` 末尾加 `pid->dout_last = 0.0f; pid->d_lpf_alpha = 0.2f;`。

### 4.2 PID 失能开关（便于调参）
[imcalib/Tool/pid.h](imcalib/Tool/pid.h) struct 加 `uint8_t enable;`（默认 1）。
[imcalib/Tool/pid.c:71](imcalib/Tool/pid.c#L71) `pid_calc` 入口加 `if (!pid->enable) return 0;`。

### 4.3 任务节拍稳态化
所有 `osDelay(1)` 改为 `vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(CTRL_PERIOD_MS));`，task 入口加 `static TickType_t xLastWake; if (!xLastWake) xLastWake = xTaskGetTickCount();`。

### 4.4 Phase 4 验证
- Vofa 同时画 `pid->pout/iout/dout`，D 项应明显平滑
- 阶跃 5° 目标，响应无明显高频抖动
- 长时间运行 >5 min 无 HardFault

---

## 关键文件清单

**新建**
- [imcalib/User/common_defs.h](imcalib/User/common_defs.h)

**修改**
- [imcalib/Task/IMU.h](imcalib/Task/IMU.h) — 删宏 / 加 G_Offset / Mahony 参数
- [imcalib/Task/IMU.c](imcalib/Task/IMU.c) — 四元数系数、float 后缀、零偏减、gravity、Mahony 守卫、IMU_Calibrate
- [imcalib/Task/surface_control_task.h](imcalib/Task/surface_control_task.h) — 4 舵枚举、宏、struct 维度
- [imcalib/Task/surface_control_task.c](imcalib/Task/surface_control_task.c) — prev_tick fix、X 翼混控、Servo_Output_All、Low_Pass 返回值、Vision 临界区
- [imcalib/Tool/pid.h](imcalib/Tool/pid.h) — xFeedForward 改内嵌、enable、d_lpf
- [imcalib/Tool/filter.h](imcalib/Tool/filter.h) — 删 dT 宏
- [Core/Src/tim.c](Core/Src/tim.c) — TIM4 CH1 ConfigChannel + PB6 GPIO
- [imcalib/Task/TotalControl.c](imcalib/Task/TotalControl.c) — 枚举改名连带
- [imcalib/Task/CallBack_Task.c](imcalib/Task/CallBack_Task.c) — Vision 写侧无需改（仅读侧加临界区）
- [imcalib/User/user_lib.c](imcalib/User/user_lib.c) — 枚举改名连带

---

## 风险与回滚

1. **gravity 单位变更**：A_World 量级从 g 变 m/s²，仅影响 Vofa 显示，控制环不依赖
2. **X 翼正负号**：必须台架先单轴阶跃校 4 个 SIGN，**绝不直接上飞行**
3. **Mahony Ki**：建议从 0.005 起步，过大易被加速度低频干扰
4. **每个 Phase 提交一次 git commit + tag**，便于单步回滚
5. **gyro 零偏校准期间飞镖必须静止**，2 秒内任何抖动都会污染零偏值

## 端到端验证流程

1. **Phase 0/1 完成后**：编译过 → 上电 → Vofa 看姿态 → 进入 Stable → 看 PID 三分量是否正常
2. **Phase 2 完成后**：上电 → 等校准蜂鸣 → 静置 30s 看 yaw 漂移率
3. **Phase 3 完成后**：台架单舵测试 → 阶跃 SIGN 校准 → 三轴姿态扰动看 4 舵响应方向
4. **Phase 4 完成后**：Vofa 看 D 项平滑度 → 阶跃响应 → 长时间稳定性
5. **整机验收**：固定到测试架，给目标姿态阶跃，观察 4 舵协调收敛

IMU.h — 把原来陀螺/加速度共用的 IMU_KF_R=5000 拆成两路独立可调:

改前	改后	效果
陀螺 GYR_KF_R	5000(延迟~70ms)	30(延迟~6ms)	保住相位,融合拿到实时角速度
加速度 ACC_KF_R	5000	500(fc~7Hz)	脏信号适度平滑,够用
IMU.c:316-318/337-339 — acc/gyr 两处卡尔曼调用改用各自的新宏。

Mahony 参数复核保持 PI(Kp=2/Ki=0.01/Kd=0)并补了调参注释。

逻辑就是我们讨论的:陀螺以前进 Mahony 前被重滤波废掉了实时性,现在让它干净低延迟地进融合,把"去噪"这件事交给 Mahony 的预测-校正去做,而不是用笨重低通先把相位毁掉。改动只是宏和参数,不涉及结构,编译无风险(IMU_KF_Q/R 已无残留引用)。

FFC 前馈接入 + 串级内外环增益拉开 + 死区软化。卡尔曼状态观测器我已记入项目 TODO,后续有需求再做。
