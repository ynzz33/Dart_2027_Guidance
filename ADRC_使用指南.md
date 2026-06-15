# ADRC自抗扰控制器 - 使用指南

## 📋 目录
1. [ADRC原理简介](#adrc原理简介)
2. [文件说明](#文件说明)
3. [集成步骤](#集成步骤)
4. [参数调优指南](#参数调优指南)
5. [调试技巧](#调试技巧)
6. [常见问题](#常见问题)

---

## ADRC原理简介

### 为什么需要ADRC？

传统PID的矛盾：
- **Kp大了** → 响应快但抖动
- **Kp小了** → 稳定但响应慢
- **遇到扰动** → 被动响应，总是慢一拍

ADRC的核心思想：
1. **不依赖精确模型** → 通过ESO实时估计"总扰动"
2. **主动补偿扰动** → 知道风多大，直接抵消
3. **非线性增益** → 误差大时猛打，误差小时轻柔

### ADRC三大组件

```
                ┌─────────────┐
    目标 ──────▶│  TD跟踪微分器 │──────▶ 平滑目标 + 目标微分
                └─────────────┘
                        │
                        ▼
                ┌─────────────┐
                │  ESO状态观测器 │◀──── 测量值 + 控制量
                └─────────────┘
                        │
                ┌───────┴───────┐
                ▼               ▼
           状态估计          扰动估计
           (角度/角速度)      (z3)
                │               │
                ▼               │
                ┌─────────────┐ │
    目标微分 ──▶│ NLSEF非线性  │◀┘
                │  误差反馈    │
                └─────────────┘
                        │
                        ▼
                    控制输出
                 (已补偿扰动)
```

### 各组件作用

| 组件 | 作用 | 比喻 |
|------|------|------|
| **TD** | 安排过渡过程，不让目标突变 | "你慢慢来，我跟得上" |
| **ESO** | 估计系统状态和总扰动 | "我知道风在吹我" |
| **NLSEF** | 非线性控制 + 扰动补偿 | "该出手时出手，该收手时收手" |

---

## 文件说明

```
imcalib/Tool/
├── adrc.h      # ADRC头文件（数据结构、接口声明）
└── adrc.c      # ADRC实现（TD、ESO、NLSEF算法）

ADRC_使用指南.md  # 本文档
```

### 关键数据结构

```c
ADRC_t adrc_ctrl[3][2];  // [通道][环]
// 通道: ADRC_PITCH=0, ADRC_ROLL=1, ADRC_YAW=2
// 环:   ADRC_ANGLE_LOOP=0(外环), ADRC_GYRO_LOOP=1(内环)
```

---

## 集成步骤

### 第一步：添加文件

将 `adrc.h` 和 `adrc.c` 添加到项目中。

### 第二步：修改 `surface_control_task.c`

在 `surface_control_task` 函数中，将：

```c
Euler_pid_Cale(delta_time);
```

改为：

```c
if (adrc_enable)
{
    Euler_ADRC_Cale(delta_time);  // 使用ADRC
}
else
{
    Euler_pid_Cale(delta_time);   // 使用原PID
}
```

### 第三步：添加头文件

在 `surface_control_task.c` 顶部添加：

```c
#include "adrc.h"
```

### 第四步：添加切换命令（可选）

通过Vofa或调试器切换：

```c
// 运行时切换
adrc_enable = 1;  // 切换到ADRC
adrc_enable = 0;  // 切换回PID
```

---

## 参数调优指南

### 调参顺序

**重要：按以下顺序调参，不要跳步！**

```
1. ESO带宽 (w0)      ← 最重要，先调这个
2. TD速度 (r)        ← 影响跟踪速度
3. NLSEF带宽 (wc)    ← 影响响应速度
4. ESO增益 (b0)      ← 影响扰动补偿强度
```

### 参数详解

#### 1. ESO带宽 (w0) - 核心参数

```c
// 在 adrc.c 的 ADRC_Init_AngleLoop 中
ESO_SetBandwidth(&adrc->eso, 15.0f);  // 角度环
ESO_SetBandwidth(&adrc->eso, 80.0f);  // 角速度环
```

**调参指南：**

| w0值 | 效果 | 适用场景 |
|------|------|---------|
| 5~10 | 跟踪慢，噪声小 | 噪声大的系统 |
| 15~30 | 中等 | **角度环推荐** |
| 50~100 | 跟踪快，噪声敏感 | **角速度环推荐** |
| >150 | 可能振荡 | 谨慎使用 |

**调参方法：**
1. 从较小值开始（如10）
2. 逐渐增大，观察扰动估计z3
3. z3应该能跟踪真实扰动，但不要太抖
4. 如果z3抖动厉害，减小w0

#### 2. TD速度 (r) - 跟踪速度

```c
adrc->td.r = 100.0f;  // 角度环
adrc->td.r = 300.0f;  // 角速度环
```

**调参指南：**

| r值 | 效果 |
|-----|------|
| 50 | 跟踪慢，过渡平滑 |
| 100~200 | 中等，**角度环推荐** |
| 200~500 | 快速，**角速度环推荐** |
| >1000 | 可能超调 |

**调参方法：**
1. 如果目标变化时系统抖动，减小r
2. 如果响应太慢，增大r
3. r太大时，TD输出会接近阶跃，失去平滑作用

#### 3. NLSEF带宽 (wc) - 控制响应

```c
NLSEF_SetBandwidth(&adrc->nlsef, 12.0f);  // 角度环
NLSEF_SetBandwidth(&adrc->nlsef, 50.0f);  // 角速度环
```

**调参指南：**

| wc值 | 效果 |
|------|------|
| 5~10 | 响应慢，稳定 |
| 10~20 | 中等，**角度环推荐** |
| 30~60 | 响应快，**角速度环推荐** |
| >100 | 可能振荡 |

#### 4. 控制增益 (b0) - 扰动补偿强度

```c
adrc->eso.b0 = 1.0f;  // 默认值
```

**含义：** 系统实际控制增益的估计值

**调参方法：**
1. 如果系统响应慢（控制力度不够），增大b0（如1.5~2.0）
2. 如果系统振荡（控制力度过大），减小b0（如0.5~0.8）
3. 通常b0=1.0附近即可

### 飞镖项目推荐参数

#### Yaw通道（重点调优）

```c
// 角度环（外环）
ADRC_Init_AngleLoop(&adrc_ctrl[ADRC_YAW][ADRC_ANGLE_LOOP], ADRC_YAW);
adrc_ctrl[ADRC_YAW][ADRC_ANGLE_LOOP].td.r = 80.0f;
ESO_SetBandwidth(&adrc_ctrl[ADRC_YAW][ADRC_ANGLE_LOOP].eso, 20.0f);
NLSEF_SetBandwidth(&adrc_ctrl[ADRC_YAW][ADRC_ANGLE_LOOP].nlsef, 10.0f);
adrc_ctrl[ADRC_YAW][ADRC_ANGLE_LOOP].eso.b0 = 1.0f;

// 角速度环（内环）
ADRC_Init_GyroLoop(&adrc_ctrl[ADRC_YAW][ADRC_GYRO_LOOP], ADRC_YAW);
adrc_ctrl[ADRC_YAW][ADRC_GYRO_LOOP].td.r = 250.0f;
ESO_SetBandwidth(&adrc_ctrl[ADRC_YAW][ADRC_GYRO_LOOP].eso, 80.0f);
NLSEF_SetBandwidth(&adrc_ctrl[ADRC_YAW][ADRC_GYRO_LOOP].nlsef, 40.0f);
adrc_ctrl[ADRC_YAW][ADRC_GYRO_LOOP].eso.b0 = 1.0f;
```

#### Pitch通道

```c
// 角度环
ESO_SetBandwidth(&adrc_ctrl[ADRC_PITCH][ADRC_ANGLE_LOOP].eso, 15.0f);
NLSEF_SetBandwidth(&adrc_ctrl[ADRC_PITCH][ADRC_ANGLE_LOOP].nlsef, 8.0f);

// 角速度环
ESO_SetBandwidth(&adrc_ctrl[ADRC_PITCH][ADRC_GYRO_LOOP].eso, 60.0f);
NLSEF_SetBandwidth(&adrc_ctrl[ADRC_PITCH][ADRC_GYRO_LOOP].nlsef, 30.0f);
```

---

## 调试技巧

### 1. 观测ESO状态估计

通过Vofa观察：

```c
// 在 Vofa 发送函数中添加
float z1, z2;
ADRC_GetStateEstimate(ADRC_YAW, ADRC_ANGLE_LOOP, &z1, &z2);
Vofa_SendData(z1, z2, adrc_ctrl[ADRC_YAW][ADRC_ANGLE_LOOP].eso.z3, ...);
```

**正常现象：**
- z1应该接近实际角度（可能更平滑）
- z2应该接近实际角速度
- z3应该能跟踪扰动（如气动阻力）

**异常诊断：**
- z1/z2抖动厉害 → ESO带宽w0太大，减小
- z1/z2滞后严重 → ESO带宽w0太小，增大
- z3一直为0 → ESO没有正确估计扰动
- z3剧烈振荡 → 需要调整alpha/delta参数

### 2. 观测扰动估计

```c
// Vofa观测
float disturbance_yaw = ADRC_GetDisturbance(ADRC_YAW, ADRC_ANGLE_LOOP);
float disturbance_pitch = ADRC_GetDisturbance(ADRC_PITCH, ADRC_ANGLE_LOOP);
```

**扰动z3的含义：**
- 正值 → 系统有正向偏差（需要负向补偿）
- 负值 → 系统有负向偏差（需要正向补偿）
- 变化 → 有动态扰动（如气动变化）

### 3. 分步调试

**步骤1：只开ESO，不开控制**
```c
// 临时修改：让ESO只观测，不输出控制
float ADRC_Calc_Debug(ADRC_t *adrc, float target, float feedback, float dt)
{
    // 只运行ESO
    ESO_Update(&adrc->eso, feedback, 0.0f, dt);

    // 返回0，不输出控制
    return 0.0f;
}
```

观察z1/z2/z3是否合理。

**步骤2：小增益测试**
```c
// 临时减小控制增益
adrc->eso.b0 = 0.1f;  // 从0.1开始
NLSEF_SetBandwidth(&adrc->nlsef, 2.0f);  // 很小的控制带宽
```

观察系统是否稳定。

**步骤3：逐步增大**
逐渐增大b0和wc，直到达到满意效果。

### 4. 对比测试

```c
// 在Vofa中同时发送PID和ADRC输出
float pid_output = Euler_pid_Cale_Debug(delta_time);
float adrc_output = Euler_ADRC_Cale_Debug(delta_time);

Vofa_SendData(pid_output, adrc_output, ...);
```

---

## 常见问题

### Q1：ADRC输出抖动

**原因：**
- ESO带宽w0太大
- fal函数的delta太小
- 噪声没有滤波

**解决：**
```c
// 1. 减小ESO带宽
ESO_SetBandwidth(&adrc->eso, 10.0f);  // 从20减到10

// 2. 增大fal的delta
adrc->eso.delta = 0.05f;  // 从0.01增到0.05
adrc->nlsef.delta = 0.05f;

// 3. 对测量值滤波
feedback = KalmanFilter(&kf, feedback, 0.1f, 5.0f);
```

### Q2：系统响应慢

**原因：**
- ESO带宽w0太小
- NLSEF带宽wc太小
- b0太小

**解决：**
```c
// 1. 增大ESO带宽
ESO_SetBandwidth(&adrc->eso, 30.0f);  // 从15增到30

// 2. 增大NLSEF带宽
NLSEF_SetBandwidth(&adrc->nlsef, 15.0f);  // 从8增到15

// 3. 增大b0
adrc->eso.b0 = 1.5f;  // 从1.0增到1.5
```

### Q3：稳态误差

**原因：**
- ESO没有正确估计扰动
- b0太小

**解决：**
```c
// 1. 调整ESO的alpha参数
adrc->eso.alpha2 = 0.3f;  // 更小的指数，对小误差更敏感

// 2. 减小delta
adrc->eso.delta = 0.005f;

// 3. 适当增大b0
adrc->eso.b0 = 1.2f;
```

### Q4：目标跳变时超调

**原因：**
- TD的r太大
- NLSEF的wc太大

**解决：**
```c
// 1. 减小TD速度
adrc->td.r = 50.0f;  // 从100减到50

// 2. 增大TD滤波因子
adrc->td.h0 = 0.01f;  // 从0.005增到0.01

// 3. 减小NLSEF带宽
NLSEF_SetBandwidth(&adrc->nlsef, 8.0f);  // 从12减到8
```

### Q5：与原PID切换时跳变

**原因：**
- ADRC和PID输出不连续

**解决：**
```c
// 切换时平滑过渡
float transition_time = 0.5f;  // 0.5秒过渡
static float blend = 0.0f;

if (adrc_enable)
{
    if (blend < 1.0f) blend += dt / transition_time;
}
else
{
    if (blend > 0.0f) blend -= dt / transition_time;
}

float pid_out = Euler_pid_Cale_Debug(delta_time);
float adrc_out = Euler_ADRC_Cale_Debug(delta_time);
float final_out = blend * adrc_out + (1.0f - blend) * pid_out;
```

---

## 进阶调参

### 自适应b0

如果系统特性变化大（如不同飞行阶段），可以让b0自适应：

```c
// 根据角速度调整b0
float gyro = fabsf(Surface.current_gyro_Euler[NOW][YAW]);
if (gyro > 100.0f)
    adrc->eso.b0 = 1.5f;  // 高速时增大
else if (gyro > 50.0f)
    adrc->eso.b0 = 1.2f;
else
    adrc->eso.b0 = 1.0f;
```

### 通道解耦

如果yaw和pitch耦合严重，可以在ESO中考虑耦合：

```c
// 简化的耦合补偿
float coupling = 0.1f * Surface.output_gyro_Euler[NOW][PITCH];
ESO_Update(&adrc->eso, feedback, control_out + coupling, dt);
```

### 前馈增强

结合视觉目标变化率：

```c
// TD的x2就是目标微分，可以加到前馈
float ff = adrc->td.x2 * 0.3f;  // 前馈增益
float u = ADRC_Calc(...) + ff;
```

---

## 总结

### ADRC调参口诀

```
ESO带宽定基调，
TD速度管跟踪，
NLSEF带宽管响应，
b0大小管补偿。

先小后大慢慢来，
观测z3是关键，
抖了就减响应慢，
稳了再加效果显。
```

### 快速上手

1. 用默认参数先跑
2. 观察z3是否合理
3. 根据现象调整w0
4. 逐步优化其他参数

### 与PID对比

| 指标 | PID | ADRC |
|------|-----|------|
| 调参难度 | 简单但难调好 | 参数多但规律清晰 |
| 抗扰能力 | 被动响应 | 主动补偿 |
| 非线性处理 | 困难 | 天然支持 |
| 鲁棒性 | 一般 | 强 |
| 计算量 | 小 | 中等 |

---

**祝调试顺利！🚀**
