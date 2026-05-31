# X 翼三轴解耦 + Pitch 优先最小能量控制分配

> 📍 本文是详细方案文档，进度总览与 TODO 见 [PROGRESS.md](PROGRESS.md)。
>
> **状态：✅ 已实现**（核心目标完成）。代码已落地于 [surface_control_task.c](imcalib/Task/surface_control_task.c) / [surface_control_task.h](imcalib/Task/surface_control_task.h)，与本文方案逐行核对一致。
> **待办**：台架 Vofa 实测验证（见文末清单）；"可选扩展"本次未做（roll 优先于 yaw、抗积分饱和）。

## Context（为什么做这个改动）

用户要把控制过程的 **yaw / roll / pitch 解耦**，并做"最小能量规划"，使得**调 roll/yaw 时对 pitch 的影响最小**（pitch 是飞镖的弹道/射程主轴，需优先保护）。

读代码后的关键结论 —— **线性区其实已经解耦了**：

当前 X 翼混控矩阵（[surface_control_task.c:379-382](imcalib/Task/surface_control_task.c#L379-L382)，去掉 SIGN 后的逻辑符号阵 A）：

```
        p    r    y
UL  [  +1   +1   -1 ]
UR  [  +1   -1   +1 ]
DL  [  -1   +1   +1 ]
DR  [  -1   -1   -1 ]
```

满足 **AᵀA = 4·I**（三列两两正交）。即在理想对称、SIGN_xx 已正确标定的前提下，pitch / roll / yaw 产生的力矩本来就互相正交 —— 调 roll/yaw 在数学上不产生 pitch 力矩。**所以真正的耦合不在混控公式，而在饱和。**

**耦合的真正来源 = 每片舵独立硬裁剪**：原先对每片舵单独 `abs_limit(±60)`。一旦某片 `+p+r-y` 超过 60 被砍，它**连带把自己那份 pitch 分量也砍掉了** → roll/yaw 的大修正会"漏"进 pitch。

数值例（limit=60, p=40, r=40, y=0）：
- 理想 ℓ = A·[40,40,0]ᵀ = [80, 0, 0, −80]，UL/DR 超限。
- **朴素裁剪** → [60,0,0,−60]，反解 pitch=(60+0−0+60)/4=**30**（被 roll 饱和从 40 污染到 30）。
- **本方案** → pitch 全保留=40，roll 自动缩到 20，yaw=0。pitch 不动。

## 方案：Pitch 优先级最小能量控制分配

在"逻辑偏转"空间（未乘 SIGN，因 `abs_limit` 对称、`|SIGN·ℓ|=|ℓ|`，限幅只取决于 |ℓ|）把每片舵拆成 **pitch 分量 P_i** 与 **横侧(roll+yaw)分量 L_i**：

```
P[UL]=+p  L[UL]=+r−y
P[UR]=+p  L[UR]=−r+y
P[DL]=−p  L[DL]=+r+y
P[DR]=−p  L[DR]=−r−y
```

1. **pitch 最高优先**：先 `abs_limit(&p, LIMIT)`，保证 pitch 单舵可实现、永不被牺牲。
2. **求最大横侧比例 k∈[0,1]**，使 `|P_i + k·L_i| ≤ LIMIT` 四片都成立。闭式解（因 |P_i|=|p|≤LIMIT，分子恒 ≥0）：
   ```
   k = min( 1,  min_i  (LIMIT − sign(L_i)·P_i) / |L_i|  )    // 仅对 L_i≠0 的片
   ```
3. **输出** `ℓ_i = P_i + k·L_i`，再乘 SIGN_xx 写入 `output_angle_Servo[NOW][i]`。

**物理意义 = 最小能量/最小改动分配**：在所有"满足舵机限幅 **且 pitch 力矩完全不变**"的分配里，选最接近期望 roll/yaw 的那个（roll、yaw 等比缩放，方向不变，只损失幅度）。pitch 永不被 roll/yaw 饱和污染。运行成本 O(1)：无矩阵求逆，仅 4 次比较/除法，适合 1 kHz 任务。

## 已落地的改动（对照实际代码）

### 1. [surface_control_task.h](imcalib/Task/surface_control_task.h)（X 翼宏区）
- ✅ 新增 `#define SERVO_ANGLE_LIMIT 60.0f`（统一替代散落的 `60.0f` 字面量）—— [.h:43](imcalib/Task/surface_control_task.h#L43)
- ✅ 新增 `extern float servo_lat_scale;`（Vofa 可观测：横侧保留比例 k，1=未饱和，<1=正在为保 pitch 缩 roll/yaw）—— [.h:126](imcalib/Task/surface_control_task.h#L126)

### 2. [surface_control_task.c](imcalib/Task/surface_control_task.c)
- ✅ 新增全局 `float servo_lat_scale = 1.0f;` —— [.c:42](imcalib/Task/surface_control_task.c#L42)
- ✅ 新增分配函数 `Servo_Mix_PitchPriority()`（位于 `Wing_Control_VECTOR_NOZZLE` 之前）—— [.c:323-350](imcalib/Task/surface_control_task.c#L323-L350)：

```c
/* Pitch 优先最小能量控制分配:
 * pitch 全额保留,roll/yaw 等比缩到舵机余量内 → 调 roll/yaw 不污染 pitch。
 * 写入 Surface.output_angle_Servo[NOW][0..3](已含 SIGN,已在 ±SERVO_ANGLE_LIMIT 内)。*/
void Servo_Mix_PitchPriority(float p, float r, float y)
{
    abs_limit(&p, SERVO_ANGLE_LIMIT);                           /* 1) pitch 分量优先*/

    float P[4], L[4];                                           /* 2) 拆 pitch 分量 / 横侧分量(逻辑符号阵) */
    P[UP_LEFT]    = +p;  L[UP_LEFT]    = +r - y;
    P[UP_RIGHT]   = +p;  L[UP_RIGHT]   = -r + y;
    P[DOWN_LEFT]  = -p;  L[DOWN_LEFT]  = +r + y;
    P[DOWN_RIGHT] = -p;  L[DOWN_RIGHT] = -r - y;

    float k = 1.0f;                                             /* 3) 求最大横侧比例 k */
    for (int i = 0; i < 4; i++)
    {
        float aL = (L[i] < 0.0f) ? -L[i] : L[i];                // 绝对值
        if (aL < 1e-6f) continue;                               /* 该片无横侧分量,不构成约束 */
        float sgnL = (L[i] < 0.0f) ? -1.0f : 1.0f;              //拿到原来的符号以便复原
        float ki = (SERVO_ANGLE_LIMIT - sgnL * P[i]) / aL;      //限幅减去原来的值再除以横侧分量，得到比例
        if (ki < k) k = ki;                                     //在比例内就按原样,超出就按比例缩放,保证在限幅内
    }
    if (k < 0.0f) k = 0.0f;
    servo_lat_scale = k;

    float SGN[4];                                               /* 4) 最终的合成 + 物理方向 SIGN 写入 */
    SGN[UP_LEFT]   = SIGN_UL;  SGN[UP_RIGHT]   = SIGN_UR;
    SGN[DOWN_LEFT] = SIGN_DL;  SGN[DOWN_RIGHT] = SIGN_DR;
    for (int i = 0; i < 4; i++)
        Surface.output_angle_Servo[NOW][i] = SGN[i] * (P[i] + k * L[i]);
}
```

- ✅ `if(DART_TYPE == VECTOR_NOZZLE)` 混控块替换为「调用分配函数 + 兜底限幅」—— [.c:403-415](imcalib/Task/surface_control_task.c#L403-L415)：

```c
if(DART_TYPE == VECTOR_NOZZLE)   //x翼
{
    if (Guidance_State==Stable||Guidance_State==Terminal)
    {
        /* Pitch 优先最小能量分配:pitch 全保,roll/yaw 等比缩进限幅,不污染 pitch。
         * SIGN_xx 仍在函数内逐片乘上,标定流程不变。 */
        Servo_Mix_PitchPriority(Surface.output_gyro_Euler[NOW][PITCH],
                                Surface.output_gyro_Euler[NOW][ROLL],
                                Surface.output_gyro_Euler[NOW][YAW]);
    }
    for (int i = 0; i < 4; i++)                    /* 安全网:分配已保证在限内,此处仅兜底 FP 误差 */
        abs_limit(&Surface.output_angle_Servo[NOW][i], SERVO_ANGLE_LIMIT);
}
```

**未改动（确认照旧）**：下游 `Wing_Control_VECTOR_NOZZLE`/`Wing_UL..DR_Control`（PWM 写入照旧）、Start/End/Self_Text_State 对 `output_angle_Servo` 的覆盖、PID、IMU、FIXED_WING 分支。SIGN_xx 标定流程不变（本方案在逻辑空间运算，SIGN 含义不变）。

## 前提与边界（诚实声明）

- 解耦正确性依赖 **SIGN_xx 已在台架按单轴阶跃正确标定** + airframe 近似对称（AᵀA≈对角）。本方案保证"软件不再因饱和制造耦合"，但不能消除物理气动不对称带来的耦合。
- 若台架实测发现明显不对称，下一步可用实测有效性矩阵的加权伪逆（CMSIS `mat_inv` 已在 [filter.h:12-20](imcalib/Tool/filter.h#L12-L20) alias）替换理想 X 阵 —— 列为后续，不在本次。

## 可选扩展（未做，留待后续）

- **roll 优先于 yaw**（或反之）：把 L 再拆两段，先填 roll 余量再填 yaw。当前是 roll/yaw 同比缩。
- **抗积分饱和**：k<1 时按 k 回算冻结/泄放 roll/yaw 内外环积分，防 windup（与 memory 中"输出加速率限制"TODO 并列）。

## 验证清单（待台架执行，配合 Vofa）

1. 画 4 路 `Surface.output_angle_Servo[NOW][0..3]` + `servo_lat_scale`。
2. **纯 pitch 阶跃**：4 舵按 +p/+p/−p/−p 动，`servo_lat_scale` 恒=1。
3. **大 roll/yaw 致饱和**：`servo_lat_scale` 掉到 <1；从输出反解 pitch 分量 `(ℓ_UL+ℓ_UR−ℓ_DL−ℓ_DR)/4` 应 = 原 p 不变 —— **核心验证：roll/yaw 饱和不动 pitch**。
4. 对照：临时切回旧的朴素裁剪块，同输入下反解 pitch 被污染（如 40→30）。
5. 长跑 >5 min 无 HardFault。
