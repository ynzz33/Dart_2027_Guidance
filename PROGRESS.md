# Dart_2027 飞控 — 开发进度与 TODO 总览

> 项目进度的单一入口。详细技术方案见文末「[详细方案文档](#详细方案文档)」。
> 本文件与 Claude 的 memory（`control-tuning-progress` / `control-approach-preferences`）**双向同步**，改进度时两边保持一致。
> 最后更新：2026-06-06

## 项目简介

STM32G431 + BMX055 + FreeRTOS 的 Dart 飞镖型飞行器飞控。X 翼布局（4 舵机，TIM4 CH1–CH4 / PB6–PB9），串级 PID 自稳（角度环 → 角速度环），Mahony 姿态融合。磁力计不启用（场景磁干扰大），PNG 视觉暂不接入主控制环。

---

## 控制方案偏好（决策原则）

做控制优化时遵循以下既定偏好，提方案默认按此：

1. **保留串级 PID，不退单环**（角度环 → 角速度环）。
2. **滤波在输入端（传感器/融合）解决，不在输出端反馈环内加低通**。
3. **「更好的输入」靠传感器融合（Mahony/卡尔曼），不是更重的低通**。

**Why：** 重低通虽更平滑但相位滞后大（R=5000 → 延迟≈70ms），滞后是闭环震荡主因，降增益压不住；纯 P 串级在数学上等价于单环 PD + 陀螺阻尼（内环 P 即阻尼系数，本身就是「用角速度预判」），且抗扰更强；输出反馈环内低通 = 又引入一个延迟源。

**怎么应用：** 优先从传感器融合质量和控制器结构入手，不要用「加重低通」或「输出端滤波」掩盖震荡。

---

## 进度时间线

### 硬件适配与基础功能（已完成 — 见 git log）
- 适配新版硬件；解算改为 X 翼。
- SPI 通信正常；舵面解算极性标定完成（极性已对）。
- 控制输出平稳化；yaw 自稳初步有效果，手持已明显稳定。

### 飞控三模块优化（详见 [plan-flight-control-overhaul.md](plan-flight-control-overhaul.md)）
4-Phase 总方案，多数随 X 翼改造落地（逐条改动以代码为准）：
- **Phase 0** 公共宏统一（`common_defs.h`）。
- **Phase 1** 关键 Bug 修复：`prev_tick` 重置致 dt≡0（I/D 失效）、四元数积分系数 `0.125`→`0.5`、FFC 空指针、重力常数、Mahony 改回标准 PI、float 全 `f` 后缀、数据有效性守卫、Vision 读侧临界区。
- **Phase 2** IMU 解算重构 + gyro 2 秒静态零偏校准。
- **Phase 3** X 翼 4 舵输出改造（TIM4 CH1–CH4，标准混控，SIGN_xx 台架标定）。
- **Phase 4** 控制算法精修：D 项低通、PID `enable` 开关、任务节拍 `vTaskDelayUntil` 稳态化。

### 输入端滤波拆分（2026-05-30）
- [IMU.h](imcalib/Task/IMU.h)：陀螺/加速度共用的卡尔曼 R 拆成两路——`GYR_KF_R` 5000→30（保相位/低延迟），`ACC_KF_R` 5000→500（适度平滑）。
- [IMU.c](imcalib/Task/IMU.c)：acc/gyr 两处卡尔曼调用改用各自新宏。
- Mahony 参数复核保持 PI（Kp=2 / Ki=0.01 / Kd=0）。

### 控制器端改动（2026-05-30）
- **FFC 前馈接入**：按偏好**与 `pid_calc` 解耦**，不内嵌进 PID。在 [pid.c](imcalib/Tool/pid.c) `Euler_pid_Cale` 里用独立 `FeedForwardController` 对每环目标算前馈再 `+=` 叠加；顺带修了 `FeedForwardController` 更新顺序 bug（二阶项失效）、`num1/num2` double→float。**默认 num1/num2=0 时前馈恒为 0，已接入但未启用**，需在 `pid_init` 给对应轴设非零值才生效。
- **死区软化**：新增 `Deadband_Soften`（连续减区，C0 连续），`pid_calc` 删掉死区 `return 0` 硬切断改走完整流程；附带修了原 `return 0` 致 err 历史/积分不更新、出死区瞬间 D 项尖刺的问题。

### 控制分配解耦（2026-05-31，详见 [plan-pitch-priority-mixing.md](plan-pitch-priority-mixing.md)）
- 实现 **Pitch 优先最小能量控制分配** `Servo_Mix_PitchPriority(p,r,y)`。诊断：X 翼混控矩阵本身线性解耦（AᵀA=4I），pitch 耦合的真正来源是每片舵独立 `abs_limit` 饱和会连带砍掉该片的 pitch 分量。
- 方案：pitch 先限幅全额保留，roll/yaw 横侧分量统一乘 `k=min(1, minᵢ(LIMIT−sgn(Lᵢ)·Pᵢ)/|Lᵢ|)` 缩进舵机余量；反解 pitch 恒=p、与 k 无关（已独立数学核验）。
- 配套：[surface_control_task.h](imcalib/Task/surface_control_task.h) 新增 `SERVO_ANGLE_LIMIT 60.0f` + `servo_lat_scale`（Vofa 观测 k）。下游 Wing/PID/SIGN_xx 标定全不变。

### 世界系 pitch/yaw 解算 / roll 反旋（2026-06-02，输出端）
- 台架确认 IMU 欧拉 pitch/yaw 是 **ZYX 世界系参考**（绕纵轴横滚 90°，PITCH 读数几乎不变）→ PID 输出是「世界系 pitch/yaw 力矩需求」，但 X 翼舵面产生**机体系**力矩，缺一层 roll 反旋。
- 新增 `Roll_Derotate_PitchYaw(Pw,Yw,&Pb,&Yb)`（[surface_control_task.c](imcalib/Task/surface_control_task.c)，置于 `Servo_Mix_PitchPriority` 前）：Δ=当前roll−Stable_roll，`Pb=cosΔ·Pw+sinΔ·Yw, Yb=−sinΔ·Pw+cosΔ·Yw`；roll 通道（绕纵轴）不动，Δ=0 恒等。VECTOR_NOZZLE 调用点先反旋再送混控。
- 用户初选「输入端转换」，台架确认欧拉角为世界系后改为**输出端净版**（输入端会多一层多余 R(Δ)、因 pitch/yaw 增益不等而交叉耦合）。
- 配套：`ROLL_WORLD_COMP_SIGN`（±1 翻号宏）+ `Roll_World_Comp_Flag`（运行时直通开关，0=旧行为便于 A/B）+ Vofa 观测量 `roll_world_delta / roll_world_pb / roll_world_yb`。下游混控/PID/IMU/SIGN_xx 标定全不变。

### 末制导视线目标锁存 / 帧间 IMU 航位推算（2026-06-03，输入端）
- **症状**：末制导镜头反馈超过目标后，镖体在同一水平线上来回首振、不趋近目标（镜头随机头刚性转动一越过目标就读数翻号，机体却只绕质心摆头）。
- **诊断**：视觉 ~20Hz（`SAMPLE_RATE`）、控制 1kHz（`CTRL_PERIOD_MS`）。原 `Guidance_Terminal` 每 tick 重算 `target=v+current`，送 `pid_calc` 后外环误差 `set−get` 把 `current` 精确相消，只剩被零阶保持 50ms 的视觉误差 `v.y`；新鲜 IMU 姿态被代数抵消、不进外环 → 控制环 50ms 盲转过冲、下帧符号翻转 → 极限环首振，左右气动力相消、净侧力≈0 故质心不平移。
- **方案（方向A，贴合「输入端/融合解决」偏好）**：仅在视觉**新帧到达**那一刻用当时姿态锁存世界系视线方向 `los_world=v+current`，帧间保持不变；目标喂锁存值 → 外环误差=`los_world−current`，机体一转误差即减、50ms 内闭环。本质＝用 1kHz IMU 把 20Hz 视觉的空档补上（航位推算），非输出端低通、不动 PID 增益、与 roll 反旋 / Pitch 优先分配独立叠加。
- **落地**：[CallBack_Task.h](imcalib/Task/CallBack_Task.h) / [.c](imcalib/Task/CallBack_Task.c) 新增 `Vision_Recog_Cnt`（仅识别成功帧递增，控制端据此判新帧）；[surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal` 重写为「新帧锁存＋帧间保持」，新增全局 `los_world_target[3]`（Vofa 可观测，[.h](imcalib/Task/surface_control_task.h) 导出）；PITCH 保留原 `<-10°` 门控；丢目标(FAILURE) 解锁并保持当前姿态。

### 控制分配重写：可调三轴限幅 + 真正最小能量分配（2026-06-03，输出端）
- **动机**：原 `Servo_Mix_PitchPriority` 本质是「pitch 优先缩横侧」的启发式饱和，且 `k` 公式写反成倒数（[.c:412](imcalib/Task/surface_control_task.c#L412) `|L|/(LIMIT−P)` 应为 `(LIMIT−P)/|L|`）实际没正常工作；调用点 roll 还误传 `target_angle_Euler[ROLL]`(≈0) 而非 PID 输出。
- **拆两交付物 + 运行时开关 `Alloc_Mode`**（0=旧对照/1=三轴限幅/2=最小能量，仿 `Roll_World_Comp_Flag` 风格，Vofa 在线可切）：
  - **交付A `Servo_Mix_AxisLimit`**：pitch/roll/yaw 各自独立可调限幅（`AXIS_LIMIT_*`）→ 理想 X 逻辑阵 → ×SIGN。O(1) 无矩阵，替代写死单 pitch。
  - **交付B `Servo_Mix_MinEnergy`**：真正的带约束最小能量分配。`u0=Bᵀ(BBᵀ)⁻¹v`（CMSIS `arm_mat_inverse_f32`）→ 1 维零空间余子式 `n` 投影进舵机限幅盒（`Bn=0` 不改力矩）→ 不可达按 pitch>yaw>roll 优先级二分缩 yaw/roll → 奇异退回交付A。舵效阵 `Alloc_B[3][4]` 默认理想 X 阵+预留台架辨识接口；`ALLOC_GAIN=4` 使理想阵下与交付A/旧版同幅度、复用 PID 标定。
- **决策**：① 通用矩阵框架+默认理想B；② B=理想阵+预留辨识；③ **roll 三轴统一接 `output_gyro_Euler[ROLL]`**（修正原误传目标角）。
- **落地**：[surface_control_task.c](imcalib/Task/surface_control_task.c)（两新函数+三辅助+调用点 switch 分派）、[.h](imcalib/Task/surface_control_task.h)（`AXIS_LIMIT_*`/`ALLOC_U_MAX`/`ALLOC_GAIN` 宏、`Alloc_Mode`/`Alloc_B`/`alloc_*` Vofa extern、函数声明）。旧 `Servo_Mix_PitchPriority` 保留作 Mode0 对照。下游 PWM/枚举/SIGN/状态机覆盖/历史移位全不变。
- **手算自检**：理想阵零空间 `n=[-4,-4,-4,-4]∝[1,1,1,1]`；`v=[10,0,0]→u0=[10,10,-10,-10]→Bu=[40,0,0]=v×gain` ✓。
- **诚实边界**：理想 B 是结构近似（未建模舵间耦合/左右不等/失速）；无空速无法动压调度；台架辨识前建议先用 Mode1 飞通再切 Mode2。

### 末制导新帧判定改用显式标志位 `Vision_New_Data_flag`（2026-06-06，输入端）
- **动机**：把 [06-03 视线锁存](#末制导视线目标锁存--帧间-imu-航位推算2026-06-03输入端) 里隐式的「计数器对比 `Vision_Recog_Cnt != last_recog_cnt`」新帧判定，换成显式、语义清晰、可在别处复用的标志位（标准生产者-消费者）。
- **落地**：
  - [CallBack_Task.h](imcalib/Task/CallBack_Task.h) `Vision_Rx_Buf_t` 新增 `Vision_New_Data_flag`；[CallBack_Task.c](imcalib/Task/CallBack_Task.c) `Vision_Receive` 在识别成功(0x5A)与丢目标(0x7A)两分支收到新数据即置 1（生产者，UART 中断 ~20Hz）。
  - [surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`（消费者，控制 1kHz）：`if (Vision_New_Data_flag==1 && recognize==SUCCESS)` 才更新目标、**块末才置 0**；帧间(flag==0)不进，目标靠 `target_angle_Euler[NOW]` 槽保持不变（`Euler_Updata` 只移位 LAST/LLAST、不动 NOW）。
  - **移除中间变量 `los_world_target`**：直接写 `Surface.target_angle_Euler[NOW][YAW]=v.y+current`、PITCH 同（保留 `<-10°` 门控）。`.c` 全局定义已删（[.h](imcalib/Task/surface_control_task.h) 的 `extern` 声明成孤儿、待清理）。
  - 丢目标(FAILURE) 分支移出 flag 门控、**每 tick 跑**（`Vision_Cmd_Work` + 目标回中=当前姿态），同原始行为。
- **影响**：`Vision_Recog_Cnt` 降级为纯统计计数、不再判新帧；Vofa 观测「锁存视线目标」改看 `Surface.target_angle_Euler[NOW][YAW]`。并发上 flag 单字节读写原子、处理后清 0（极偶然处理中来帧会丢一次触发，但数据已入 `Vision_Rx_Data`、下帧补，滞后≤50ms）。

---

## 当前 TODO

### ⏳ 待 Vofa 台架验证
- **视线锁存（方向A）**：末制导对静止/缓动目标，镖体应单调转向并稳定指向、不再同线首振；Vofa 拉 `Surface.target_angle_Euler[NOW][YAW]` 应为帧间水平台阶（保持不变）、仅识别成功新帧到达瞬间阶跃更新；丢目标(FAILURE) 每 tick 回中、重新捕获到 SUCCESS 帧立即重锁。
- **视觉单位核验（方向C，未做）**：`v.x/v.y` 是 `int16`，PNG 按像素用（×FOV→rad），而 `Guidance_Terminal` 锁存按度直接加；Vofa 看 `Surface.target_angle_Euler[NOW][YAW]` 量级是否与欧拉角同量级（几度~几十度），若达几百说明视觉发的是像素、需 ×FOV 转度，否则锁存目标偏大、镖体会稳定指向过大角度。
- **控制分配三档对照（新）**：Vofa 切 `Alloc_Mode` 0/1/2 都不超 ±65、roll 现接 PID 输出无突跳；Mode2 零空间自检 `n=[-4,-4,-4,-4]`、`alloc_singular_flag≡0`；可达区 Mode2 能量 Σu² ≤ Mode1；强制大 v 看 `alloc_infeasible=1`/`alloc_v_scale<1`/pitch 保住；长跑无 NaN。台架辨识 `Alloc_B` 后把调用点经验系数 `0.85/1.05` 设 1。
- **Pitch 优先分配**（仅 Mode0 对照保留；完整验证清单见 [plan-pitch-priority-mixing.md](plan-pitch-priority-mixing.md#验证清单待台架执行配合-vofa)）：纯 pitch 阶跃时 `servo_lat_scale`≡1；大 roll/yaw 饱和时 k<1 而从输出反解的 pitch 分量不变；长跑 >5 min 无 HardFault。
- **roll→世界 pitch/yaw 反旋 SIGN**：`Roll_World_Comp_Flag=0` 行为同旧版（直通对照）；置 1 且 Δ≈0 时恒等（`roll_world_delta`≈0、pb≈Pw、yb≈Yw）；横滚 +90° 给纯世界 pitch（Pw>0,Yw=0）应得 pb≈0、yb≈±Pw 且机头朝**正确**世界方向修正，反了则把 `ROLL_WORLD_COMP_SIGN` 翻成 −1 重编。

### 🔜 下一步（待 Vofa 验证后）
- **启用并标定 FFC** `num1/num2`：先内环角速度环，阶跃目标看超调/滞后。
- **串级内外环增益拉开**：目前外环 P=0.7 > 内环 P=0.2，与「内环>外环」偏好相反，需重标。
- **输出加速率限制**（rate limit，非环内低通）。

### 💤 后续（暂缓，有需求再做）
- **卡尔曼状态观测器**：带角速度状态的状态空间卡尔曼，预测姿态，替代当前一阶标量 `KalmanFilter`。
- Pitch 分配可选扩展：roll 优先于 yaw（当前 roll/yaw 同比缩）、抗积分饱和（k<1 时回算冻结/泄放横侧积分防 windup）。

---

## 详细方案文档

| 文档 | 内容 | 状态 |
|---|---|---|
| [plan-flight-control-overhaul.md](plan-flight-control-overhaul.md) | 飞控三模块优化总方案（IMU 解算 / 控制算法 / X 翼输出 / 精修，4 Phase，逐条改前→改后 + 验证流程） | 多数已落地 |
| [plan-pitch-priority-mixing.md](plan-pitch-priority-mixing.md) | X 翼三轴解耦 + Pitch 优先最小能量控制分配（含数学核验、落地代码、验证清单） | ✅ 已实现，待台架验证 |

---

## 与 Claude memory 的同步

本文件是项目内（git 可同步）的进度镜像，对应 Claude 的两条 memory：

- `control-tuning-progress`（project）← → 本文件「进度时间线」+「当前 TODO」
- `control-approach-preferences`（feedback）← → 本文件「控制方案偏好」

更新任一侧时同步另一侧，保持一致。
