# Dart_2027 飞控 — 开发进度与 TODO 总览

> 项目进度的单一入口。详细技术方案见文末「[详细方案文档](#详细方案文档)」。
> 本文件与 Claude 的 memory（`control-tuning-progress` / `control-approach-preferences`）**双向同步**，改进度时两边保持一致。
> 📖 代码速查地图见 [CODE_OVERVIEW.md](CODE_OVERVIEW.md)（答题/接手前先读，含环境/任务/数据流/坐标系/状态机/分配器/全局/陷阱）。
> 🌐 跨 AI 工作规则/约束见 [AGENTS.md](AGENTS.md)（任何 AI 接手前先读：沟通方式、改动哲学、文档/验证纪律、红线）。
> 最后更新：2026-06-27（分配器散落全局整合为 `Alloc` 结构体；清理末制导 LPF/距离增益死代码 + `#if 0` 封存增益调度函数；三维→二维直通 + 调参哲学转向「内环>外环纯P串级」+ 末制导主动滑翔→扎 + PNG 拆分接口；新建跨 AI [AGENTS.md](AGENTS.md)；修复已删 plan 文件链接。前次：2026-06-23 全量审计 v2）
>
> ⚠️ **本项目处于高频调参期**（几乎每个 commit 都在改 PID 增益/分配档/门控）。本文档**只记结构、方案与"为什么"，不追具体数值**——所有增益/阈值/默认档**以代码为准**（pid.c `pid_init`、surface_control_task.c/.h 顶部全局与宏）。文中出现的数值均为"某时刻快照"，可能已变。

## 项目简介

STM32G431 + BMX055 + FreeRTOS 的 Dart 飞镖型飞行器飞控。X 翼布局（4 舵机，TIM3 CH2 + TIM4 CH2–CH4），串级 PID 自稳（角度环 → 角速度环）+ 可选 LADRC 单环二阶自抗扰（`ladrc_mode` 切档），Mahony 姿态融合，视觉/IMU 紧耦合 6 态 EKF（vision_ins.c）给不漂的速度。磁力计不启用（场景磁干扰大），PNG 比例导引拆分接口已就绪但主环仍 `#if 0`。

> **当前实际在飞的链路（2026-06-27 快照，以代码为准）**：三轴**直通**串级 PID（`ladrc_mode=0`、`vel_pursuit_mode=0`，06-22 试的欧拉运动学耦合变换已于 06-23「三维转二维」撤回，作 TODO 留注释）→ 控制分配 `Alloc_Mode=2`（最小能量）→ X 翼 4 舵。调参哲学已转向「**内环>外环、I/D≈0 的纯 P 串级**」（Gyro 0.8~1.1 > Angle 0.25~0.45）。末制导 pitch 用**主动滑翔→扎**（`pitch_glide_mode=1`）。

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

### 飞控三模块优化（原 `plan-flight-control-overhaul.md`，已删除→内容并入本时间线 / git 历史）
4-Phase 总方案，多数随 X 翼改造落地（逐条改动以代码为准）：
- **Phase 0** 公共宏统一（`common_defs.h`）。
- **Phase 1** 关键 Bug 修复：`prev_tick` 重置致 dt≡0（I/D 失效）、四元数积分系数 `0.125`→`0.5`、FFC 空指针、重力常数、Mahony 改回标准 PI、float 全 `f` 后缀、数据有效性守卫、Vision 读侧临界区。
- **Phase 2** IMU 解算重构 + gyro 2 秒静态零偏校准。
- **Phase 3** X 翼 4 舵输出改造（TIM4 CH1–CH4，标准混控，SIGN_xx 台架标定）。
- **Phase 4** 控制算法精修：D 项低通、PID `enable` 开关、任务节拍 `vTaskDelayUntil` 稳态化。

### 输入端滤波拆分（2026-05-30）
- [IMU.h](imcalib/Task/IMU.h)：陀螺/加速度共用的卡尔曼 R 拆成两路——`GYR_KF_R` 与 `ACC_KF_R` 独立可调。
- [IMU.c](imcalib/Task/IMU.c)：acc/gyr 两处卡尔曼调用改用各自新宏。
- Mahony 参数复核保持 PI（Kd=0）。
- ⚠️ **现状勘误（2026-06-07 核对代码）**：`GYR_KF_R=ACC_KF_R=1000`（非本条最初记的 30/500）——用户实测 R=30 欠滤波（"滤不动"），关键在 Q:R 比例，**有意保持 1000**；Mahony `Kp=10`（非 2）。以代码为准。

### 控制器端改动（2026-05-30）
- **FFC 前馈接入**：按偏好**与 `pid_calc` 解耦**，不内嵌进 PID。在 [pid.c](imcalib/Tool/pid.c) `Euler_pid_Cale` 里用独立 `FeedForwardController` 对每环目标算前馈再 `+=` 叠加；顺带修了 `FeedForwardController` 更新顺序 bug（二阶项失效）、`num1/num2` double→float。**默认 num1/num2=0 时前馈恒为 0，已接入但未启用**，需在 `pid_init` 给对应轴设非零值才生效。
- **死区软化**：新增 `Deadband_Soften`（连续减区，C0 连续），`pid_calc` 删掉死区 `return 0` 硬切断改走完整流程；附带修了原 `return 0` 致 err 历史/积分不更新、出死区瞬间 D 项尖刺的问题。

### 控制分配解耦（2026-05-31，原 `plan-pitch-priority-mixing.md`，已删除→见本时间线后续「控制分配重写」「交付A 升级」条 / git 历史）
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
- **手算自检**：理想阵零空间 `n=[4,4,-4,-4]∝[1,1,-1,-1]`；`v=[10,0,0]→u0=[10,10,10,10]→Bu=[40,0,0]=v×gain` ✓。
- **诚实边界**：理想 B 是结构近似（未建模舵间耦合/左右不等/失速）；无空速无法动压调度；台架辨识前建议先用 Mode1 飞通再切 Mode2。

### 末制导新帧判定改用显式标志位 `Vision_New_Data_flag`（2026-06-06，输入端）
- **动机**：把 [06-03 视线锁存](#末制导视线目标锁存--帧间-imu-航位推算2026-06-03输入端) 里隐式的「计数器对比 `Vision_Recog_Cnt != last_recog_cnt`」新帧判定，换成显式、语义清晰、可在别处复用的标志位（标准生产者-消费者）。
- **落地**：
  - [CallBack_Task.h](imcalib/Task/CallBack_Task.h) `Vision_Rx_Buf_t` 新增 `Vision_New_Data_flag`；[CallBack_Task.c](imcalib/Task/CallBack_Task.c) `Vision_Receive` 在识别成功(0x5A)与丢目标(0x7A)两分支收到新数据即置 1（生产者，UART 中断 ~20Hz）。
  - [surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`（消费者，控制 1kHz）：`if (Vision_New_Data_flag==1 && recognize==SUCCESS)` 才更新目标、**块末才置 0**；帧间(flag==0)不进，目标靠 `target_angle_Euler[NOW]` 槽保持不变（`Euler_Updata` 只移位 LAST/LLAST、不动 NOW）。
  - **移除中间变量 `los_world_target`**：直接写 `Surface.target_angle_Euler[NOW][YAW]=v.y+current`、PITCH 同（保留 `<-10°` 门控）。`.c` 全局定义已删（[.h](imcalib/Task/surface_control_task.h) 的 `extern` 声明成孤儿、待清理）。
  - 丢目标(FAILURE) 分支移出 flag 门控、**每 tick 跑**（`Vision_Cmd_Work` + 目标回中=当前姿态），同原始行为。
- **影响**：`Vision_Recog_Cnt` 降级为纯统计计数、不再判新帧；Vofa 观测「锁存视线目标」改看 `Surface.target_angle_Euler[NOW][YAW]`。并发上 flag 单字节读写原子、处理后清 0（极偶然处理中来帧会丢一次触发，但数据已入 `Vision_Rx_Data`、下帧补，滞后≤50ms）。

### 交付A 升级为可配置优先级逐级缩放（2026-06-07，输出端）
- **问题**：原交付A `Servo_Mix_AxisLimit`=「三轴各自限幅(40/40/60) → 理想阵合成 → 每片硬 clip 到 ±60」。轴级限幅和=140≫单舵 60，roll/yaw 一大单片即超限触发 clip，硬 clip **无差别砍**该片 pitch 分量 → 退回「饱和砍 pitch」老问题，比旧函数还粗糙（用户指出）。
- **方案**：重写为**字典序（逐级）优先级缩放**。新增运行时可配全局 `Alloc_Prio[3]`（轴枚举[0]最高，默认 `{PITCH,YAW,ROLL}`，调试器 Watch 在线改——Vofa 无下行通道）。按优先级逐级求各轴保留比 `k=clamp(minᵢ (LIMIT−sgn(termᵢ)·baseᵢ)/|termᵢ|, 0,1)`，`base` 累加已定轴贡献；高优先级写入后低优先级只在每片剩余舵机余量叠加、**绝不回削** → 高优先轴不被低优先轴污染、单舵自动 ≤LIMIT 无需粗暴 clip。
- **落地**：[surface_control_task.c](imcalib/Task/surface_control_task.c) 重写 `Servo_Mix_AxisLimit`（含 `Alloc_Prio` 排列合法性校验，非法退默认）；全局补 `Alloc_Prio[3]` 定义与**补回 `servo_lat_scale` 定义**（此前 .h 有 `extern` 无 .c 定义、是重构遗留的悬空声明，写它会链接错），语义泛化为「最低优先轴保留比」；[.h](imcalib/Task/surface_control_task.h) 加 `extern uint8_t Alloc_Prio[3]`。`AXIS_LIMIT_*` 保留作各轴需求前置上限。
- **手算自检**（prio={PITCH,YAW,ROLL}，三轴全饱和）：pitch 级 k=1 全额；yaw 级受已占余量限部分保留；roll 级因某片已饱和 k=0 被挤。反解 **pitch 力矩满额、低优先轴逐级让步**，任意片 ≤±`SERVO_ANGLE_LIMIT` ✓（Plan agent 独立复核 + 数值验证）。
- **影响**：`Servo_Mix_MinEnergy` 奇异退回调用 AxisLimit → 退回质量升级为优先级缩放解；Mode0 旧 `Servo_Mix_PitchPriority`（k 公式反写 bug）仍仅作对照不动；调用点 switch / 下游 PWM/SIGN 不变。

### 内环角速度 pitch/roll 轴配对修正（2026-06-07，台架确认）
- **症状**：roll 自稳一开始能稳一点，随后发散成自旋；yaw 自稳一直有效、roll 不行。
- **根因**：串级内环要求「角速度 = 外环角度的导数」（同轴）。欧拉角解算里 `Euler[ROLL]`=绕机体 X 角，其角速度是四元数积分用的 `gx=G_Rad[X]=G[PITCH]`（chipX）；`Euler[PITCH]`=绕机体 Y 角、角速度=`G[ROLL]`（chipY）。但内环 roll 喂的是 `current_gyro_Euler[ROLL]=G[ROLL]`、pitch 喂的是 `G[PITCH]` → **roll/pitch 内环角速度反馈各自接到了对方的物理轴**。yaw（机体 Z）自洽故有效；roll 内环读不到真正的自旋角速度 → 退化成纯 P 无阻尼 + 俯仰角速度串扰 → 起初 P 能拉、随后发散自旋。
- **台架确认**：纯绕纵轴（自旋轴）滚转时 `G[PITCH]`（chipX）跳、`G[ROLL]` 几乎不动 → 纵轴=chipX=`Euler[ROLL]` 轴。
- **修复（方案B，用户最终选定）**：用户没用「只对调内环」的方案A，而是在**源头对调** [IMU.c:326-327](imcalib/Task/IMU.c#L326) 寄存器 `G[ROLL]←chipX(rx[2,1])`、`G[PITCH]←chipY(rx[4,3])`，使陀螺命名与物理轴对齐（`G[ROLL]`=纵轴）。但只动源头会让四元数 `gx=G_Rad[X]=chipY` → 把 chipY 当机体 X → **欧拉角 pitch/roll 物理含义被一起对调**。故方案B 必须**同步对调四元数取轴**：[IMU.c:64-65](imcalib/Task/IMU.c#L64) `gx=G_Rad[ROLL]`(chipX)、`gy=G_Rad[PITCH]`(chipY)，使 `(gx,gy,gz)=(chipX,chipY,chipZ)` 与源头对调前一致 → 欧拉角回正；`current_gyro_Euler` 保持直通（[IMU.c:345](imcalib/Task/IMU.c#L345)），内环 roll 自动拿到 `G[ROLL]`=chipX=纵轴。**净效果与方案A 数学等价**（四元数/欧拉角同原始 + 内环拿对物理轴）。
- **方案B 连带处理**：源头对调改了 `G[PITCH]/G[ROLL]`、`G_Rad[X]/[Y]` 的物理含义，排查全部消费点——① **PNG 制导** [PNG_Task.c:50-51](imcalib/Task/PNG_Task.c#L50) 直接吃 `G_Rad[X]/[Y]` 做视线角速度补偿，改取 `G_Rad[ROLL]/[PITCH]` 取回原 chipX/chipY，**保持制导行为与对调前一致**；② [IMU.c:22](imcalib/Task/IMU.c#L22) KF3 在 `#if 0` 内禁用、零偏标定逐通道独立——无影响；③ Vofa/视觉遥测（[TotalControl.c:147-149](imcalib/Task/TotalControl.c#L147)、[CallBack_Task.c:189-191](imcalib/Task/CallBack_Task.c#L189)）仅显示、代码不动，曲线 `G_PITCH` 现为 chipY、`G_ROLL` 现为 chipX（=纵轴，更直观）。

### PID 角度误差环绕（2026-06-07）
- **动机**：roll/yaw 外环用 `atan2` 欧拉角 ∈[-180,180]，是周期量；目标/当前跨 ±180° 边界时 `err=set−get` 出现 ~360° 假跳变 → 反向猛打诱发自旋。pitch 是 `asin`∈[-90,90] 不环绕。
- **落地**：[pid.h](imcalib/Tool/pid.h) `pid_t` 加按通道开关 `uint8_t angle_wrap` + 声明 `Angle_Wrap_180`；[pid.c](imcalib/Tool/pid.c) 新增 `Angle_Wrap_180`（归一到 (-180,180]），`pid_calc` 在 `err=set−get` 后、`max_err` 判断前 `if(angle_wrap) err=Angle_Wrap_180(err)`（放 max_err 前免假跳误触发）；`pid_init` 仅给 `Angle/ROLL`、`Angle/YAW` 外环置 1。内环角速度环 / mahony 不开（非周期量）；角度外环 kd=0，wrap 不引 D 尖刺。

### 坐标系统一 + 四元数/欧拉角重定向（2026-06-07，姿态解算）
- **根因**：用户重新明确机体系为 ENU（X=右/东、Y=前/北、Z=上/天；pitch 绕 X 抬头+、roll 绕 Y 右滚+、yaw 绕 Z 右偏+）。原解算两处不自洽：① **加速度系与陀螺系差一个 X↔Y 对调**——加速度 `A[X]=chipY(右)/A[Y]=chipX(前)`，但四元数取陀螺 `gx=G_Rad[ROLL]=chipX(前)` 把"前"当机体 X → Mahony `e=a×t` 在两个对调的系做叉乘、修正错 → 姿态漂移（这才是"解算又不对"）；② 欧拉角用标准 ZYX（roll 绕 X/pitch 绕 Y），与"pitch 绕 X、roll 绕 Y"不符。
- **修复（[IMU.c](imcalib/Task/IMU.c) / [IMU.h](imcalib/Task/IMU.h)）**：
  - **统一机体系 X=右/Y=前/Z=上**：四元数取陀螺改 `gx=G_Rad[PITCH](chipY=右)、gy=G_Rad[ROLL](chipX=前)、gz=G_Rad[YAW](chipZ=上)`，使陀螺系=加速度系、Mahony 自洽（[IMU.c:66-68](imcalib/Task/IMU.c#L66)）。
  - **欧拉角提取重写**（[IMU.c:172-174](imcalib/Task/IMU.c#L172)）：`PITCH=asin(2(yz+wx))`、`ROLL=atan2(2(wy−xz),1−2(x²+y²))`、`YAW=atan2(2(xy−wz),1−2(x²+z²))`，三轴极性=抬头+/右滚+/右偏+。
  - **内环角速度反馈**移到姿态算法、用原始测量按用户极性（[IMU.c:74-76](imcalib/Task/IMU.c#L74)）：pitch=+gx、roll=+gy、**yaw=−gz**（Z 上右手下 +gz=左偏，故 yaw 右+取负）；pitch/roll 直通=与方案B 一致，内外环同号 by construction（从同一组机体角速度派生）。
  - **可配符号宏**（[IMU.h](imcalib/Task/IMU.h)）`ACC_SIGN_X/Y/Z`、`GYR_SIGN_X/Y/Z`，默认=当前硬件，台架按验证表锁定芯片物理正向。
- **数值核验**（纯 Python，非硬件）：抬头/右滚/右偏 +30°→对应轴各 +30；纯绕前轴横滚 pitch 恒 0；组合旋转三轴正确解耦；Mahony 注入 +8° roll 误差收敛回 0（证 `e=a×v, ω+=Kp·e` 负反馈）。**关键发现**：原 Mahony 符号其实是对的（`pid_calc` 的 `set−get` 已把它翻正），真 bug 只在陀螺取轴 → 改动外科手术式（积分公式/Mahony/R/A_World 均不动）。
- **诚实边界**：yaw 对齐右+后角度/速率/目标整体翻号 → yaw PID 输出或翻 → 舵面响应或反转，台架单轴确认、反了翻 yaw 舵面 SIGN（本次不动混控符号）。BMX055 加速度/陀螺封装轴向本就不同，各轴物理符号只能台架锁定。下游状态机(pitch 抬头+/过0)、发射判定(A[Y])、PNG 取轴(G_Rad[ROLL/PITCH] raw)、控制分配/roll 反旋全不变。

### X 翼 pitch 解算几何对齐（2026-06-08，输出端）
- **解算（现行正确做法）**：X 翼 4 片成 X(45°，从尾看 UL135/UR45/DR315/DL225)，每片偏转产生切向力，对三轴力矩贡献 **pitch∝cosθ=[UL−,UR+,DR+,DL−]、yaw∝sinθ=[+,+,−,−]、roll=常数[+,+,+,+]**（三模态两两正交，第 4 模态 `[+,−,+,−]` 隔片交替=零空间纯阻力）。逻辑阵配物理 `SIGN=[−1,+1,+1,−1]`（左右镜像安装）后，pitch 指令落到舵令 `u=[−,+,+,−]`=真俯仰、与 roll/yaw 正交解耦。
- **三处把 pitch 列对齐为 `[+1,+1,+1,+1]`（四片同号）**：[surface_control_task.c](imcalib/Task/surface_control_task.c) `Servo_Mix_AxisLimit` 的 C 阵 / `Alloc_B` pitch 行 / `Servo_Mix_PitchPriority` 的 `P[]`。SIGN 不动（每片三轴共享、只修整片装反，轴间配对由逻辑阵列决定）；`Alloc_B` 零空间相应为 `n=[4,4,−4,−4]∝[1,1,−1,−1]`。
- **待台架**：`Alloc_Mode=1` 纯 pitch 阶跃应见明确抬/低头、掉速小；抬头方向那 bit 台架定（现 `[+1,+1,+1,+1]` 反了则整体取负）。pitch/yaw 调用点清零行已注释 → 三轴放开（旧"pitch 掐死"描述已过时）。
- *(背景：原 pitch 列 `[+1,+1,−1,−1]` 经 SIGN 落零空间→只减速不产生俯仰；先前以 BBᵀ=4I “证明解耦”仅算法对自身假设 B 自洽、非物理正确，以本条几何为准。)*

### 制导段抖动修复：D 项对测量微分 + 视觉目标斜坡（2026-06-10，输入端/源头）
- **症状**：目标附近抖动，**主要在末制导(视觉介入)段**；纯陀螺自稳段不明显。想给小 D 加阻尼反而抖得更严重。
- **根因 = 微分冲击（derivative kick）**：[pid.c](imcalib/Tool/pid.c) `pid_calc` 的 D 原对**误差** `e=set−get` 求导。纯陀螺段目标恒定→`de/dt=−d(角度)/dt` 纯阻尼正常；末制导段目标每 50ms 被视觉新帧阶跃刷新（[surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`）→新帧那拍 `dout=d·Δset/dt=d·Δ·1000` 脉冲尖刺、20Hz 周期激励→抖；Δ 再小也炸(d=0.005、Δ=0.5° 仍出 2.5 脉冲)。纯陀螺段无 setpoint 阶跃故不抖——与症状吻合。
- **方案1（根治，[pid.c](imcalib/Tool/pid.c)）**：D 改对**测量(反馈)微分** `dout=−d·(get[NOW]−get[LAST])/dt`，只对反馈量求导、不对目标求导→目标阶跃不再进 D；反馈是物理连续量、D 只对真实运动求导=纯阻尼。D 不经死区软化(死区内 P 不拉、保留 D 阻尼压残余运动)。零延迟、非低通——PLC 默认做法，原 Phase4.1 计划的 D 项低通因此暂不需要。
- **方案3（治视觉低帧率源，[surface_control_task.c](imcalib/Task/surface_control_task.c) / [.h](imcalib/Task/surface_control_task.h)）**：在 **setpoint 端**(非反馈环、不引相位滞后)给视觉锁存目标加**速率限制斜坡**。新增全局 `vision_los_final[3]`(世界系视线终点，Vofa 可观测)、静态 `Target_Slew(cur,target,max_step,wrap)`(YAW 差值经 `Angle_Wrap_180` 走最短弧)；`Guidance_Terminal` 重写为「视觉新帧只更新终点 `vision_los_final` + 每 tick `target` 朝终点斜坡逼近 `VISION_TARGET_SLEW_DPS·dT`」。帧间终点不变、斜坡到达后 `target≡终点`，与原「锁存+航位推算」等价，仅消去切换瞬间阶跃；FAILURE 终点+目标都对齐当前(斜坡 d=0=原就地保持)。
- **参数**：`VISION_TARGET_SLEW_DPS`=150°/s(=0.15°/tick) 初值，台架可调——大→更跟手(接近阶跃、削 kick 弱)、小→更平滑(滞后增大，过小会跟不上视线角速度致脱靶)。
- **未编译**(Keil/MDK 工程需在 IDE 里编)。两项独立叠加、互不冲突，均合「输入端/源头解决、不在反馈环加低通」偏好。

### 末制导俯仰能量管理 + 速度预测完善（2026-06-12）
- **问题**:末制导俯仰为纯追尾(鼻先指向视线 LOS)。识别到引导灯(机体俯仰≈-5°才看得到)时,俯仰与期望入射俯冲角(=速度方向=撞击姿态、正向撞击、≈-25~-30°)误差大;直接打到位→机体俯仰远比速度方向陡→大负迎角掉升力→重力往下掉损能→滑翔距离不够→打不稳。
- **三决策(用户定)**:① 距离调度=视觉回传(像素面积+距离**一起用**);② 速度预测**作入射角参考(锁定初速)**;③ 俯仰控制权=**设俯冲上限(改动最小)**:俯仰仍跟 LOS,叠加随接近度放开的最陡俯冲限幅。
- **A 速度预测完善（[filter.c](imcalib/Tool/filter.c) / [IMU.c](imcalib/Task/IMU.c)）**:修 `Kalman_Vel_Calc` 输出索引 bug(速度=状态0=`res[0]`,原误取 `res[1]`=加速度);新增 `Kalman_Vel_Set` 入段锚定接口;IMU 取消注释速度积分/机体速度;俯冲入段(`Stable→Terminal` 置 `Vel_Reanchor_Flag`)用「姿态前向(R_matrix_T 第1行)×标称速度 `V_NOM_MS`」锚定世界速度 → 弹道角 `gamma_pitch_deg=atan2(Vz,√(Vx²+Vy²))` 起始≈机体俯仰、随重力演化。纯积分无 ZUPT 会漂、终端段短+锚定可接受;`V_NOM` 只影响 γ 演化速率不影响初始 γ。
- **B 俯冲限幅（[surface_control_task.c](imcalib/Task/surface_control_task.c)）**:新增 `Pitch_Dive_Floor`,`θ_floor=max(L_sched, γ−AOA_MARGIN_DEG)`。L_sched 按接近度 s∈[0,1] 从 `PITCH_DIVE_LIMIT_FAR_DEG`(-8°)线性放开到 `PITCH_INCIDENT_DEG`(-27°)。`Guidance_Terminal` 末**仅识别成功**时钳俯仰目标≥θ_floor(丢目标保持原"持当前");调用点去掉旧 `pitch>-20→p_body=0` 硬掐(改由 θ_floor 承担,pitch 全程受控浅滑翔更省能)。迎角项 γ−AOA_MARGIN 防距离/面积被骗时仍守住(大负迎角=损能根因);终端 γ≈θ→迎角 0=正向撞击。
- **接近度 s 分段合成(面积+距离一起用)**:远段用距离 `dist_cm`(标定准、连续,s:0→`DIVE_SCHED_SWITCH`),近段用面积 `area`(blob 大、近场更可靠,s:`DIVE_SCHED_SWITCH`→1),`dist_cm` 决定走哪段;两者都无(dist_cm=0)退化按 γ 自调度。
- **C 视觉协议（[CallBack_Task.c](imcalib/Task/CallBack_Task.c) / .h）**:新增**独立 6 字节包** `0x5B + dist_hi + dist_lo + area_hi + area_lo + 0xA6`(dist/area 均 uint16 大端),**不动** 0x5A 识别包(仍 6 字节 x,y)。`Vision_Rx_Buf_t` 加 `dist_cm/area`;`Vision_Receive` 加 0x5B 分支(只更新 dist/area,不置 recognize/New_Data);缓冲仍 6 字节、`Size==6`。OpenMV 端 `send_distance(dist_cm,area)`(`dist_cm=DIST_K/sqrt(px)`)已配套。
- **未编译**(Keil/MDK)。Vofa/调试器 Watch 观测 `gamma_pitch_deg` / `pitch_dive_floor` / `closeness_s` / `Vision_Rx_Data.dist_cm/area`。新增宏(全待台架实测):`PITCH_INCIDENT_DEG/PITCH_DIVE_LIMIT_FAR_DEG/AOA_MARGIN_DEG/GAMMA_FAR_DEG/DIST_ACQUIRE_CM/DIST_NEAR_CM/AREA_NEAR/AREA_IMPACT/DIVE_SCHED_SWITCH`(surface_control_task.h)、`V_NOM_MS`(IMU.h)。

### 速度预测/弹道角 γ 单位量纲修复（2026-06-13）
- **症状**:实测 `gamma_pitch_deg` 不准、速度预测失效,末制导只能靠视觉钳位（[surface_control_task.c](imcalib/Task/surface_control_task.c) `Pitch_Dive_Floor` 的 `dist_cm==0` 分支已临时降级 `s=0.0f`、注释"实测 γ 不够准"）。
- **根因 = A_World 去重力量纲不一致**:[IMU.c](imcalib/Task/IMU.c) 原 `a_no_gravity = a_raw − gravity·R_col3`,而 `a_raw` 单位是 **g**（静止‖a‖≈1、`ACC_LSB_16G=1/2048`、Mahony `acc_dev=|‖a‖−1|` 门控也以 1g 为基准）、`gravity=GRAVITY_MS2=9.80665` 是 **m/s²**——g 减 m/s²（静止水平误出 1−9.8=−8.8）→ A_World/速度积分/γ 全错。**2026-06-07 审计即记"A_World量纲g与m/s²混减"**,当时 PNG 未接主环遂留待;2026-06-12 启用速度预测把 A_World 拉进 γ 末制导路径却没跟着修 → γ 不准。
- **修复（[IMU.c](imcalib/Task/IMU.c)）**:改 `a_no_gravity = gravity·(a_raw − R_col3)`(先 ×GRAVITY_MS2 把 `a_raw` 转 m/s²、再扣机体系重力投影 `gravity·R_col3`),静止线加速度=0(速度不漂)、量纲与 `V_NOM_MS`(m/s)及积分 `v+=dT·a` 一致;`a_raw_x/y/z` 局部量本身不动 → Mahony 仍按 g 单位归一化、`acc_dev` 门控不受影响(外科手术式)。
- **未编译**(Keil/MDK 需 IDE 编)。**待 Vofa 验证**:静置 `gamma_pitch_deg`≈0(原应漂)、鼻先下压 γ 变负且≈机体俯仰、纯横滚不大改 γ;验准后可把 `Pitch_Dive_Floor` `dist_cm==0` 分支改回按 γ 自调度([surface_control_task.c](imcalib/Task/surface_control_task.c) line 161)、`L_aoa=γ−AOA_MARGIN` 重新可信。
- **残余漂移源(次要、终端段短可接受)**:加速度零偏 `A_Offset` 标定了但未回扣主环([IMU.h](imcalib/Task/IMU.h) 注释)、发射后 `acc_trust=0` 纯陀螺 coast 致姿态/A_World 方向漂——若验后仍明显漂再议回扣 `A_Offset` / 加 ZUPT。

### 末制导比例化逼近（替固定斜坡）+ PN 视线率超前补偿（2026-06-14，setpoint 端）
- **动机**：06-10 的视觉目标斜坡用**固定速率** `VISION_TARGET_SLEW_DPS·dT`——远近误差一个速率，用户实测**太固定**（误差大跟不上、误差小又过冲），改为**比例化逼近**。
- **比例化逼近（[surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`）**：每 tick 把斜坡起点 `vision_los_current[]` 重置为当前姿态，`Target_Slew` 的 `max_step` 从固定值改为 `|终点−当前|/N`（YAW N=15、PITCH N=25）。净效果＝**目标 = 当前姿态 + LOS误差/N**，即把视线误差分 N 段逐拍弥补：误差大走得快、误差小走得慢、自然收敛，不再固定速率过冲/欠跟。旧宏 `VISION_TARGET_SLEW_DPS` 废弃（[.h:68-73](imcalib/Task/surface_control_task.h#L68-L73) 注释保留说明）。
- **PN 视线率超前补偿（[.c](imcalib/Task/surface_control_task.c) / [.h:90-98](imcalib/Task/surface_control_task.h#L90-L98)）**：锁存的世界系视线终点帧间差分得惯性视线率 λ̇（`vision_los_rate[]`，纯视觉、不依赖会漂的 IMU 积分速度），**仅识别成功**时 `target += PN_LEAD_K·λ̇`（YAW 全程、PITCH 仅俯冲到位 <-8° 后）——驱动 λ̇→0＝碰撞航线，既提前命中又给外环超前相位压猎振。`LOS_RATE_LIMIT_DPS=30`°/s 限幅防丢帧/视觉跳变爆冲；丢目标清 λ̇、复位首帧标志（重捕不吃陈旧 λ̇）。配套常值配平迎角前馈 `AOA_TRIM_DEG`（默认 0=关；本工程 γ 取姿态前向≡θ、θ−γ≡0 无迎角信息时退化用）。
- **参数（[.h](imcalib/Task/surface_control_task.h)，全待台架/试飞实测）**：`PN_LEAD_K=0.05`（先小增益验方向再加大，过大易被视觉噪声激励）、`LOS_RATE_LIMIT_DPS=30`、`AOA_TRIM_DEG=0`；比例分段数 N=15(YAW)/25(PITCH) 写在 .c 调用点。
- **遗留留痕（未动代码，记录待清）**：λ̇ 差分含裸魔数 `/9.5f`（[.c:218-219](imcalib/Task/surface_control_task.c#L218-L219)，经验压噪、含义未注明）；`Target_Slew` 上方注释 [.c:244-247](imcalib/Task/surface_control_task.c#L244-L247) 仍按旧"恒速率斜坡"描述、**已过时**；pitch 门控 `pitch_control_limit_deg=-8°`（[.c:186](imcalib/Task/surface_control_task.c#L186)）与 [.c:205](imcalib/Task/surface_control_task.c#L205) 注释"<-10°"对不上。
- **未编译**(Keil/MDK)。**待 Vofa/试飞验证**：① 比例化——误差大快速逼近不过冲、误差小平滑收敛不抖，调 N 看跟手/平滑权衡；② PN——`vision_los_rate` 随视线转动有值、丢帧/丢目标归零不爆冲，小 `PN_LEAD_K` 先验超前方向（应提前于纯追尾命中）再加大；③ 回归：丢目标(FAILURE)/未俯冲到位仍就地保持（目标=当前）。

### 视觉/IMU 紧耦合 EKF（vision_ins.c/.h，2026-06-15+）
- **动机**：纯积分速度无外部观测必线性发散；旧 `Kalman_Vel_Calc`（filter.c 二阶卡尔曼）已 `#if 0` 停用；ZUPT 只在地面静止时成立，飞行段没有零速时刻。唯一能在飞行中钉住速度的外部观测是视觉（看固定靶，给方位+距离）。
- **方案**：6 态线性 KF `x=[p(3),v(3)]`，世界系 ENU，相对固定靶。IMU 加速度做输入预测（1kHz）、视觉笛卡尔位置量测更新（~30Hz）、静止零速更新。姿态不进状态（沿用 Mahony 松耦合），加速度零偏不进状态（靠地面标定+ZUPT refine）。
- **量测**：视觉帧识别成功时，像素→机体系视线单位向量 + dist_cm→距离，合成 `z=−range·u_world` 作 3 维位置量测。量测噪声各向异性：`R=σ⊥²·I+(σr²−σ⊥²)·u·uᵀ`（方位准、测距粗）。零速更新：物理静止时量测 v=0。
- **落地**：[vision_ins.c](imcalib/Tool/vision_ins.c) / [.h](imcalib/Tool/vision_ins.h)（新文件）。[IMU.c](imcalib/Task/IMU.c) 内调用：predict 每拍 + 视觉新帧位置更新 + 静止零速更新，单任务零竞争。EKF 输出回写 `IMU_Data.Velocity[World]`，供机体速度映射/弹道角 γ/Vofa。
- **取代**：旧 `Kalman_Vel_Calc/Set/Init`（filter.c）已 `#if 0` 禁用；`gamma_pitch_fwd_deg`（姿态前向估计）取代会漂的速度版 `gamma_pitch_deg` 供末制导用。

### LADRC 线性自抗扰控制器（adrc.c/.h 重写，2026-06-21+）
- **动机**：原非线性 ADRC 用 fal/fst + α/δ 参数，每轴 2 环 ×(TD+ESO+NLSEF+b0+α/δ) 十几个旋钮，没法系统地调。
- **方案**：全部换成高志强(Gao)带宽法 LADRC：单环二阶 = 三阶线性扩张状态观测器(LESO) + 线性状态误差反馈(LSEF) + 扰动补偿。整轴只剩 3 个旋钮：wc(控制带宽)、wo(观测带宽≈3~5×wc)、b0(控制增益估计)。
- **阻尼源**：默认用实测陀螺（`use_gyro_damp`），而非 LESO 估的 z2——相位准、不依赖 b0，roll 不易高频抖。
- **落地**：[adrc.c](imcalib/Tool/adrc.c) / [.h](imcalib/Tool/adrc.h)（文件名不变，内部全换）。`LADRC_Init` 按通道给默认参数；`LADRC_Calc` 单拍计算；`Euler_LADRC_Cale` 三轴并行。[surface_control_task.c](imcalib/Task/surface_control_task.c) 新增 `ladrc_mode` 运行时切档（0=全PID/1=三轴LADRC/3=仅Roll LADRC）。
- **Roll 已标定**：wc=10.5, wo=52.5, b0=55, deadband=1°, max_output=±15°。pitch/yaw 占位默认待启用。

### LQR 状态反馈控制器（lqr.c/.h 新建，2026-06-27，**未编译/待台架**）
- **动机**：尝试用 LQR 一步 6态→4舵替代「PID 算三轴力矩 + 混控器分配」两步链路（K 已含 X 翼混控几何）。移植自 [lqr_czn/dart_attitude_LQR_v1.m](lqr_czn/dart_attitude_LQR_v1(1)(1).m) + [移植指南](lqr_czn/MCU_LQR_PORTING_GUIDE(1).md)。
- **方案**：状态 `x=[roll,pitch,yaw 误差(rad), p,q,r(rad/s)]`（err=测量−期望），控制律 `u=-K_d·x`，4 舵偏(rad)；`K_d[4][6]` 由 MATLAB `dlqr` 算出（连续双积分器 A + `B=I⁻¹·G·k_aero`，c2d 离散）。
- **落地**：[lqr.c](imcalib/Tool/lqr.c) / [.h](imcalib/Tool/lqr.h)。`LQR_Update` 纯解算（与 MATLAB 对拍用）；`Euler_LQR_Cale` 桥接：取 Surface 姿态/角速度→组 x→解算→换序+×SIGN+转度，**直接写 `output_angle_Servo`、绕过 `output_gyro_Euler` 与 `Servo_Mix_*`**。[surface_control_task.c](imcalib/Task/surface_control_task.c) 新增 `lqr_mode` 切档（0=关默认/1=LQR；优先级高于 ladrc/vel_pursuit，混控分派加 `lqr_mode!=1` 跳过）。`LQR_Init` 已挂 TotalInitTask。
- **K 矩阵粘贴区**：`dart_lqr_K[4][6]` 与 MATLAB 同名同形；调好 Q/R/惯量/速度后跑脚本 Step5，把打印的 4 行整块覆盖粘贴即可。当前是脚本**占位参数**导出值，台架前必须用真实惯量/气动/速度重跑更新。
- **⚠ 上车前必做**：① 新增 `lqr.c` 手动加入 Keil/eIDE 工程编译列表（AI 编不了）；② 按指南 §9 逐轴阶跃验符号——MATLAB 舵号(delta1=右上/2=左上/3=左下/4=右下)≠工程索引(UL/UR/DR/DL)，已在 `K_ROW_TO_SERVO[]` 换序，符号反了在 `SIGN_xx`/`gyro_sign[]` 翻，**别改 K 行序**（飞镖一次性，G 符号反=正反馈）。
- **pitch 仅制导段受控**（开关 `lqr_pitch_terminal_only`，lqr.c 默认 1，2026-06-27 加）：`Euler_LQR_Cale` 组完状态后、`LQR_Update` 前，非 `Terminal` 段把 pitch 状态分量 `x[1]`(pitch_err)/`x[4]`(q) 清零 → 一步解出的 4 舵**不含 pitch 通道成分**，Stable 等自稳阶段 pitch 不打舵、只 roll/yaw 受控；制导段(Terminal)恢复全三轴控；`err_deg[1]` 保留真值供观测。=0 回全程控 pitch 做 A/B。作者诉求：「只在制导段才控 pitch，其他时候舵机 pitch 不受控」。**未编译/待台架。**

### ZUPT 零速更新 + 地面零偏在线对准（IMU.c，2026-06-15+）
- **问题**：纯积分速度无外部观测→任何加速度零偏/姿态残差都被无限积分而漂（实测"漂移远大于真实运动"）。
- **方案**：发射前（状态机 Self_Text/Start、地面静止）用零速观测把速度钉回 0（ZUPT=融合，非低通），并把"静止残差 a_raw−R_col3"慢速喂给机体系零偏 `A_Offset` 在线对准（不依赖标定时是否水平）；发射后冻结零偏、停 ZUPT（防匀速飞行 ‖a‖≈1g 被误判静止而错误归零）。
- **判据**：|‖a‖−1g|（用去零偏模长）与角速度双小、持续 `ZUPT_HOLD_CNT`(100) 拍才确认静止。`imu_is_static` 全局标志供 Vofa 观测。
- **落地**：[IMU.c](imcalib/Task/IMU.c) `IMU_Attitude_Algorithm` 内 ZUPT 段 + [IMU.h](imcalib/Task/IMU.h) 新增宏 `ZUPT_*` / `ACC_BIAS_LPF_K`。

### 视线角半径归一化 + 距离增益调度（surface_control_task.c/.h，2026-06-15+）
- **视线角归一化**：远处引导灯 blob 半径小（~5px），同样像素偏移对应更大实际角度；近处 blob 大（~30px），偏移对应更小角度。`Vision_Angle_Normalize(angle, radius)` 归一化到 `REF_RADIUS`(15px)，使控制增益不随距离变化。
- **距离增益**：`Yaw_Gain_ByDist(dist_cm)` / `Pitch_Gain_ByDist(dist_cm)` 纯距离线性插值增益，取代旧的面积+距离双段合成（已注释）。YAW 远处增益大（补偿视觉距离效应）、近处小（防过冲）；PITCH 反向。
- **落地**：[surface_control_task.c](imcalib/Task/surface_control_task.c) 新增 3 个静态函数 + [.h](imcalib/Task/surface_control_task.h) 新增宏 `REF_RADIUS*` / `YAW_GAIN_*` / `PITCH_GAIN_*`。

### 视觉目标 LPF 平滑（替 Target_Slew 主路径，surface_control_task.c，2026-06-15+）
- **动机**：06-14 比例化逼近用 `Target_Slew`（每拍斜坡），实测仍有抖动。改用一阶低通滤波 `target = k×final + (1−k)×target_last`，更平滑、无阶跃、自然收敛。
- **方案**：`Guidance_Terminal` 内 LPF 代替 `Target_Slew` 主路径。k 由距离增益缩放（远处小 k 保守、近处大 k 跟手）。`Target_Slew` 函数保留但不再主路径调用。
- **落地**：[surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal` 重写为 LPF 路径。

### 速度矢量追踪三级串级（surface_control_task.c，2026-06-15+）
- **方案**：外环速度方向追目标方向→输出 body 目标角；中环镖头追 body 目标角→输出 rate_cmd（复用角度环 PID）；内环角速度追 rate_cmd→输出舵偏（复用 gyro 环 PID）。`vel_pursuit_mode=0` 时退化为原两级 PID。
- **落地**：[surface_control_task.c](imcalib/Task/surface_control_task.c) 新增 `Velocity_Pursuit_Cale()` + `vel_pursuit_pid[2]`（[pid.c](imcalib/Tool/pid.c)）。当前 `vel_pursuit_mode=0`（未启用）。

### 参数勘误（2026-06-23 审计核对代码）
以下参数代码实际值与旧文档不一致，**以代码为准**：
- `PN_LEAD_K` = **0.5**（旧记 0.05）
- `LOS_RATE_LIMIT_DPS` = **40**°/s（旧记 30）
- `pitch_control_limit_deg` = **−5.0**°（旧记 −8.0）
- `Alloc_Prio` 默认 = **{YAW, ROLL, PITCH}**（旧记 {PITCH,YAW,ROLL}）
- Alloc_B pitch 行 / C 阵 pitch 列 = **[−1,−1,−1,−1]**（旧记 [+1,+1,+1,+1]，SIGN 翻转后等效）
- PID 增益：Angle PITCH=0.2/ROLL=1.8/YAW=0.55；Gyro PITCH=0.30/ROLL=0.2/YAW=0.5（CODE_OVERVIEW 表过时）
- 死区：Angle PITCH=1.0/ROLL=0.5/YAW=0.0；Gyro PITCH=0.0/ROLL=1.0/YAW=1.0
- angle_wrap：ROLL/YAW 外环均 **=0**（旧记 =1，实际未启用环绕）
- D 项：pid.c:188 测量微分被注释，:189 用**误差微分** `d*(e_now-e_last)/dt`（PROGRESS 06-10 记的"对测量微分"已不成立）

### 三维耦合控制 →「二维直通」回退（2026-06-23，控制器结构）
- **背景**：06-22（commit d38ef2d）曾在 `Euler_pid_Cale` 外环与内环之间插入**欧拉运动学变换 T(φ,θ)**，把外环输出的「世界系欧拉角速率 `[φ̇,θ̇,ψ̇]`」映射成机体角速度 `[ωx,ωy,ωz]` 再喂内环（内环反馈是机体陀螺），意图消除有横滚时 pitch/yaw 的天然耦合，并省掉外部 `Roll_Derotate_PitchYaw`。
- **回退（commit e5b9778「将三维转化为2维」）**：变换**整段注释、改回直通** `body_rate_cmd[axis]=temp[axis]`。原因：① 本工程陀螺轴序非标准 ZYX（`gx=PITCH, gy=ROLL, gz=YAW`，见 [IMU.c](imcalib/Task/IMU.c)），标准 T(φ,θ) 套不上、需按实际轴序重推；② 飞镖小角度/小滚转下耦合可忽略，直通已够用且更稳（"yaw 也稳定了一点"）。变换公式作 **TODO 框架代码**留在 [pid.c](imcalib/Tool/pid.c) `Euler_pid_Cale` 注释里，待轴序标定后再启用（台架判据：固定 pitch/yaw 目标手动滚转 20°，看是否出现意外 yaw 漂移）。
- **同 commit 连带**：① PID 增益 + 死区大改（死区 Angle→1.0、Gyro→0.0）；② **PNG 拆分接口** `PNG_Apply_Lead_Yaw/Pitch`（[PNG_Task.c](imcalib/Task/PNG_Task.c)）+ `PNG_Mode=1`（用 vision_ins EKF 世界系 p/v 叉乘**直接算视线率** ω_yaw/ω_pitch，替代纯视觉帧差分），供 `Guidance_Terminal` 分轴门控（俯冲未到位只喂 yaw）——**但主路径仍 `#if 0` 关闭**，先验证纯 PID 基础跟踪。

### 调参哲学转向「内环>外环纯 P 串级」（2026-06-24~25，commit 4d46d01/4f605eb「改变了调参方式」）
- **变化**：增益整体拍平，**内环 P 反超外环**、I/D 基本归零——Angle(外环) PITCH/ROLL/YAW≈`0.25/0.45/0.45`，Gyro(内环)≈`0.80/0.90/1.10`（**快照，以代码为准**）。呼应既有 TODO「串级内外环增益拉开」与控制偏好「纯 P 串级=单环 PD+陀螺阻尼」。
- **roll 内环陀螺反馈 ÷2**：[pid.c](imcalib/Tool/pid.c) `Euler_pid_Cale` 内环 roll 喂 `current_gyro_Euler[NOW][ROLL]/2.0f`（经验压抖/降 roll 内环等效增益，留痕待解释）。
- **pitch 外环门控收紧**：pitch 外环+内环仅在 `Guidance_State>Stable && 视觉识别成功` 时更新，否则内环保持上拍（Stable 段不控俯仰）；yaw/roll 每拍都算。
- **分配器默认 `Alloc_Mode` 1→2**（最小能量 `Servo_Mix_MinEnergy` 为默认；Mode0 `Servo_Mix_PitchPriority` 调用点已注释，仅留 1/2 两档）。

### 末制导 pitch「主动滑翔→扎」（2026-06-27，commit a13f2b4 + 未提交，setpoint 端）
- **动机**：纯追尾/门限放开 pitch 易过早扎下、大负迎角掉升力损能、射程不够（与 06-12 能量管理同一痛点，换更简洁的实现）。
- **方案**：新增 `pitch_glide_mode`（默认 **1**，可调试器 Watch 切回 0 做 A/B）。`Guidance_Terminal` 里按**世界系看灯视线俯角 φ**(`vision_los_final[NOW][PITCH]`) 在两段间插值：远端（视线浅，φ≥`GLIDE_LOS_HI_DEG`=−12°）`blend=0` → pitch 目标住 `THETA_GLIDE_DEG`(+2° 抬头)**压平轨迹增程**；近端（视线陡，φ≤`GLIDE_LOS_LO_DEG`=−25°）`blend=1` → 平滑过渡到追视觉 pitch 目标**扎下去**（与 `PITCH_INCIDENT_DEG≈−27°` 衔接）。`pitch_glide_target=(1−blend)·THETA_GLIDE + blend·视觉目标`。
- **落地**：[surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`（`pitch_glide_mode` 分支，旧 `pitch_control_limit_deg` 门限逻辑保留作 `=0` 对照）+ [.h](imcalib/Task/surface_control_task.h) 新宏 `THETA_GLIDE_DEG/GLIDE_LOS_HI_DEG/GLIDE_LOS_LO_DEG`、新全局 `pitch_glide_mode/target/blend`（Vofa/Watch 可观测）。`AXIS_LIMIT_PITCH` 20→30。
- **未编译**(Keil/MDK)。**待台架/试飞**：远端是否真压平增程、过渡是否平滑无台阶、`THETA_GLIDE_DEG` 过大是否失速反掉得更快；blend 随接近度 0→1。

### 末制导 LPF/距离增益死代码清理 + 增益调度函数 `#if 0` 封存（2026-06-27，未提交）
- **动机**：LPF 早已被旁路（`Low_Pass_Filter` 调用注释、`lpf_yaw/pitch` 直接 `=vision_los_final[NOW]`），距离增益 `k_yaw/k_pitch` 算了不用——一堆死中间变量留在主控制路径。按「少建变量、不留垃圾全局」清掉。
- **清理（[surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`）**：删 `lpf_yaw/lpf_pitch/lpf_inited` 及首拍初始化、`yaw_gain/pitch_gain/k_max/k_min/k_yaw/k_pitch` 全部死变量；YAW/PITCH 目标直接取 `vision_los_final[NOW]`（航位推算保留、无额外平滑、glide→扎逻辑不变）；删过时的「一阶低通原理」大段注释及各处注释掉的调用行（`Vision_Angle_Normalize`/`Low_Pass_Filter`/`raw_rate`）。
- **封存（[.c](imcalib/Task/surface_control_task.c) 240–416 整块 `#if 1`→`#if 0`）**：随之无人调用的 5 函数 `Vision_Angle_Normalize`/`Target_Slew`/`Pitch_Dive_Floor`/`Yaw_Gain_ByDist`/`Pitch_Gain_ByDist` 整块封存（保留可回退）；连带只被它们写的 4 个 Vofa 全局 `pitch_dive_floor`/`closeness_s`/`yaw_distance_gain`/`pitch_distance_gain` 定义注释、[.h](imcalib/Task/surface_control_task.h) 对应 extern 注释。`vision_los_rate` **保留**（PNG_Task.c 仍引用，删了会链接错）。
- **行为等价性**：识别成功(SUCCESS)主路径**完全不变**（清理前 `lpf_*` 在 SUCCESS 即恒等于 `vision_los_final[NOW]`）。**唯一细微变化**：丢目标(FAILURE)时 YAW 目标从「保持上一拍锁存视线」改为「就地保持当前姿态」——与 PITCH、与上方 FAILURE 分支、与文档「丢目标就地保持」一致（原 `static lpf_yaw` 的冻结副作用本就与 FAILURE 分支矛盾）。
- **未编译**(Keil/MDK)。纯死代码清理，不动控制律/增益/分配/状态机；要恢复增益调度把 `#if 0` 改回 `#if 1` 并接回调用即可。

### 控制分配散落全局整合为 `Alloc` 结构体（2026-06-27，未提交）
- **动机**：分配器相关 15 个散落全局（`Alloc_Mode`/`Alloc_Prio`/`Alloc_B` + `alloc_u0/u_out/alpha/u0_span/v_scale/p_scale/infeasible/singular_flag` + `servo_lat_scale`）散在文件顶部，按「少建变量、相关变量归 typedef」整合。
- **落地**：[surface_control_task.h](imcalib/Task/surface_control_task.h) 新增 `Alloc_t`（风格同 `Surface_t`），那批 extern 收成单条 `extern Alloc_t Alloc`；[.c](imcalib/Task/surface_control_task.c) 定义区合成一个 `Alloc_t Alloc = {…}`（C99 指定初始化，默认值不变：Mode=2、Prio={ROLL,YAW,PITCH}、B=理想阵、v/p/lat_scale=1），全部引用点 `Alloc_Mode→Alloc.Mode`、`alloc_u0→Alloc.u0`… 机械改名。
- **零牵连**：grep 确认仅 surface_control_task.c/.h 引用，PNG_Task/TotalControl/Vofa 打包都不碰；分配器内部静态函数名（`alloc_minnorm_solve`/`alloc_try` 等）不在改名范围、未受影响。**纯重命名重构，无行为变化**。
- **顺带修文档勘误**：CODE_OVERVIEW `Alloc.Prio` 默认以代码为准＝`{ROLL,YAW,PITCH}`（旧文档误记 `{YAW,ROLL,PITCH}`）。
- **未编译**(Keil/MDK)。

---

### 全量代码审计 + 大问题修复（2026-06-07）
对全部应用层源码 + Keil 工程环境做了一次完整通读，产出 [CODE_OVERVIEW.md](CODE_OVERVIEW.md) 项目地图。本轮**已落地修复**：
- **控制分配默认档 `Alloc_Mode` 0→1**：原默认走 Mode0 `Servo_Mix_PitchPriority`，其 k 公式反写（[.c](imcalib/Task/surface_control_task.c) 写成 `aL/(LIMIT−P)`、应为 `(LIMIT−P)/aL`）且无 `k≤1` 上钳；在 `p_body=y_body=0` 只过 roll 的现状下，把 roll 指令压成**二次方畸变**（输出∝`r·|r|/60`，小误差权限被严重削弱、大误差才突起作用）——正是一直在调的 roll 轴被它拖累。改默认到修好的 Mode1（`Servo_Mix_AxisLimit` 逐级优先级缩放，roll-only 下 k=1 线性正确），符合本文档"Mode1 作生产分配器"的本意。
- **同时修正 Mode0 公式**（用户选"双保险"）：`ki=(LIMIT−sgn·P)/aL` + 加 `k≤1`，旧对照函数也变正确。
- **`Guidance_Terminal` 视觉并发修复**：原 `taskENTER_CRITICAL` 取的快照 `v` 是**死变量**，后续仍直接读 live `Vision_Rx_Data`（与 UART 视觉中断 ~20Hz 写多字段撕裂读）；改为全程用 `v`，清 flag 仍打真 struct。
- **Vofa 遥测修正**：第 10-12 路原发 `output_angle_Servo[NOW][PITCH/ROLL/YAW]`（X翼 enum 下=舵 UL/UR/DR、被误标 P/R/Y），改发真正的 PID 内环输出 `output_gyro_Euler[NOW][P/R/Y]`，调参才看得到控制器输出。
- **[surface_control_task.h](imcalib/Task/surface_control_task.h) 舵机通道尾注释纠正**（`Servo_DR/DL_Channel` 原注释与宏名相反）。

**文档同步勘误**（代码现状 vs 旧文档）：① `GYR_KF_R/ACC_KF_R` 现均 **1000**（非旧记 30/500，有意，见 [输入端滤波拆分](#输入端滤波拆分2026-05-30)）；② Mahony `Kp=10`（非 2）；③ X翼左上舵 UL 因硬件接 **TIM3 CH2/PB5**（非旧计划 TIM4 CH1/PB6）；④ 舵机 enum 实为 `{UL,UR,DR,DL}`（DR 在 DL 前，与混控矩阵列序一致）；⑤ 计划 Phase4 三项里仅任务节拍 `vTaskDelayUntil` 已落地，**D 项低通 / FFC 启用仍未做**。

---

### 2026-07-15：修复视觉内录结束时过早断电

- **根因**：`Guidance_End()` 发送视觉停录命令并切到 `PROCESS_OK` 后，`get_current_State()` 原先下一拍立即执行 `Power_OFF`，绕过了 `Guidance_Process_OK()` 的保存等待，导致视觉端日志和视频文件可能均为 0KB。
- **修改**：移除 `get_current_State()` 的立即断电路径，保留 `Guidance_Process_OK()` 的延时断电。未编译、未上板验证；待确认视觉端能收到 `(1,5)` 停录反馈并正常生成文件内容。

## 当前 TODO

### 🔬 2026-06-07 审计新发现
- **分配器切档验证**：默认已切 Mode1，确认 Stable 下 roll 自稳线性、`servo_lat_scale` 未饱和≡1；Mode0 修公式后作对照，未饱和时应与 Mode1 一致。
- **任务优先级/同步**：`IMU_Task` 与 `Total_Control_Task` 同为 `osPriorityIdle`、各自 1kHz、无同步 → 控制可能用上一拍姿态。建议把 IMU 优先级抬高于控制、或合并、或用信号量/任务通知同步。
- ✅ **已解决**：TIM4 预分频改 **169**（与 TIM3 一致，时基统一，原 167 的 1.19% 差已消，用户改）；删除无用的 **`Servo_PWM_Limit`**（量纲统一为角度后失效，输出由上游 `abs_limit(±60)` 约束）；**视觉单位/轴映射经确认正确**（视觉发像素、`Vision_Receive` 接收时转度 `/160*72`、`/120*54`，下游按度用）。
- 🗑️ **已决定不再跟踪（2026-06-08）**：① **加速度标定**——用户确认现状正常，删除待办（仅经 A_World→速度→PNG、未接主环，不影响自稳）；② **IMU SPI 阻塞读超时**——不作处理；③ **FFC 前馈**——见下「🔜 下一步」决策。

### ✅ 已 Vofa 确认（2026-06-08）
- **坐标系统一 / 四元数+欧拉角重定向（2026-06-07）**：经 Vofa 确认**坐标系已统一**——Euler 三轴极性正确、单轴动作与 `current_gyro_Euler` 同号、长跑不漂。（若日后发现 yaw 控制方向反，仍按原结论翻 yaw 舵面 SIGN。）
- **内环角速度 pitch/roll 轴配对（方案B，2026-06-07）**：**roll 自稳已正常**（git「roll 调稳了很多」）——方案B 源头+四元数同步对调生效，roll 能阻尼自旋、不再渐进发散，yaw 不变、PNG 行为不变。pitch/yaw 通道已放开（清零行已注释），pitch 解算几何已对齐（见时间线 2026-06-08 条）。

### ⏳ 待 Vofa 台架验证
- **🆕 末制导 pitch 主动滑翔→扎（2026-06-27）**：`pitch_glide_mode=1`。Vofa/Watch 拉 `pitch_glide_blend`（应随接近 0→1）、`pitch_glide_target`、`vision_los_final[NOW][PITCH]`(视线俯角 φ)。验：① 远端（φ 浅）pitch 目标住 `THETA_GLIDE_DEG`(+2°) 压平、滑翔增程不过早扎；② 近端（φ≤−25°）平滑过渡到追视觉目标扎下、与入射角 −27° 衔接、无台阶；③ `THETA_GLIDE_DEG` 过大是否失速反掉得更快（保守起步往上调）；④ 切 `pitch_glide_mode=0` 退回旧 `pitch_control_limit_deg` 门限对照。
- **🆕 二维直通 vs 三维耦合变换（2026-06-23 回退）**：当前直通（变换 `#if`/注释关）。台架判据——固定 pitch/yaw 目标、手动滚转 ±20°，看是否出现**意外 yaw/pitch 漂移**：若明显→说明需按本工程陀螺轴序（gx=pitch/gy=roll/gz=yaw）重推 T(φ,θ) 并启用；若可忽略→直通够用，保持。
- **🆕 调参哲学转向「内环>外环纯 P」（2026-06-24~25）**：确认拍平后的纯 P 串级 roll/yaw/pitch 自稳质量（无超调、无低频摇摆、无高频抖）；roll 内环 `÷2` 是否仍需要（去掉看是否更抖）。**增益以 [pid.c](imcalib/Tool/pid.c) `pid_init` 为准，文档不追数值。**
- **LADRC roll 自稳（2026-06-21+，当前 `ladrc_mode=0` 全 PID，需切 3 验证）**：`ladrc_mode=3` 仅 roll 用 LADRC、pitch/yaw 仍 PID。Vofa 拉 `ladrc_ctrl[LADRC_ROLL].z1`(角度估计)/`z2`(角速度估计)/`z3`(总扰动)/`u`(输出)；roll 自稳应与 PID 同级或更好（无超调、无高频抖）。调参：先定 wc(10.5)→wo=5×wc(52.5)→调 b0(55)。`ladrc_mode=1` 三轴全 LADRC 待 pitch/yaw 参数标定后再试。
- **vision_ins EKF 速度验证（2026-06-15+）**：① 静置 `vins_out.v_world` 应≈0（ZUPT 钉死）；② 手持移动后停→速度应快速回零；③ 俯冲入段 `Vel_Reanchor_Flag` 锚定后 `gamma_pitch_fwd_deg` 应≈机体俯仰且不漂；④ `vins_out.range_m` 应随距离减小单调递减。调参：`VINS_SIGMA_ACC`(1.0) 控制 IMU/视觉信任比，漂得快就调大。
- **ZUPT + 零偏在线对准（2026-06-15+）**：上电静置→`imu_is_static` 应在 ~100ms 内变 1、`A_Offset` 慢速收敛；拿起移动→`imu_is_static` 应变 0、零偏冻结。发射后不应误判静止。
- **视线角半径归一化 + 距离增益（2026-06-15+）**：① `Vision_Rx_Data.Euler_norm` 应随 blob 半径变化而归一化（远处小 blob→放大、近处大 blob→缩小）；② `yaw_distance_gain` 远处>近处、`pitch_distance_gain` 近处>远处；③ 远近切换应平滑无台阶。
- **LPF 视觉目标平滑（2026-06-15+，⚠️ 当前主路径被旁路）**：设计是 `target = k×vision_los_final + (1−k)×target_last`（k 由距离增益缩放：远小 k 保守、近大 k 跟手），丢目标冻结。**但 2026-06-27 核对：`Low_Pass_Filter` 调用已注释、`lpf_yaw/pitch` 直接 `=vision_los_final[NOW]`、k 算了未用**（调试期临时态）。需要平滑时把注释切回再验：`Surface.target_angle_Euler[NOW]` 平滑跟踪 `vision_los_final`、无阶跃。
- **末制导比例化逼近 + PN 视线率超前补偿（2026-06-14）**：① 比例化逼近——Vofa 拉 `vision_los_final[NOW]`（终点，仅新帧阶跃）与 `Surface.target_angle_Euler[NOW]`（目标），目标应＝当前姿态+LOS误差/N、误差大快速逼近不过冲、误差小平滑收敛不抖；调 N（YAW 15 / PITCH 25，写在 .c 调用点）看跟手/平滑权衡。② PN——`vision_los_rate[YAW/PITCH]` 随视线转动有值、丢帧/丢目标归零不爆冲、被 `LOS_RATE_LIMIT_DPS`(40) 限住；`PN_LEAD_K`(0.5) 先小增益验超前方向对（命中应提前于纯追尾）再加大，`AOA_TRIM_DEG` 默认 0 暂不引入。③ 回归——丢目标(FAILURE)/pitch 未俯冲到位(≥-5°) 仍就地保持（目标=当前、不打舵）。**注：PN 超前当前 `#if 0` 关闭，先用纯 PID 跟踪验证基础性能。**
- **末制导俯仰能量管理 + 速度预测（2026-06-12）**：① γ 对不对——静置 `gamma_pitch_deg`≈0;手持鼻先下压 γ 应变负且≈机体俯仰;纯横滚不应大改 γ。② 限幅起作用——`closeness_s` 随距离变小(远段)/面积变大(近段)从 0→1,`pitch_dive_floor` 从≈-8 平滑放开到≈-27;给一个很低的视觉 LOS,俯仰目标应被钳在 θ_floor 不下探。③ 正向撞击——末段 `gamma_pitch_deg` 与 `current_angle_Euler[PITCH]` 收敛(迎角→0)。④ 协议——`Vision_Rx_Data.dist_cm/area` 在 0x5B 包到达时更新且随远近变化;0x5A/0x7A/0x9A 仍正常、不串包。⑤ 标定宏 `DIST_ACQUIRE_CM/DIST_NEAR_CM/AREA_NEAR/AREA_IMPACT/V_NOM_MS` 打靶实测;远/近段在切换点 s=`DIVE_SCHED_SWITCH` 应平滑衔接(若有台阶调 `AREA_NEAR`↔`DIST_NEAR_CM` 对应)。⑥ 回归:roll/yaw 自稳与既有视觉视线锁定不变。
- **D 项对测量微分 / ~~视觉目标斜坡~~（2026-06-10）**：① 末制导段重新给 D 一个小阻尼值，应**不再**出现 20Hz 抖动（原微分冲击已消）；纯陀螺自稳段 D 阻尼行为正常。② ~~视觉目标斜坡~~ **已被 2026-06-14 比例化逼近替代**（`VISION_TARGET_SLEW_DPS` 废弃），验证改看上面 06-14 条。③ 丢目标(FAILURE) 目标=当前、误差≈0 不打舵。
- **角度环绕（2026-06-07）**：roll/yaw 目标设在接近 ±180° 处、令当前姿态跨边界，输出不应出现 ~360° 假误差导致的反向猛打；±0 附近常规自稳行为不变（wrap 不触发）。
- **视线锁存（方向A）**：末制导对静止/缓动目标，镖体应单调转向并稳定指向、不再同线首振；Vofa 拉 `Surface.target_angle_Euler[NOW][YAW]` 应为帧间水平台阶（保持不变）、仅识别成功新帧到达瞬间阶跃更新；丢目标(FAILURE) 每 tick 回中、重新捕获到 SUCCESS 帧立即重锁。
- ✅ **视觉单位核验（方向C，已确认）**：视觉发的是**像素**，已在 `Vision_Receive` 接收时转成度（`Euler[YAW]=y/160*72`、`Euler[PITCH]=x/120*54`），`Guidance_Terminal` 锁存即为度、量纲一致，无需再 ×FOV。
- **控制分配档对照（当前默认 `Alloc_Mode=2` 最小能量；Mode0 调用点已注释，仅 1/2 可选）**：Vofa 切 `Alloc_Mode` 1/2 输出都不超 ±`SERVO_ANGLE_LIMIT`(60)、roll 接 PID 输出无突跳；**Mode1**（`Servo_Mix_AxisLimit` 逐级优先级缩放，`Alloc_Prio` 默认 **{YAW,ROLL,PITCH}**，调试器在线改）：高优先轴力矩保住、低优先轴逐级让步，填非法排列自动退默认不崩，`servo_lat_scale` 未饱和≡1/饱和<1；**Mode2**（默认）零空间自检 `n=[4,4,-4,-4]`、`alloc_singular_flag≡0`、可达区能量 Σu² ≤ Mode1、强制大 v 看 `alloc_infeasible`/pitch 保住；长跑无 NaN。台架辨识 `Alloc_B` 后把调用点经验系数设 1。
- **roll→世界 pitch/yaw 反旋 SIGN**（pitch/yaw 已放开，可验）：`Roll_World_Comp_Flag=0` 行为同旧版（直通对照）；置 1 且 Δ≈0 时恒等（`roll_world_delta`≈0、pb≈Pw、yb≈Yw）；横滚 +90° 给纯世界 pitch（Pw>0,Yw=0）应得 pb≈0、yb≈±Pw 且机头朝**正确**世界方向修正，反了则把 `ROLL_WORLD_COMP_SIGN` 翻成 −1 重编。

### 🔜 下一步（待 Vofa 验证后）
- **LADRC pitch/yaw 标定**：roll 验证通过后，逐步启用 pitch(`ladrc_mode` 扩展) / yaw LADRC，替代串级 PID。
- **速度矢量追踪启用**：`vel_pursuit_mode=1` 三级串级（速度方向→角度→角速度），待 EKF 速度验证准后试。
- **PN 超前补偿启用**：当前 `#if 0`（`Guidance_Terminal` 内）。拆分接口 `PNG_Apply_Lead_Yaw/Pitch` 已就绪（`PNG_Mode=1` 用 vision_ins EKF 世界系 p/v 叉乘直接算视线率，分轴门控：俯冲未到位只喂 yaw）；待基础视觉跟踪验证通过、标定好 `K_Dyn` 后打开。
- ✅ **串级内外环增益拉开（2026-06-24~25 已落地，待验收）**：调参哲学已转向「内环>外环、I/D≈0 纯 P 串级」（Gyro 0.8~1.1 > Angle 0.25~0.45）。验收并入上面「调参哲学转向」验证项；如仍有低频摇摆/高频抖再继续重标。
- **输出加速率限制**（输出端 rate limit，非环内低通）。注：setpoint 端已用 LPF 平滑视觉目标；此处指对最终舵量/力矩输出再加 rate limit。
- 🚫 **前馈 FFC 暂不做**（2026-06-08 决定）：实际效果不大、且不好验证是否有效，先不用前馈。代码已是关闭态（[pid.c](imcalib/Tool/pid.c) `Euler_pid_Cale` 里 `+=` 调用已注释、`num1/num2=0`），结构保留备用，需要时再启用。

### 💤 后续（暂缓，有需求再做）
- **卡尔曼状态观测器**：带角速度状态的状态空间卡尔曼，预测姿态，替代当前一阶标量 `KalmanFilter`。
- Pitch 分配可选扩展：roll 优先于 yaw（当前 roll/yaw 同比缩）、抗积分饱和（k<1 时回算冻结/泄放横侧积分防 windup）。

---

## 详细方案文档

| 文档 | 内容 | 状态 |
|---|---|---|
| [CODE_OVERVIEW.md](CODE_OVERVIEW.md) | **全项目代码地图**：环境/构建、RTOS 任务、数据流水线、坐标系约定、制导状态机、串级PID、控制分配三档、关键全局、已知陷阱。答题/接手前速查 | ✅ 持续维护 |
| [AGENTS.md](AGENTS.md) | **跨 AI 工作规则/约束**：沟通方式、改动哲学、控制方案偏好、文档/验证纪律、红线、跨项目适配。任何 AI（含本项目外）接手前先读 | ✅ 2026-06-27 新建 |
| ~~plan-flight-control-overhaul.md~~ | 飞控三模块优化总方案（4 Phase）——**已删除**，内容并入「进度时间线」与 git 历史 | 🗑️ 已删 |
| ~~plan-pitch-priority-mixing.md~~ | X 翼三轴解耦 + Pitch 优先控制分配——**已删除**，见时间线「控制分配重写」「交付A 升级」条 | 🗑️ 已删 |

---

## 与 Claude memory / AGENTS.md 的同步

本文件是项目内（git 可同步）的进度镜像。三处保持一致：

- **本文件**「进度时间线」+「当前 TODO」← → Claude memory 的项目/进度类条目。
- **[AGENTS.md](AGENTS.md)**「§4 控制方案偏好」← → 本文件「控制方案偏好」← → Claude memory 的偏好/反馈类条目（[[user-profile]] 等）。

更新任一侧时同步其它侧。memory 实际条目以 memory 目录为准（高频调参期，数值一律以代码为准）。
### 2026-07-14：LQR 固定速度 + 积分分离

保留现有 `K_d[4][6]`，`lqr_use_scheduled_K=0` 时固定设计速度生成 K；新增 `LQR_t.integral` 统一管理积分状态、增益、限幅、积分分离和舵面饱和回退。积分分离开关为 1，大姿态误差冻结积分，输出饱和回退本拍积分。未编译，待 Keil/台架验证。

### 2026-07-14：BMI088 加速度计支持（BMX055 可切换）

- **动机**：硬件换用 BMI088（6 轴 IMU，acc+gyro），需与原 BMX055 代码并存、运行时宏切换。
- **BMX055 vs BMI088 关键差异**：
  - Acc 寄存器地址不同（BMX055: 0x02 起始 / BMI088: 0x12 起始）
  - BMI088 acc 有电源状态机（0x7D=0x04 开、0x7C=0x00 active），BMX055 无
  - BMI088 acc 数据 16-bit（敏感度≈0.00718 g/LSB @±24g），BMX055 12-bit 左对齐
  - **BMI088 acc 读取必须含 dummy byte**：发完地址后多发1字节dummy，数据才正确。单字节读和多字节读均需 dummy。burst 读 = 8字节（1地址+1dummy+6数据），数据从 rx[2] 开始
  - Gyro 两芯片完全共用（寄存器、协议、±2000°/s），gyro 读不需要 dummy
- **落地**：
  - `common_defs.h`：`USE_BMX055`/`USE_BMI088` 选型宏（当前 `USE_BMI088=1`）
  - `surface_control_task.h`：`ACC_LSB`/`ACC_SAT_G` 按芯片自动切换（方便调参）
  - `IMU.c`：新增 `BMI088_Init_Acc()`（软复位→使能→active→配 ODR/量程，对照官方例程 `bmi088_lib/`）+ `BMI088_Read_Acc()`（burst 读 8 字节，数据从 rx[2] 开始）
  - `IMU_Init()` / `IMU_Data_Read()` 用 `#if USE_BMI088` 分流
- **踩坑记录**：① 寄存器 0x7C/0x7D 写反（active/suspend 搞混）；② 误以为多字节读不需要 dummy byte（实际需要，7字节版数据错位1字节，角度正常是因为 acc_trust=0 时姿态纯靠陀螺）；③ 数据格式误判为 12-bit 右对齐（实际 16-bit）；④ init 缺软复位+chip ID 验证（例程要求写完读回验证）。**核心教训：必须对照官方例程的寄存器地址和 SPI 时序，不能凭经验猜。**
- **已验证**：BMI088 acc 读取正常（chip ID=0x1E，静止 acc 值≈±1g）。未编译（Keil/MDK），待台架完整验证。
- **切换方法**：改 `common_defs.h` 的 `USE_BMX055=1/USE_BMI088=0` 即可切回 BMX055。
