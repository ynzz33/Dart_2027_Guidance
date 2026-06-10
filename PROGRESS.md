# Dart_2027 飞控 — 开发进度与 TODO 总览

> 项目进度的单一入口。详细技术方案见文末「[详细方案文档](#详细方案文档)」。
> 本文件与 Claude 的 memory（`control-tuning-progress` / `control-approach-preferences`）**双向同步**，改进度时两边保持一致。
> 📖 代码速查地图见 [CODE_OVERVIEW.md](CODE_OVERVIEW.md)（答题/接手前先读，含环境/任务/数据流/坐标系/状态机/分配器/全局/陷阱）。
> 最后更新：2026-06-10（制导段抖动修复：D 项改对测量微分消微分冲击 kick + 视觉目标斜坡平滑视觉 50ms 阶跃。前次：X 翼 pitch 解算几何对齐）

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
- [IMU.h](imcalib/Task/IMU.h)：陀螺/加速度共用的卡尔曼 R 拆成两路——`GYR_KF_R` 与 `ACC_KF_R` 独立可调。
- [IMU.c](imcalib/Task/IMU.c)：acc/gyr 两处卡尔曼调用改用各自新宏。
- Mahony 参数复核保持 PI（Kd=0）。
- ⚠️ **现状勘误（2026-06-07 核对代码）**：`GYR_KF_R=ACC_KF_R=1000`（非本条最初记的 30/500）——用户实测 R=30 欠滤波（"滤不动"），关键在 Q:R 比例，**有意保持 1000**；Mahony `Kp=10`（非 2）。以代码为准。

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
- **D 项对测量微分 / 视觉目标斜坡（2026-06-10）**：① 末制导段重新给 D 一个小阻尼值，应**不再**出现 20Hz 抖动（原微分冲击已消）；纯陀螺自稳段 D 阻尼行为正常。② Vofa 拉 `vision_los_final[YAW]` 与 `Surface.target_angle_Euler[NOW][YAW]`：终点应为帧间台阶(仅新帧阶跃)，目标应**平滑斜坡**跟随终点、无 50ms 台阶；视线角速度大时若目标跟不上则调大 `VISION_TARGET_SLEW_DPS`。③ 丢目标(FAILURE) 目标=当前、误差≈0 不打舵。
- **角度环绕（2026-06-07）**：roll/yaw 目标设在接近 ±180° 处、令当前姿态跨边界，输出不应出现 ~360° 假误差导致的反向猛打；±0 附近常规自稳行为不变（wrap 不触发）。
- **视线锁存（方向A）**：末制导对静止/缓动目标，镖体应单调转向并稳定指向、不再同线首振；Vofa 拉 `Surface.target_angle_Euler[NOW][YAW]` 应为帧间水平台阶（保持不变）、仅识别成功新帧到达瞬间阶跃更新；丢目标(FAILURE) 每 tick 回中、重新捕获到 SUCCESS 帧立即重锁。
- ✅ **视觉单位核验（方向C，已确认）**：视觉发的是**像素**，已在 `Vision_Receive` 接收时转成度（`Euler[YAW]=y/160*72`、`Euler[PITCH]=x/120*54`），`Guidance_Terminal` 锁存即为度、量纲一致，无需再 ×FOV。
- **控制分配三档对照（含 Pitch 优先）**：Vofa 切 `Alloc_Mode` 0/1/2 输出都不超 ±`SERVO_ANGLE_LIMIT`(60)、roll 现接 PID 输出无突跳；**Mode1 优先级缩放（Pitch 优先即在此：`Alloc_Prio` 默认 {PITCH,YAW,ROLL}，不另设单独验证项）**：调试器改 `Alloc_Prio` 排列，高优先轴力矩应保住、低优先轴逐级让步，填非法排列(如{0,0,1})自动退默认不崩，`servo_lat_scale` 未饱和≡1/饱和<1；Mode2 零空间自检 `n=[4,4,-4,-4]`、`alloc_singular_flag≡0`、可达区能量 Σu² ≤ Mode1、强制大 v 看 `alloc_infeasible`/pitch 保住；长跑无 NaN。台架辨识 `Alloc_B` 后把调用点经验系数 `0.85/1.05` 设 1。
- **roll→世界 pitch/yaw 反旋 SIGN**（pitch/yaw 已放开，可验）：`Roll_World_Comp_Flag=0` 行为同旧版（直通对照）；置 1 且 Δ≈0 时恒等（`roll_world_delta`≈0、pb≈Pw、yb≈Yw）；横滚 +90° 给纯世界 pitch（Pw>0,Yw=0）应得 pb≈0、yb≈±Pw 且机头朝**正确**世界方向修正，反了则把 `ROLL_WORLD_COMP_SIGN` 翻成 −1 重编。

### 🔜 下一步（待 Vofa 验证后）
- **串级内外环增益拉开**：目前外环 P=0.7 > 内环 P=0.2，与「内环>外环」偏好相反，需重标。
- **输出加速率限制**（输出端 rate limit，非环内低通）。注：**setpoint 端**速率限制已于 2026-06-10 对视觉目标做了（方案3 视觉目标斜坡）；此处指对最终舵量/力矩输出再加 rate limit，位置不同、可独立。
- 🚫 **前馈 FFC 暂不做**（2026-06-08 决定）：实际效果不大、且不好验证是否有效，先不用前馈。代码已是关闭态（[pid.c](imcalib/Tool/pid.c) `Euler_pid_Cale` 里 `+=` 调用已注释、`num1/num2=0`），结构保留备用，需要时再启用。

### 💤 后续（暂缓，有需求再做）
- **卡尔曼状态观测器**：带角速度状态的状态空间卡尔曼，预测姿态，替代当前一阶标量 `KalmanFilter`。
- Pitch 分配可选扩展：roll 优先于 yaw（当前 roll/yaw 同比缩）、抗积分饱和（k<1 时回算冻结/泄放横侧积分防 windup）。

---

## 详细方案文档

| 文档 | 内容 | 状态 |
|---|---|---|
| [CODE_OVERVIEW.md](CODE_OVERVIEW.md) | **全项目代码地图**：环境/构建、RTOS 任务、数据流水线、坐标系约定、制导状态机、串级PID、控制分配三档、关键全局、已知陷阱。答题/接手前速查 | ✅ 2026-06-07 新建 |
| [plan-flight-control-overhaul.md](plan-flight-control-overhaul.md) | 飞控三模块优化总方案（IMU 解算 / 控制算法 / X 翼输出 / 精修，4 Phase，逐条改前→改后 + 验证流程） | 多数已落地 |
| [plan-pitch-priority-mixing.md](plan-pitch-priority-mixing.md) | X 翼三轴解耦 + Pitch 优先最小能量控制分配（含数学核验、落地代码、验证清单） | ✅ 已实现，待台架验证 |

---

## 与 Claude memory 的同步

本文件是项目内（git 可同步）的进度镜像，对应 Claude 的两条 memory：

- `control-tuning-progress`（project）← → 本文件「进度时间线」+「当前 TODO」
- `control-approach-preferences`（feedback）← → 本文件「控制方案偏好」

更新任一侧时同步另一侧，保持一致。
