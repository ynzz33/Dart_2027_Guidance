# Dart_2027 飞控 — 代码总览 / 项目地图

> 给 Claude（及任何接手者）**答题前先读这一篇**就能掌握整个项目的速查地图。
> 进度与决策见 [PROGRESS.md](PROGRESS.md)；跨 AI 工作规则/约束见 [AGENTS.md](AGENTS.md)。（旧 `plan-flight-control-overhaul.md` / `plan-pitch-priority-mixing.md` 已删除，内容并入 PROGRESS 时间线 + git 历史。）
> 本文基于 **2026-06-27 复审** 整理，力求与代码一致；改代码后请同步本文。
>
> ⚠️ **高频调参期**：PID 增益/分配档/门控几乎每个 commit 都在变。本文中所有具体数值（增益、阈值、默认档位）均为**某时刻快照、可能已过时**——一律**以代码为准**（pid.c `pid_init`、surface_control_task.c/.h）。本文价值在**结构、数据流、坐标系、陷阱**这些不常变的骨架。

---

## 0. 一句话定义

STM32G431 + BMX055/BMI088(可切换) + FreeRTOS 的 **Dart 飞镖型飞行器飞控**：X 翼 4 舵面，串级 PID（角度环→角速度环）+ 可选 LADRC 单环二阶自抗扰（`ladrc_mode` 切档）自稳 + 末制导，Mahony 姿态融合，视觉/IMU 紧耦合 6 态 EKF（vision_ins.c）给不漂的速度，视觉（OpenMV）经 UART 给视线角+距离+面积。比赛镖（RoboMaster 飞镖）场景。IMU 芯片通过 `common_defs.h` 的 `USE_BMX055`/`USE_BMI088` 宏切换（acc 初始化/读取/敏感度自动适配，gyr 共用）。

---

## 1. 构建环境（Keil MDK-ARM）

| 项 | 值 | 备注 |
|---|---|---|
| MCU | STM32G431（Cortex-M4F） | 单精度硬件 FPU |
| 主频 | **170 MHz** | HSE 12MHz → PLL(/3 ×85 /2)，FLASH latency 4，VOS1 Boost |
| 总线 | AHB=APB1=APB2=170MHz | 定时器核时钟 170MHz |
| FPU | **硬件单精度已启用**（工程 `FPU2 / RvdsVP=2`） | FreeRTOS 用 **ARM_CM4F 端口**（保存 FPU 上下文，多任务用 float 安全）。`configENABLE_FPU 0` 在 CM4F 端口下**被忽略，无害**（只 CM33/CM23 端口读它） |
| 优化 | **-O1**（`Optim=2`），C99，警告开（非 error） | 1kHz 控制环在 170MHz 上余量充足 |
| 预处理宏 | `USE_HAL_DRIVER, STM32G431xx` | **未定义 `ARM_MATH_CM4`**；但 CMSIS-DSP 用的是**预编译库**，无影响 |
| CMSIS-DSP | `arm_cortexM4lf_math.lib`（CM4+小端+FPU）**已链接** | `arm_mat_inverse_f32` 等矩阵函数可用（Mode2/卡尔曼用，目前未激活路径） |
| RTOS | FreeRTOS V10.3.1 + CMSIS-OS v1 | tick 1000Hz，heap_4 = **15360 B**，`configMAX_PRIORITIES=7`，抢占式 |
| 已知 eIDE 坑 | 见 memory `eide-isnullorundefined-crash` | 与代码无关 |

---

## 2. 目录与文件职责

```
imcalib/                ← 用户应用代码（核心都在这）
├── Task/
│   ├── IMU.c/.h               BMX055/BMI088 SPI 读取 + Mahony 姿态解算（最核心，USE_BMI088 切芯片；BMI088 acc 需 dummy byte，8字节burst读）
│   ├── surface_control_task.c/.h  制导状态机 + 串级PID调度 + 混控/控制分配 + 舵机PWM（最大）
│   ├── TotalControl.c/.h      主控制编排：Vofa()+surface_control_task()+遥测；IMU_Task()=读+解算
│   ├── CallBack_Task.c/.h     所有中断回调（UART视觉/镖头、定时器、EXTI按键）+ 视觉协议解析 + 遥测发送
│   ├── PNG_Task.c/.h          比例导引（Proportional Navigation）—— **当前未接主控制环**
│   ├── Init_Config.c/.h       TotalInitTask() 上电初始化序列
│   ├── Button.c/.h            按键消抖+长短按状态机（TIM15 ISR 驱动）
│   └── buzzer.c/.h            无源蜂鸣器 PWM 音乐（消息式，播放在 SelfTest 任务）
├── Tool/
│   ├── pid.c/.h               PID（位置/增量）+ 死区软化 + 角度环绕 + 前馈FFC（FFC 当前关）+ 速度方向外环
│   ├── adrc.c/.h              LADRC 线性自抗扰控制器（单环二阶 LESO+LSEF，文件名不变内部全换）
│   ├── lqr.c/.h               LQR 姿态控制器（6态→4舵一步解算 u=-K_d·x，含混控；lqr_mode 切档，未编译/待台架）
│   ├── lqi_torque.c/.h         LQI 力矩控制器（9态→3轴力矩 N·m；lqi_mode 切档，未编译/待台架）★ NEW
│   ├── torque_allocator.c/.h   Pitch 保护型零空间舵面分配器（力矩→4舵，未编译/待台架）★ NEW
│   ├── filter.c/.h            标量卡尔曼(激活) + 2D卡尔曼(速度EKF已#if0禁用) + 低通 + CMSIS矩阵宏别名
│   ├── vision_ins.c/.h        视觉/IMU 紧耦合 6 态 EKF(世界系 p,v)，给不漂的速度+距离估计
│   ├── ADC_Battery.c/.h       电池电压 ADC（DMA + 标量卡尔曼）
│   └── Vofa_send.c/.h         Vofa+ 上位机 2/4/8/16/24/32 通道 float 发送
└── User/
    ├── common_defs.h          公共宏：dT/M_PI/RAD2DEG/单位换算/传感器量程
    ├── mytype.h               u8/s16/fp32 等类型别名
    └── user_lib.c/.h          PWM_Init() + Self_Text_Task()/Process()

Core/                   ← CubeMX 生成的 HAL 层
├── Src/  main.c(时钟+外设初始化序), app_freertos.c(任务创建/任务体), stm32g4xx_it.c(中断向量),
│         tim/adc/spi/usart/dma/gpio.c(外设配置), system_*, hal_msp, timebase_tim
└── Inc/  FreeRTOSConfig.h, main.h, *_hal_conf.h 等
MDK-ARM/                ← Keil 工程(.uvprojx)、编译产物
Drivers/ Middlewares/   ← HAL 库、CMSIS、FreeRTOS、DSP 库
python_vision_script/   ← OpenMV 视觉脚本（不编进 Keil）：识别主脚本 Guidance_vision_scrpit.py + PC 端视频测试 video_test.py（识别管线 1:1 复刻）+ convert_mjpeg.py + 使用/识别两篇 md
```

> `.cmsis/` 目录是厂商设备模板库（ARMCMx 各核），**与本项目无关**，忽略。

---

## 3. RTOS 任务结构（`Core/Src/app_freertos.c`）

| 任务 | 优先级 | 栈 | 周期 | 职责 |
|---|---|---|---|---|
| Init_Task | **Normal**（最高） | 512 | 一次性 | 跑 `TotalInitTask()` 后 `osThreadTerminate` 自删 |
| SelfTest_Task | Idle | 512 | 100ms | 自检 `Self_Text_Task()`（仅 Self_Text_State）+ `Buzzer_play_task()` |
| IMU_Task | Idle | 512 | 1ms (`vTaskDelayUntil`) | 先 `IMU_Calibrate()`（2s 静态零偏）一次，再循环 `IMU_Task()`=读+解算 |
| Total_Control_Task | Idle | 512 | 1ms (`vTaskDelayUntil`) | `TotalControl()`=Vofa+`surface_control_task()`+视觉遥测+ADC |

> ⚠️ **IMU 与 Control 同为 osPriorityIdle、同优先级、各自 1kHz、无同步**：执行先后不确定，控制环可能用**上一拍**姿态（≤1ms 滞后）。初始化顺序靠 Init_Task(Normal) 先跑完再让 Idle 任务跑（正确）。改进项见 PROGRESS TODO。

### 初始化序列 `TotalInitTask()`（`Init_Config.c`）
`ALL_CS_Free` → 启 TIM6/7 中断 → `PWM_Init()`(启 TIM2/3/4 PWM) → `pid_init()` → `Q[NOW][0]=1` → 三路 UART 空闲DMA接收(huart1半双工/huart2调试/huart3视觉，关半传输中断) → `ADC_Init()` → `IMU_Init()`(BMX055或BMI088,按 `USE_BMI088` 宏切换) → `PNG_Init()` → `Kalman_Vel_Init()` → 上电 → 目标欧拉角初值 30/30/30（随即被状态机覆盖）。

---

## 4. 数据流水线（每 1ms）

```
BMX055/BMI088(SPI2阻塞读) ─IMU_Data_Read─► IMU_Data.A/G(原始,去饱和/NaN,标量Kalman,减陀螺零偏)
                                         │
                    IMU_Attitude_Algorithm│(Mahony PI 融合 → 四元数积分 → 欧拉角)
                                         │
                    ZUPT(发射前零速+零偏在线refine) + A_World(去重力,世界系加速度)
                                         │
                    VisInsEKF_Predict(1kHz) + UpdateVision(~30Hz) + UpdateZeroVel(静止)
                                         ▼
              Surface.current_angle_Euler[NOW]  +  current_gyro_Euler[NOW]
              IMU_Data.Velocity[World/Body]  +  vins_out.range_m/vc
                                         │
   get_current_State()→Guidance_State   │   get_current_Target()→target_angle_Euler[NOW]
                                         ▼
         Euler_pid_Cale() 或 Euler_LADRC_Cale()(ladrc_mode切档)
         外环角度→temp[]→内环角速度→ output_gyro_Euler[NOW]
                                         │  (三轴全放开,pitch/yaw经Roll_Derotate反旋)
                                         ▼
        Servo_Mix_*(Alloc.Mode 分派) → output_angle_Servo[NOW][0..3] (度,含SIGN,±60)
                                         ▼
        Wing_Control_VECTOR_NOZZLE → Wing_UL/UR/DL/DR_Control → Finally_Angle(µs) → __HAL_TIM_SET_COMPARE
```

---

## 5. 坐标系与轴向约定（**极易混淆，答题必看**）

- **机体系 = 右手 ENU**：X=右/东、Y=前/北、Z=上/天。
- **欧拉角极性**：PITCH 绕 X 抬头为正；ROLL 绕 Y 右滚为正；YAW 绕 Z 右偏为正。
  - `Euler[PITCH]=asin(...)`∈[-90,90]（不环绕）；`Euler[ROLL]/[YAW]=atan2(...)`∈[-180,180]（**周期量，外环 `angle_wrap` 当前=0 未启用环绕，函数保留**）。
- **陀螺喂四元数**：`gx=G_Rad[PITCH]`(chipY=右)、`gy=G_Rad[ROLL]`(chipX=前/纵轴)、`gz=G_Rad[YAW]`(chipZ=上)。
- **内环角速度反馈**（`current_gyro_Euler`）：pitch=+gx、roll=+gy、**yaw=−gz**（右手系 +gz=左偏，故 yaw 右+取负）。
- **加速度寄存器映射**：`A[X](右)=accY寄存器`、`A[Y](前)=accX寄存器`、`A[Z](上)=accZ寄存器`；静止 `A[Z]≈+g`。
- **可配符号宏**（`IMU.h`）：`ACC_SIGN_X/Y/Z`（默认 +/+/−）、`GYR_SIGN_X/Y/Z`（默认 +/+/−）。某轴方向反了只翻对应宏，不动公式。
- **enum 顺序**：`{PITCH=0, ROLL=1, YAW=2}`；`{X=0,Y=1,Z=2}`；`{NOW=0,LAST=1,LLAST=2}`；舵机 `{UP_LEFT=0, UP_RIGHT=1, DOWN_RIGHT=2, DOWN_LEFT=3}`（**注意 DR 在 DL 前**，与混控矩阵列序一致）。
- 磁力计（`M[]`）读取代码在，但 **MAG 不启用**（场景磁干扰大）。

---

## 6. 制导状态机（`get_current_State` / `get_current_Target`）

`enum {Self_Text_State=0, Start, Stable, Terminal, End, PROCESS_OK}`

```
Self_Text_OK ─► Start ──(A_Normed[Y]≥0.8 或 A[Y]≤−1，连续5)──► Stable(请求视觉内录)
                                                                  │ Euler[PITCH]≤0，连续5
                                                                  ▼
End ◄──(A_Normed[Y]≥0.9 且 A[Y]≥1.5，连续5；冲击检测)── Terminal(视觉制导)
 │ cnt>200
 ▼ Vision 停录 → PROCESS_OK
```
- **舵机仅在** `Guidance_State==Terminal || ==Self_Text_State || Stable_Flag==1` **时驱动**，否则回中(0)。
- `End` 阶段发送视觉停录命令后进入 `PROCESS_OK`；断电必须走 `Guidance_Process_OK()` 的延时路径，不能在状态切换后的下一拍立即 `Power_OFF`，否则视觉端来不及关闭文件，SD 卡上的日志/视频可能保持 0KB。
- `Stable_Flag` 在 Stable 且 `Euler[PITCH]≤30` 时置 1。
- `Self_Text_State` 下四舵置 30°（自检摆舵）；`Start` 下置 0。
- 各阶段目标：Start/Stable → ROLL/YAW 锁 `Stable_Euler_Angle`、PITCH=当前（只阻尼）；Terminal → ROLL 自稳，YAW/PITCH 视觉视线锁存（见 §9）。

---

## 7. 串级 PID（`pid.c` / `pid_init`）

每轴两环：`surface_control_pid[Angle][axis]`（外环角度）→ `surface_control_pid[Gyro][axis]`（内环角速度）。当前激活的是「镖体1」一组（镖体2/3 注释着）。

**调参哲学（2026-06-24~25 转向，重要）**：增益整体拍平为**「内环>外环、I/D≈0 的纯 P 串级」**——内环 P 反超外环（呼应控制偏好「纯 P 串级=单环 PD+陀螺阻尼」）。下表为 **2026-06-27 快照，⚠️ 频繁变动，以 [pid.c](imcalib/Tool/pid.c) `pid_init` 为准**：

| 轴 | 外环(Angle) P/I/D | 内环(Gyro) P/I/D | 外死区 | 内死区 |
|---|---|---|---|---|
| PITCH | 0.25 / 0 / 0 | 0.80 / 0 / 0 | 1.0° | 0° |
| ROLL | 0.45 / 0 / 0 | 0.90 / 0 / 0 | 1.0° | 0° |
| YAW | 0.45 / 0 / 0 | 1.10 / 0 / 0 | 1.0° | 0° |

**`Euler_pid_Cale` 当前结构（2026-06-23「三维转二维」后）**：
- **直通**：外环输出（世界系欧拉角速率）**直接喂内环**（机体陀螺）。06-22 试过的欧拉运动学变换 T(φ,θ)（世界系角速率→机体角速度解耦）已**撤回、整段注释**，因本工程陀螺轴序非标准 ZYX（`gx=pitch/gy=roll/gz=yaw`）套不上标准公式；作 TODO 框架留在函数注释里，待轴序标定后再启用。小角度/小滚转下直通够用。
- **pitch 外环门控**：pitch 外环+内环仅在 `Guidance_State>Stable && 视觉识别成功` 时更新，否则内环保持上拍（Stable 不控俯仰）；**yaw/roll 每拍都算**。
- **roll 内环陀螺 ÷2**：内环 roll 喂 `current_gyro_Euler[NOW][ROLL]/2.0f`（经验压抖/降 roll 内环等效增益，留痕待解释）。

- **死区软化** `Deadband_Soften`：误差落入死区连续归零（C0 连续，无硬切断尖刺）。
- **角度环绕** `Angle_Wrap_180`：ROLL/YAW 外环 `angle_wrap` 当前均设为 **0（未启用）**。函数保留，需要时置 1 即开。
- **前馈 FFC**：结构在（`xFeedForward` 已内嵌、空指针 bug 已修），但 `Euler_pid_Cale` 里调用被注释 + `num1/num2=0` → **当前不生效**。**2026-06-08 决定暂不启用前馈**，代码保留备用。
- **D 项**（2026-06-23 审计核对）：pid.c:188 测量微分 `−d·(get[NOW]−get[LAST])/dt` 被**注释**，:189 实际用**误差微分** `d·(e_now−e_last)/dt`（对误差 e=set−get 求导）。PROGRESS 06-10 记的"对测量微分"已不成立——代码可能在后续调试中切回了误差微分。误差微分在目标阶跃时会有微分冲击 kick，但当前视觉目标已用 LPF 平滑（非阶跃），故 kick 被源头消解。
- `max_err` 字段全为 0 → 该保护从不触发（依赖 `MaxOutput` 限幅）。
- **速度方向外环** `vel_pursuit_pid[2]`（PITCH/YAW）：速度矢量追踪模式用，MaxOutput=45°，当前 `vel_pursuit_mode=0`（未启用）。

### LADRC 线性自抗扰（`adrc.c/.h`，可选替代 PID）

`ladrc_mode` 运行时切档（surface_control_task.c，**当前值=0 全 PID**）：
- **0**=全 PID（安全默认，当前在用）
- **1**=三轴全 LADRC（`Euler_LADRC_Cale`）
- **3**=仅 Roll 用 LADRC，Pitch/Yaw 仍 PID

单环二阶 LADRC = 三阶 LESO + LSEF + 扰动补偿，整轴 3 个旋钮：wc / wo / b0。Roll 已标定：wc=10.5, wo=52.5, b0=55, deadband=1°, max=±15°。阻尼默认用实测陀螺（`use_gyro_damp`），非 LESO z2。详见 [adrc.c](imcalib/Tool/adrc.c) 头部注释。

### LQR 状态反馈（`lqr.c/.h`，可选替代 PID + 混控，**未编译/待台架**）

`lqr_mode` 运行时切档（surface_control_task.c，**当前值=1 正调 LQR**，以代码为准；优先级高于 ladrc_mode/vel_pursuit_mode）：
- **0**=关（走 PID/LADRC，安全默认）
- **1**=LQR 一步解算：`u=-K_d·x`，6 态 `[roll,pitch,yaw 误差(rad), p,q,r(rad/s)]` → 4 舵偏(rad)。
- **pitch 仅制导段受控** `lqr_pitch_terminal_only`（lqr.c，默认 1）：非 `Terminal` 段把 pitch 状态分量 `x[1]`(pitch_err)/`x[4]`(q) 清零 → LQR 解出的 4 舵**不含 pitch 通道成分**（Stable 等阶段 pitch 不打舵），roll/yaw 照常自稳；`err_deg[1]` 保留真值供观测。=0 回全程控 pitch 做 A/B。

**与 PID/LADRC 的本质区别**：PID/LADRC 是两步（先算三轴力矩 `output_gyro_Euler` → 再过 `Servo_Mix_*` 混控）；LQR 的 `K_d[4][6]` 已把 X 翼混控几何 G 烘焙进模型，**一步替代 PID+混控两步**，`Euler_LQR_Cale` 直接写 `output_angle_Servo[NOW][...]`、绕过 `output_gyro_Euler` 与 `Servo_Mix_*`（调用点已加 `lqr_mode!=1` 跳过混控）。

**K 矩阵** = [lqr.c](imcalib/Tool/lqr.c) 的 `dart_lqr_K[4][6]`（MATLAB 同名同形「粘贴区」）：在 [lqr_czn/dart_attitude_LQR_v1.m](lqr_czn/dart_attitude_LQR_v1(1)(1).m) 调好 Q/R/惯量/速度，跑 Step5 把打印的 4 行整块覆盖粘贴即可。⚠ MATLAB 舵号(delta1=右上,2=左上,3=左下,4=右下) ≠ 工程索引(UL/UR/DR/DL)，在 `K_ROW_TO_SERVO[]` 换序；上车前必按移植指南 §9 逐轴阶跃验符号（飞镖一次性，G 符号反=正反馈）。详见 [lqr.c](imcalib/Tool/lqr.c) 头注 + [lqr_czn/MCU_LQR_PORTING_GUIDE(1).md](lqr_czn/MCU_LQR_PORTING_GUIDE(1).md)。

> 未编译：新增 `lqr.c` 需手动加入 Keil/eIDE 工程编译列表（AI 编不了）。

### LQI 力矩控制器 + Pitch 保护型零空间分配（`lqi_torque.c/.h` + `torque_allocator.c/.h`，2026-07-23 新增，**未编译/待台架**）

**与 LQR 的本质区别**：LQR 一步输出 4 舵角（K 含混控），LQI 拆成两步——先 LQI 输出 3 轴物理力矩 N·m，再零空间分配器把力矩翻译成 4 舵角。

`lqi_mode` 运行时切档（surface_control_task.c，**当前值=0 默认关**，优先级高于 `lqr_mode`）：
- **0**=关（走 LQR，默认安全路径）
- **1**=LQI 力矩控制 + Pitch 保护零空间分配

**状态与输出**：
- 状态 `xa[9]` = `[e_roll, e_pitch, e_yaw, p, q, r, ∫e_r, ∫e_p, ∫e_y]`（rad, rad/s, rad·s）
- LQI 输出 `tau[3]` = `[Mx, My, Mz]`（N·m）
- K_lqi[3][9] 由 MATLAB dlqr 生成（[matlab_script/](matlab_script/)目录）
- 舵面顺序全线统一为 `[UL, UR, DR, DL]`，C 端不再换序

**H_tau 力矩矩阵**（3×4，N·m/rad）：从舵面几何（`r_i × n_i`）× 动压 × 面积 × 舵效显式计算，替代旧集总 G 矩阵。当前所有气动参数为占位符（待 SolidWorks/CFD）。

**零空间分配算法**（`torque_allocator.c`）：
1. 先满足 Roll/Yaw 力矩（2×4 子矩阵伪逆 → delta0）
2. 在 Roll/Yaw 零空间内优化（2×2 解析求解）：
   - 最小化 Pitch 力矩（权重 λ_pitch=100）
   - 最小化总舵面动作（权重 λ_servo=1）
3. 舵面限幅 + 统一缩放 + 回算实际力矩
4. 饱和/不可达 → 冻结积分

**关键文件**：
- MATLAB：[matlab_script/dart_lqi_parameters.m](matlab_script/dart_lqi_parameters.m)（参数配置）、[dart_attitude_lqi_torque_pitch_protected.m](matlab_script/dart_attitude_lqi_torque_pitch_protected.m)（主脚本）、[dart_lqi_export_c.m](matlab_script/dart_lqi_export_c.m)（C 导出）
- C：[lqi_torque.c](imcalib/lqi_tool/lqi_torque.c)/[.h](imcalib/lqi_tool/lqi_torque.h)（控制器）、[torque_allocator.c](imcalib/lqi_tool/torque_allocator.c)/[.h](imcalib/lqi_tool/torque_allocator.h)（分配器）
- 表：[lqi_gain_table.h](imcalib/lqi_tool/lqi_gain_table.h)（K_lqi 表）、[lqi_geometry_table.h](imcalib/lqi_tool/lqi_geometry_table.h)（H_tau 表 + 零空间）
- 计划：[PLAN_LQI_TORQUE_PITCH_PROTECTED.md](PLAN_LQI_TORQUE_PITCH_PROTECTED.md)

**⚠ 占位符清单（需 SolidWorks/CFD/台架数据后重新生成所有表）**：
- 交叉惯量 Ixy/Ixz/Iyz（当前 = 0）
- 舵面位置 r_i（当前从 r_ac=0.150/a_ac=0.120 反推）
- 舵面面积 S_i（当前 0.005 m²）
- 舵效系数 C_Fδ（当前 5.0，占位 ≈ CLα）
- 气动阻尼/恢复系数（当前全 0，未启用）

> 未编译：新增 4 个 .c 文件需手动加入 Keil/eIDE 工程编译列表（AI 编不了）。

---

## 8. 混控 / 控制分配（`surface_control_task.c`，**当前重点**）

X 翼逻辑符号阵（enum 列序 UL,UR,DR,DL）：**pitch `[−1,−1,−1,−1]`**（四片同号，SIGN 翻转后等效）、roll `[+1,−1,−1,+1]`、yaw `[−1,+1,−1,+1]`。物理装配符号 `SIGN=[UL−1, UR+1, DR+1, DL−1]`（左右舵镜像安装；台架单轴阶跃标定，某片整体反了翻它的号；SIGN 每片三轴共享，轴间配对结构由逻辑阵的列决定、不靠 SIGN）。Alloc.B pitch 行同为 `[−1,−1,−1,−1]`。

**X 翼解算几何（pitch 为何四片同号）**：4 片成 X(45°，从尾看 UL135°/UR45°/DR315°/DL225°)，每片偏转产生切向力，对力矩贡献 **pitch∝cosθ=[UL−,UR+,DR+,DL−]、yaw∝sinθ=[+,+,−,−]、roll=常数**（三模态两两正交，第 4 模态 `[+,−,+,−]` 隔片交替=零空间纯阻力）。故 pitch 列四片同号，配 SIGN 后 pitch 指令落到舵令 `u=[−,+,+,−]`=真俯仰、与 roll/yaw 解耦。*(2026-06-08 校正：原 pitch 列 `[+1,+1,−1,−1]` 经 SIGN 落零空间、只减速不俯仰。)*

**`Alloc.Mode` 运行时分配器**（调试器/初值切换；**默认现为 2**；Mode0 调用点已注释，实际只有 1/2 可选）：

| Mode | 函数 | 说明 |
|---|---|---|
| 0 | `Servo_Mix_PitchPriority` | pitch 优先启发式饱和缩横侧。k 公式已修正 + 加 `k≤1`。**调用点已注释、当前不可达**，仅留函数作历史对照。 |
| 1 | `Servo_Mix_AxisLimit` | 各轴前置限幅(P30/R15/Y30)→**逐级(字典序)优先级缩放**（`Alloc.Prio` 默认 **{ROLL,YAW,PITCH}**，调试器在线改）→×SIGN→兜底限幅60。高优先轴不被低优先轴污染。 |
| **2（默认）** | `Servo_Mix_MinEnergy` | 真·带约束最小能量：CMSIS 伪逆 `u0=Bᵀ(BBᵀ)⁻¹v` → 零空间投影进限幅盒 → 不可达按 pitch>yaw>roll 二分缩。奇异退回 Mode1。舵效阵 `Alloc.B` 默认理想阵、可台架辨识替换。 |

- **Roll_Derotate_PitchYaw**：把世界系 pitch/yaw 力矩按当前 roll 反旋到机体系（`Δ=current_roll−Stable_roll`）。**仅 `Guidance_State==Terminal` 时调用**（Stable 段直通不反旋）；调用点 `p_body/y_body=0` 清零行已注释 → pitch/yaw 三轴放开。
- Vofa 观测量（均 `Alloc` 结构体字段）：`Alloc.lat_scale`(最低优先轴保留比 k)、`Alloc.u0/u_out`、`Alloc.alpha/v_scale/p_scale`、`Alloc.infeasible/singular_flag`。

### ⭐ 当前"实际在飞什么"（极重要，2026-06-27 快照）
三轴全部放开（调用点清零行已注释），全接 PID 内环输出，按 `Alloc.Mode` 分派；Stable 与 Terminal 都打舵（且需 `imu_is_static==0`）。
- **控制器**：`ladrc_mode=0`→PID 串级（默认在用）；`=3`→roll 用 LADRC、pitch/yaw 仍 PID；`=1`→三轴全 LADRC。`vel_pursuit_mode=0`（速度矢量追踪未启用）。**外环→内环直通**（无欧拉运动学变换，见 §7）。
- **分配**：`Alloc.Mode=2`（最小能量，默认）。`Roll_Derotate_PitchYaw` 仅 Terminal 段反旋 pitch/yaw、roll 直通。
- **末制导**：① 视线锁存（方向A）+ 航位推算；② 视觉目标平滑的 **LPF/距离增益已删除**（2026-06-27 清理，`target`=锁存视线终点，见 §9）；③ pitch 用**主动滑翔→扎** `pitch_glide_mode=1`（远滑翔增程、近扎下，见 §9），pitch 外环仅 Terminal+识别成功才控；④ PN 超前补偿 `#if 0`（关闭）；丢目标就地保持。
- 配合 2026-06-08 pitch 解算几何对齐，pitch 才真正产生俯仰力矩。

---

## 9. 视觉 / 通信（`CallBack_Task.c`）

| UART | 实例 | 对端 | 模式 |
|---|---|---|---|
| huart1 | USART1 | 镖头触发板 | **单线半双工** DMA，CRC8-MAXIM |
| huart2 | USART2 | PC / Vofa | DMA（调试+遥测） |
| huart3 | USART3 | 视觉 OpenMV | DMA 空闲事件，6 字节帧 |

- 视觉帧（均 6 字节）：`0x5A..0xA5`=识别成功（x,y 像素），`0x5B..0xA6`=距离+面积（dist_cm,area 均 uint16 大端，2026-06-12 新增，独立于识别包），`0x7A..0xA7`=丢目标，`0x9A..0xA9`=录制状态。`0x5B` 包只更新 `Vision_Rx_Data.dist_cm/area`、不置 recognize/New_Data；OpenMV `send_distance(dist_cm=DIST_K/sqrt(px), area)` 配套。
- **像素→度转换在 `Vision_Receive` 内做**：`Euler[YAW]=y/160*72`、`Euler[PITCH]=x/120*54`（x→PITCH、y→YAW）。✅ **已确认**：视觉发的是**像素**、接收时即转成度，轴映射正确，下游 `Guidance_Terminal` 锁存按度用、量纲一致（无需再 ×FOV）。
- `Vision_New_Data_flag`：ISR(~20Hz)产生、`Guidance_Terminal`(1kHz)消费后清 0（生产者-消费者）。
- **视线锁存（方向A）+ 航位推算（2026-06-15+；LPF/距离增益 2026-06-27 已删除）**：新帧到达瞬间把世界系视线锁存到 `vision_los_final=vision_euler+current`（终点，帧间不变、航位推算）。**原一阶低通平滑 + 距离增益缩放 k 已整段删除**（`lpf_*`/`yaw_gain`/`pitch_gain`/`k_*` 死变量全清），现 `target` 直接 = `vision_los_final[NOW]`（无额外平滑）。读侧用临界区快照 `v`；丢目标(FAILURE)终点+目标对齐当前(就地保持)。*(`Target_Slew`/`Vision_Angle_Normalize`/`*_Gain_ByDist` 均已 `#if 0` 封存。)*
- **PN 视线率超前补偿（2026-06-14，当前 `Guidance_Terminal` 内 `#if 0` 关闭）**：锁存的世界系视线终点帧间差分得惯性视线率 λ̇（`vision_los_rate[]`，纯视觉、不依赖会漂的 IMU 积分速度）。**拆分接口** `PNG_Apply_Lead_Yaw/Pitch`（[PNG_Task.c](imcalib/Task/PNG_Task.c)）支持分轴门控（俯冲未到位只喂 yaw）；`PNG_Mode=1` 改用 **vision_ins EKF 世界系 p/v 叉乘直接算 LOS 率** ω_yaw/ω_pitch（替代纯帧差分），`PNG_Mode=0` 仍走 `vision_los_rate`。`LOS_RATE_LIMIT_DPS`=40、`PNG_LEAD_LIMIT_DEG`=8 限幅。**当前关闭，先用纯 PID 跟踪验证基础性能，标定好 `K_Dyn` 后再打开。**
- **末制导 pitch 能量管理（2026-06-12 `Pitch_Dive_Floor` → 2026-06-27 主动滑翔→扎，当前用后者）**：现行 `pitch_glide_mode=1`——按**世界系看灯视线俯角 φ**(`vision_los_final[NOW][PITCH]`) 在两段插值：远端（φ≥`GLIDE_LOS_HI_DEG`=−12°）`blend=0` → pitch 目标住 `THETA_GLIDE_DEG`(+2°)**压平增程**；近端（φ≤`GLIDE_LOS_LO_DEG`=−25°）`blend=1` → 平滑过渡到追视觉 pitch 目标**扎下**（衔接 `PITCH_INCIDENT_DEG≈−27°`）。`pitch_glide_blend/target` Vofa 可观测。切 `pitch_glide_mode=0` 退回旧 `pitch_control_limit_deg` 门限逻辑。*(旧 `Pitch_Dive_Floor`/接近度 s/弹道角 γ 调度的代码仍在，但现行 pitch 目标由 glide→dive 主导。)*
- **视线角半径归一化（2026-06-15+，⚠️ 2026-06-27 已 `#if 0` 封存、调用已删）**：`Vision_Angle_Normalize(angle, radius)` 把视线角按 blob 半径归一化到 `REF_RADIUS`(15px)，消除远近 blob 大小差异。当前主路径不再调用。
- **距离增益调度（2026-06-15+，⚠️ 2026-06-27 已 `#if 0` 封存）**：`Yaw_Gain_ByDist(dist_cm)` / `Pitch_Gain_ByDist(dist_cm)` 纯距离线性插值增益（YAW 远大近小、PITCH 反向）。随 LPF 删除一并停用、不再被调用。
- **（历史/底层，已被 glide→dive 取代为主路径，⚠️ 2026-06-27 已 `#if 0` 封存）末制导俯仰能量管理 `Pitch_Dive_Floor`（2026-06-12）**：随接近度放开的最陡俯冲限幅 `θ_floor=max(L_sched(s), γ−AOA_MARGIN)`。接近度 s 远段用 `dist_cm`、近段用 `area`，缺则按弹道角 γ。现行 pitch 目标由 `pitch_glide_mode=1` 主导；物理意图一致（远保射程、近放开到入射角、终端迎角→0 正向撞击）。
- **速度预测（2026-06-15+ 改用 EKF）**：旧 `Kalman_Vel_Calc`（filter.c 二阶卡尔曼）已 `#if 0` 停用。改用 `vision_ins.c` 6 态 EKF：IMU 加速度 1kHz 预测 + 视觉笛卡尔位置 ~30Hz 更新 + 静止零速更新。EKF 输出 `vins_out.v_world` 回写 `IMU_Data.Velocity[World]`，供机体速度映射/弹道角 γ/Vofa。俯冲入段(`Stable→Terminal`)用「姿态前向×`V_NOM_MS`」`VisInsEKF_SetVel` 锚定初速 → `gamma_pitch_fwd_deg`=姿态前向估计（不漂，取代会漂的速度版 `gamma_pitch_deg`）。ZUPT 在发射前自动归零速度+对准零偏。
- **速度/γ 单位量纲修复（2026-06-13）**：`A_World` 去重力原写 `a_raw − gravity·R_col3`，`a_raw` 单位是 **g**(静止≈1)、`gravity=GRAVITY_MS2` 是 **m/s²**，量纲不一致(静止误出 ≈−8.8)使 `A_World`/速度/`gamma_pitch_deg` 全错——即"实测 γ 不准"的根因(2026-06-07 审计已记此项、当时 PNG 未接主环遂留待)。改为 `gravity·(a_raw − R_col3)`(先把 `a_raw`×g 转 m/s² 再扣重力)，静止线加速度=0、量纲与 `V_NOM_MS` 一致；`a_raw_x/y/z` 本身不动→不影响 Mahony 的 g 单位归一化/门控。残余漂移源(次要、待台架核验)：加速度零偏 `A_Offset` 未回扣、发射后纯陀螺 coast 姿态漂。
- **PNG 比例导引**：`PNG_Guidance` 在 `TotalControl` 里被注释，**未接主环**；`Velocity[Body]` 现已随速度预测算出（但 PNG 仍未接）。

---

## 10. 关键全局变量速查

| 变量 | 定义处 | 含义 |
|---|---|---|
| `IMU_Data` | IMU.c | 全部 IMU 状态（A/G/Q/Euler/A_World/零偏/calib_done/A_Offset/Vel_Dir） |
| `Surface` | surface_control_task.c | 控制状态总仓（current/target/output Euler、output_angle_Servo、Finally_Angle、Stable_Euler_Angle、Guidance_cnt） |
| `Guidance_State` | surface_control_task.c | 制导状态机当前态 |
| `ladrc_mode` | surface_control_task.c | LADRC 切档（0=全PID/1=三轴LADRC/3=仅Roll LADRC） |
| `lqr_mode` / `lqr_pitch_terminal_only` / `lqr_ctrl` / `dart_lqr_K[4][6]` | surface_control_task.c / lqr.c | LQR 切档（0=关/1=一步6态→4舵含混控，绕过 Servo_Mix_*；**当前=1**）/ pitch 仅 Terminal 段受控开关(默认1) / 状态+舵偏观测仓 / K 矩阵(MATLAB 粘贴区)。未编译/待台架 |
| `ladrc_ctrl[3]` | adrc.c | LADRC 控制器实例（wc/wo/b0/z1/z2/z3/u 等） |
| `Alloc`（结构体 `Alloc_t`） | surface_control_task.c | 控制分配状态（整合原 `Alloc_Mode/Prio/B`、`alloc_*`、`servo_lat_scale`）：`.Mode` 档位 / `.Prio[3]` 优先级(默认{ROLL,YAW,PITCH}) / `.B[3][4]` 舵效阵 / `.u0/.u_out/.alpha/.u0_span/.v_scale/.p_scale/.lat_scale/.infeasible/.singular_flag` Vofa 观测 |
| `Vision_Rx_Data` | CallBack_Task.c | 视觉接收（ISR 写、控制读，含 New_Data_flag/dist_cm/area/radius/Euler_norm） |
| `vision_los_final[2][3]` / `vision_los_rate[3]` | surface_control_task.c | 末制导世界系视线终点（视觉新帧锁存，`target` 直接取用、无额外平滑）/ 惯性视线率 λ̇（终点帧间差分，PN 用，当前 `#if 0`） |
| `vins_out` | vision_ins.c | EKF 输出（p_world/v_world/range_m/vc/locked） |
| `gamma_pitch_fwd_deg` / `gamma_pitch_deg` | IMU.c | 弹道角γ姿态前向估计°(不漂，常用) / 速度积分版°(已停用，保留) |
| `Vel_Reanchor_Flag` / `imu_is_static` | IMU.c | 俯冲入段锚定请求位 / ZUPT 静止标志 |
| ~~`pitch_dive_floor` / `closeness_s`~~ | surface_control_task.c | 旧 `Pitch_Dive_Floor` 的 Vofa 观测，2026-06-27 随函数 `#if 0` 封存（定义已注释） |
| `pitch_glide_mode` / `pitch_glide_target` / `pitch_glide_blend` | surface_control_task.c | 末制导 pitch 主动滑翔→扎（默认1）/ 当前 pitch 目标° / 滑翔→扎过渡系数 0..1 |
| ~~`yaw_distance_gain` / `pitch_distance_gain`~~ | surface_control_task.c | 旧距离增益 Vofa 观测，2026-06-27 随 `*_Gain_ByDist` `#if 0` 封存（定义已注释） |
| `surface_control_pid[2][3]` / `mahony_pid[3]` / `vel_pursuit_pid[2]` | pid.c | PID 实例（含速度方向外环） |
| `temp[3]` | pid.c | 外环→内环中转（Vofa 观测） |
| `DART_TYPE` | surface_control_task.c | `VECTOR_NOZZLE`(X翼,激活) / `FIXED_WING`(飞翼,#if0 不编译) |

---

## 11. 已知问题 / 陷阱（答题时警惕；详见 PROGRESS「当前 TODO」）

- **IMU/Control 同 Idle 优先级无同步** → 控制可能用上一拍姿态。
- **GYR_KF_R=1000（有意）**：用户实测 R=30 欠滤波（滤不动），关键是 Q:R 比例；与早期文档"30"不同，以代码为准。
- **外环→内环直通、无坐标系变换**：欧拉运动学变换 T(φ,θ) 已撤回（06-23 三维转二维），陀螺轴序非标准 ZYX、未推导。大滚转时 pitch/yaw 可能耦合；小角度够用。变换框架注释留在 [pid.c](imcalib/Tool/pid.c) `Euler_pid_Cale`，待轴序标定再启用。
- **roll 内环陀螺反馈 ÷2**：`Euler_pid_Cale` 内环 roll 喂 `current_gyro_Euler[NOW][ROLL]/2.0f`（经验值，留痕待解释）。
- **D 项用误差微分（非测量微分）**：pid.c 测量微分行被注释、用误差微分 `d*(e_now-e_last)/dt`。当前 I/D≈0（纯 P 调参），且视觉目标非阶跃，微分冲击影响小。
- **视觉目标 LPF/距离增益已删除（2026-06-27 清理）**：原 `lpf_*`/`yaw_gain`/`pitch_gain`/`k_*` 中间量全清，`target` 直接=`vision_los_final`；`Target_Slew`/`Vision_Angle_Normalize`/`*_Gain_ByDist`/`Pitch_Dive_Floor` 一并 `#if 0` 封存（见 §9）。
- **angle_wrap 未启用**：ROLL/YAW 外环 `angle_wrap=0`，函数保留但未开。跨 ±180° 边界时可能出现假误差。
- 失效/未启用：MAG、PNG 主环(`#if 0`，拆分接口已就绪)、FFC、3D IMU 卡尔曼(`#if 0`)、旧速度卡尔曼 `Kalman_Vel_Calc`(`#if 0`，已被 vision_ins EKF 取代)、PN 超前补偿(`#if 0`)、速度矢量追踪(`vel_pursuit_mode=0`)、LADRC(`ladrc_mode=0`)、LQR(`lqr_mode=0`，且 lqr.c 未加入工程编译)、欧拉运动学变换(注释)。
- 死代码/卫生：`FIXED_WING` 整段 `#if 0`；`PWM_Init` 启了若干未配置/无 GPIO 通道（实际用的 4 路正常）；`Button.c` `Press_Long_Cnt!=0`（数组地址恒真，实际不触发越界）；`filter.c` 注释 GBK 乱码；末制导 `Target_Slew`/`Vision_Angle_Normalize`/`Yaw|Pitch_Gain_ByDist`/`Pitch_Dive_Floor` 已 `#if 0` 封存（增益调度块，2026-06-27）；`Servo_Mix_PitchPriority`(Mode0) 调用点已注释、不可达。

## 12. 2026-06-07 本次审计已修复

- **`Alloc_Mode` 默认 0→1**（启用修好的 Mode1 `Servo_Mix_AxisLimit`，roll-only 线性正确）。
- **Mode0 `Servo_Mix_PitchPriority` k 公式修正** `(LIMIT−sgn·P)/aL` + 加 `k≤1` 上钳（原反写致 roll 二次畸变）。
- **`Guidance_Terminal` 视觉读取改用临界区快照 `v`**（原快照是死变量、实际读 live struct 与 ISR 撕裂）。
- **Vofa 第 10-12 路改发 `output_gyro_Euler[P/R/Y]`**（原误发 `output_angle_Servo[0..2]`、X翼下=舵 UL/UR/DR、看不到真正的 PID 内环输出）。
- **`surface_control_task.h` 舵机通道注释纠正**（DR/DL 尾注释原与宏名相反）。

**审计后跟进（2026-06-07 同日）**：TIM4 预分频统一为 **169**（与 TIM3 一致，时基差已消，用户改）；删除无用的 **`Servo_PWM_Limit`**（量纲统一为角度后失效，输出由上游 ±60 约束）；**视觉单位/轴映射经用户确认正确**（像素，接收时转度）。

**2026-06-08 文档维护**：清理 TODO——**加速度标定**（用户确认现状正常）、**IMU SPI 阻塞读超时**（不处理）移出已知问题；**前馈 FFC** 决定暂不启用（效果不大 / 不易验证，代码保留关闭态）；**坐标系统一 + roll 自稳**经 Vofa 确认正常（详见 [PROGRESS.md](PROGRESS.md) 「已 Vofa 确认」）。

**2026-06-27 复审（本次）**：跟进 06-23~06-27 代码——① 三维耦合变换撤回为**直通**（§7）；② 调参哲学转向「内环>外环纯 P 串级」、增益表重写为快照+以代码为准（§7）；③ `Alloc_Mode` 默认 1→**2**、Mode0 调用点注释（§8）；④ `Roll_Derotate` 改为仅 Terminal 段、非旁路（§8 ⭐）；⑤ 末制导 pitch 改**主动滑翔→扎** `pitch_glide_mode=1`（§9/§10）；⑥ LPF 视觉目标平滑主路径**被旁路**（§9/§11）；⑦ PNG 拆分接口 `PNG_Apply_Lead_Yaw/Pitch` + `PNG_Mode=1` 就绪但仍 `#if 0`（§9）；⑧ 修复指向已删 plan 文件的链接、新增 [AGENTS.md](AGENTS.md) 入口。**全文数值改为"快照、以代码为准"口径**（高频调参期）。

**2026-06-23 全量审计 v2**：新增 vision_ins EKF / LADRC / ZUPT / 距离增益 / LPF 目标平滑 / 速度矢量追踪等模块文档；修正增益表、分配器默认值、D 项描述、angle_wrap 状态、参数值。详见 [PROGRESS.md](PROGRESS.md)「参数勘误」。

**2026-06-08 X 翼 pitch 解算几何对齐**：pitch 逻辑列对齐为四片同号 `[+1,+1,+1,+1]`（`Servo_Mix_AxisLimit` 的 C 阵 / `Alloc_B` / `Servo_Mix_PitchPriority` 三处），使 pitch 指令落到真俯仰模态、与 roll/yaw 正交解耦（X 翼几何见 §8）；SIGN 不动。pitch/yaw 调用点清零行已注释 → 三轴放开。

## 13. 2026-06-10 制导段抖动修复（D 项对测量微分 + 视觉目标斜坡）

- **症状**：目标附近抖动，主要在**末制导(视觉介入)段**；纯陀螺自稳段不明显。想给小 D 加阻尼反而抖得更厉害。
- **根因 = 微分冲击（derivative kick）**：[pid.c](imcalib/Tool/pid.c) 的 D 原本对**误差** `e=set−get` 求导。纯陀螺段目标恒定(`Stable_Euler_Angle`)，`de/dt=−d(角度)/dt` 是纯阻尼；末制导段目标每 50ms 被视觉新帧阶跃刷新([surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`)，那一拍 `dout=d·Δ/dt=d·Δ·1000` 是脉冲尖刺、20Hz 周期激励 → 抖；Δ 越小 d 也救不回(d=0.005、Δ=0.5° 仍出 2.5 脉冲)。
- **方案1（根治，pid.c）**：D 改对**测量微分** `−d·(get[NOW]−get[LAST])/dt`，只对反馈量求导 → 目标阶跃不再进 D、纯阻尼；不经死区软化、死区内仍阻尼；零延迟非低通。
- **方案3（视觉源，surface_control_task .h/.c）**：setpoint 端给锁存目标加**斜坡**(速率限制)。新增 `vision_los_final[3]`(世界系视线终点)、`Target_Slew`(差值含 YAW 角度环绕)；`Guidance_Terminal` 改为「视觉帧更新终点 + 每 tick `target` 斜坡逼近终点」，把 50ms 阶跃摊平。宏 `VISION_TARGET_SLEW_DPS`=150°/s 可台架调(大→跟手、小→平滑滞后)。与原航位推算等价、仅消阶跃。*(2026-06-14：固定速率斜坡已升级为「比例化逼近」+ PN 视线率超前补偿，`VISION_TARGET_SLEW_DPS` 废弃，见 §9。)*
- **未编译**(Keil 工程，需在 MDK 里编)。两项均合"输入端/源头解决、不在反馈环加低通"偏好。

---
*维护约定：本文与 PROGRESS.md / memory `control-tuning-progress` 三方同步；改代码后更新对应小节。*
### LQR+I 积分扩展（2026-07-14 → 2026-07-20 重写为 PID 积分并环）

积分机制已从 LQR 自维护 `lqr_ctrl.integral.err` 重写为 **PID 积分并环**：
- **3 个专用 `pid_i_for_lqr[3]`**（PITCH/ROLL/YAW，`pid_t` 类型），P=D=0 纯积分，ki 默认 0.1（宏 `LQR_I_KI_DEFAULT`）
- **积分分离**：`|err_deg| ≥ 0.5°`（`LQR_I_SEPARATION_DEG_DEFAULT`）→ 清零 iout；`< 0.5°` → `pid_calc()` 累积。分离阈值是唯一门控（PID deadband=0）
- **并环**：`增强误差 = err_deg + pid_i_for_lqr[axis].iout`（度），再 DEG2RAD 进 LQR 状态 x[0..2]
- **符号**：`pid_calc(set=当前角度, get=目标角度)` → PID 误差 = current−target = LQR err_deg，同号
- **抗饱和**：舵面饱和时回退本拍 iout 到保存值，重算 LQR
- 旧 `LQR_Update()` 中的积分贡献代码块已移除（积分已预叠加在 x 中）
- 积分贡献 `pid_i_for_lqr[axis].iout` 写 `lqr_ctrl.integral.err` 供 Vofa 观测

参数以 `lqr.h` 宏（`LQR_I_SEPARATION_DEG_DEFAULT`/`LQR_I_LIMIT_DEG_DEFAULT`/`LQR_I_KI_DEFAULT`）和 `lqr.c` `LQR_Init` 代码为准，未编译、待台架。
