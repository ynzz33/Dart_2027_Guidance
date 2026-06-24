# Dart_2027 飞控 — 代码总览 / 项目地图

> 给 Claude（及任何接手者）**答题前先读这一篇**就能掌握整个项目的速查地图。
> 进度与决策见 [PROGRESS.md](PROGRESS.md)；详细技术方案见 [plan-flight-control-overhaul.md](plan-flight-control-overhaul.md) / [plan-pitch-priority-mixing.md](plan-pitch-priority-mixing.md)。
> 本文基于 **2026-06-23 全量审计 v2** 整理，力求与代码一致；改代码后请同步本文。

---

## 0. 一句话定义

STM32G431 + BMX055 + FreeRTOS 的 **Dart 飞镖型飞行器飞控**：X 翼 4 舵面，串级 PID（角度环→角速度环）+ 可选 LADRC 单环二阶自抗扰（`ladrc_mode` 切档）自稳 + 末制导，Mahony 姿态融合，视觉/IMU 紧耦合 6 态 EKF（vision_ins.c）给不漂的速度，视觉（OpenMV）经 UART 给视线角+距离+面积。比赛镖（RoboMaster 飞镖）场景。

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
│   ├── IMU.c/.h               BMX055 SPI 读取 + Mahony 姿态解算（最核心）
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
`ALL_CS_Free` → 启 TIM6/7 中断 → `PWM_Init()`(启 TIM2/3/4 PWM) → `pid_init()` → `Q[NOW][0]=1` → 三路 UART 空闲DMA接收(huart1半双工/huart2调试/huart3视觉，关半传输中断) → `ADC_Init()` → `IMU_Init()`(BMX055寄存器) → `PNG_Init()` → `Kalman_Vel_Init()` → 上电 → 目标欧拉角初值 30/30/30（随即被状态机覆盖）。

---

## 4. 数据流水线（每 1ms）

```
BMX055(SPI2阻塞读) ─IMU_Data_Read─► IMU_Data.A/G(原始,去饱和/NaN,标量Kalman,减陀螺零偏)
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
        Servo_Mix_*(Alloc_Mode 分派) → output_angle_Servo[NOW][0..3] (度,含SIGN,±60)
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
- `Stable_Flag` 在 Stable 且 `Euler[PITCH]≤30` 时置 1。
- `Self_Text_State` 下四舵置 30°（自检摆舵）；`Start` 下置 0。
- 各阶段目标：Start/Stable → ROLL/YAW 锁 `Stable_Euler_Angle`、PITCH=当前（只阻尼）；Terminal → ROLL 自稳，YAW/PITCH 视觉视线锁存（见 §9）。

---

## 7. 串级 PID（`pid.c` / `pid_init`）

每轴两环：`surface_control_pid[Angle][axis]`（外环角度）→ `surface_control_pid[Gyro][axis]`（内环角速度）。当前激活的是「镖体1」一组（镖体2/3 注释着）：

| 轴 | 外环(Angle) P/I/D | 内环(Gyro) P/I/D | 外死区 | 内死区 |
|---|---|---|---|---|
| PITCH | 0.2 / 0 / 0.05 | 0.30 / 0 / 0 | 1.0° | 0° |
| ROLL | 1.8 / 0.5 / 0.5 | 0.2 / 0 / 0 | 0.5° | 1.0° |
| YAW | 0.55 / 0.5 / 0.2 | 0.5 / 0 / 0 | 0° | 1.0° |

- **死区软化** `Deadband_Soften`：误差落入死区连续归零（C0 连续，无硬切断尖刺）。
- **角度环绕** `Angle_Wrap_180`：ROLL/YAW 外环 `angle_wrap` 当前均设为 **0（未启用）**。函数保留，需要时置 1 即开。
- **前馈 FFC**：结构在（`xFeedForward` 已内嵌、空指针 bug 已修），但 `Euler_pid_Cale` 里调用被注释 + `num1/num2=0` → **当前不生效**。**2026-06-08 决定暂不启用前馈**，代码保留备用。
- **D 项**（2026-06-23 审计核对）：pid.c:188 测量微分 `−d·(get[NOW]−get[LAST])/dt` 被**注释**，:189 实际用**误差微分** `d·(e_now−e_last)/dt`（对误差 e=set−get 求导）。PROGRESS 06-10 记的"对测量微分"已不成立——代码可能在后续调试中切回了误差微分。误差微分在目标阶跃时会有微分冲击 kick，但当前视觉目标已用 LPF 平滑（非阶跃），故 kick 被源头消解。
- `max_err` 字段全为 0 → 该保护从不触发（依赖 `MaxOutput` 限幅）。
- **速度方向外环** `vel_pursuit_pid[2]`（PITCH/YAW）：速度矢量追踪模式用，MaxOutput=45°，当前 `vel_pursuit_mode=0`（未启用）。

### LADRC 线性自抗扰（`adrc.c/.h`，可选替代 PID）

`ladrc_mode` 运行时切档（surface_control_task.c）：
- **0**=全 PID（安全默认）
- **1**=三轴全 LADRC（`Euler_LADRC_Cale`）
- **3**=仅 Roll 用 LADRC，Pitch/Yaw 仍 PID

单环二阶 LADRC = 三阶 LESO + LSEF + 扰动补偿，整轴 3 个旋钮：wc / wo / b0。Roll 已标定：wc=10.5, wo=52.5, b0=55, deadband=1°, max=±15°。阻尼默认用实测陀螺（`use_gyro_damp`），非 LESO z2。详见 [adrc.c](imcalib/Tool/adrc.c) 头部注释。

---

## 8. 混控 / 控制分配（`surface_control_task.c`，**当前重点**）

X 翼逻辑符号阵（enum 列序 UL,UR,DR,DL）：**pitch `[−1,−1,−1,−1]`**（四片同号，SIGN 翻转后等效）、roll `[+1,−1,−1,+1]`、yaw `[−1,+1,−1,+1]`。物理装配符号 `SIGN=[UL−1, UR+1, DR+1, DL−1]`（左右舵镜像安装；台架单轴阶跃标定，某片整体反了翻它的号；SIGN 每片三轴共享，轴间配对结构由逻辑阵的列决定、不靠 SIGN）。Alloc_B pitch 行同为 `[−1,−1,−1,−1]`。

**X 翼解算几何（pitch 为何四片同号）**：4 片成 X(45°，从尾看 UL135°/UR45°/DR315°/DL225°)，每片偏转产生切向力，对力矩贡献 **pitch∝cosθ=[UL−,UR+,DR+,DL−]、yaw∝sinθ=[+,+,−,−]、roll=常数**（三模态两两正交，第 4 模态 `[+,−,+,−]` 隔片交替=零空间纯阻力）。故 pitch 列四片同号，配 SIGN 后 pitch 指令落到舵令 `u=[−,+,+,−]`=真俯仰、与 roll/yaw 解耦。*(2026-06-08 校正：原 pitch 列 `[+1,+1,−1,−1]` 经 SIGN 落零空间、只减速不俯仰。)*

**`Alloc_Mode` 运行时三档分配器**（调试器/初值切换；**默认现为 1**）：

| Mode | 函数 | 说明 |
|---|---|---|
| 0 | `Servo_Mix_PitchPriority` | pitch 优先启发式饱和缩横侧。k 公式**已于 2026-06-07 修正**（原反写成 `aL/(LIMIT−P)` 致 roll 二次畸变）+ 加 `k≤1` 上钳。pitch_limit=15。作对照。 |
| **1（默认）** | `Servo_Mix_AxisLimit` | 各轴前置限幅(P40/R15/Y25)→**逐级(字典序)优先级缩放**（`Alloc_Prio` 默认 **{YAW,ROLL,PITCH}**，调试器在线改）→×SIGN→兜底限幅60。高优先轴不被低优先轴污染。 |
| 2 | `Servo_Mix_MinEnergy` | 真·带约束最小能量：CMSIS 伪逆 `u0=Bᵀ(BBᵀ)⁻¹v` → 零空间投影进限幅盒 → 不可达按 pitch>yaw>roll 二分缩。奇异退回 Mode1。舵效阵 `Alloc_B` 默认理想阵、可台架辨识替换。 |

- **Roll_Derotate_PitchYaw**：把世界系 pitch/yaw 力矩按当前 roll 反旋到机体系（`Δ=current_roll−Stable_roll`）。**当前因 p_body=y_body=0 被旁路**。
- Vofa 观测量：`servo_lat_scale`(最低优先轴保留比 k)、`alloc_u0/alloc_u_out`、`alloc_alpha/v_scale/p_scale`、`alloc_infeasible/singular_flag`。

### ⭐ 当前"实际在飞什么"（极重要）
`surface_control_task.c` 调用点 `p_body/y_body/r_body` 清零行**已注释 → 三轴全部放开**：pitch/yaw 经 `Roll_Derotate_PitchYaw` 反旋（当前 Terminal 且 pitch>-5° 时条件反旋）、roll 直通，全接 PID 内环输出（或 LADRC，`ladrc_mode` 切档），按 `Alloc_Mode`(默认1) 分派；Stable 与 Terminal 都打舵。
- **控制链路**：`ladrc_mode=0`→PID 串级（默认）；`ladrc_mode=3`→roll 用 LADRC、pitch/yaw 仍 PID；`ladrc_mode=1`→三轴全 LADRC。`vel_pursuit_mode=0`（速度矢量追踪未启用）。
- **末制导**：`Guidance_Terminal` 用 LPF 平滑视觉目标（非阶跃）；PN 超前补偿 `#if 0`（关闭）；俯仰有 `Pitch_Dive_Floor` 能量管理；丢目标就地保持。
- 配合 2026-06-08 pitch 解算几何对齐（§8），pitch 才真正产生俯仰力矩。

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
- **视线锁存（方向A）+ LPF 平滑（2026-06-15+）**：新帧到达瞬间把世界系视线锁存到 `vision_los_final=vision_euler+current`（终点，帧间不变）；每 tick 用**一阶低通滤波** `target = k×final + (1−k)×target_last` 平滑逼近（k 由距离增益缩放：远处小 k 保守、近处大 k 跟手）。读侧用临界区快照 `v`；丢目标(FAILURE)终点+目标都对齐当前(就地保持)、冻结滤波器。*(替代 2026-06-14 的 `Target_Slew` 比例化逼近——函数保留但主路径已改用 LPF。)*
- **PN 视线率超前补偿（2026-06-14，当前 `#if 0` 关闭）**：锁存的世界系视线终点帧间差分得惯性视线率 λ̇（`vision_los_rate[]`，纯视觉、不依赖会漂的 IMU 积分速度），**仅识别成功**时 `target += PN_LEAD_K·λ̇`（YAW 全程、PITCH 仅俯冲到位 <-5° 后）→ 驱动 λ̇→0＝碰撞航线。`LOS_RATE_LIMIT_DPS`=**40**°/s 限幅；`PN_LEAD_K`=**0.5**（先验方向）、`AOA_TRIM_DEG`=0。**当前关闭，先用纯 PID 跟踪验证基础性能，标定好 K_Dyn 后再打开。**
- **视线角半径归一化（2026-06-15+）**：`Vision_Angle_Normalize(angle, radius)` 把视线角按 blob 半径归一化到 `REF_RADIUS`(15px)，消除远近 blob 大小差异对视线角的影响。写入 `Vision_Rx_Data.Euler_norm` 供 Guidance_Terminal 用。
- **距离增益调度（2026-06-15+）**：`Yaw_Gain_ByDist(dist_cm)` / `Pitch_Gain_ByDist(dist_cm)` 纯距离线性插值增益。YAW 远处增益大(1.3)近处小(0.7)；PITCH 反向远(0.7)近(1.2)。取代旧的面积+距离双段合成（已注释）。
- **末制导俯仰能量管理（2026-06-12）**：俯仰仍跟视觉 LOS，但 `Guidance_Terminal` 末叠加随接近度放开的最陡俯冲限幅 `θ_floor=max(L_sched(s), γ−AOA_MARGIN)`（仅识别成功时钳）。接近度 s 分段：远段用 `dist_cm`、近段用 `area`（0x5B 包），缺则按弹道角 γ 自调度。`γ=gamma_pitch_deg` 由速度预测算（见下）。调用点旧 `pitch>-20→p_body=0` 已撤、pitch 全程受控。物理：远处禁陡俯冲保射程、接近放开到入射角(-27°)、终端 γ≈θ→迎角0=正向撞击。
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
| `ladrc_ctrl[3]` | adrc.c | LADRC 控制器实例（wc/wo/b0/z1/z2/z3/u 等） |
| `Alloc_Mode` / `Alloc_Prio[3]` / `Alloc_B[3][4]` | surface_control_task.c | 分配器档位 / 优先级(默认{YAW,ROLL,PITCH}) / 舵效阵 |
| `Vision_Rx_Data` | CallBack_Task.c | 视觉接收（ISR 写、控制读，含 New_Data_flag/dist_cm/area/radius/Euler_norm） |
| `vision_los_final[2][3]` / `vision_los_rate[3]` | surface_control_task.c | 末制导世界系视线终点（视觉新帧锁存，LPF 平滑逼近）/ 惯性视线率 λ̇（终点帧间差分，PN 用，当前 `#if 0`） |
| `vins_out` | vision_ins.c | EKF 输出（p_world/v_world/range_m/vc/locked） |
| `gamma_pitch_fwd_deg` / `gamma_pitch_deg` | IMU.c | 弹道角γ姿态前向估计°(不漂，常用) / 速度积分版°(已停用，保留) |
| `Vel_Reanchor_Flag` / `imu_is_static` | IMU.c | 俯冲入段锚定请求位 / ZUPT 静止标志 |
| `pitch_dive_floor` / `closeness_s` | surface_control_task.c | 末制导俯仰俯冲下限θ_floor° / 接近度s∈[0,1] |
| `yaw_distance_gain` / `pitch_distance_gain` | surface_control_task.c | 距离增益（Vofa 可观测） |
| `surface_control_pid[2][3]` / `mahony_pid[3]` / `vel_pursuit_pid[2]` | pid.c | PID 实例（含速度方向外环） |
| `temp[3]` | pid.c | 外环→内环中转（Vofa 观测） |
| `DART_TYPE` | surface_control_task.c | `VECTOR_NOZZLE`(X翼,激活) / `FIXED_WING`(飞翼,#if0 不编译) |

---

## 11. 已知问题 / 陷阱（答题时警惕；详见 PROGRESS「当前 TODO」）

- **IMU/Control 同 Idle 优先级无同步** → 控制可能用上一拍姿态。
- **GYR_KF_R=1000（有意）**：用户实测 R=30 欠滤波（滤不动），关键是 Q:R 比例；与早期文档"30"不同，以代码为准。
- **D 项用误差微分（非测量微分）**：pid.c:188 测量微分被注释，:189 用误差微分 `d*(e_now-e_last)/dt`。PROGRESS 06-10 记的"对测量微分"已不成立。当前视觉目标已用 LPF 平滑（非阶跃），微分冲击被源头消解，暂无抖动。
- **angle_wrap 未启用**：ROLL/YAW 外环 `angle_wrap=0`，函数保留但未开。跨 ±180° 边界时可能出现假误差。
- 失效/未启用：MAG、PNG 主环(`#if 0`)、FFC、3D IMU 卡尔曼(`#if 0`)、旧速度卡尔曼 `Kalman_Vel_Calc`(`#if 0`，已被 vision_ins EKF 取代)、PN 超前补偿(`#if 0`)、速度矢量追踪(`vel_pursuit_mode=0`)。
- 死代码/卫生：`FIXED_WING` 整段 `#if 0`；`PWM_Init` 启了若干未配置/无 GPIO 通道（实际用的 4 路正常）；`Button.c` `Press_Long_Cnt!=0`（数组地址恒真，实际不触发越界）；`filter.c` 注释 GBK 乱码；`Target_Slew` 函数保留但主路径已改用 LPF。

## 12. 2026-06-07 本次审计已修复

- **`Alloc_Mode` 默认 0→1**（启用修好的 Mode1 `Servo_Mix_AxisLimit`，roll-only 线性正确）。
- **Mode0 `Servo_Mix_PitchPriority` k 公式修正** `(LIMIT−sgn·P)/aL` + 加 `k≤1` 上钳（原反写致 roll 二次畸变）。
- **`Guidance_Terminal` 视觉读取改用临界区快照 `v`**（原快照是死变量、实际读 live struct 与 ISR 撕裂）。
- **Vofa 第 10-12 路改发 `output_gyro_Euler[P/R/Y]`**（原误发 `output_angle_Servo[0..2]`、X翼下=舵 UL/UR/DR、看不到真正的 PID 内环输出）。
- **`surface_control_task.h` 舵机通道注释纠正**（DR/DL 尾注释原与宏名相反）。

**审计后跟进（2026-06-07 同日）**：TIM4 预分频统一为 **169**（与 TIM3 一致，时基差已消，用户改）；删除无用的 **`Servo_PWM_Limit`**（量纲统一为角度后失效，输出由上游 ±60 约束）；**视觉单位/轴映射经用户确认正确**（像素，接收时转度）。

**2026-06-08 文档维护**：清理 TODO——**加速度标定**（用户确认现状正常）、**IMU SPI 阻塞读超时**（不处理）移出已知问题；**前馈 FFC** 决定暂不启用（效果不大 / 不易验证，代码保留关闭态）；**坐标系统一 + roll 自稳**经 Vofa 确认正常（详见 [PROGRESS.md](PROGRESS.md) 「已 Vofa 确认」）。

**2026-06-23 全量审计 v2**：新增 vision_ins EKF / LADRC / ZUPT / 距离增益 / LPF 目标平滑 / 速度矢量追踪等模块文档；修正增益表（PID gains 全部过时→更新）、分配器默认值（Alloc_Prio / Alloc_B pitch 符号 / C 阵）、D 项描述（测量微分→误差微分）、angle_wrap 状态、PN_LEAD_K/LOS_RATE_LIMIT_DPS/pitch_control_limit_deg 参数值。详见 [PROGRESS.md](PROGRESS.md)「参数勘误」。

**2026-06-08 X 翼 pitch 解算几何对齐**：pitch 逻辑列对齐为四片同号 `[+1,+1,+1,+1]`（`Servo_Mix_AxisLimit` 的 C 阵 / `Alloc_B` / `Servo_Mix_PitchPriority` 三处），使 pitch 指令落到真俯仰模态、与 roll/yaw 正交解耦（X 翼几何见 §8）；SIGN 不动。pitch/yaw 调用点清零行已注释 → 三轴放开。

## 13. 2026-06-10 制导段抖动修复（D 项对测量微分 + 视觉目标斜坡）

- **症状**：目标附近抖动，主要在**末制导(视觉介入)段**；纯陀螺自稳段不明显。想给小 D 加阻尼反而抖得更厉害。
- **根因 = 微分冲击（derivative kick）**：[pid.c](imcalib/Tool/pid.c) 的 D 原本对**误差** `e=set−get` 求导。纯陀螺段目标恒定(`Stable_Euler_Angle`)，`de/dt=−d(角度)/dt` 是纯阻尼；末制导段目标每 50ms 被视觉新帧阶跃刷新([surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`)，那一拍 `dout=d·Δ/dt=d·Δ·1000` 是脉冲尖刺、20Hz 周期激励 → 抖；Δ 越小 d 也救不回(d=0.005、Δ=0.5° 仍出 2.5 脉冲)。
- **方案1（根治，pid.c）**：D 改对**测量微分** `−d·(get[NOW]−get[LAST])/dt`，只对反馈量求导 → 目标阶跃不再进 D、纯阻尼；不经死区软化、死区内仍阻尼；零延迟非低通。
- **方案3（视觉源，surface_control_task .h/.c）**：setpoint 端给锁存目标加**斜坡**(速率限制)。新增 `vision_los_final[3]`(世界系视线终点)、`Target_Slew`(差值含 YAW 角度环绕)；`Guidance_Terminal` 改为「视觉帧更新终点 + 每 tick `target` 斜坡逼近终点」，把 50ms 阶跃摊平。宏 `VISION_TARGET_SLEW_DPS`=150°/s 可台架调(大→跟手、小→平滑滞后)。与原航位推算等价、仅消阶跃。*(2026-06-14：固定速率斜坡已升级为「比例化逼近」+ PN 视线率超前补偿，`VISION_TARGET_SLEW_DPS` 废弃，见 §9。)*
- **未编译**(Keil 工程，需在 MDK 里编)。两项均合"输入端/源头解决、不在反馈环加低通"偏好。

---
*维护约定：本文与 PROGRESS.md / memory `control-tuning-progress` 三方同步；改代码后更新对应小节。*
