# Dart_2027 飞控 — 代码总览 / 项目地图

> 给 Claude（及任何接手者）**答题前先读这一篇**就能掌握整个项目的速查地图。
> 进度与决策见 [PROGRESS.md](PROGRESS.md)；跨 AI 工作规则/约束见 [AGENTS.md](AGENTS.md)。（旧 `plan-flight-control-overhaul.md` / `plan-pitch-priority-mixing.md` 已删除，内容并入 PROGRESS 时间线 + git 历史。）
> 本文基于 **2026-06-27 复审** 整理，力求与代码一致；改代码后请同步本文。
>
> ⚠️ **高频调参期**：PID 增益/分配档/门控几乎每个 commit 都在变。本文中所有具体数值（增益、阈值、默认档位）均为**某时刻快照、可能已过时**——一律**以代码为准**（pid.c `pid_init`、surface_control_task.c/.h）。本文价值在**结构、数据流、坐标系、陷阱**这些不常变的骨架。

---

## 0. 一句话定义

STM32G431 + BMX055/BMI088(可切换) + FreeRTOS 的 **Dart 飞镖型飞行器飞控**：**LQI 力矩控制（`lqi_mode=1` 恒激活）→ 力矩→舵角分配器（torque_allocator）→ X 翼 4 舵面**，Mahony 姿态融合，视觉/IMU 紧耦合 6 态 EKF（vision_ins.c）给不漂的速度，视觉（OpenMV）经 UART 给视线角+距离+面积。比赛镖（RoboMaster 飞镖）场景。IMU 芯片通过 `common_defs.h` 的 `USE_BMX055`/`USE_BMI088` 宏切换（acc 初始化/读取/敏感度自动适配，gyr 共用）。

> **2026-08-11 主链路收敛**：控制 = **LQI 唯一激活**（9 态→3 轴力矩 N·m）+ `lqi_alloc_mode=0` 简单伪逆分配。**LQR（lqr_tool/）、ADRC/LADRC（Tool/adrc.c）、PID 串级（Euler_pid_Cale）、Servo_Mix_* 分配器、pitch_glide 滑翔、末端锁定 TermLock 均已弃用/未实现，代码留存仅供对照**——文档下方这些旧描述均已标注。详见过时项清单。**以代码为准。**

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
│   ├── pid.c/.h               PID（位置/增量）+ 死区软化 + 角度环绕 + 前馈FFC——**弃用留存**（Euler_pid_Cale 无调用；mahony_pid 仍被 IMU 用）
│   ├── adrc.c/.h              LADRC 线性自抗扰（单环二阶 LESO+LSEF）——**弃用留存**
│   ├── filter.c/.h            标量卡尔曼(激活) + 2D卡尔曼(速度EKF已#if0禁用) + CMSIS矩阵宏别名
│   ├── vision_ins.c/.h        视觉/IMU 紧耦合 6 态 EKF(世界系 p,v)，给不漂的速度+距离估计
│   ├── vision_ekf.c/.h          6态 bearing-only 非线性EKF(位置+速度，姿态借用Mahony，不用距离)，与 vision_ins 并行，ekf_mode 切换
│   ├── ADC_Battery.c/.h       电池电压 ADC（DMA + 标量卡尔曼）
│   └── Vofa_send.c/.h         Vofa+ 上位机 2/4/8/16/24/32 通道 float 发送
├── lqr_tool/                  **弃用留存**：LQR 姿态控制器 + MATLAB Coder 生成 K 表（LQR_K_Dart_d*）——不再使用
├── lqi_tool/                  **当前激活链路**：
│   ├── lqi_torque.c/.h         LQI 力矩控制器（9态→3轴力矩 N·m；lqi_mode=1 恒激活）★
│   ├── torque_allocator.c/.h  舵面分配器（Torque_Allocate_Simple 简单 pinv 当前用；Pitch 保护未启用）★
│   ├── lqi_gain_table.h       K_lqi[3][9]（MATLAB dlqr 导出，与速度无关）
│   └── lqi_geometry_table.h   H_tau_Vref[3][4] + 零空间 N_ry（舵效矩阵，占位参数待台架）
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
| IMU_Task | **Low**（2026-08-12 抬） | 512 | 1ms (`vTaskDelayUntil`) | 先 `IMU_Calibrate()`（2s 静态零偏）一次，再循环 `IMU_Task()`=读+解算 |
| Total_Control_Task | Idle | 512 | 1ms (`vTaskDelayUntil`) | `TotalControl()`=Vofa+`surface_control_task()`+视觉遥测+ADC |

> ⚠️ **2026-08-12 已修复（时序变更，待台架回归）**：原 IMU 与 Control 同为 osPriorityIdle、无同步、控制环可能用**上一拍**姿态（启动顺序随机 0/1 拍滞后）。现 IMU_Task 优先级抬到 **osPriorityLow**（高于 Control/Idle），每 tick IMU 先跑、Control 后用最新姿态，0 滞后确定性化。初始化顺序仍靠 Init_Task(Normal) 先跑完。

### 初始化序列 `TotalInitTask()`（`Init_Config.c`）
`ALL_CS_Free` → 启 TIM6/7 中断 → `PWM_Init()`(启 TIM2/3/4 PWM) → `pid_init()` → `Q[NOW][0]=1` → 三路 UART 空闲DMA接收(huart1半双工/huart2调试/huart3视觉，关半传输中断) → `ADC_Init()` → `IMU_Init()`(BMX055或BMI088,按 `USE_BMI088` 宏切换) → `PNG_Init()` → `Kalman_Vel_Init()` → 上电 → 目标欧拉角初值 30/30/30（随即被状态机覆盖）。

---

## 4. 数据流水线（每 1ms）

```
BMX055/BMI088(SPI2阻塞读) ─IMU_Data_Read─► IMU_Data.A/G(原始,去饱和/NaN,标量Kalman,减陀螺零偏)
                                         │
                    a_corr = a − A_Offset │(Mahony 用去偏值)
                    IMU_Attitude_Algorithm│(Mahony PI 融合 → 四元数积分 → 欧拉角)
                                         │
                    ZUPT(仅 Self_Text/Start,发射前零速) + A_World(gravity·(a_corr−R_col3))
                                         │
                    VisInsEKF_Predict(1kHz) + UpdateVision(~30Hz) + UpdateZeroVel(静止)
                                         ▼
              Surface.current_angle_Euler[NOW]  +  current_gyro_Euler[NOW]
              IMU_Data.Velocity[World/Body]  +  vins_out.range_m/vc
                                         │
   get_current_State()→Guidance_State   │   get_current_Target()→target_angle_Euler[NOW]
                                         ▼
        LQI(9态→3轴力矩, pitch误差=0只留阻尼, yaw带积分) → lqi_ctrl.torque_cmd_Nm[3]
                                         │  (lqi_mode=1 恒激活; LQR 分支弃用)
                                         ▼
        torque_allocator(Torque_Allocate_Simple) → output_angle_Servo[NOW][0..3] (度,±60)
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
Self_Text_OK ─► Start ──(Euler[PITCH]∈Shot_Pitch±5 且 Euler[ROLL]∈Shot_Roll±6，连续50)──► 对准检测(Guidance_flag[1])
    │                                                                                     │ V_DART_Lqi≥1.5 且 Body[Y]>0.3 且 |Body[X]|<0.5
    ▼                                                                                     ▼
Stable(自稳,锁 Stable_Euler_Angle) ──(Euler[PITCH]≤Shot_Pitch−10 且 视觉识别成功,连续5)──► Terminal(视觉制导)
                                                                                            │ V_DART_Lqi<4 或 A[Y]<−0.8,连续50
                                                                                            ▼
                                                                    End(停录,2000拍) ──► PROCESS_OK(延时断电5000拍)
```
- **舵机仅在** `Guidance_State==Terminal || ==Self_Text_State || Stable_Flag==1` **时驱动**，否则回中(0)。
- `End` 阶段发送视觉停录命令后进入 `PROCESS_OK`；断电必须走 `Guidance_Process_OK()` 的延时路径，不能在状态切换后的下一拍立即 `Power_OFF`，否则视觉端来不及关闭文件，SD 卡上的日志/视频可能保持 0KB。
- `Stable_Flag` 在 `get_current_Target()` 的 Stable 分支置 1（每拍刷新）。
- `Self_Text_State` 下四舵置 30°（自检摆舵）；`Start` 下置 0。
- 各阶段目标：Start/Stable → ROLL/YAW 锁 `Stable_Euler_Angle`、PITCH=当前（只阻尼）；Terminal → ROLL 自稳，YAW/PITCH 视觉视线锁存（见 §9），pitch 目标在 LQI 里误差恒 0（方案A，不主动追）。

---

## 7. 串级 PID（`pid.c` / `pid_init`，**已弃用留存 2026-08-11**）

> **现状**：`Euler_pid_Cale` 在 `surface_control_task()` **无调用点**（已由 LQI 取代）；`mahony_pid[3]` 仍被 IMU.c 的 Mahony 用（必须保留）。以下为历史结构记录，供对照。下表为 **2026-06-27 快照，⚠️ 以 [pid.c](imcalib/Tool/pid.c) `pid_init` 为准**：

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

### LADRC 线性自抗扰（`adrc.c/.h`，**已弃用留存 2026-08-11**）

~~`ladrc_mode` 运行时切档~~——**不再使用**。`Init_Config.c` 的 `LADRC_Init_All()` 已注释；`Euler_LADRC_Cale` 无调用点。代码留存仅供对照。原方案：单环二阶 LADRC = 三阶 LESO + LSEF + 扰动补偿，wc/wo/b0 三旋钮，阻尼用实测陀螺。

### LQR 状态反馈（`lqr.c/.h`，**已弃用留存 2026-08-11**）

**不再使用**。`lqr_mode` 从未定义（仅 .h extern 已注释）；`surface_control_task()` 调用点是 `if (lqi_mode==1) Euler_LQI_Cale else Euler_LQR_Cale`，`lqi_mode` 恒 1 → 恒走 LQI。`LQR_Init()` 已从 Init_Config.c 注释；LQR 代码（含 MATLAB Coder 生成的 `LQR_K_Dart_d` 系列）留存仅供对照。原方案：`u=-K_d·x` 6 态→4 舵一步含混控。

**与 PID/LADRC 的本质区别**：PID/LADRC 是两步（先算三轴力矩 `output_gyro_Euler` → 再过 `Servo_Mix_*` 混控）；LQR 的 `K_d[4][6]` 已把 X 翼混控几何 G 烘焙进模型，**一步替代 PID+混控两步**，`Euler_LQR_Cale` 直接写 `output_angle_Servo[NOW][...]`、绕过 `output_gyro_Euler` 与 `Servo_Mix_*`（调用点已加 `lqr_mode!=1` 跳过混控）。

**K 矩阵** = [lqr.c](imcalib/Tool/lqr.c) 的 `dart_lqr_K[4][6]`（MATLAB 同名同形「粘贴区」）：在 [lqr_czn/dart_attitude_LQR_v1.m](lqr_czn/dart_attitude_LQR_v1(1)(1).m) 调好 Q/R/惯量/速度，跑 Step5 把打印的 4 行整块覆盖粘贴即可。⚠ MATLAB 舵号(delta1=右上,2=左上,3=左下,4=右下) ≠ 工程索引(UL/UR/DR/DL)，在 `K_ROW_TO_SERVO[]` 换序；上车前必按移植指南 §9 逐轴阶跃验符号（飞镖一次性，G 符号反=正反馈）。详见 [lqr.c](imcalib/Tool/lqr.c) 头注 + [lqr_czn/MCU_LQR_PORTING_GUIDE(1).md](lqr_czn/MCU_LQR_PORTING_GUIDE(1).md)。

> 未编译：新增 `lqr.c` 需手动加入 Keil/eIDE 工程编译列表（AI 编不了）。

### LQI 力矩控制器 + 舵面分配（`lqi_torque.c/.h` + `torque_allocator.c/.h`，**当前唯一激活链路**）

**与旧 PID/LQR 的本质区别**：拆成两步——先 LQI 输出 3 轴物理力矩 N·m，再分配器把力矩翻译成 4 舵角（`torque_allocator`）。

`lqi_mode`（lqi_torque.c 定义）**恒 = 1**：LQI 力矩控制 + 分配（surface_control_task.c 调用点）。`lqi_alloc_mode = 0`（**简单伪逆 pinv(H_tau) 全轴最小舵量**；`1`=零空间 Pitch 保护，未用）。`lqi_mode=0` 的 else 分支 `Euler_LQR_Cale` 为弃用留存。

**状态与输出**：
- 状态 `xa[9]` = `[e_roll, e_pitch, e_yaw, p, q, r, ∫e_r, ∫e_p, ∫e_y]`（rad, rad/s, rad·s）
- **pitch 方案A（2026-08-11）**：`e_pitch ≡ 0`（依托初始动力、不追目标角度），只保留 pitch 角速度阻尼 `q`；pitch 原 `q/=cnt` 阻尼削减残留已删（俯冲段裸奔发散根因）
- LQI 输出 `tau[3]` = `[Mx, My, Mz]`（N·m）
- K_lqi[3][9] 由 MATLAB dlqr 生成（[matlab_script/](matlab_script/)目录）
- 舵面顺序全线统一为 `[UL, UR, DR, DL]`，C 端不再换序

**积分（2026-08-11 修复，当前仅 YAW 有效）**：
- 阈值 `LQI_INTEG_THRESHOLD_RAD` = **0.5°**（原 `0.008726646*3` 实际 1.5°，注释写 0.5° → 积分被分离闸门每拍清零、永远积不起来）
- YAW 积分限幅 `LQI_INTEG_LIMIT_YAW` = **5°·s**（原 2° clamp 死）
- `Euler_LQI_Cale` §3.5 积分门控：ROLL/PITCH 恒清零冻结；YAW 非 Terminal 段冻结、Terminal 段放行。LQI_Update 内积分分离 + 限幅接管。

**H_tau 力矩矩阵**（3×4，N·m/rad）：`H_tau = Vs · H_tau_Vref`。**Vs 固定 = 6（2026-08-11）**——因 EKF 速度不准（方向已验证、幅度不准），固定 Vs 使舵效可预测；⚠ Vs=6 ≠ "速度 6"（真按 V=6 应为 (6/6)²=1），是把 H_tau 整体放大 6 倍 = 等效舵效标定系数，**待台架**。速度平方调度保留 `#if 0`（EKF 速度验证准后启用：先低通再平方）。真实飞行速度 **6~10 m/s**（用户 2026-08-11）。

**分配器 `torque_allocator.c`**：
- `Torque_Allocate_Simple`（当前）：pinv 整个 3×4 H_tau，三轴力矩全满足、最小舵量
- `Torque_Allocate_PitchProtected`（`lqi_alloc_mode=1` 未用）：先满足 Roll/Yaw → 零空间压低 Pitch → 限幅
- 舵面限幅 ±60°、饱和/不可达 → 冻结积分

**关键文件**：
- MATLAB：[matlab_script/dart_lqi_parameters.m](matlab_script/dart_lqi_parameters.m)、[dart_attitude_lqi_torque_pitch_protected.m](matlab_script/dart_attitude_lqi_torque_pitch_protected.m)、[dart_lqi_export_c.m](matlab_script/dart_lqi_export_c.m)
- C：[lqi_torque.c](imcalib/lqi_tool/lqi_torque.c)/[.h](imcalib/lqi_tool/lqi_torque.h)、[torque_allocator.c](imcalib/lqi_tool/torque_allocator.c)/[.h](imcalib/lqi_tool/torque_allocator.h)
- 表：[lqi_gain_table.h](imcalib/lqi_tool/lqi_gain_table.h)（K_lqi）、[lqi_geometry_table.h](imcalib/lqi_tool/lqi_geometry_table.h)（H_tau + 零空间）

**⚠ 占位符清单（需 SolidWorks/CFD/台架数据后重新生成所有表）**：
- 交叉惯量 Ixy/Ixz/Iyz（当前 = 0）
- 舵面位置 r_i（当前从 r_ac=0.150/a_ac=0.120 反推）
- 舵面面积 S_i（当前 0.005 m²）
- 舵效系数 C_Fδ（当前 5.0，占位 ≈ CLα）
- 气动阻尼/恢复系数（当前全 0，未启用）
- **Vs=6 实际舵效**（待台架确认放大倍数）

> 编译：eIDE 工程（`.eide/eide.yml` srcDirs 含 `imcalib/lqi_tool`）**已含 LQI 文件**；MDK-ARM/*.uvprojx 是旧版**不含** lqi/adrc/lqr——**别用 MDK 编译**。

---

## 8. 混控 / 舵面分配（**当前 = LQI → torque_allocator，旧 Servo_Mix_* 体系已不存在**）

> **2026-08-11 关键修正**：文档此前描述的 `Alloc.Mode` 三档分配器（`Servo_Mix_AxisLimit/MinEnergy`）、`Roll_Derotate_PitchYaw`、`Alloc` 全局——**在代码中只有 .h 声明、无任何 .c 定义/调用**（已全部注释清理）。当前分配 = `Euler_LQI_Cale` 内 `torque_allocator.c` 的 `Torque_Allocate_Simple`（`lqi_alloc_mode=0`），LQR 分支已弃用。

**X 翼几何背景（供分配器 H_tau 理解）**：4 片成 X(45°，从尾看 UL135°/UR45°/DR315°/DL225°)，每片偏转产生切向力，对力矩贡献 **pitch∝cosθ=[UL−,UR+,DR+,DL−]、yaw∝sinθ=[+,+,−,−]、roll=常数**（三模态两两正交）。物理装配符号 `SIGN=[UL−1, UR+1, DR+1, DL−1]`（左右舵镜像安装，台架标定）。此几何已烘焙进 MATLAB 生成的 `H_tau_Vref` 表，不再由 C 端逻辑阵显式表达。

**分配链路**：LQI 输出三轴力矩 `tau[3]` → `Torque_Allocate_Simple`（pinv(H_tau)×tau → 4 舵 rad → 转度 ±60 限幅）→ `Surface.output_angle_Servo[NOW][UL/UR/DR/DL]` → `Wing_Control_VECTOR_NOZZLE` 写 PWM。`lqi_alloc_mode=1` 的零空间 Pitch 保护分配器已实现但**未启用**。

**Vofa 观测（`lqi_ctrl` 结构体）**：`attitude_error_rad[0..2]`、`torque_achieved_Nm[0..2]`、`servo_cmd_deg[0..3]`、`torque_angle/rate/integral_Nm`、`servo_sat_mask`、`allocator_infeasible`、`pitch_moment_Nm`。

### ⭐ 当前"实际在飞什么"（2026-08-11 快照，以代码为准）
Stable/Terminal 段且 `imu_is_static==0` → **LQI 力矩控制**（`lqi_mode=1`）→ `Torque_Allocate_Simple` 分配 → 4 舵。
- **控制器**：LQI 9 态（pitch 误差恒 0、仅阻尼；yaw 制导段带积分；roll 自稳）。LQR/ADRC/PID 串级均弃用。
- **分配**：`lqi_alloc_mode=0`（简单 pinv），H_tau 用固定 `Vs=6` 缩放（待台架标舵效）。
- **末制导**：① 视线锁存（方向A）+ 航位推算（`vision_los_final`，无 LPF/距离增益/pitch_glide——均已删/弃用）；② PN 超前 `#if 0`（关闭）；③ 丢目标就地保持。

---

## 9. 视觉 / 通信（`CallBack_Task.c`）

| UART | 实例 | 对端 | 模式 |
|---|---|---|---|
| huart1 | USART1 | 镖头触发板 | **单线半双工** DMA，CRC8-MAXIM |
| huart2 | USART2 | PC / Vofa | DMA（调试+遥测） |
| huart3 | USART3 | 视觉 OpenMV | DMA 空闲事件，6 字节帧 |

- 视觉帧（均 6 字节）：`0x5A..0xA5`=识别成功（x,y 像素），`0x5B..0xA6`=距离+面积（dist_cm,area 均 uint16 大端，2026-06-12 新增，独立于识别包），`0x7A..0xA7`=丢目标，`0x9A..0xA9`=录制状态。`0x5B` 包只更新 `Vision_Rx_Data.dist_cm/area`、不置 recognize/New_Data；OpenMV `send_distance(dist_cm=DIST_K/sqrt(px), area)` 配套。
- **像素→度转换在 `Vision_Receive` 内做**：`Euler[PITCH]=y/160*36`、`Euler[YAW]=x/120*27`（**y→PITCH、x→YAW**，2026-08-11 按代码修正——旧文档写反为 x→PITCH、y→YAW）。✅ **以代码为准**：视觉 OpenMV 发的是像素（x=水平偏移、y=垂直偏移），接收时即转成度，下游 `Guidance_Terminal` 锁存按度用、量纲一致（无需再 ×FOV）。EKF 视觉量测同轴约定（`az=x_px`、`el=y_px`）。
- `Vision_New_Data_flag`：ISR(~20Hz)产生、`Guidance_Terminal`(1kHz)消费后清 0（生产者-消费者）。
- **视线锁存（方向A）+ 航位推算（2026-06-15+；LPF/距离增益 2026-06-27 已删除）**：新帧到达瞬间把世界系视线锁存到 `vision_los_final=vision_euler+current`（终点，帧间不变、航位推算）。**原一阶低通平滑 + 距离增益缩放 k 已整段删除**（`lpf_*`/`yaw_gain`/`pitch_gain`/`k_*` 死变量全清），现 `target` 直接 = `vision_los_final[NOW]`（无额外平滑）。读侧用临界区快照 `v`；丢目标(FAILURE)终点+目标对齐当前(就地保持)。*(`Target_Slew`/`Vision_Angle_Normalize`/`*_Gain_ByDist` 均已 `#if 0` 封存。)*
- **PN 视线率超前补偿（2026-06-14，当前 `Guidance_Terminal` 内 `#if 0` 关闭）**：锁存的世界系视线终点帧间差分得惯性视线率 λ̇（`vision_los_rate[]`，纯视觉、不依赖会漂的 IMU 积分速度）。**拆分接口** `PNG_Apply_Lead_Yaw/Pitch`（[PNG_Task.c](imcalib/Task/PNG_Task.c)）支持分轴门控（俯冲未到位只喂 yaw）；`PNG_Mode=1` 改用 **vision_ins EKF 世界系 p/v 叉乘直接算 LOS 率** ω_yaw/ω_pitch（替代纯帧差分），`PNG_Mode=0` 仍走 `vision_los_rate`。`LOS_RATE_LIMIT_DPS`=40、`PNG_LEAD_LIMIT_DEG`=8 限幅。**当前关闭，先用纯 PID 跟踪验证基础性能，标定好 `K_Dyn` 后再打开。**
- **末制导 pitch（2026-08-11 收敛为 LQI 方案A，glide/门限均弃用）**：~~pitch 主动滑翔→扎 `pitch_glide_mode`~~ **代码中无此实现**（`pitch_glide_mode/target/blend` 为悬空 extern，已注释；`GLIDE_*`/`THETA_GLIDE_*` 宏未使用）。现行：LQI 里 **pitch 误差恒 0、只保留角速度阻尼**（依托初始动力、不追目标角度），pitch 增程靠镖架初始动力 + 气动滑翔自然实现，不做主动俯仰姿态调度。~~`Pitch_Dive_Floor`/`pitch_control_limit_deg` 门限~~ 同样弃用（`#if 0` 封存，符号已注释）。
- **视线角半径归一化（2026-06-15+，⚠️ 2026-06-27 已 `#if 0` 封存、调用已删）**：`Vision_Angle_Normalize(angle, radius)` 把视线角按 blob 半径归一化到 `REF_RADIUS`(15px)，消除远近 blob 大小差异。当前主路径不再调用。
- **距离增益调度（2026-06-15+，⚠️ 2026-06-27 已 `#if 0` 封存）**：`Yaw_Gain_ByDist(dist_cm)` / `Pitch_Gain_ByDist(dist_cm)` 纯距离线性插值增益（YAW 远大近小、PITCH 反向）。随 LPF 删除一并停用、不再被调用。
- **（历史/底层，已弃用，⚠️ 2026-06-27 `#if 0` 封存）末制导俯仰能量管理 `Pitch_Dive_Floor`（2026-06-12）**：随接近度放开的最陡俯冲限幅 `θ_floor=max(L_sched(s), γ−AOA_MARGIN)`。已被 2026-08-11 的 LQI 方案A（pitch 误差恒 0、不主动调度俯仰）取代。物理意图参考：远保射程、近放开到入射角、终端迎角→0 正向撞击。
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
| `lqi_mode`（**恒=1 激活**）/ `lqi_alloc_mode`（**0 简单pinv**）/ `lqi_ctrl` | lqi_torque.c | LQI 力矩控制档位 / 分配器档位 / 控制器观测仓（attitude_error_rad、torque_*_Nm、servo_cmd_deg、sat/infeasible 等） |
| `V_DART_Lqi` / `lqi_ctrl.cached_V` | lqi_torque.c | EKF 速度缓存（50Hz，H_tau 用；当前 Vs 固定，未参与） |
| ~~`ladrc_mode` / `ladrc_ctrl[3]`~~ | ~~surface_control_task.c / adrc.c~~ | **已弃用**（2026-08-11）：LADRC 切档无定义、无调用；`ladrc_ctrl` 留存供对照 |
| ~~`lqr_mode` / `lqr_ctrl` / `dart_lqr_K`~~ | ~~surface_control_task.c / lqr.c~~ | **已弃用**（2026-08-11）：`lqr_mode` 从未定义、extern 已注释；LQR 代码留存 |
| ~~`Alloc`（`Alloc_t`）~~ | ~~surface_control_task.c~~ | **已弃用**：悬空声明已注释，无定义无使用（分配器 = torque_allocator） |
| `Vision_Rx_Data` | CallBack_Task.c | 视觉接收（ISR 写、控制读，含 New_Data_flag/dist_cm/area/radius/Euler_norm） |
| `vision_los_final[2][3]` / `vision_los_rate[3]` | surface_control_task.c | 末制导世界系视线终点（视觉新帧锁存，`target` 直接取用、无额外平滑）/ 惯性视线率 λ̇（终点帧间差分，PN 用，当前 `#if 0`） |
| `vins_out` | vision_ins.c | EKF 输出（p_world/v_world/range_m/vc/locked） |
| `ekf_out` | vision_ekf.c | EKF 输出（同构 VinsOut_t，ekf_mode=1 时替代 vins_out） |
| `ekf_mode` | IMU.c | 速度估计切换：0=旧6态KF(vision_ins), 1=新6态bearing-only非线性EKF(不用距离) |
| `gamma_pitch_fwd_deg` / `gamma_pitch_deg` | IMU.c | 弹道角γ姿态前向估计°(不漂，常用) / 速度积分版°(已停用，保留) |
| `Vel_Reanchor_Flag` / `imu_is_static` | IMU.c | 俯冲入段锚定请求位 / ZUPT 静止标志 |
| ~~`pitch_dive_floor` / `closeness_s`~~ | surface_control_task.c | 旧 `Pitch_Dive_Floor` 的 Vofa 观测，已 `#if 0` 封存（定义已注释） |
| ~~`pitch_glide_mode` / `pitch_glide_target` / `pitch_glide_blend`~~ | surface_control_task.c | **已弃用**（2026-08-11）：悬空 extern 已注释，无实现（pitch 走 LQI 方案A） |
| ~~`yaw_distance_gain` / `pitch_distance_gain`~~ | surface_control_task.c | 旧距离增益 Vofa 观测，已 `#if 0` 封存（定义已注释） |
| `surface_control_pid[2][3]` / `mahony_pid[3]` / `vel_pursuit_pid[2]` | pid.c | PID 实例（含速度方向外环） |
| `temp[3]` | pid.c | 外环→内环中转（Vofa 观测） |
| `DART_TYPE` | surface_control_task.c | `VECTOR_NOZZLE`(X翼,激活) / `FIXED_WING`(飞翼,#if0 不编译) |

---

## 11. 已知问题 / 陷阱（答题时警惕；详见 PROGRESS「当前 TODO」）

- ~~**IMU/Control 同 Idle 优先级无同步**~~ → **2026-08-12 已修复**（IMU 抬到 osPriorityLow，见 §3 注），待台架回归。
- **NaN/Inf 防线（2026-08-12 加固，正常路径行为不变）**：① EKF 速度回写（IMU.c）非有限→全轴置 0；② 分配器入口（torque_allocator.c）力矩非有限→不可达+回中；③ `abs_limit`（pid.c）非有限→归 0（防 NaN 穿透到定时器）；④ `vis_dt_cnt` 改 uint32 防 65.5s 回绕。分配器 `Torque_Allocate_Simple` 死计算 `iS*` 已删（A1）。
- **GYR_KF_R / ACC_KF_R = 10（IMU.h，以代码为准）**：旧文档记 1000 已过时。R=10 为当前调参值，Q:R 比例决定滤波；调参以代码为准。
- **视觉轴映射**：**x→YAW、y→PITCH**（`Euler[0]=pitch=y/160*36`、`Euler[1]=yaw=x/120*27`）——2026-08-11 按代码修正，旧文档写反。
- **EKF 速度幅度不准（方向已验证）**：H_tau 动压调度因此固定 `Vs=6`（舵效系数，待台架）；真实速度 6~10 m/s。
- **LQI 积分仅 YAW 有效**：阈值 0.5°、限幅 5°·s（2026-08-11 修正，原 1.5°/2° 导致积分积不起来）。
- **angle_wrap 未启用**：LQI 里 yaw 误差恒环绕（`Angle_Wrap_180`）；roll 按 `lqi_ctrl.roll_wrap`（默认 0）。
- **Pitch 方案A**：LQI 里 pitch 误差恒 0、只留角速度阻尼——依托初始动力，不追目标角度。
- **加速度零偏新公式（2026-08-20）**：`A_Offset = mean(a) − g_ref_body`（发射架已知姿态 `Shot_Pitch/Roll` 构造理论重力），取代旧 `mean(a) − R_col3`（依赖含零偏的 Mahony 输出）。Mahony 始终用去偏加速度 `a_corr` 做归一化、ZUPT 仅在 `Self_Text||Start` 时允许、零偏标定后立即冻结不在线 refine。待台架验证。
- 失效/未启用（2026-08-11 现状）：MAG、PNG 主环(`#if 0`)、PN 超前补偿(`#if 0`)、LQR(弃用)、LADRC/ADRC(弃用)、PID 串级 `Euler_pid_Cale`(无调用)、Servo_Mix_* 分配器(无定义)、`vel_pursuit_mode`(悬空已注释)、欧拉运动学变换(注释)。
- 死代码/卫生：`FIXED_WING` 整段 `#if 0`；`PWM_Init` 启了若干未配置/无 GPIO 通道（实际用的 4 路正常）；`Button.c` `Press_Long_Cnt!=0`（数组地址恒真，实际不触发越界）；`filter.c` 注释 GBK 乱码；~~`TermLock`（末端锁定）死代码~~ **已删除（2026-08-12）**；悬空声明已从 .h 注释清理。

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
