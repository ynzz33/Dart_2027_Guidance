# Dart_2027 飞控 — 代码总览 / 项目地图

> 给 Claude（及任何接手者）**答题前先读这一篇**就能掌握整个项目的速查地图。
> 进度与决策见 [PROGRESS.md](PROGRESS.md)；详细技术方案见 [plan-flight-control-overhaul.md](plan-flight-control-overhaul.md) / [plan-pitch-priority-mixing.md](plan-pitch-priority-mixing.md)。
> 本文基于 **2026-06-07 全量通读源码 + 工程文件**整理，力求与代码一致；改代码后请同步本文。

---

## 0. 一句话定义

STM32G431 + BMX055 + FreeRTOS 的 **Dart 飞镖型飞行器飞控**：X 翼 4 舵面，串级 PID（角度环→角速度环）自稳 + 末制导，Mahony 姿态融合，视觉（OpenMV）经 UART 给视线角。比赛镖（RoboMaster 飞镖）场景。

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
│   ├── pid.c/.h               PID（位置/增量）+ 死区软化 + 角度环绕 + 前馈FFC（FFC 当前关）
│   ├── filter.c/.h            标量卡尔曼(激活) + 2D/3D卡尔曼(未用) + 低通 + CMSIS矩阵宏别名
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
                                         ▼
              Surface.current_angle_Euler[NOW]  +  current_gyro_Euler[NOW]
                                         │
   get_current_State()→Guidance_State   │   get_current_Target()→target_angle_Euler[NOW]
                                         ▼
              Euler_pid_Cale()  外环角度PID→temp[]→内环角速度PID→ output_gyro_Euler[NOW]
                                         │  (p_body,y_body 当前掐死=0；r_body=roll，Terminal时=0)
                          Roll_Derotate_PitchYaw (当前被旁路)
                                         ▼
        Servo_Mix_*(Alloc_Mode 分派) → output_angle_Servo[NOW][0..3] (度,含SIGN,±60)
                                         ▼
        Wing_Control_VECTOR_NOZZLE → Wing_UL/UR/DL/DR_Control → Finally_Angle(µs) → __HAL_TIM_SET_COMPARE
```

---

## 5. 坐标系与轴向约定（**极易混淆，答题必看**）

- **机体系 = 右手 ENU**：X=右/东、Y=前/北、Z=上/天。
- **欧拉角极性**：PITCH 绕 X 抬头为正；ROLL 绕 Y 右滚为正；YAW 绕 Z 右偏为正。
  - `Euler[PITCH]=asin(...)`∈[-90,90]（不环绕）；`Euler[ROLL]/[YAW]=atan2(...)`∈[-180,180]（**周期量，外环开 angle_wrap**）。
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

每轴两环：`surface_control_pid[Angle][axis]`（外环角度）→ `surface_control_pid[Gyro][axis]`（内环角速度）。当前激活的是「镖体1」一组（镖体2 注释着）：

| 轴 | 外环(Angle) P/I/D | 内环(Gyro) P/I/D | 死区 |
|---|---|---|---|
| PITCH | 0.1 / 0 / 0 | 0.08 / 0 / 0 | 外5° 内5° |
| ROLL | 0.2 / 0 / 0（wrap） | 0.1 / **0.05** / 0 | 0 |
| YAW | 3.1 / 0 / 0（wrap） | 1.05 / 0 / **0.1** | 内2° |

- **死区软化** `Deadband_Soften`：误差落入死区连续归零（C0 连续，无硬切断尖刺）。
- **角度环绕** `Angle_Wrap_180`：仅 ROLL/YAW **外环**开（周期角防 ±180 假跳变）。
- **前馈 FFC**：结构在（`xFeedForward` 已内嵌、空指针 bug 已修），但 `Euler_pid_Cale` 里调用被注释 + `num1/num2=0` → **当前不生效**。**2026-06-08 决定暂不启用前馈**（效果不大、不易验证是否有效），代码保留备用。
- **D 项对测量微分**（derivative on measurement，2026-06-10）：`pid_calc` 的 D = `−d·(get[NOW]−get[LAST])/dt`，只对反馈量求导、不对误差求导 → 目标阶跃(视觉~20Hz 锁存)不再进 D，消除微分冲击 kick（「制导段一加 D 就抖、纯陀螺自稳段不抖」的根因，见 §13）；D 不经死区软化，死区内仍提供阻尼。零延迟、非低通，合"不在反馈环加低通"偏好。原计划 Phase4.1 的 D 项低通因此暂不需要。
- `max_err` 字段全为 0 → 该保护从不触发（依赖 `MaxOutput` 限幅）。

---

## 8. 混控 / 控制分配（`surface_control_task.c`，**当前重点**）

X 翼逻辑符号阵（enum 列序 UL,UR,DR,DL）：**pitch `[+1,+1,+1,+1]`**、roll `[+1,−1,−1,+1]`、yaw `[−1,+1,−1,+1]`。物理装配符号 `SIGN=[UL−1, UR+1, DR+1, DL−1]`（左右舵镜像安装；台架单轴阶跃标定，某片整体反了翻它的号；SIGN 每片三轴共享，轴间配对结构由逻辑阵的列决定、不靠 SIGN）。

**X 翼解算几何（pitch 为何四片同号）**：4 片成 X(45°，从尾看 UL135°/UR45°/DR315°/DL225°)，每片偏转产生切向力，对力矩贡献 **pitch∝cosθ=[UL−,UR+,DR+,DL−]、yaw∝sinθ=[+,+,−,−]、roll=常数**（三模态两两正交，第 4 模态 `[+,−,+,−]` 隔片交替=零空间纯阻力）。故 pitch 列四片同号，配 SIGN 后 pitch 指令落到舵令 `u=[−,+,+,−]`=真俯仰、与 roll/yaw 解耦。*(2026-06-08 校正：原 pitch 列 `[+1,+1,−1,−1]` 经 SIGN 落零空间、只减速不俯仰。)*

**`Alloc_Mode` 运行时三档分配器**（调试器/初值切换；**默认现为 1**）：

| Mode | 函数 | 说明 |
|---|---|---|
| 0 | `Servo_Mix_PitchPriority` | pitch 优先启发式饱和缩横侧。k 公式**已于 2026-06-07 修正**（原反写成 `aL/(LIMIT−P)` 致 roll 二次畸变）+ 加 `k≤1` 上钳。pitch_limit=15。作对照。 |
| **1（默认）** | `Servo_Mix_AxisLimit` | 各轴前置限幅(P40/R40/Y60)→**逐级(字典序)优先级缩放**（`Alloc_Prio` 默认 {PITCH,YAW,ROLL}，调试器在线改）→×SIGN→兜底限幅60。高优先轴不被低优先轴污染。 |
| 2 | `Servo_Mix_MinEnergy` | 真·带约束最小能量：CMSIS 伪逆 `u0=Bᵀ(BBᵀ)⁻¹v` → 零空间投影进限幅盒 → 不可达按 pitch>yaw>roll 二分缩。奇异退回 Mode1。舵效阵 `Alloc_B` 默认理想阵、可台架辨识替换。 |

- **Roll_Derotate_PitchYaw**：把世界系 pitch/yaw 力矩按当前 roll 反旋到机体系（`Δ=current_roll−Stable_roll`）。**当前因 p_body=y_body=0 被旁路**。
- Vofa 观测量：`servo_lat_scale`(最低优先轴保留比 k)、`alloc_u0/alloc_u_out`、`alloc_alpha/v_scale/p_scale`、`alloc_infeasible/singular_flag`。

### ⭐ 当前"实际在飞什么"（极重要）
`surface_control_task.c` 调用点 `p_body/y_body/r_body` 清零行**已注释 → 三轴全部放开**：pitch/yaw 经 `Roll_Derotate_PitchYaw` 反旋、roll 直通，全接 PID 内环输出，按 `Alloc_Mode`(默认1) 分派；Stable 与 Terminal 都打舵。
- `Guidance_Terminal` 末尾仍**无条件把 pitch/yaw/roll 目标盖成 `Stable_Euler_Angle`**（覆盖前面视觉锁存写入的目标）→ Terminal 实际是"稳到 Stable 姿态"，**视觉制导段当前为死代码**（疑调试态；要让视觉生效需去掉那三行覆盖）。
- 配合 2026-06-08 pitch 解算几何对齐（§8），pitch 才真正产生俯仰力矩。

---

## 9. 视觉 / 通信（`CallBack_Task.c`）

| UART | 实例 | 对端 | 模式 |
|---|---|---|---|
| huart1 | USART1 | 镖头触发板 | **单线半双工** DMA，CRC8-MAXIM |
| huart2 | USART2 | PC / Vofa | DMA（调试+遥测） |
| huart3 | USART3 | 视觉 OpenMV | DMA 空闲事件，6 字节帧 |

- 视觉帧：`0x5A..0xA5`=识别成功（x,y 像素），`0x7A..0xA7`=丢目标，`0x9A..0xA9`=录制状态。
- **像素→度转换在 `Vision_Receive` 内做**：`Euler[YAW]=y/160*72`、`Euler[PITCH]=x/120*54`（x→PITCH、y→YAW）。✅ **已确认**：视觉发的是**像素**、接收时即转成度，轴映射正确，下游 `Guidance_Terminal` 锁存按度用、量纲一致（无需再 ×FOV）。
- `Vision_New_Data_flag`：ISR(~20Hz)产生、`Guidance_Terminal`(1kHz)消费后清 0（生产者-消费者）。
- **视线锁存（方向A）+ 目标斜坡（2026-06-10）**：新帧到达瞬间把世界系视线锁存到 `vision_los_final=vision_euler+current`（终点，帧间不变）；`target` 每 tick 经 `Target_Slew` 朝终点**斜坡逼近**（`VISION_TARGET_SLEW_DPS`=150°/s，setpoint 端速率限制）→ 把 20Hz 视觉的 50ms 目标阶跃摊平，消除对外环 P / D 的周期性台阶冲击。帧间终点不变、斜坡到达后 `target≡终点` → 外环误差=终点−当前，机体一转误差即减，与原航位推算等价（仅消去切换瞬间阶跃）。读侧用临界区快照 `v`；丢目标(FAILURE)终点+目标都对齐当前(斜坡 d=0，就地保持)。
- **PNG 比例导引**：`PNG_Guidance` 在 `TotalControl` 里被注释，**未接主环**；且 `Velocity[Body]` 从未算 → `V_c` 恒被钳到 `Vc_min`。

---

## 10. 关键全局变量速查

| 变量 | 定义处 | 含义 |
|---|---|---|
| `IMU_Data` | IMU.c | 全部 IMU 状态（A/G/Q/Euler/A_World/零偏/calib_done） |
| `Surface` | surface_control_task.c | 控制状态总仓（current/target/output Euler、output_angle_Servo、Finally_Angle、Stable_Euler_Angle、Guidance_cnt） |
| `Guidance_State` | surface_control_task.c | 制导状态机当前态 |
| `Alloc_Mode` / `Alloc_Prio[3]` / `Alloc_B[3][4]` | surface_control_task.c | 分配器档位 / 优先级 / 舵效阵 |
| `Vision_Rx_Data` | CallBack_Task.c | 视觉接收（ISR 写、控制读，含 New_Data_flag） |
| `vision_los_final[3]` | surface_control_task.c | 末制导世界系视线终点（视觉新帧锁存，`target` 斜坡逼近它；Vofa 可观测） |
| `surface_control_pid[2][3]` / `mahony_pid[3]` | pid.c | PID 实例 |
| `temp[3]` | pid.c | 外环→内环中转（Vofa 观测） |
| `DART_TYPE` | surface_control_task.c | `VECTOR_NOZZLE`(X翼,激活) / `FIXED_WING`(飞翼,#if0 不编译) |

---

## 11. 已知问题 / 陷阱（答题时警惕；详见 PROGRESS「当前 TODO」）

- **IMU/Control 同 Idle 优先级无同步** → 控制可能用上一拍姿态。
- **GYR_KF_R=1000（有意）**：用户实测 R=30 欠滤波（滤不动），关键是 Q:R 比例；与早期文档"30"不同，以代码为准。
- 失效/未启用：MAG、PNG 主环、FFC、速度卡尔曼、3D IMU 卡尔曼(`#if 0`)、`Kalman_Vel_Calc`。
- 死代码/卫生：`FIXED_WING` 整段 `#if 0`；`PWM_Init` 启了若干未配置/无 GPIO 通道（实际用的 4 路正常）；`Button.c` `Press_Long_Cnt!=0`（数组地址恒真，实际不触发越界）；`filter.c` 注释 GBK 乱码。

## 12. 2026-06-07 本次审计已修复

- **`Alloc_Mode` 默认 0→1**（启用修好的 Mode1 `Servo_Mix_AxisLimit`，roll-only 线性正确）。
- **Mode0 `Servo_Mix_PitchPriority` k 公式修正** `(LIMIT−sgn·P)/aL` + 加 `k≤1` 上钳（原反写致 roll 二次畸变）。
- **`Guidance_Terminal` 视觉读取改用临界区快照 `v`**（原快照是死变量、实际读 live struct 与 ISR 撕裂）。
- **Vofa 第 10-12 路改发 `output_gyro_Euler[P/R/Y]`**（原误发 `output_angle_Servo[0..2]`、X翼下=舵 UL/UR/DR、看不到真正的 PID 内环输出）。
- **`surface_control_task.h` 舵机通道注释纠正**（DR/DL 尾注释原与宏名相反）。

**审计后跟进（2026-06-07 同日）**：TIM4 预分频统一为 **169**（与 TIM3 一致，时基差已消，用户改）；删除无用的 **`Servo_PWM_Limit`**（量纲统一为角度后失效，输出由上游 ±60 约束）；**视觉单位/轴映射经用户确认正确**（像素，接收时转度）。

**2026-06-08 文档维护**：清理 TODO——**加速度标定**（用户确认现状正常）、**IMU SPI 阻塞读超时**（不处理）移出已知问题；**前馈 FFC** 决定暂不启用（效果不大 / 不易验证，代码保留关闭态）；**坐标系统一 + roll 自稳**经 Vofa 确认正常（详见 [PROGRESS.md](PROGRESS.md) 「已 Vofa 确认」）。

**2026-06-08 X 翼 pitch 解算几何对齐**：pitch 逻辑列对齐为四片同号 `[+1,+1,+1,+1]`（`Servo_Mix_AxisLimit` 的 C 阵 / `Alloc_B` / `Servo_Mix_PitchPriority` 三处），使 pitch 指令落到真俯仰模态、与 roll/yaw 正交解耦（X 翼几何见 §8）；SIGN 不动。pitch/yaw 调用点清零行已注释 → 三轴放开。

## 13. 2026-06-10 制导段抖动修复（D 项对测量微分 + 视觉目标斜坡）

- **症状**：目标附近抖动，主要在**末制导(视觉介入)段**；纯陀螺自稳段不明显。想给小 D 加阻尼反而抖得更厉害。
- **根因 = 微分冲击（derivative kick）**：[pid.c](imcalib/Tool/pid.c) 的 D 原本对**误差** `e=set−get` 求导。纯陀螺段目标恒定(`Stable_Euler_Angle`)，`de/dt=−d(角度)/dt` 是纯阻尼；末制导段目标每 50ms 被视觉新帧阶跃刷新([surface_control_task.c](imcalib/Task/surface_control_task.c) `Guidance_Terminal`)，那一拍 `dout=d·Δ/dt=d·Δ·1000` 是脉冲尖刺、20Hz 周期激励 → 抖；Δ 越小 d 也救不回(d=0.005、Δ=0.5° 仍出 2.5 脉冲)。
- **方案1（根治，pid.c）**：D 改对**测量微分** `−d·(get[NOW]−get[LAST])/dt`，只对反馈量求导 → 目标阶跃不再进 D、纯阻尼；不经死区软化、死区内仍阻尼；零延迟非低通。
- **方案3（视觉源，surface_control_task .h/.c）**：setpoint 端给锁存目标加**斜坡**(速率限制)。新增 `vision_los_final[3]`(世界系视线终点)、`Target_Slew`(差值含 YAW 角度环绕)；`Guidance_Terminal` 改为「视觉帧更新终点 + 每 tick `target` 斜坡逼近终点」，把 50ms 阶跃摊平。宏 `VISION_TARGET_SLEW_DPS`=150°/s 可台架调(大→跟手、小→平滑滞后)。与原航位推算等价、仅消阶跃。
- **未编译**(Keil 工程，需在 MDK 里编)。两项均合"输入端/源头解决、不在反馈环加低通"偏好。

---
*维护约定：本文与 PROGRESS.md / memory `control-tuning-progress` 三方同步；改代码后更新对应小节。*
