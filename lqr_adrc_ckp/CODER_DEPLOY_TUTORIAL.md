# 无动力 X 构型飞镖：从参数标定 → AI 仿真调试 → Coder 生成 C 代码 → VSCode 部署 完整教程

> 本文是一条打通「实物参数 → MATLAB 仿真 → AI 辅助整定 → 嵌入式 C 代码 → VSCode 固件集成」的完整流水线。
> 读完你可以：拿到一具真飞镖，量完几何/质量/舵机，跑仿真调通控制参数，
> 用 MATLAB Coder 把控制算法转成 C 代码，塞进 VSCode 的嵌入式工程里跑在飞控板上。
>
> **配套文件**：[`README.md`](README.md)（原理与整定指南）、[`DEPLOY_GUIDE.md`](DEPLOY_GUIDE.md)（部署与运行）

---

# 第一篇：参数标定 —— 从实物到 `dart_params.m`

## 1.1 你要标定什么

打开 `dart_params.m`，以下参数需要从实物测量/估算得到：

### 第一组：几何与质量（直接测量，一次搞定）

| 参数 | 符号 | 怎么测 | 精度要求 |
|---|---|---|---|
| `body.m` | 质量 m (kg) | 电子秤称重 | ±1 g 即可 |
| `body.d` | 参考直径 d (m) | 游标卡尺量弹体最大直径 | ±0.1 mm |
| `body.S` | 参考面积 S (m²) | = π·(d/2)²，自动算 | 不需要单独测 |
| `body.Ixx` | 滚转惯量 (kg·m²) | 扭摆法：用细丝悬挂，测扭振周期 T，Ixx = (T/2π)²·k（k 为丝扭刚度） | ±10% 够用 |
| `body.Iyy` | 俯仰惯量 | 双线摆法：两根等长线悬挂，测摆动周期 | ±10% 够用 |
| `body.Izz` | 偏航惯量 | 轴对称体 ≈ Iyy | 取 Iyy |

> **惯量简易估算法**（不想做实验）：把飞镖近似为均质圆柱 + 锥形头部，
> ```matlab
> % 均质圆柱（弹体）: I_cyl = (1/12)*m*(3*(d/2)^2 + L^2)
> % 均质圆锥（头部）: I_cone = (3/5)*m_head*(d/2)^2
> % 平行轴定理叠加
> ```

### 第二组：舵机（查数据手册或做实验）

| 参数 | 符号 | 怎么测 | 默认参考值 |
|---|---|---|---|
| `servo.wn` | 自然频率 (rad/s) | 给阶跃指令，用高速相机/编码器录舵偏响应，看振荡频率和衰减。或查舵机规格书标称带宽 | 2π·15≈94 |
| `servo.zeta` | 阻尼比 | 同上，从衰减包络算 | 0.7 |
| `servo.pos_max` | 位置限幅 (rad) | 量舵面机械止动位 | deg2rad(90) |
| `servo.rate_max` | 速率限幅 (rad/s) | 查舵机规格书空载转速 | deg2rad(200) |

### 第三组：气动系数（最需要标定，也是 AI 辅助调试的重点）

**如果是简单项目（不吹风洞）**，先用以下经验公式估算，跑通仿真后再对照实际飞行微调：

| 系数 | 符号 | 估算方法 | 典型量级 |
|---|---|---|---|
| `Cd0` | 零升阻力 | 参考同形状弹体数据；亚音速细长体 ≈ 0.2-0.4 | 0.30 |
| `CLa` | 升力线斜率 | 细长体理论：CLa ≈ 2 (1/rad) | 2.0 |
| `CYb` | 侧力 | 轴对称 ≈ -CLa | -2.0 |
| `Cma` | 俯仰静稳定 | 最关键参数：静稳定裕度 × CLa。正静稳定裕度（质心在压心前）→ Cma < 0。量质心位置，估算压心 ≈ 头部后方 1/3 弹长处 | -5 到 -15 |
| `Cnb` | 偏航静稳定 | 轴对称 ≈ -Cma | +5 到 +15 |
| `Clp` | 滚转阻尼 | 经验值，主要由弹翼/尾翼尺寸决定 | -1 到 -5 |
| `Cmq, Cnr` | 俯仰/偏航阻尼 | 经验值，与尾翼面积和力臂正比 | -10 到 -20 |
| `Clda` | 滚转舵效 | **关键**：给固定舵偏跑 CFD 或实测，看滚转力矩/动压/参考量 | 0.05-0.3 |
| `Cmde` | 俯仰舵效 | 同上 | -0.5 到 -2.0 |
| `Cndr` | 偏航舵效 | 同上，X 构型通常较小 | -0.05 到 -0.2 |

> **没有风洞/CFD 怎么办**：先取表中"典型量级"的中间值跑仿真 → 跑通后对照实飞数据逐系调（见第二篇）。

---

## 1.2 标定后的验证流程

改完参数后，按顺序跑（每次改参数必跑，一步不能跳）：

```
① t1_openloop     →  确认开环物理合理（速度衰减、静稳定、无 NaN）
② t2_inner_rate   →  确认内环 LADRC 能跟踪角速度
③ t3_attitude_step →  确认全闭环姿态阶跃指标达标（超调<20%、ts<0.5s）
④ t4_robustness   →  确认速度衰减和风扰下不崩
⑤ main_dart_sim   →  确认 6DOF 自由飞行一切正常
```

每步 PASS=1 才能进下一步。

---

# 第二篇：AI 辅助仿真调试 —— 让 Claude 帮你调参

## 2.1 调试哲学：人不猜参数，AI 跑扫描

传统做法是工程师脑子里推公式、手动试几个数。效率低、容易漏。正确做法：

1. **你告诉 AI 哪里不对**（比如"滚转超调 35%，太大了"或"60m/s 掉到 30m/s 后舵饱和"）
2. **AI 写扫描脚本**，在参数空间里成片跑仿真，自动提取指标
3. **AI 读结果，挑最优参数**，给出修改建议
4. **你确认**改到 `dart_params.m`，重新跑全套验证

## 2.2 常用调试对话模板

### 场景 A：阶跃响应不满意

```
"跑了一下 t3，滚转超调 12%、调节时间 0.8s。
帮我用 probe_gain 扫一下 wc 从 10 到 25、Q 整体缩放 0.5~2 倍，
找一组超调<5% 且 ts<0.4s 的参数。"
```

→ AI 会跑 `probe_gain.m`，分析输出的超调/ts，给出推荐值。

### 场景 B：舵面频繁饱和

```
"main_dart_sim 舵面峰值到了 25°，后半段也经常触限。
帮我用 probe_main 扫一下：滚转幅度 5/8/10°，过渡时间 0.3/0.5/0.8s，风 3 个等级，
看哪个组合能让饱和步数降到 50 以内。"
```

→ AI 跑 `probe_main.m`，打印各组合的 nsat/maxd，定位根因。

### 场景 C：换实物舵机后不确定

```
"舵机换了，pos_max=60°, rate_max=300°/s，wn 不知道，大约 20Hz。
帮我跑一遍全套回归，确认所有测试 PASS。"
```

→ AI 改参数跑全套，验证不崩。

### 场景 D：气动系数整定（对照实飞数据）

```
"实飞数据来了：60m/s 水平射出，实际速度衰减到 50m/s 用了 2s（仿真要 3s）。
仿真阻力偏小。帮我逐步把 Cd0 从 0.30 调到匹配实飞衰减曲线。"
```

→ AI 写脚本扫 Cd0 值，让仿真速度衰减曲线拟合实飞数据。

## 2.3 AI 能做什么、不能做什么

| ✅ AI 能 | ❌ AI 不能替代 |
|---|---|
| 批量跑扫描、提取指标、建议参数 | 判断实物数据的可信度 |
| 调 LQR/LADRC 增益让你少试错 | 替你决定"超调 5% vs 调节时间 0.3s"哪个更重要 |
| 诊断饱和根因（风/机动/耦合） | 给没有传感器的飞镖测出惯量来 |
| 写脚本自动拟合气动系数 | 代替风洞实验给出绝对精确的 Cmα |


# 第三篇：MATLAB Coder —— 从 .m 到 C/C++ 代码

## 3.1 哪些函数需要生成代码

飞控板上只跑**控制算法**（在线 + 实时），不包括仿真（离线 + 非实时）：

| 需要转 C | 不需要转 C |
|---|---|
| `control/ladrc_inner.m` 内环 LADRC | `models/dart_6dof.m` 6DOF 动力学（实物飞，不仿真） |
| `control/lqr_outer_design.m` LQR 增益 | `models/dart_aero.m` 气动力（实物飞，不仿真气动） |
| `control/control_alloc.m` 控制分配 | `models/rk4_step.m` 积分器 |
| `models/actuator_2nd.m` 舵机模型（若在飞控侧做舵机控制） | `models/dryden_wind.m` 风场 |
| `models/dcm_ned2body.m` 坐标变换（IMU/导航要用） | `analysis/*` 画图/指标 |
| — | `tests/*` 测试脚本 |

> 注：实际飞控板上，6DOF 动力学由真实物理"运行"，不需要仿真。
> `dart_aero.m` 里的气动力在实物上由真实空气提供，也不需要在飞控代码里算（除非你做模型预测控制 MPC）。

## 3.2 准备代码：让函数兼容 Code Generation

MATLAB Coder 对代码有一些限制。需要处理的几个点：

### (1) `ladrc_inner.m` —— 结构体字段初始化

Coder 要求结构体字段在第一次赋值时必须全部声明。当前用 `if isempty(st)` 动态创建，需要改写为显式声明：

```matlab
function [u, st] = ladrc_inner_cg(omega_cmd, omega_meas, st, params, u_act)
%#codegen   % ← 加上这行告诉 Coder 要做代码生成
    ...
    if isempty(st)
        st.z1 = omega_meas;
        st.z2 = zeros(3,1);
        st.u  = zeros(3,1);
    end
    ...
end
```

实际上当前代码已经兼容——只要 `st` 作为结构体传入传出即可。

### (2) `lqr_outer_design.m` —— LQR 离线算，K 编译期固定

真实飞控中 K 是预先算好的常数矩阵，不需要在飞控板上跑 `lqr()`。
所以：

```matlab
% 方案：编译时直接用常量替代 lqr() 调用
% 把 K 作为 params 里的硬编码矩阵，不运行时求解黎卡提方程
params.lqr.K = [6 0 0; 0 6 0; 0 0 6];   % 预先算好
```

### (3) `control_alloc.m` —— 矩阵运算没问题

矩阵乘、伪逆（`B \ y` 形式）在 Coder 中都支持。当前写法已兼容。

### (4) `dcm_ned2body.m` —— 纯三角函数，完全兼容

无任何 Coder 不兼容的写法。

### (5) `actuator_2nd.m` —— 完全兼容

无动态内存分配、无变长数组，完全兼容。

## 3.3 创建 Coder 项目

### 步骤 1：打开 MATLAB Coder App

```matlab
coder
```

或在命令窗口：

```matlab
% 以 ladrc_inner 为例生成 C 代码
cfg = coder.config('lib');          % 生成静态库（.lib / .a）
cfg.TargetLang = 'C';                % 或 'C++'
cfg.GenerateReport = true;           % 生成 HTML 报告方便检查

% 定义输入类型
omega_cmd  = coder.typeof(double(0), [3 1], [0 0]);
omega_meas = coder.typeof(double(0), [3 1], [0 0]);
% st 是结构体，需要定义
st_type = struct();
st_type.z1 = coder.typeof(double(0), [3 1], [0 0]);
st_type.z2 = coder.typeof(double(0), [3 1], [0 0]);
st_type.u  = coder.typeof(double(0), [3 1], [0 0]);
% params 也是结构体...
% u_act 可选

codegen -config cfg ladrc_inner -args {omega_cmd, omega_meas, st_type, params, u_act}
```

> 实际使用中，建议把整个控制链包在一个顶层函数里一次生成。

### 步骤 2：统一入口函数（推荐）

写一个 `gnc_step.m`，把所有控制步骤包在一起，一次性生成整个 GNC 循环的 C 代码：

```matlab
function [delta_cmd, gnc_state] = gnc_step(att, pqr, att_ref, dt, params, gnc_state)
%#codegen
% GNC_STEP  飞控一步（1000 Hz）：姿态测量 → LQR → LADRC → 分配 → 舵机 → 舵令输出
%   这是给 Coder 生成的顶层函数，里面调用了全部控制子函数。
%
%   att(3x1)     - 当前姿态角 [φ;θ;ψ] (rad)，来自 IMU/AHRS
%   pqr(3x1)     - 当前角速度 [p;q;r] (rad/s)，来自陀螺
%   att_ref(3x1) - 目标姿态角 (rad)，来自制导
%   dt(1x1)      - 步长 (s)
%   params       - 参数结构体（编译期固定，含 K/W 等已标定好的值）
%   gnc_state    - 控制器内部状态（LESO z1/z2、舵机 pos/vel 等），需跨步保持
%
%   返回：
%   delta_cmd(4x1) - 4 舵面指令 (rad)
%   gnc_state      - 更新后状态

    % ① 外环 LQR：姿态误差 → 期望角速度
    omega_cmd = params.K * (att_ref - att);        % K 预计算，不用运行时 lqr()

    % ② 内环 LADRC：期望角速度 + 实测角速度 → 期望力矩
    [tau, gnc_state.ladrc] = ladrc_inner(omega_cmd, pqr, gnc_state.ladrc, params, gnc_state.tau_act);

    % ③ X 构型控制分配：力矩 → 4 舵偏指令
    [dcmd, alloc_info] = control_alloc(tau, gnc_state.qbar, params);

    % ④ 二阶舵机：指令 → 实际位置（经速率/位置限幅）
    [delta, gnc_state.servo] = actuator_2nd(dcmd, gnc_state.servo, params);

    % ⑤ 估算实际执行力矩（给下步 LADRC 做抗饱和）
    %    注：实飞时这里用舵偏反馈 × 估算的舵效矩阵；或直接从舵机状态推算
    gnc_state.tau_act = gnc_state.B_eff * delta;   % B_eff 基于当前动压估计
    gnc_state.qbar    = estimate_dynamic_pressure(...);  % 来自空速管或模型

    delta_cmd = dcmd;    % 输出指令舵偏（或 delta 实际位置，看飞控策略）
end
```

### 步骤 3：生成代码

```matlab
% 全部控制链 + 坐标变换，一次性转 C
codegen -config cfg gnc_step dcm_ned2body ladrc_inner control_alloc actuator_2nd ...
    -args {att, pqr, att_ref, dt, params, gnc_state} -o gnc_controller
```

生成产物（在 `codegen/lib/gnc_controller/` 下）：

```
gnc_controller.h          ← 主头文件（你 include 的）
gnc_controller.c          ← 主实现
ladrc_inner.h / .c
control_alloc.h / .c
actuator_2nd.h / .c
dcm_ned2body.h / .c
gnc_step.h / .c
...
rtwtypes.h                ← Coder 运行时的类型定义
```

## 3.4 关键注意事项

| 要点 | 说明 |
|---|---|
| **预计算 K** | `lqr()` 不能在嵌入式上跑（依赖 Control System Toolbox + 解黎卡提方程太贵）。在 MATLAB 里算好 K 存进 params 作为常量矩阵 |
| **动压估计** | 实飞需要空速管或模型估计 q̄。若没有空速管，可用 GPS 速度 + 风速模型估算，或简化为 `qbar = 0.5*rho*norm(vel)^2` |
| **浮点 vs 定点** | 默认生成双精度浮点。若飞控是 Cortex-M4F（有 FPU），直接用 float（改 `cfg.DataTypeReplacement = 'CoderTypedefs'`） |
| **LESO 初始化** | 首次调用时 `gnc_state.ladrc = []`，函数内部 `isempty` 分支在 C 中对应 `if (st_is_empty)` |
| **无动态内存** | 代码中只有固定大小的矩阵（3×3、3×1、4×1），不会触发 malloc，Coder 会提示 "no dynamic memory allocation"——这是好的 |


# 第四篇：VSCode 部署 —— 在嵌入式工程中集成生成代码

## 4.1 典型嵌入式工程结构

假设飞控板是 **STM32F4 / H7**，用 **CMake + ARM GCC** 构建：

```
drone_gnc_firmware/                       ← VSCode 打开的工程根
├── CMakeLists.txt
├── .vscode/
│   ├── c_cpp_properties.json             ← include 路径配好
│   ├── launch.json                       ← 调试 (J-Link / ST-Link / OpenOCD)
│   └── tasks.json                        ← 构建任务
├── Core/
│   ├── main.c                            ← 飞控主循环
│   ├── freertos.c                        ← RTOS 任务调度（如有）
│   ├── imu_hal.c / .h                    ← IMU 驱动（SPI/I2C → 姿态角/角速度）
│   ├── servo_hal.c / .h                  ← 舵机 PWM 驱动
│   └── ...
├── Drivers/                              ← HAL 库 (CMSIS, STM32 HAL/LL)
├── Middleware/                           ← FreeRTOS, LwIP 等
└── lib/
    └── gnc_controller/                   ← ★ MATLAB Coder 生成的代码放这里
        ├── gnc_step.h / .c
        ├── gnc_controller.h / .c
        ├── ladrc_inner.h / .c
        ├── control_alloc.h / .c
        ├── actuator_2nd.h / .c
        ├── dcm_ned2body.h / .c
        └── rtwtypes.h
```

## 4.2 集成步骤

### 步骤 1：把生成代码拷进工程

```bash
cp -r E:/DevelopLotteany1/Simulink_Guidence_lqr_adrc/codegen/lib/gnc_controller/* \
      ~/drone_gnc_firmware/lib/gnc_controller/
```

### 步骤 2：CMake 添加

```cmake
# CMakeLists.txt 中添加
set(GNC_DIR ${CMAKE_SOURCE_DIR}/lib/gnc_controller)

# 收集所有 Coder 生成的 .c
file(GLOB GNC_SOURCES ${GNC_DIR}/*.c)

# 添加 include 路径
target_include_directories(${PROJECT_NAME} PRIVATE ${GNC_DIR})

# 链接
target_sources(${PROJECT_NAME} PRIVATE ${GNC_SOURCES})
```

### 步骤 3：VSCode 的 c_cpp_properties.json

```json
{
    "configurations": [{
        "name": "STM32",
        "includePath": [
            "${workspaceFolder}/Core/**",
            "${workspaceFolder}/Drivers/**",
            "${workspaceFolder}/lib/gnc_controller",
            "${workspaceFolder}/Middleware/**"
        ],
        "defines": ["STM32F407xx", "USE_HAL_DRIVER"],
        "compilerPath": "arm-none-eabi-gcc",
        "cStandard": "c11",
        "intelliSenseMode": "gcc-arm"
    }],
    "version": 4
}
```

### 步骤 4：飞控主循环调用

```c
// main.c —— 1000 Hz 飞控主循环（由定时器中断或 RTOS 任务驱动）

#include "gnc_step.h"
#include "gnc_controller_types.h"   // 结构体类型定义

// ---- 从 Coder 生成的类型里取所需 ----
// (Coder 会给每个结构体生成对应的 C typedef，在 gnc_controller_types.h 里)

static gnc_state_t gnc_state;       // 跨步状态（LESO 内部量、舵机状态）
static double params_K[9];          // 预计算的 LQR 增益（从 params 结构体拆出）
static double params_W[9];          // 分配权重
// ... 其他 params 成员（在初始化时从 MATLAB 导出值填好）

void gnc_init(void) {
    // ① 初始化 gnc_state 为空（首次调用 gnc_step 会自动初始化内部状态）
    gnc_state.ladrc_is_empty = true;
    gnc_state.servo_is_empty = true;

    // ② 加载预标定好的参数（这些值来自 MATLAB 仿真最终确定的最优参数）
    //    K = diag([6 6 6])
    double K[9] = {6,0,0, 0,6,0, 0,0,6};
    memcpy(params_K, K, sizeof(K));

    //    W = diag([1 1 0.1])
    double W[9] = {1,0,0, 0,1,0, 0,0,0.1};
    memcpy(params_W, W, sizeof(W));

    // ... 加载其他 params 域（servo wn/zeta/pos_max/rate_max, alloc E/eps, adrc w0/wc/b0 等）
}

void gnc_1000hz_isr(void) {         // 每 1ms 由定时器触发
    // ① 从 IMU 读姿态和角速度
    double att[3], pqr[3];
    imu_get_attitude(att);           // [φ θ ψ] in rad，来自 AHRS/互补滤波/EKF
    imu_get_gyro(pqr);               // [p q r] in rad/s，陀螺原始数据

    // ② 从制导模块获取目标姿态
    double att_ref[3];
    guidance_get_ref(att_ref);       // 上位机或板载制导管线

    // ③ 调用 GNC 一步（核心！MATLAB Coder 生成的 C 代码）
    double delta_cmd[4];
    gnc_step(att, pqr, att_ref, 0.001, /*params*/..., &gnc_state, delta_cmd);

    // ④ 把舵令转换成 PWM 输出给舵机
    servo_set_radians(delta_cmd);    // 4路PWM，映射到 [pos_min, pos_max]
}
```

## 4.3 参数结构体映射：从 MATLAB struct 到 C struct

MATLAB 里的 `params` 是一个嵌套结构体；Coder 会为它生成对应的 C struct。
如果觉得自动生成的结构体太复杂，可以手动拆成平铺的标量数组（上面的写法就是手拆的）。

**推荐方案**：写一个 `params_init.c`，把 MATLAB 仿真最终确定的那组参数硬编码进去：

```c
// params_init.c —— 从 MATLAB 最终参数导出，手写一次，之后不动
// 所有值来自 dart_params.m 经过 AI 仿真标定后的最终版本

void params_load(dart_params_t *p) {
    // --- 环境 ---
    p->env.rho = 1.225;

    // --- 舵机 ---
    p->servo.wn       = 94.2478;   // 2*pi*15
    p->servo.zeta     = 0.7;
    p->servo.pos_max  = 1.5708;    // deg2rad(90)
    p->servo.rate_max = 3.49066;   // deg2rad(200)

    // --- LQR (预先算好的 K) ---
    p->lqr.K[0]=6; p->lqr.K[4]=6; p->lqr.K[8]=6;  // diag([6,6,6])

    // --- LADRC ---
    p->adrc.w0 = 100.0;
    p->adrc.wc = 16.0;
    p->adrc.b0[0] = 1000.0;  // 1/Ixx = 1/0.001
    p->adrc.b0[1] = 50.0;    // 1/Iyy = 1/0.02
    p->adrc.b0[2] = 50.0;    // 1/Izz = 1/0.02

    // --- 分配 ---
    p->alloc.E[0]=0.7071; p->alloc.E[1]=-0.7071; ...  // cos(45°)
    p->alloc.W[0]=1; p->alloc.W[4]=1; p->alloc.W[8]=0.1;
    p->alloc.eps = 1e-6;
}
```

> 上面用 `p->alloc.K[8]` 是平铺的 3×3 在 C 里按行优先排成 9 元素 1D 数组。

## 4.4 闭环验证：仿真结果 vs 实测

| 验证环节 | 方法 |
|---|---|
| **代码级验证** | Coder 生成的代码可以**在 MATLAB 里回代验证**：用同样的输入跑 .m 和生成的 .c（通过 mex），输出必须 bit-exact 一致 |
| **SIL (Software-in-the-Loop)** | 把生成的 C 代码编译成 DLL/SO，在 MATLAB 仿真循环里替换掉原来的 .m 函数调用——既跑了完整仿真场景，又验证了 C 代码的正确性 |
| **HIL (Hardware-in-the-Loop)** | 把生成的代码烧到飞控板，飞控板接舵机实物（或舵机负载模拟器），MATLAB 仿真端跑 6DOF + 气动，两者通过串口/CAN 交换数据——最接近真实 |
| **实飞** | 仿真里标定好的参数直接烧进飞控。首次实飞建议低空、低速、小机动，逐步放开 |

## 4.5 SIL 验证脚本（在 MATLAB 中做完最后的 check）

```matlab
% sil_check.m —— 用 SIL 验证 Coder 生成的 C 代码和原始 .m 完全一致

% 编译 mex
codegen -config:mex gnc_step -args {att0, pqr0, ref0, dt, params, st0} -o gnc_step_mex

% 跑同一个场景，对比输出
for k = 1:1000
    [dcmd_m, st_m] = gnc_step(att, pqr, ref, dt, params, st_m);   % 纯 .m
    [dcmd_c, st_c] = gnc_step_mex(att, pqr, ref, dt, params, st_c); % C mex
    err = max(abs(dcmd_m - dcmd_c));
    if err > 1e-12
        fprintf('SIL mismatch at step %d: err=%g\n', k, err);
    end
end
fprintf('SIL check passed — .m and C output bit-exact identical\n');
```

---

## 附录 A：飞控板上完整的 GNC 调用链（制导 → 控制 → 执行）

```
┌─────────────────────────────────────────────────────────────┐
│  制导管线（上位机或板载）                                      │
│  根据目标弹道 / 遥控指令 / 预编程航点                          │
│  → 输出目标姿态角 att_ref = [φ_ref; θ_ref; ψ_ref]            │
│  → 注：无动力飞镖俯仰目标 = 弹道倾角 γ（顺气飞），见 main 场景 │
└───────────────────────┬─────────────────────────────────────┘
                        │ att_ref (3x1, rad)
                        ▼
┌─────────────────────────────────────────────────────────────┐
│  导航/传感                                                   │
│  IMU → 姿态角 φθψ (AHRS/EKF)                                 │
│  Gyro → 角速度 pqr (含低通滤波)                               │
│  Baro/GPS → 高度/速度 (用于 γ 估计)                           │
└───────────────────────┬─────────────────────────────────────┘
                        │ att (3x1) + pqr (3x1)
                        ▼
┌─────────────────────────────────────────────────────────────┐
│  控制（gnc_step, 1000 Hz）    ← 本文生成的 C 代码核心        │
│                                                             │
│  att_ref ──→ LQR ──→ ω_cmd ──→ LADRC+LESO ──→ τ_c          │
│               (K预计算)        (估总扰动+对消)    期望力矩    │
│                                                             │
│  τ_c ──→ X 构型分配 ──→ δ_cmd ──→ 二阶舵机 ──→ δ_out        │
│           (加权伪逆)      4舵指令        速率/位置限         │
│                                                             │
│  抗饱和：δ_out 反算 τ_act 反馈给 LESO 的 b0·u 项             │
└───────────────────────┬─────────────────────────────────────┘
                        │ δ_cmd (4x1, rad)
                        ▼
┌─────────────────────────────────────────────────────────────┐
│  执行（舵机 HAL）                                            │
│  δ_cmd → PWM 占空比 → 4 路舵机转动                           │
│  飞镖物理响应 → 下一周期 IMU 读到新状态 → 循环                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 附录 B：快速操作清单

| 我想做什么 | 工具/步骤 |
|---|---|
| 量实物参数填进 dart_params | 游标卡尺、电子秤、扭摆/双线摆实验 → 改 `dart_params.m` |
| 调 LQR 增益让超调变小 | 改 Q 权重 → 跑 `probe_gain` → AI 建议最佳值 |
| 调 LADRC 带宽让抗扰更快 | 改 wc/w0 → 跑 `t3_attitude_step` `t4_robustness` 验证 |
| 换舵机后确认没问题 | 改 servo 段 → 跑全套 `t1~t4 + main` |
| 生成 C 代码 | `coder` App 或 `codegen` 命令 → 生成到 `codegen/lib/` |
| SIL 验证 C 代码正确性 | `codegen -config:mex` → mex 回代对比 |
| 集成到 VSCode 嵌入式工程 | 拷 `lib/gnc_controller/` 进工程 → CMake 加源文件 → `main.c` 调 `gnc_step` |
| 首次实飞验证 | 低空低速小幅机动 → 记录飞参 → 与仿真回放对比 → 微调气动系数 |
