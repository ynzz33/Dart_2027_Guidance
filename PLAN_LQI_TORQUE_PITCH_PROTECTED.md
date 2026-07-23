# 飞镖 LQI 力矩控制与 Pitch 保护型零空间分配实施计划

## 0. 文档目的

本计划用于指导其他 AI 或工程人员，在新 Git 分支中重建飞镖姿态控制器。

目标控制链路：

~~~text
IMU 姿态/角速度
        ↓
姿态 LQI
        ↓
三轴期望力矩 [Mx, My, Mz]，单位 N·m
        ↓
Pitch 保护型零空间舵面分配
        ↓
四片舵面角度 [UL, UR, DR, DL]
        ↓
舵机 PWM
~~~

核心目标：

1. MATLAB 中 LQI 只输出物理三轴力矩，不直接输出四片舵角。
2. 使用真实惯量、质心和舵面相对质心的位置建立力矩模型。
3. Pitch 默认只保留很弱的角速度阻尼，不主动追踪 Pitch 角度。
4. Roll/Yaw 控制时，在满足 Roll/Yaw 力矩的前提下，利用零空间尽量减小 Pitch 力矩和舵面动作。
5. 删除旧姿态 PID、LADRC 和旧四舵角 LQR 路径，不保留旧控制器作为运行时回退。
6. 气动阻尼、恢复力矩、完整能量优化等缺少可靠数据的部分先保留接口，不强行加入主控制模型。

---

## 1. 已确认的工程事实

### 1.1 坐标和轴序

以下约定已经在工程中验证过，实施时不得擅自修改：

- 工程机体系为当前已验证的右手 ENU 坐标系。
- 工程姿态数组轴序为 PITCH=0、ROLL=1、YAW=2。
- LQI 数学模型内部统一使用 [ROLL, PITCH, YAW]。
- IMU 角速度最终必须转换到数学模型顺序 [p, q, r]。
- 不允许因为 MATLAB 数学顺序与工程数组顺序不同而修改已经验证过的符号。

### 1.2 舵面顺序

工程和日志舵面顺序统一为：

~~~text
[UL, UR, DR, DL]
~~~

新的 MATLAB 和嵌入式零空间分配器统一采用这个顺序。任何旧模型顺序都必须通过明确的排列矩阵转换，不能通过修改 G 矩阵或随意翻转符号解决。

### 1.3 惯量

当前真实惯量来自 SolidWorks，单位为 kg·m²：

~~~matlab
Ixx = 307131.230e-9;
Iyy = 1580238.787e-9;
Izz = 1589776.981e-9;
~~~

优先使用完整惯量矩阵：

~~~matlab
I_body = [ Ixx, -Ixy, -Ixz;
          -Ixy,  Iyy, -Iyz;
          -Ixz, -Iyz,  Izz ];
~~~

如果暂时没有可靠的交叉惯量，设置 Ixy=Ixz=Iyz=0，并明确标注为暂缺数据，不能伪造数值。

---

## 2. 范围和非目标

### 2.1 本阶段必须完成

- 姿态/角速度 LQI；
- 三轴物理力矩输出；
- 质心和舵面位置进入舵面力矩矩阵；
- Pitch 低权重；
- Pitch 积分默认关闭；
- 嵌入式 Pitch 保护型零空间分配；
- 舵面限幅、实际力矩回算、积分抗饱和；
- 外部扰动力矩仿真；
- MATLAB 到 C 的增益和矩阵导出；
- 删除旧姿态 PID、LADRC 和旧 LQR 调用路径；
- MATLAB 仿真和嵌入式单元测试。

### 2.2 本阶段暂不强行完成

以下内容只保留参数接口和 TODO，不允许因为有占位数字就直接启用：

- 从单次抛物线日志直接拟合完整气动阻尼；
- 未验证的 Pitch/Yaw 静态恢复系数；
- 未准确测量的舵机速度模型；
- 在线非线性扰动观测器；
- 纯 IMU 的位置、速度和目标状态完整估计；
- 复杂六自由度在线 MPC；
- 严格意义上的总飞行能量最优控制。

姿态 LQI 可以最小化控制代价，但没有准确的速度、迎角、阻力和舵机功耗模型时，不能声称已经最小化整段飞行能量。

---

## 3. Git 分支和文件策略

### 3.1 分支

从当前工程新建：

~~~text
feature/lqi-torque-pitch-protected
~~~

所有删除和新增均在该分支完成，不修改原分支。

### 3.2 建议新增文件

MATLAB：

~~~text
matlab/lqi/dart_attitude_lqi_torque_pitch_protected.m
matlab/lqi/dart_lqi_parameters.m
matlab/lqi/dart_lqi_export_c.m
~~~

嵌入式：

~~~text
imcalib/lqi_tool/lqi_torque.c
imcalib/lqi_tool/lqi_torque.h
imcalib/lqi_tool/torque_allocator.c
imcalib/lqi_tool/torque_allocator.h
imcalib/lqi_tool/lqi_gain_table.h
imcalib/lqi_tool/torque_geometry_table.h
~~~

实际目录可以按工程风格调整，但 MATLAB、LQI 控制器和分配器必须分开。

### 3.3 删除范围

新分支中删除旧姿态控制路径：

- 旧四舵角直接 LQR；
- 旧 LQR 增益调度和 MATLAB Coder 生成的旧 K 文件；
- 姿态 PID 调用路径；
- 姿态 LADRC 调用路径；
- 旧 Servo_Mix 姿态控制路径；
- lqr_mode、ladrc_mode 等旧姿态控制运行时切换。

删除前必须进行引用搜索。若某个 PID 函数仍被非姿态功能使用，只删除姿态依赖，不删除无关功能。

新控制器数据无效时不切回旧控制器，安全动作固定为：

~~~text
冻结积分 → 舵面回中 → 设置错误标志
~~~

---

## 4. MATLAB 姿态模型

### 4.1 状态定义

数学模型统一使用：

~~~matlab
x = [e_roll;
     e_pitch;
     e_yaw;
     p;
     q;
     r];
~~~

单位：

~~~text
姿态误差：rad
角速度：rad/s
~~~

积分状态：

~~~matlab
z = integral([e_roll; e_pitch; e_yaw]);
xa = [x; z];
~~~

控制输出：

~~~matlab
tau = [Mx; My; Mz];       % N·m
tau_cmd = -K_lqi * xa;
~~~

新的 MATLAB 脚本禁止出现 LQI 直接输出四舵角的主路径。

### 4.2 姿态误差

推荐使用四元数误差：

~~~text
q_error = inverse(q_target) ⊗ q_current
~~~

再转换为三维小角度误差向量。

为兼容当前工程，第一版可以暂时使用 Euler 误差，但必须：

- Roll/Yaw 做角度环绕；
- 仿真初始角度保持在小角度范围；
- 明确注明这是兼容版本，不是最终大机动姿态误差方案。

动态目标误差只用于控制器跟踪，不能用于气动参数标定。

### 4.3 非线性刚体转动模型

仿真模型使用：

~~~matlab
I_body * omega_dot = ...
    tau_aero ...
  + tau_surface ...
  + tau_dist ...
  - cross(omega, I_body * omega);
~~~

其中：

- tau_aero：可选气动阻尼和恢复力矩；
- tau_surface：舵面实际力矩；
- tau_dist：外部扰动力矩；
- cross(omega,I_body*omega)：刚体陀螺耦合项。

零角速度附近线性化时，陀螺耦合项可以暂时不进入 LQI 的线性 A 矩阵，但必须保留在非线性仿真中。

### 4.4 线性模型

线性模型：

~~~matlab
eta_dot   = omega;
omega_dot = A_omega_eta(V) * eta ...
          + A_omega_rate(V) * omega ...
          + inv(I_body) * tau ...
          + inv(I_body) * tau_dist;
~~~

因此：

~~~matlab
A = [zeros(3), eye(3);
     A_omega_eta(V), A_omega_rate(V)];

B_tau = [zeros(3);
         inv(I_body)];

E_dist = B_tau;
~~~

第一版没有可靠气动数据时：

~~~matlab
A_omega_eta(V)  = zeros(3,3);
A_omega_rate(V) = zeros(3,3);
~~~

保留以下开关：

~~~matlab
enable_roll_damping
enable_pitch_damping
enable_yaw_damping
enable_pitch_restoring
enable_yaw_restoring
enable_gyro_coupling
enable_servo_dynamics
~~~

未标定参数默认关闭。

### 4.5 质心和舵面位置

所有舵面位置必须相对质心定义：

~~~matlab
r_surface(:,1) = r_UL_cg;
r_surface(:,2) = r_UR_cg;
r_surface(:,3) = r_DR_cg;
r_surface(:,4) = r_DL_cg;
~~~

每片舵面参数包含：

- 舵面面积；
- 气动力方向；
- 舵效系数；
- 安装方向符号；
- 相对质心的位置。

单片舵面力矩：

~~~matlab
dF_i = qbar * S_i * C_F_delta_i * delta_i * force_direction_i;
tau_i = cross(r_surface(:,i), dF_i);
~~~

最终：

~~~matlab
tau_surface = H_tau(V) * delta;
delta = [delta_UL; delta_UR; delta_DR; delta_DL];
~~~

若舵效只能使用旧模型估计值，标记为 nominal_estimate，并通过不确定性仿真验证，不能当作已经标定完成。

### 4.6 矩阵检查

每次 MATLAB 运行必须检查：

~~~matlab
rank(H_tau)
rank(H_tau([ROLL,YAW],:))
svd(H_tau)
cond(H_tau)
rank(ctrb(A,B_tau))
~~~

重点检查：

1. 三轴是否可控；
2. Roll/Yaw 子矩阵是否满秩；
3. 某速度下是否接近奇异；
4. 四片舵面是否存在有效冗余；
5. 舵面顺序和符号是否与工程一致。

---

## 5. Pitch 保护型 LQI

### 5.1 LQI 增广

~~~matlab
C_I = [eye(3), zeros(3,3)];

A_aug = [A,   zeros(6,3);
         C_I, zeros(3,3)];

B_aug = [B_tau;
         zeros(3,3)];

[A_d, B_d] = c2d(A_aug, B_aug, Ts, 'zoh');
K_lqi = dlqr(A_d, B_d, Q_aug, R_tau);
~~~

### 5.2 Pitch 权重

默认原则：

~~~text
Pitch 角度权重：低
Pitch 角速度权重：低
Pitch 积分权重：0
Pitch 力矩权重：高
~~~

示意配置：

~~~matlab
Q_aug = diag([
    Q_roll_angle,
    Q_pitch_angle_small,
    Q_yaw_angle,
    Q_p,
    Q_q_small,
    Q_r,
    Q_Iroll,
    0,
    Q_Iyaw
]);

R_tau = diag([
    R_roll,
    R_pitch_large,
    R_yaw
]);
~~~

具体数值必须通过仿真调节，禁止直接照抄旧 Q/R。

### 5.3 Pitch 输出保护

即使 LQI 计算出 tau_pitch，也必须执行：

~~~matlab
tau_pitch = clamp(tau_pitch, -M_pitch_max, M_pitch_max);
~~~

默认：

- 不使用 Pitch 积分；
- 只保留弱角速度阻尼；
- 不主动追踪抛物线期间的 Pitch 角度；
- Pitch 目标变化不能直接造成大幅舵面动作。

### 5.4 积分抗饱和

MATLAB 和嵌入式实现必须一致：

~~~text
舵面未饱和：正常积分
舵面饱和且误差继续推动饱和：暂停积分
误差方向有助于退出饱和：允许积分释放
Pitch 保护触发：冻结 Pitch 积分
目标丢失：冻结对应轴积分
~~~

---

## 6. 嵌入式 Pitch 零空间分配

### 6.1 重要原理

完整矩阵的零空间满足：

~~~matlab
H_tau * N = 0;
~~~

它不能改变已经指定的完整三轴力矩。

本项目需要的不是完整 H_tau 的零空间，而是：

~~~text
固定 Roll/Yaw 力矩，
利用 Roll/Yaw 子矩阵的零空间降低 Pitch 力矩。
~~~

### 6.2 Roll/Yaw 子问题

定义：

~~~matlab
H_ry = H_tau([ROLL,YAW], :);    % 2×4
h_pitch = H_tau(PITCH, :);      % 1×4
tau_ry = tau_cmd([ROLL,YAW]);
~~~

先求满足 Roll/Yaw 的舵面解：

~~~matlab
delta0 = pinv(H_ry) * tau_ry;
~~~

求 Roll/Yaw 子矩阵零空间：

~~~matlab
N_ry = null(H_ry);               % 通常为 4×2
~~~

通解：

~~~matlab
delta = delta0 + N_ry * eta;
~~~

### 6.3 优化目标

~~~matlab
J = lambda_pitch * (h_pitch * delta)^2 ...
  + lambda_servo  * (delta' * W_delta' * W_delta * delta);
~~~

含义：

- 第一项最小化 Pitch 力矩；
- 第二项最小化总舵面动作；
- W_delta 可以让某些舵面更保守。

把 delta=delta0+N_ry*eta 代入后，得到固定维度二维最小二乘问题。

### 6.4 嵌入式实现

STM32G431 不使用通用 QP。采用固定维度解析计算：

~~~text
1. 计算 delta0
2. 读取预计算或当前 N_ry
3. 解 2×2 对称线性方程
4. 得到 eta
5. 得到 delta
6. 检查四舵面限幅
7. 必要时统一缩放
8. 回算实际三轴力矩
9. 设置饱和和不可行标志
~~~

若矩阵接近奇异：

- 不求逆；
- 使用标称速度矩阵；
- 设置 allocator_infeasible；
- 舵面回到安全输出；
- 记录错误原因。

### 6.5 Pitch 硬约束

最终必须检查：

~~~matlab
abs(h_pitch * delta) <= M_pitch_max
~~~

若不满足：

1. 优先降低 Pitch 相关舵面动作；
2. 尽量保持 Roll/Yaw；
3. 设置 allocator_infeasible；
4. 不允许无提示地输出大 Pitch 力矩。

限幅优先级固定为：

~~~text
舵面物理安全限幅
    > Pitch 力矩保护
    > Roll/Yaw 力矩精度
    > 舵面能量最小化
~~~

### 6.6 关于整体能量

零空间只能在当前 Roll/Yaw 力矩任务下减少 Pitch 力矩和舵面动作，不能单独保证总飞行能量最优。

当前阶段使用以下能量代理：

~~~text
Pitch 力矩平方积分
四舵面偏转平方积分
舵面变化率平方积分
总舵面动作
~~~

没有可靠阻力模型前，不得把这些代理量称为真实飞行能量。

---

## 7. 嵌入式结构和接口

建议新增：

~~~c
typedef struct
{
    float attitude_error_rad[3];
    float body_rate_rad_s[3];
    float integral_error[3];

    float torque_cmd_Nm[3];
    float torque_achieved_Nm[3];
    float torque_error_Nm[3];

    float servo_cmd_deg[4];
    float pitch_moment_Nm;
    float pitch_moment_limit_Nm;

    uint8_t servo_sat_mask;
    uint8_t allocator_infeasible;
    uint8_t state_valid;
} LQI_Control_t;
~~~

接口：

~~~c
void LQI_Init(void);

void LQI_Update(
    float dt,
    const float attitude_error_rad[3],
    const float body_rate_rad_s[3],
    const float torque_achieved_Nm[3],
    float torque_cmd_Nm[3]);

void Torque_Allocator_Update50Hz(void);

void Torque_Allocate_PitchProtected(
    const float torque_cmd_Nm[3],
    float servo_cmd_deg[4],
    float torque_achieved_Nm[3]);
~~~

控制任务顺序：

1. 读取 IMU 姿态和机体系角速度；
2. 读取当前目标姿态；
3. 形成 [ROLL,PITCH,YAW] 误差；
4. 执行 LQI；
5. 执行 Pitch 保护型零空间分配；
6. 回算实际三轴力矩；
7. 根据饱和状态更新积分器；
8. 限制舵角；
9. 输出 PWM；
10. 记录控制状态。

异常时：

~~~text
状态无效 → 冻结积分 → 舵面回中 → 设置错误标志
~~~

LQI 不负责估计动态目标。视觉或其他制导层负责生成目标姿态；LQI 只负责姿态和角速度跟踪。

---

## 8. 速度调度

气动力矩矩阵通常随速度变化：

~~~matlab
H_tau(V) ≈ V^2 * H_tau_ref
~~~

MATLAB 可输出：

- K_lqi(V)；
- H_tau(V)；
- H_ry(V)；
- N_ry(V)。

推荐使用少量速度表和线性插值，不使用高阶多项式拟合全部 K。

嵌入式规则：

1. 速度无效时使用标称速度；
2. 速度限幅；
3. 50 Hz 更新矩阵和增益；
4. 1 kHz 使用最近一次完整参数；
5. 使用双缓冲；
6. 速度变化过小时不重复更新；
7. 记录实际调度速度和有效标志。

如果当前速度估计尚未验证，第一版固定使用标称速度，先验证零空间算法。

---

## 9. 外部扰动

MATLAB 加入：

~~~matlab
x_next = A_d*x + B_d*tau + E_d*d;
~~~

扰动场景：

- 恒定 Pitch 外部力矩；
- 恒定 Roll/Yaw 外部力矩；
- 阶跃扰动力矩；
- 短时冲击；
- 有界随机扰动；
- 舵面控制效能下降；
- 惯量和质心偏差。

第一版嵌入式不加入复杂扰动观测器。只用 IMU 时，角速度微分噪声容易被误判为外部力矩。先使用 LQI 积分处理慢变化和恒定扰动。

---

## 10. MATLAB 输出

脚本必须输出：

1. 连续 A、B_tau、E_dist；
2. 离散 A_d、B_d、E_d；
3. LQI 增益 K_lqi；
4. H_tau(V)；
5. H_ry(V)；
6. N_ry(V) 或计算零空间所需参数；
7. Pitch 力矩上限；
8. 舵面角度上限；
9. 速度表；
10. 闭环极点；
11. 力矩分配误差；
12. 舵面饱和统计；
13. Pitch 力矩平方积分；
14. 舵面动作平方积分；
15. 抛物线射程和速度损失代理指标。

C 导出优先使用静态表：

~~~c
static const float lqi_K_table[NUM_SPEED][3][9];
static const float torque_H_table[NUM_SPEED][3][4];
~~~

导出文件必须注明：

- 状态顺序；
- 力矩顺序；
- 舵面顺序；
- 单位；
- 采样周期；
- Q/R 配置；
- 速度范围；
- 生成时间。

---

## 11. MATLAB 验证

### 11.1 线性检查

- 惯量矩阵正定；
- 控制矩阵维度正确；
- ctrb 秩满足要求；
- H_tau 满秩；
- H_ry 满秩；
- 速度范围内没有严重奇异点；
- 所有闭环离散极点位于单位圆内；
- 增益没有 NaN/Inf。

### 11.2 Pitch 保护对比

分别仿真：

1. 只 Roll；
2. 只 Yaw；
3. Roll+Yaw；
4. 不使用零空间；
5. 使用完整三轴伪逆；
6. 使用 Pitch 保护零空间。

比较：

- Roll/Yaw 力矩误差；
- Pitch 实际力矩；
- Pitch 舵面动作；
- 总舵面动作；
- 舵面饱和次数；
- 实际实现力矩误差。

验收目标：使用零空间后，在 Roll/Yaw 任务基本不变的情况下，Pitch 力矩和舵面动作下降。

### 11.3 LQI 抗扰

验证：

- 无扰动收敛；
- 恒定扰动下无明显稳态误差；
- 舵面饱和后积分不发散；
- Pitch 保护触发后 Pitch 积分不继续推动控制；
- 目标变化不会产生异常 Pitch 冲击。

### 11.4 抛物线仿真

至少提供简化抛物线仿真，比较：

- 无控制射程；
- 普通 Roll/Yaw 控制射程；
- Pitch 保护控制射程；
- Pitch 力矩平方积分；
- 舵面动作平方积分；
- 速度损失。

没有可靠阻力模型时，结果只能标记为代理指标。

---

## 12. 嵌入式验证

### 12.1 编译检查

- 新 C/H 文件加入 Keil 工程；
- 数组维度一致；
- 单位转换明确；
- 无动态内存；
- 无矩阵越界；
- 不在 1 kHz 中使用不必要的 double；
- C 数组与 MATLAB 数值一致。

### 12.2 分配器单元测试

验证：

1. Roll 力矩方向；
2. Yaw 力矩方向；
3. Pitch 保护关闭时分配正常；
4. Pitch 保护开启后 Pitch 力矩下降；
5. Roll/Yaw 误差在允许范围内；
6. 四舵角不超限；
7. 奇异矩阵返回错误；
8. 速度无效时使用标称参数；
9. 舵面饱和时能回算实际力矩。

### 12.3 台架测试

顺序：

1. 舵面回中；
2. 单轴 Roll；
3. 单轴 Yaw；
4. Roll/Yaw 同时控制；
5. 检查 Pitch 力矩；
6. 对比零空间开启前后；
7. 检查积分抗饱和；
8. 检查速度调度；
9. 检查异常状态回中。

---

## 13. 实施顺序

### 阶段 A：MATLAB 基础模型

1. 创建新分支；
2. 新建参数文件；
3. 填入惯量；
4. 填入质心和舵面位置；
5. 建立 H_tau；
6. 检查矩阵秩和符号；
7. 建立简单 LQI；
8. 输出三轴力矩；
9. 完成闭环仿真。

### 阶段 B：Pitch 保护

1. 设置 Pitch 低权重；
2. 关闭 Pitch 积分；
3. 设置 Pitch 力矩上限；
4. 编写 MATLAB 零空间参考实现；
5. 对比普通伪逆和零空间分配；
6. 确定嵌入式解析公式。

### 阶段 C：嵌入式

1. 删除旧姿态控制调用；
2. 添加 LQI 结构体；
3. 添加静态 K 表；
4. 添加 H_tau 表；
5. 添加 Roll/Yaw 子矩阵零空间计算；
6. 添加舵面限幅；
7. 添加实际力矩回算；
8. 添加积分抗饱和；
9. 添加异常状态处理。

### 阶段 D：调度和扰动

1. 先固定标称速度；
2. 验证固定速度版本；
3. 加入速度表；
4. 加入速度有效性和回退；
5. MATLAB 加入外部扰动；
6. 暂不加入在线扰动观测器。

### 阶段 E：台架和飞行

1. MATLAB 数值检查；
2. Keil 编译；
3. 无动力台架；
4. 单轴低权限测试；
5. Roll/Yaw 测试；
6. 检查 Pitch 力矩和舵面动作；
7. 低权限飞行；
8. 再决定是否增加气动阻尼或 Pitch 权限。

---

## 14. 最终验收标准

必须同时满足：

1. MATLAB 输出三轴 N·m 力矩，而不是四舵角；
2. 嵌入式完成力矩到四舵角的转换；
3. 舵面顺序始终为 [UL,UR,DR,DL]；
4. 惯量、质心和舵面力矩臂进入模型；
5. Pitch 积分默认关闭；
6. Pitch 力矩有硬限制；
7. Roll/Yaw 控制时零空间能够降低 Pitch 力矩；
8. 舵面饱和时积分不会继续累积；
9. 速度无效时回退标称参数；
10. 奇异或不可行分配返回错误；
11. MATLAB、嵌入式和日志单位统一；
12. 测试结果区分 MATLAB 已验证和硬件待验证。

---

## 15. 给实施 AI 的硬性要求

1. 实施前先读取 AGENTS.md、CODE_OVERVIEW.md 和 PROGRESS.md。
2. 不修改已经确认正确的坐标、符号和舵面顺序。
3. 不把旧四舵角 K 矩阵直接改名当作新 LQI。
4. 不把 Pitch 状态强制写成零。
5. 不把动态目标误差用于气动参数标定。
6. 不把未经验证的阻尼参数直接启用。
7. 不用输出低通掩盖控制振荡。
8. 每次只改变一个主要变量并记录原因。
9. MATLAB、嵌入式和日志统一单位、轴序和舵面顺序。
10. 先做 MATLAB 数值验证，再做 C 编译和台架验证。
11. 没有实际执行的编译、烧录和飞行测试不得声称完成。

