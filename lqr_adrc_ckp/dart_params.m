function params = dart_params()
% DART_PARAMS  无动力 X 构型飞镖仿真参数（一键初始化）
%   params = dart_params() 返回包含所有子系统参数的结构体。
%   单位统一 SI（m, kg, s, rad）；角度内部一律用弧度。
%   气动系数为“细长轴对称弹体”的合理估算值，可随时替换为实测/CFD 数据。
%
%   字段分组：env / body / speed / aero / servo / sensor / lqr / adrc / alloc / sim

%% ---------- 环境常数 ----------
params.env.g   = 9.81;      % 重力加速度 (m/s^2)
params.env.rho = 1.225;     % 海平面空气密度 (kg/m^3)

%% ---------- 几何 / 质量 / 惯量 ----------
params.body.m   = 0.5;                          % 质量 (kg)
params.body.d   = 0.05;                         % 参考直径/特征长度 (m)
params.body.S   = pi*(params.body.d/2)^2;       % 参考面积 (m^2)
params.body.Ixx = 1.0e-3;                       % 滚转惯量（细长体，小）(kg·m^2)
params.body.Iyy = 2.0e-2;                       % 俯仰惯量
params.body.Izz = 2.0e-2;                       % 偏航惯量（轴对称 Izz=Iyy）
params.body.I   = diag([params.body.Ixx params.body.Iyy params.body.Izz]);

%% ---------- 速度剖面 ----------
params.speed.V0 = 60;       % 初速 (m/s)
params.speed.Vf = 20;       % 期望末速量级 (m/s)，由阻力自然衰减得到

%% ---------- 气动系数（细长轴对称弹体典型估算，可替换） ----------
params.aero.Cd0  = 0.30;    % 零升阻力系数
params.aero.CLa  = 2.0;     % 升力线斜率 dCL/dα (1/rad)
params.aero.CYb  = -2.0;    % 侧力 dCY/dβ (1/rad)
params.aero.Cma  = -8.0;    % 俯仰静稳定 dCm/dα (1/rad)，负=静稳定
params.aero.Cnb  =  8.0;    % 偏航静稳定 dCn/dβ (1/rad)，正=方向稳定
params.aero.Clp  = -3.0;    % 滚转阻尼（含 d/2V 无量纲化约定）
params.aero.Cmq  = -15.0;   % 俯仰阻尼
params.aero.Cnr  = -15.0;   % 偏航阻尼
params.aero.Clda = 0.15;    % 滚转舵效 dCl/dδa
params.aero.Cmde = -1.2;    % 俯仰舵效 dCm/dδe
params.aero.Cndr = -0.10;   % 偏航舵效 dCn/dδr —— 故意取小：偏航效率极低

%% ---------- 舵机（二阶 + 限幅/速率） ----------
params.servo.wn       = 2*pi*15;        % 自然频率 (rad/s)，~15 Hz 带宽
params.servo.zeta     = 0.7;            % 阻尼比
params.servo.pos_max  = deg2rad(90);    % 位置限幅 ±90°（舵机总行程 180°）
params.servo.rate_max = deg2rad(200);   % 速率限幅 ±200°/s

%% ---------- 传感器 ----------
params.sensor.gyro_noise = 0.01;        % 陀螺噪声 RMS (rad/s)
params.sensor.lpf_fc     = 50;          % 角速度一阶低通截止 (Hz)
params.sensor.use_yaw    = true;        % 偏航是否可测（true=测量，仅增稳）

%% ---------- 外环 LQR（模型 [φ̇;θ̇;ψ̇]=[p;q;r] => A=0, B=I） ----------
% 提纲起点 Q=diag(10,10,1)/R=diag(1,1,0.1) => K≈3.16，姿态环 τ≈0.32s、ts≈2.7s 过慢。
% 整定：保持 10:10:1 相对权重，提增益至 K=6（Q=diag(36,36,3.6)）；与内环 wc=16 配合，
% 闭环 ωn≈√(wc·K)≈9.8 rad/s、ζ≈0.82 => 滚转/俯仰阶跃 OS<1%、ts≈0.38s（见 README 整定指南）。
params.lqr.Q = diag([36 36 3.6]);       % 姿态角权重（K=6，偏航同步）
params.lqr.R = diag([1  1  0.1]);       % 角速度指令权重
params.lqr.omega_max = deg2rad([200;120;120]);  % 角速度指令限幅（防大误差饱和失稳）
% 增益 K 由 lqr_outer_design.m 计算

%% ---------- 内环 LADRC（二阶 LESO，逐通道） ----------
% wc：提纲起点 30 会使初始角加速度需求 wc·K·θ 远超舵机可产生的角加速度 → 饱和失稳；
%     整定至 16，使各级带宽匹配（外环 K=6 ≪ 内环 wc=16 ≪ 舵机 ωn≈94），全程不饱和。
params.adrc.w0 = 100;       % 观测器带宽 (rad/s)，约 6×wc
params.adrc.wc = 16;        % 控制器带宽 (rad/s)
params.adrc.b0 = [1/params.body.Ixx, 1/params.body.Iyy, 1/params.body.Izz];  % 各通道控制增益 b0=1/I

%% ---------- X 构型控制分配 ----------
% 4 个舵面（尾视：1=右上 2=左上 3=左下 4=右下），舵轴与体 X 轴成 45°。
% 几何混控矩阵 E (3x4)：行=[滚转;俯仰;偏航]，滚转/俯仰含 cos45°，偏航为差动（效率由 Cndr 体现）。
c45 = cosd(45);
params.alloc.E = [  c45  -c45  -c45   c45;     % 滚转：相邻反号
                    c45   c45  -c45  -c45;     % 俯仰：上正下负
                    1    -1     1    -1   ];   % 偏航：差动
params.alloc.W = diag([1 1 0.1]);   % 输出加权：偏航稍轻（Cndr=-0.1 效率较低），但 ±90° 舵机舵权充裕；
                                    % 0.1 折中：比旧 0.05 给更多偏航主动控制权，但不至于在强侧风下挤占滚转/俯仰
params.alloc.eps = 1e-6;            % 加权最小二乘正则项

%% ---------- 仿真设置 ----------
params.sim.Ts   = 1e-3;     % 步长 (s) => 1000 Hz
params.sim.Tend = 3.0;      % 默认时长 (s)

%% ---------- 风场（Dryden 紊流 + 常值侧风）----------
% 【风扰从哪来、影响什么】风改变飞镖感受到的相对气流方向：侧风→侧滑角 β、上下风→攻角 α，
% 由此产生额外气动力矩扰动姿态。控制器（尤其 LADRC 的扩张状态观测器）负责把它估出并压住。
params.wind.sigma  = 1.5;            % 紊流强度 RMS (m/s)，中等湍流量级
params.wind.L      = [200;200;50];   % 紊流尺度长度 [纵;横;垂] (m)，低空典型值
params.wind.steady = [0;4;0];        % 常值侧风（NED：北/东/地），此处东向 4 m/s

end
