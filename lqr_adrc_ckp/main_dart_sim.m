% =========================================================================
% MAIN_DART_SIM   无动力 X 构型飞镖：全系统 6DOF 自由飞行标准场景（顶层入口）
% =========================================================================
% 【这个脚本是干什么的】
%   前面 tests/ 里的 t1~t4 是“分项考试”（单独测内环、测姿态阶跃、测鲁棒性），
%   而本脚本是“综合实飞演示”：把飞镖当成真的从发射管里射出去，让它在空中一边
%   走弹道、一边被风吹、一边做滚转机动，把【完整控制链 + 完整 6DOF 动力学】端到端
%   跑一遍，最后出一张“综合体检报告”大图。
%
% 【和 att_bench(姿态台) 有什么不同？为什么要两套？】
%   - att_bench：把飞行速度“焊死”在 [V;0;0]（攻角/侧滑恒为 0、动压恒定），只演化转动+
%     姿态，像把飞镖架在风洞天平上测姿态环——干净、好测超调/调节时间，但不掉速、不走位移。
%   - 本脚本：真正的 12 状态自由飞行（位置+速度+姿态+角速度全演化），速度/弹道由气动力与
%     重力自然决定，风通过改变攻角α/侧滑β“真实地”吹它——这才是飞镖真实的样子。
%
% 【关键：无动力弹体怎么飞才“对”？（这是本场景设计的核心，务必看懂）】
%   飞镖没有动力，射出去就是一条抛物弹道、会往下掉。如果硬让它“机头一直水平/上扬某固定角”，
%   而速度方向却在往下偏 → 攻角 α 越来越大 → 静稳定力矩 Cmα·α 暴增 → 舵面拼命对抗直到饱和(失控)。
%   正确飞法是【俯仰跟踪弹道倾角 γ】：让机头顺着速度方向走，攻角 α≈0、阻力最小、最省舵。
%   所以本场景：俯仰自动跟踪弹道(顺气流)，偏航只增稳(目标0)，把“主动机动”放在不影响弹道的
%   【滚转】通道上演示姿态跟踪——既物理合理、又能展示控制器的跟踪与抗扰本事。
%   （提纲要求的“俯仰/滚转姿态角阶跃超调<20%、调节<0.5s”属姿态环指标，已由 t3/t4 在姿态台严格验证。）
%
% 【完整信号链（每 1ms / 1000Hz 走一遍）】
%   ① 传感器：陀螺测角速度(加噪声+一阶低通)，姿态角理想可测(IMU)
%   ② 外环 LQR：姿态角误差 → 期望角速度，叠加“参考微分前馈”(滚转指令在动时提前给速度，跟得更快)
%   ③ 内环 LADRC：期望角速度 → 期望力矩，并用扩张状态观测器(LESO)实时估计/对消“总扰动”
%   ④ X 构型控制分配：3 个期望力矩 → 4 个舵面偏角(用实时动压调度)
%   ⑤ 二阶舵机：±90°位置限、±200°/s速率限
%   ⑥ 6DOF 非线性动力学：实际舵偏+风 → 力/力矩 → RK4 积分出下一时刻 12 状态
%
% 【场景设计】初速 60 m/s 水平射出，初始姿态小扰动 [1.5,-1,1]°(模拟出管扰动)；
%   俯仰跟踪弹道(顺气流)、偏航增稳；1~3s 平滑滚转到 +10° 再平滑回 0；全程 Dryden 紊流 σ=1.0+常值东向侧风 2.5 m/s。
%   注：无动力抛射弹道会下坠，重力把势能转成动能 → 空速可能不降反升；提纲”60→20 大范围”属长
%   射程/大阻力工况，速度区间鲁棒性由 t4_robustness 按速度点扫频严格验证，这里只做真实飞行片段。
%
% 【怎么运行】(详见 DEPLOY_GUIDE.md)
%   MATLAB 命令窗口：  cd 到本项目目录;  addpath(genpath(pwd));  main_dart_sim
%   或命令行：  matlab -batch "cd('E:/DevelopLotteany1/Simulink_Guidence_lqr_adrc'); addpath(genpath(pwd)); main_dart_sim"
% =========================================================================
clear; clc; close all;
projroot = fileparts(mfilename('fullpath'));      % 本文件所在目录=项目根
addpath(genpath(projroot));
params = dart_params();                            % 一键载入全部参数

% ---------------- 仿真基本设置 ----------------
Ts = params.sim.Ts;            % 步长 1e-3 s（1000 Hz）
T  = 4.0;                      % 仿真总时长 (s)
N  = round(T/Ts);              % 总步数
tvec = (0:N-1)'*Ts;            % 时间轴 (N×1)

% ---------------- 预先取出常用量 ----------------
K     = lqr_outer_design(params);                 % 外环 LQR 增益 K(3×3)
rho   = params.env.rho;  S = params.body.S;  d = params.body.d;
E     = params.alloc.E;                           % X 构型几何混控矩阵 (3×4)
Ceff  = [params.aero.Clda; params.aero.Cmde; params.aero.Cndr];  % 三通道舵效系数
omax  = params.lqr.omega_max;                     % 角速度指令限幅 (3×1)
gyroN = params.sensor.gyro_noise;                 % 陀螺噪声 RMS
a_lpf = exp(-2*pi*params.sensor.lpf_fc*Ts);       % 陀螺一阶低通系数

% ---------------- 预生成风序列（Dryden 紊流 + 常值侧风）----------------
% ±90° 舵机舵权充裕，用中等风展示抗扰能力（σ=1.0 + 侧风 2.5 m/s）。
% 注：最大风(σ=1.5+侧风4)仍可稳定飞行，但 X构型偏航/滚转耦合会使滚转跟踪稍超调。
windSig    = 1.0;                                  % 演示紊流强度 (m/s)
windSteady = [0, 2.5, 0];                          % 演示常值东向侧风 2.5 m/s
p_demo = params;  p_demo.wind.sigma = windSig;
windG = dryden_wind(tvec, 50, p_demo, 7);          % N×3 紊流 (NED, m/s)，seed=7 可复现
wind_series = windG + windSteady;                  % 叠加常值侧风

% ---------------- 滚转机动指令（smoothstep 平滑，使微分前馈有意义、无冲击）----------------
% smoothstep: clamp 到[0,1]后 3x²-2x³，起止斜率为0 → 指令平滑、其导数(前馈)有界。
clamp01 = @(x) max(0, min(1, x));
ss      = @(x) 3*clamp01(x).^2 - 2*clamp01(x).^3;
phiCmd  = @(t) deg2rad(10)*( ss((t-1)/0.4) - ss((t-3)/0.4) );   % 1s起平滑升到10°(0.4s完成)，3s回0

% ---------------- 初始状态 x(12) = [pN pE pD; u v w; φ θ ψ; p q r] ----------------
V0   = params.speed.V0;                            % 初速 60 m/s（沿体 X 水平射出）
att0 = deg2rad([1.5; -1; 1]);                      % 初始姿态扰动(出管小扰动，贴近真实量级)
x = [0;0;0;  V0;0;0;  att0;  0;0;0];

% ---------------- 控制器/执行器内部状态 ----------------
st_l = [];  st_s = [];                             % LADRC / 舵机 状态(空=自动初始化)
wf = zeros(3,1);                                   % 陀螺低通滤波器状态
tau_act = zeros(3,1);                              % 上一步“实际执行力矩”(抗饱和用)
phi_prev = phiCmd(-Ts);                            % 上一步滚转指令(算微分前馈用)

% ---------------- 记录容器（预分配）----------------
R.t=tvec; R.pos=zeros(N,3); R.V=zeros(N,1); R.att=zeros(N,3); R.ref=zeros(N,3);
R.pqr=zeros(N,3); R.cmd=zeros(N,3); R.tau=zeros(N,3); R.delta=zeros(N,4);
R.wind=zeros(N,3); R.z2=zeros(N,3); R.alpha=zeros(N,1); R.beta=zeros(N,1);
R.qbar=zeros(N,1); R.sat=false(N,1); R.gamma=zeros(N,1);

rng(0);                                            % 固定随机种子，结果可复现
% ======================= 主循环（1000 Hz 定步长）=======================
for k = 1:N
    t = tvec(k);
    att = x(7:9);  pqr_true = x(10:12);            % 当前真实姿态 / 角速度

    % ① 传感器：角速度加陀螺噪声 + 一阶低通；姿态角理想可测
    wm = pqr_true + gyroN*randn(3,1);
    wf = a_lpf*wf + (1-a_lpf)*wm;

    % ② 参考：俯仰跟踪弹道倾角γ(顺气流，α≈0 最省舵)；滚转机动；偏航增稳
    R_b2n = dcm_ned2body(att(1),att(2),att(3))';   % 机体→NED
    Vned  = R_b2n * x(4:6);                         % 速度在 NED 的分量
    gamma = atan2(-Vned(3), hypot(Vned(1),Vned(2)));% 弹道倾角(向上为正)
    ref   = [phiCmd(t); gamma; 0];
    % 外环 LQR + 滚转微分前馈 → 期望角速度（限幅防过激）
    omega_ff  = [(phiCmd(t)-phi_prev)/Ts; 0; 0];  phi_prev = phiCmd(t);
    omega_cmd = K*(ref - att) + omega_ff;
    omega_cmd = max(min(omega_cmd, omax), -omax);

    % ③ 内环 LADRC（传上一步实际力矩做抗饱和）→ 期望力矩
    [tau, st_l] = ladrc_inner(omega_cmd, wf, st_l, params, tau_act);

    % ④ 控制分配：用“控制器侧动压估计”(地速)→ 4 舵偏
    V = norm(x(4:6));  qbar_est = 0.5*rho*V^2;
    [dcmd, ainfo] = control_alloc(tau, qbar_est, params);

    % ⑤ 二阶舵机（±25°/±200°/s）→ 实际舵偏
    [delta, st_s] = actuator_2nd(dcmd, st_s, params);

    % 实际执行力矩（抗饱和反馈给 LADRC）
    B_eff   = diag(qbar_est*S*d*Ceff) * E;
    tau_act = B_eff * delta;

    % ⑥ 当前步风 + 记录气动信息（真实力矩由 dart_aero 内部含风动压计算）
    wind = wind_series(k,:)';
    [~, ~, info] = dart_aero(x, delta, wind, params);

    % ---- 记录本步 ----
    R.pos(k,:)=x(1:3)'; R.V(k)=info.V; R.att(k,:)=att'; R.ref(k,:)=ref';
    R.pqr(k,:)=pqr_true'; R.cmd(k,:)=omega_cmd'; R.tau(k,:)=tau'; R.delta(k,:)=delta';
    R.wind(k,:)=wind'; R.z2(k,:)=st_l.z2'; R.alpha(k)=info.alpha; R.beta(k)=info.beta;
    R.qbar(k)=info.qbar; R.sat(k)=ainfo.sat; R.gamma(k)=gamma;

    % ⑦ 6DOF 非线性动力学 RK4 积分一步 → 下一时刻状态
    x = rk4_step(x, delta, wind, params, Ts);
end

% ======================= 关键指标 =======================
hold_mask = R.t>=2.5 & R.t<=3.0;                   % 2.5~3s：滚转保持 10°
phi_track = mean(rad2deg(R.att(hold_mask,1))) - 10;
alpha_rms = rad2deg(sqrt(mean(R.alpha.^2)));       % 攻角RMS(顺气流应小)
beta_rms  = rad2deg(sqrt(mean(R.beta.^2)));        % 侧滑RMS
psi_dev   = max(abs(rad2deg(R.att(:,3))));         % 偏航增稳：全程最大偏离
maxd      = max(abs(rad2deg(R.delta(:))));         % 峰值舵偏
nsat      = sum(R.sat);                            % 触饱和步数

fprintf('================= MAIN 6DOF 自由飞行标准场景 =================\n');
fprintf('  仿真: T=%.1fs, 1000Hz, 初速 %.0f m/s, Dryden紊流σ=%.1f + 常值侧风 %g m/s(东)\n', ...
        T, V0, windSig, windSteady(2));
fprintf('  弹道: 空速 %.1f->%.1f m/s, 弹道倾角 %.1f->%.1f deg, 下坠 %.1f m\n', ...
        R.V(1), R.V(end), rad2deg(R.gamma(1)), rad2deg(R.gamma(end)), -R.pos(end,3));
fprintf('  滚转机动跟踪(2.5~3s保持10°): 误差 %+.3f deg\n', phi_track);
fprintf('  俯仰顺气流: 攻角α RMS=%.2f deg (跟踪弹道→α≈0)\n', alpha_rms);
fprintf('  偏航增稳: 全程最大 |psi|=%.2f deg (侧滑β RMS=%.2f deg)\n', psi_dev, beta_rms);
fprintf('  峰值舵偏 %.1f deg (上限25°)，触饱和步数 %d/%d\n', maxd, nsat, N);

% ======================= 出综合报告图 =======================
plot_results(R, params, projroot);
fprintf('完成。综合报告图已保存到 analysis/figs/MAIN_overview.png\n');
