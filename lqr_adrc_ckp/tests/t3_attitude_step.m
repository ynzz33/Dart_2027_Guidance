% =========================================================================
% T3_ATTITUDE_STEP   M3：全闭环姿态阶跃验证（这是整个控制方案的“总考试”）
% -------------------------------------------------------------------------
% 【这个脚本在干什么？】
%   把完整控制链跑起来，给飞镖一个“姿态角目标突然跳变”（阶跃）的指令，看它能不能
%   又快又稳地转到目标角度。完整链路（每 1ms 走一遍）：
%     传感器(陀螺测角速度,带噪声+低通)
%       → 外环 LQR（姿态角差 → 期望角速度）
%       → 内环 LADRC（期望角速度 → 期望力矩，并实时估计/对消扰动）
%       → X构型控制分配（3个力矩 → 4个舵面偏角）
%       → 二阶舵机（带 ±25°位置限 / ±200°/s速率限）
%       → 6DOF 转动方程（力矩 → 角加速度 → 角速度 → 姿态角）
%
% 【为什么用“小角度”阶跃（滚转10°、俯仰5°）？】
%   无动力飞镖靠惯性飞行，姿态控制的任务是“小幅修正姿态 + 增稳”，不是做大机动翻滚。
%   小角度下舵面不会饱和、刹车距离短，正是飞镖真实的工作工况。
%
% 【这是“姿态控制台”仿真】
%   把飞行速度固定在 50 m/s（动压恒定），只演化“转动+姿态”，不算掉高/减速。
%   这样能干净地测“姿态环”的快慢与超调，不被弹道下落干扰（速度衰减的全程鲁棒性留给 M4）。
%
% 【3 个考核指标（提纲要求）】
%   1) 超调 < 20%      —— 冲过头不能太多
%   2) 调节时间 < 0.5s —— 进入并稳定在目标 ±2% 带内的时间（@50 m/s）
%   3) 无静差          —— 最终稳稳停在目标角，残余误差≈0
%   偏航只做“增稳”（目标=0，看它在滚转/俯仰耦合下是否被压住，不主动跟踪）。
%
% 【对比演示】同一套控制器结构，分别用“提纲起点参数”和“整定后参数”各跑一遍，
%   直观看到整定（wc:30→16, K:3.16→6）如何把“慢/超调”变成“快又稳”。
% =========================================================================
clear; clc; close all;
if ~exist('projroot','var'); projroot = fileparts(fileparts(mfilename('fullpath'))); end
addpath(genpath(projroot));
params = dart_params();

% ---- 两套参数：提纲起点 vs 整定后（params 默认即整定后）----
p_outline = params;  p_outline.adrc.wc = 30;   % 提纲起点：内环带宽 30
Q_outline = diag([10 10 1]);                    % 提纲起点：K≈3.16
Q_tuned   = params.lqr.Q;                       % 整定后：K=6（diag(36,36,3.6)）
omax      = params.lqr.omega_max;               % 角速度指令限幅（两者同用，只对比 wc/Q）

% ---- 阶跃指令（单通道隔离，指标最干净）----
rollDeg = 10;  pitchDeg = 5;  T = 1.5;
ref_roll  = @(t)[deg2rad(rollDeg)*(t>=0.2);  0;  0];   % 仅滚转阶跃 10° @0.2s
ref_pitch = @(t)[0;  deg2rad(pitchDeg)*(t>=0.2);  0];  % 仅俯仰阶跃  5° @0.2s

% ---- 跑 4 次：{滚转,俯仰} × {提纲起点,整定后}；rng(0) 固定噪声，保证可比 ----
rng(0); r0_roll  = att_bench(p_outline, Q_outline, ref_roll,  T, omax);
rng(0); rT_roll  = att_bench(params,    Q_tuned,   ref_roll,  T, omax);
rng(0); r0_pitch = att_bench(p_outline, Q_outline, ref_pitch, T, omax);
rng(0); rT_pitch = att_bench(params,    Q_tuned,   ref_pitch, T, omax);

% ---- 计算性能指标 ----
mr  = perf_metrics(rT_roll.t,  rad2deg(rT_roll.att(:,1)),  0.2, 0, rollDeg);   % 整定后-滚转
mp  = perf_metrics(rT_pitch.t, rad2deg(rT_pitch.att(:,2)), 0.2, 0, pitchDeg);  % 整定后-俯仰
mr0 = perf_metrics(r0_roll.t,  rad2deg(r0_roll.att(:,1)),  0.2, 0, rollDeg);   % 提纲起点-滚转
mp0 = perf_metrics(r0_pitch.t, rad2deg(r0_pitch.att(:,2)), 0.2, 0, pitchDeg);  % 提纲起点-俯仰
yawR = max(abs(rad2deg(rT_roll.att(:,3))));    % 滚转机动时偏航耦合（增稳效果）
yawP = max(abs(rad2deg(rT_pitch.att(:,3))));   % 俯仰机动时偏航耦合
mdR  = max(abs(rad2deg(rT_roll.delta(:))));    % 滚转场景峰值舵偏
mdP  = max(abs(rad2deg(rT_pitch.delta(:))));   % 俯仰场景峰值舵偏

fprintf('=== M3 closed-loop attitude step @V=50 m/s (roll %d deg, pitch %d deg) ===\n', rollDeg, pitchDeg);
fprintf('--- outline start (wc=30, K=3.16) ---\n');
fprintf('  Roll : OS=%5.1f%%  ts=%6.3fs  ess=%6.3f deg\n', mr0.overshoot, mr0.settling, mr0.ess);
fprintf('  Pitch: OS=%5.1f%%  ts=%6.3fs  ess=%6.3f deg\n', mp0.overshoot, mp0.settling, mp0.ess);
fprintf('--- tuned (wc=16, K=6) [params default] ---\n');
fprintf('  Roll : OS=%5.1f%%  ts=%6.3fs  tr=%.3fs  ess=%6.3f deg  maxDelta=%.1f deg\n', mr.overshoot, mr.settling, mr.rise, mr.ess, mdR);
fprintf('  Pitch: OS=%5.1f%%  ts=%6.3fs  tr=%.3fs  ess=%6.3f deg  maxDelta=%.1f deg\n', mp.overshoot, mp.settling, mp.rise, mp.ess, mdP);
fprintf('  Yaw stabilization: max|psi|=%.3f deg (roll case), %.3f deg (pitch case)\n', yawR, yawP);

% ---- 判定（基于整定后）：超调<20% & 调节<0.5s & 静差<0.5deg & 有限 ----
PASS = mr.overshoot<20 && mp.overshoot<20 && mr.settling<0.5 && mp.settling<0.5 ...
    && abs(mr.ess)<0.5 && abs(mp.ess)<0.5 ...
    && all(isfinite(rT_roll.att(end,:))) && all(isfinite(rT_pitch.att(end,:)));
fprintf('>>> M3 PASS = %d\n', PASS);

% ====================== 出图 ======================
fig = figure('Position',[40 40 1180 740],'Visible','off');

% (1) 滚转角阶跃响应：参考 / 整定后 / 提纲起点
subplot(2,3,1);
  plot(rT_roll.t, rad2deg(rT_roll.ref(:,1)),'k--','LineWidth',1.2); hold on;
  plot(rT_roll.t, rad2deg(rT_roll.att(:,1)),'b','LineWidth',1.6);
  plot(r0_roll.t, rad2deg(r0_roll.att(:,1)),'r:','LineWidth',1.4); grid on;
  xlabel('时间 (s)'); ylabel('滚转 \phi (deg)');
  title(sprintf('滚转阶跃 %d° (整定后 OS=%.1f%%, ts=%.2fs)', rollDeg, mr.overshoot, mr.settling));
  legend('参考目标','整定后(wc16,K6)','提纲起点(wc30,K3.16)','Location','southeast');

% (2) 俯仰角阶跃响应
subplot(2,3,2);
  plot(rT_pitch.t, rad2deg(rT_pitch.ref(:,2)),'k--','LineWidth',1.2); hold on;
  plot(rT_pitch.t, rad2deg(rT_pitch.att(:,2)),'b','LineWidth',1.6);
  plot(r0_pitch.t, rad2deg(r0_pitch.att(:,2)),'r:','LineWidth',1.4); grid on;
  xlabel('时间 (s)'); ylabel('俯仰 \theta (deg)');
  title(sprintf('俯仰阶跃 %d° (整定后 OS=%.1f%%, ts=%.2fs)', pitchDeg, mp.overshoot, mp.settling));
  legend('参考目标','整定后','提纲起点','Location','southeast');

% (3) 角速度：指令 vs 实测（内环跟踪质量）
subplot(2,3,3);
  plot(rT_roll.t, rad2deg(rT_roll.cmd(:,1)),'b--','LineWidth',1.0); hold on;
  plot(rT_roll.t, rad2deg(rT_roll.pqr(:,1)),'b','LineWidth',1.3);
  plot(rT_pitch.t, rad2deg(rT_pitch.cmd(:,2)),'m--','LineWidth',1.0);
  plot(rT_pitch.t, rad2deg(rT_pitch.pqr(:,2)),'m','LineWidth',1.3); grid on;
  xlabel('时间 (s)'); ylabel('角速度 (deg/s)'); title('内环跟踪：角速度指令 vs 实测');
  legend('p_{cmd}','p','q_{cmd}','q','Location','northeast');

% (4) LADRC 期望力矩（滚转场景）
subplot(2,3,4);
  plot(rT_roll.t, rT_roll.tau,'LineWidth',1.3); grid on;
  xlabel('时间 (s)'); ylabel('期望力矩 (N·m)'); title('LADRC 输出力矩（滚转场景）');
  legend('L_c(滚转)','M_c(俯仰)','N_c(偏航)','Location','northeast');

% (5) 4 个舵面偏角（滚转场景），叠加 ±25° 限幅线
subplot(2,3,5);
  plot(rT_roll.t, rad2deg(rT_roll.delta),'LineWidth',1.1); grid on; hold on;
  yline( rad2deg(params.servo.pos_max),'r--','+25°');
  yline(-rad2deg(params.servo.pos_max),'r--','-25°');
  xlabel('时间 (s)'); ylabel('舵偏 (deg)'); title('4 舵面偏角（滚转场景，未饱和）');
  legend('\delta_1','\delta_2','\delta_3','\delta_4','Location','northeast');

% (6) 偏航增稳：两种机动下偏航角都被压在 0 附近
subplot(2,3,6);
  plot(rT_roll.t,  rad2deg(rT_roll.att(:,3)), 'b','LineWidth',1.3); hold on;
  plot(rT_pitch.t, rad2deg(rT_pitch.att(:,3)),'m','LineWidth',1.3); grid on;
  xlabel('时间 (s)'); ylabel('偏航 \psi (deg)'); title('偏航增稳（目标=0，耦合扰动被压住）');
  legend('滚转机动时','俯仰机动时','Location','best');

sgtitle('M3 全闭环姿态阶跃：LQR前馈 + LADRC + X构型分配 + 二阶舵机 @50 m/s');
outdir = fullfile(projroot,'analysis','figs'); if ~exist(outdir,'dir'); mkdir(outdir); end
saveas(fig, fullfile(outdir,'M3_attitude_step.png'));
fprintf('图已保存: %s\n', fullfile(outdir,'M3_attitude_step.png'));
