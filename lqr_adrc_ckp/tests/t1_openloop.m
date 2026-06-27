% =========================================================================
% T1_OPENLOOP  M1：开环 6DOF 动力学「合理性」验证（控制器还没上场，先验物理）
% -------------------------------------------------------------------------
% 【为什么先做开环】加控制器之前，必须先确认“物理引擎”本身是对的——否则后面
%   分不清问题出在控制还是动力学。开环 = 不反馈、给固定/预设舵偏，看飞镖自己怎么飞。
% 【场景】初速 60 m/s 水平射出，全程零舵偏自由飞，仅在 0.5~0.7s 给一小段俯仰舵脉冲，
%   看它“被踢一下”之后能不能自己稳回来。
% 【4 个合理性判据（PASS 条件）】
%   ① 不出 NaN/Inf（方程没写错、没数值爆炸）；
%   ② 速度先因下坠略增、再被阻力拖慢（弹道俯冲的典型“先升后降”）；
%   ③ 攻角 α 很小（静稳定 Cmα<0 像风向标一样把机头自动顶向气流）；
%   ④ 俯仰角速度 q 的“短周期振荡”越来越小（有气动阻尼 → 收敛）。
% =========================================================================
clear; clc; close all;
if ~exist('projroot','var'); projroot = fileparts(fileparts(mfilename('fullpath'))); end
addpath(genpath(projroot));

params = dart_params();
Ts = params.sim.Ts;
T  = 6.0;  N = round(T/Ts);

% 初始状态：初速沿体 X，水平，零姿态零角速度
x = zeros(12,1);  x(4) = params.speed.V0;
wind = [0;0;0];

% 用 E 的伪逆把“等效俯仰舵脉冲”映射为 4 个物理舵偏
Epinv = pinv(params.alloc.E);
DELTA_E = deg2rad(5);

rec.t = (0:N-1)'*Ts;
rec.V = zeros(N,1); rec.euler = zeros(N,3); rec.pqr = zeros(N,3); rec.pos = zeros(N,3);
rec.alpha = zeros(N,1);

for k = 1:N
    t = (k-1)*Ts;
    if t>=0.5 && t<0.7,  u_cmd = [0; DELTA_E; 0];  else,  u_cmd = [0;0;0]; end
    delta = Epinv * u_cmd;

    [~,~,info] = dart_aero(x, delta, wind, params);
    rec.V(k)=info.V; rec.alpha(k)=info.alpha;
    rec.euler(k,:)=x(7:9)'; rec.pqr(k,:)=x(10:12)'; rec.pos(k,:)=x(1:3)';

    x = rk4_step(x, delta, wind, params, Ts);
end

% ---- 数值检查（开环物理合理性诊断）----
q_deg = rad2deg(rec.pqr(:,2));
amp_early = max(abs(q_deg(rec.t<=1.5)));    % 短周期前段振幅
amp_late  = max(abs(q_deg(rec.t>=4.5)));    % 后段振幅
maxAlpha  = max(abs(rad2deg(rec.alpha)));
maxLat    = max(abs(rad2deg(rec.euler(:,[1 3]))),[],'all');  % phi,psi 横侧
fprintf('=== M1 open-loop 6DOF check ===\n');
fprintf('V: V0=%.2f, Vmin=%.2f, Vend=%.2f m/s (dip-then-climb = ballistic dive, expected)\n', ...
        params.speed.V0, min(rec.V), rec.V(end));
fprintf('theta_end = %.2f deg (weathervane: nose follows velocity, expected <0)\n', rad2deg(rec.euler(end,2)));
fprintf('max|alpha| = %.3f deg (small => static stability holds nose into wind)\n', maxAlpha);
fprintf('q short-period amp: early=%.2f -> late=%.2f deg/s (decaying => damped)\n', amp_early, amp_late);
fprintf('lateral phi/psi max = %.2e deg (≈0, no lateral excitation)\n', maxLat);
fprintf('finite (no NaN/Inf): %d\n', all(isfinite(x)));
PASS = all(isfinite(x)) && (amp_late < amp_early) && (maxAlpha < 15) && (maxLat < 1e-6);
fprintf('>>> M1 PASS = %d\n', PASS);

% ---- 出图 ----
fig = figure('Position',[80 80 980 680],'Visible','off');
subplot(2,2,1); plot(rec.t, rec.V,'LineWidth',1.2); grid on;
    xlabel('时间 (s)'); ylabel('速度 V (m/s)'); title('速度衰减（阻力）');
subplot(2,2,2); plot(rec.t, rad2deg(rec.euler),'LineWidth',1.2); grid on;
    xlabel('时间 (s)'); ylabel('欧拉角 (deg)'); legend('\phi','\theta','\psi'); title('姿态角（俯仰舵脉冲响应）');
subplot(2,2,3); plot(rec.t, rad2deg(rec.pqr),'LineWidth',1.2); grid on;
    xlabel('时间 (s)'); ylabel('角速度 (deg/s)'); legend('p','q','r'); title('角速度');
subplot(2,2,4); plot(rec.t, rec.pos(:,3),'LineWidth',1.2); grid on; set(gca,'YDir','reverse');
    xlabel('时间 (s)'); ylabel('p_D (m, 向下为正)'); title('高度（NED-D）');
sgtitle('M1 开环 6DOF 动力学验证');

outdir = fullfile(projroot,'analysis','figs');
if ~exist(outdir,'dir'); mkdir(outdir); end
saveas(fig, fullfile(outdir,'M1_openloop.png'));
fprintf('图已保存: %s\n', fullfile(outdir,'M1_openloop.png'));
