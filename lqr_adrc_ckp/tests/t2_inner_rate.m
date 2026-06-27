% =========================================================================
% T2_INNER_RATE  M2：内环 LADRC「角速度闭环」验证（阶跃跟踪 + 抗扰）
% -------------------------------------------------------------------------
% 【这一关单独考谁】只考内环 LADRC 本身，把它从整条链里“抠出来”单独测：
%   用最理想的对象 I·ω̇ = τ_LADRC + d(t)（力矩瞬间执行、不经分配/舵机/饱和），
%   这样若有问题就一定是 LADRC 的，而不是别的环节——这叫“隔离测试”。
% 【考什么】
%   ① 跟踪：给角速度阶跃指令（p:+100°/s、q:−60°/s），看实测能否准确跟上、无静差；
%   ② 抗扰：在 1.2s 往滚转通道注入一个常值扰动力矩 d，看 LESO 的 z2 能否把它估出来
%      （理论上 z2 应收敛到 d/Ixx），从而控制量自动补偿、角速度几乎不被带偏。
% 【PASS 条件】结果有限 + 三通道稳态误差 < 0.5°/s。
% =========================================================================
clear; clc; close all;
if ~exist('projroot','var'); projroot=fileparts(fileparts(mfilename('fullpath'))); end
addpath(genpath(projroot));
params = dart_params();
Ts = params.sim.Ts; T = 2.0; N = round(T/Ts);
Ivec = [params.body.Ixx; params.body.Iyy; params.body.Izz];

omega = zeros(3,1);   st = [];     % 实测=真值（M2 先验证标称跟踪，噪声留待 M3/M4）
rec.t=(0:N-1)'*Ts; rec.cmd=zeros(N,3); rec.w=zeros(N,3); rec.u=zeros(N,3); rec.z2=zeros(N,3);

d_roll = 0.05;        % 注入到滚转通道的常值扰动力矩 (N·m) @1.2s
for k=1:N
    t=(k-1)*Ts;
    % 角速度指令：p 阶跃 +100°/s @0.2s；q 阶跃 -60°/s @0.8s；r 保持 0（仅增稳）
    cmd = [deg2rad(100)*(t>=0.2); deg2rad(-60)*(t>=0.8); 0];
    d   = [d_roll*(t>=1.2); 0; 0];                 % 外部扰动力矩

    [u, st] = ladrc_inner(cmd, omega, st, params); % u = 期望力矩
    omega = omega + Ts*((u + d)./Ivec);            % 理想对象积分（力矩直接执行 + 扰动）

    rec.cmd(k,:)=cmd'; rec.w(k,:)=omega'; rec.u(k,:)=u'; rec.z2(k,:)=st.z2';
end

% ---- 指标（英文诊断）----
fprintf('=== M2 inner-loop LADRC check ===\n');
ch={'p','q','r'};
for i=1:3
  ess = rad2deg(rec.w(end,i)-rec.cmd(end,i));
  % 上升时间（到指令 90%），仅对有阶跃的通道
  fprintf('ch %s: cmd_end=%6.1f, w_end=%6.1f deg/s, ess=%7.3f deg/s\n', ...
          ch{i}, rad2deg(rec.cmd(end,i)), rad2deg(rec.w(end,i)), ess);
end
fprintf('z2_roll_end = %.2f rad/s^2 (≈ d/Ixx = %.2f, ESO estimates disturbance)\n', ...
        rec.z2(end,1), d_roll/params.body.Ixx);
ess_all = max(abs(rad2deg(rec.w(end,:)-rec.cmd(end,:))));
PASS = all(isfinite(omega)) && (ess_all < 0.5);     % 稳态误差 <0.5 deg/s
fprintf('finite=%d, max|ess|=%.3f deg/s, >>> M2 PASS = %d\n', all(isfinite(omega)), ess_all, PASS);

% ---- 出图 ----
fig=figure('Position',[80 80 1000 640],'Visible','off');
nm={'滚转 p','俯仰 q','偏航 r'};
for i=1:3
  subplot(2,3,i); plot(rec.t,rad2deg(rec.cmd(:,i)),'--','LineWidth',1.3); hold on;
  plot(rec.t,rad2deg(rec.w(:,i)),'LineWidth',1.3); grid on;
  xlabel('时间 (s)'); ylabel('角速度 (deg/s)'); title(nm{i}); legend('指令','实测','Location','best');
end
subplot(2,3,4); plot(rec.t,rec.u,'LineWidth',1.2); grid on;
  xlabel('时间 (s)'); ylabel('期望力矩 (N·m)'); legend('L_c','M_c','N_c'); title('LADRC 输出力矩');
subplot(2,3,5); plot(rec.t,rec.z2,'LineWidth',1.2); grid on;
  xlabel('时间 (s)'); ylabel('z_2'); legend('p','q','r'); title('ESO 总扰动估计 z_2');
subplot(2,3,6); plot(rec.t,rad2deg(rec.w-rec.cmd),'LineWidth',1.2); grid on;
  xlabel('时间 (s)'); ylabel('跟踪误差 (deg/s)'); legend('p','q','r'); title('角速度跟踪误差');
sgtitle('M2 内环 LADRC：角速度阶跃跟踪 + 抗扰（@1.2s 注入滚转扰动）');
outdir=fullfile(projroot,'analysis','figs'); if ~exist(outdir,'dir'); mkdir(outdir); end
saveas(fig,fullfile(outdir,'M2_inner_rate.png'));
fprintf('图已保存: %s\n', fullfile(outdir,'M2_inner_rate.png'));
