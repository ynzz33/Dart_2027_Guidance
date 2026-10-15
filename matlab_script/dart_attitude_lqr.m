function result = dart_attitude_lqr(cfg)
% 六态 -> 三轴物理力矩的纯 LQR；求 K、检查并自动覆盖 MATLAB 侧增益表。
% 轨迹仿真单独运行 dart_lqr_simulate.m，本文件不分配舵角或写下位机文件。
% 用法：result = dart_attitude_lqr(); 修改配置后可显式传入 cfg。
% x=[e_roll,e_pitch,e_yaw,p,q,r]，误差=测量-目标，单位 rad/rad/s。
% tau=-K_tau*x，单位 N·m；模型为小角度刚体、理想力矩输入。

result = struct();
active = [1, 3, 4, 5, 6];
model = struct();
design = struct();
if nargin == 0
    cfg = dart_lqr_parameters();
end

model.A = [zeros(3), eye(3); zeros(3), zeros(3)];
model.B_tau = [zeros(3); cfg.I_body_kgm2 \ eye(3)];
model.active_states = active;
sysc = ss(model.A, model.B_tau, eye(6), zeros(6,3));
sysd = c2d(sysc, cfg.Ts, 'zoh');
model.Ad = sysd.A;
model.Bd_tau = sysd.B;

% 删除不参与反馈的 pitch 角度再求解，避免把其中性极点当成失稳。
design.controllability_rank = rank(ctrb(model.A(active,active), model.B_tau(active,:)));
if design.controllability_rank ~= numel(active)
    error('DartLQR:Uncontrollable', '五个有效状态不完全可控，请检查惯量及轴序。');
end
sysd_active = c2d(ss(model.A(active,active), model.B_tau(active,:), eye(5), zeros(5,3)), cfg.Ts, 'zoh');
K_active = dlqr(sysd_active.A, sysd_active.B, cfg.Q, cfg.R_tau);
design.K_tau = zeros(3,6);
design.K_tau(:,active) = K_active;
design.controlled_poles = eig(sysd_active.A - sysd_active.B * K_active);
design.max_abs_controlled_pole = max(abs(design.controlled_poles));
design.float32_controlled_poles = eig(sysd_active.A - sysd_active.B * double(single(K_active)));
design.max_abs_float32_pole = max(abs(design.float32_controlled_poles));
design.full_poles = eig(model.Ad - model.Bd_tau * design.K_tau);
design.pitch_neutral_pole_count = sum(abs(design.full_poles - 1) < 1e-9);
if any(~isfinite(design.K_tau(:))) || ~isfinite(design.max_abs_controlled_pole) || ~isfinite(design.max_abs_float32_pole) || design.max_abs_controlled_pole >= 1 || design.max_abs_float32_pole >= 1
    error('DartLQR:UnstableDesign', 'LQR 求解结果非有限或受控状态不稳定。');
end

result.cfg = cfg;
result.model = model;
result.design = design;
result.state_order = {'e_roll','e_pitch','e_yaw','p','q','r'};
result.torque_order = {'roll','pitch','yaw'};

fprintf('纯 LQR：K_tau[3][6]，输出三轴力矩 N·m，无积分、无舵角分配。\n');
fprintf('K_tau：行 [roll,pitch,yaw]；列 [e_roll,e_pitch,e_yaw,p,q,r]。\n');
for axis = 1:3
    fprintf('  ');
    fprintf('%+.8e ',design.K_tau(axis,:));
    fprintf('\n');
end
fprintf('受控状态可控性 %d/5，最大闭环极点模 %.9f。\n', design.controllability_rank, design.max_abs_controlled_pole);
fprintf('单精度系数最大闭环极点模 %.9f。\n', design.max_abs_float32_pole);
fprintf('pitch 不追角度：完整模型中性极点数量 %d（预期 1）。\n', design.pitch_neutral_pole_count);
fprintf('K 矩阵数值检查：通过（当前理想力矩模型）；实际轴映射和控制效果待台架。\n');
result.header_path = dart_lqr_export_c(result);
end
