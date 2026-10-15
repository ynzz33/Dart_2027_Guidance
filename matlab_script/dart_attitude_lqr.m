function varargout = dart_attitude_lqr(cfg)
% 六态 -> 三轴物理力矩的纯 LQR；求 K、检查并直接覆盖代码内增益表。
% 轨迹仿真单独运行 dart_lqr_simulate.m；下位机九态→六态接入仍待完成。
% 用法：result = dart_attitude_lqr(); 修改配置后可显式传入 cfg。
% x=[e_roll,e_pitch,e_yaw,p,q,r]，误差=测量-目标，单位 rad/rad/s。
% tau=-K_tau*x，单位 N·m；模型为小角度刚体、理想力矩输入。

varargout = {};
result = struct();
model = struct();
design = struct();
if nargin == 0
    cfg = dart_lqr_parameters();
end

model.A = [zeros(3), eye(3); zeros(3), zeros(3)];
model.B_tau = [zeros(3); cfg.I_body_kgm2 \ eye(3)];
sysc = ss(model.A, model.B_tau, eye(6), zeros(6,3));
sysd = c2d(sysc, cfg.Ts, 'zoh');
model.Ad = sysd.A;
model.Bd_tau = sysd.B;

% 三轴角度及角速度均参与求解，不能再跳过 pitch 角度或把该列置零。
design.controllability_rank = rank(ctrb(model.A, model.B_tau));
if design.controllability_rank ~= 6
    error('DartLQR:Uncontrollable', '六个状态不完全可控，请检查惯量及轴序。');
end
design.K_tau = dlqr(model.Ad, model.Bd_tau, cfg.Q, cfg.R_tau);
design.controlled_poles = eig(model.Ad - model.Bd_tau * design.K_tau);
design.max_abs_controlled_pole = max(abs(design.controlled_poles));
design.float32_controlled_poles = eig(model.Ad - model.Bd_tau * double(single(design.K_tau)));
design.max_abs_float32_pole = max(abs(design.float32_controlled_poles));
if any(~isfinite(design.K_tau(:))) || ~isfinite(design.max_abs_controlled_pole) || ~isfinite(design.max_abs_float32_pole) || design.max_abs_controlled_pole >= 1 || design.max_abs_float32_pole >= 1
    error('DartLQR:UnstableDesign', 'LQR 求解结果非有限或受控状态不稳定。');
end

result.cfg = cfg;
result.model = model;
result.design = design;
result.state_order = {'e_roll','e_pitch','e_yaw','p','q','r'};
result.torque_order = {'roll','pitch','yaw'};

result.header_path = dart_lqr_export_c(result);
fprintf('K 矩阵数值检查通过，已覆盖。\n');
if nargout > 0
    varargout{1} = result;
end
end
