function sim = dart_lqr_simulate(result,sim_cfg)
% 独立轨迹仿真入口：打开本文件直接运行，或执行 sim=dart_lqr_simulate()。
% 默认读取当前参数求 K；传入 result 时复用已求解的 K 和模型，不再次求解。
% 仿真参数只在本文件中设置；自定义时完整传入 sim_cfg 三个字段。
% 小角度刚体、理想力矩输入；不模拟舵量、执行器饱和或气动。

sim = struct();
if nargin < 2
    sim_cfg.duration_s = 2.0;
    sim_cfg.x0 = deg2rad([15; 10; 10; 50; 50; 50]); % 角度误差及角速度，转为 rad/rad/s。
    sim_cfg.plot_enabled = true;
end
if nargin == 0
    result = dart_attitude_lqr();
end

N = round(sim_cfg.duration_s / result.cfg.Ts);
if N < 1
    error('DartLQR:SimulationSamples','仿真至少需要一个控制周期。');
end
sim.time_s = (0:N) * result.cfg.Ts;
sim.command_time_s = (0:N-1) * result.cfg.Ts;
sim.x = zeros(6,N+1);
sim.torque_cmd_Nm = zeros(3,N);
sim.torque_angle_Nm = zeros(3,N);
sim.torque_rate_Nm = zeros(3,N);
sim.x(:,1) = sim_cfg.x0;
for k = 1:N
    sim.torque_angle_Nm(:,k) = -result.design.K_tau(:,1:3) * sim.x(1:3,k);
    sim.torque_rate_Nm(:,k) = -result.design.K_tau(:,4:6) * sim.x(4:6,k);
    sim.torque_cmd_Nm(:,k) = sim.torque_angle_Nm(:,k) + sim.torque_rate_Nm(:,k);
    sim.x(:,k+1) = result.model.Ad * sim.x(:,k) + result.model.Bd_tau * sim.torque_cmd_Nm(:,k);
end
if any(~isfinite(sim.x(:))) || any(~isfinite(sim.torque_cmd_Nm(:)))
    error('DartLQR:NonfiniteSimulation','仿真计算产生 NaN/Inf。');
end
sim.final_angle_deg = rad2deg(sim.x(1:3,end));
sim.final_rate_deg_s = rad2deg(sim.x(4:6,end));
sim.peak_torque_Nm = max(abs(sim.torque_cmd_Nm),[],2);

fprintf('最终角度 [roll,pitch,yaw] = [%.6f, %.6f, %.6f] deg。\n',sim.final_angle_deg);
fprintf('最终角速度 [p,q,r] = [%.6f, %.6f, %.6f] deg/s。\n',sim.final_rate_deg_s);
fprintf('模型验证采用理想力矩输入；惯量轴映射、实际力矩及舵机方向待台架。\n');
if sim_cfg.plot_enabled
    plot_torque_simulation(sim);
end
end

function plot_torque_simulation(sim)
% 仅展示姿态、速率及力矩；不把理想模型结果当作舵机实测。

fig = figure('Name','Dart LQR Torque','NumberTitle','off','Color','w','Position',[100,80,900,700]);
tiledlayout(fig,3,1);
nexttile;
plot(sim.time_s,rad2deg(sim.x(1:3,:)).','LineWidth',1.2);
ylabel('角度误差 (deg)');
legend('roll','pitch（不追角度）','yaw','Location','best');
title('纯 LQR / 理想力矩输入');
grid on;
nexttile;
plot(sim.time_s,rad2deg(sim.x(4:6,:)).','LineWidth',1.2);
ylabel('角速度 (deg/s)');
legend('p / roll','q / pitch','r / yaw','Location','best');
grid on;
nexttile;
plot(sim.command_time_s,sim.torque_cmd_Nm.','LineWidth',1.2);
xlabel('时间 (s)');
ylabel('力矩指令 (N·m)');
legend('roll','pitch','yaw','Location','best');
grid on;
end
