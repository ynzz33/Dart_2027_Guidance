% ========================================================================
% 飞镖 LQR 姿态控制器 v1
% ========================================================================
% 适用对象：RoboMaster X 型四舵机可动翼面飞镖。
%
% 功能：
%   1. 解析构造第一版纯刚体姿态模型 A/B。
%   2. 使用 c2d + dlqr 计算 1 kHz 离散 LQR 增益 K_d。
%   3. 用纯 MATLAB 脚本做闭环仿真验证。
%   4. 打印可直接移植到 C 代码中的 K_d 矩阵。
%
% 第一版模型假设：
%   - 只控制姿态，不控制平动/落点。
%   - 工作点为零攻角、零标称角速度、长轴对准来流、不自转。
%   - A 矩阵为三组双积分器，不加入气动恢复/阻尼。
%   - B 矩阵由 X 型舵面混控矩阵 G、气动集总增益 k_aero、转动惯量组成。
%
% 状态定义：
%   x = [d_phi, d_theta, d_psi, p, q, r]'
%       d_phi   : roll  角相对期望/配平点偏差，rad
%       d_theta : pitch 角相对期望/配平点偏差，rad
%       d_psi   : yaw   角相对期望/配平点偏差，rad
%       p, q, r : IMU 输出的机体角速度，rad/s，右手定则
%
% 控制量定义：
%   u = [delta1, delta2, delta3, delta4]'
%       delta_i 为第 i 片全动翼面舵机偏转角，rad
%
% 控制律：
%   u = -K_d * x
%
% 上车关键注意：
%   G 矩阵符号、舵机编号、舵机安装方向必须逐一对照实物核对。
%   符号反了就是正反馈，调 Q/R 救不回来。
% ========================================================================

%% Step 0: 重置程序
clear
close all
clc
tic

%% Step 1: 参数区，正常情况下只改这里

% ---------------------- 物理常量 ----------------------
g_ac = 9.81;          % 重力加速度，m/s^2，当前 v1 暂未使用，预留给外环/第二版
rho_ac = 1.225;       % 空气密度，kg/m^3

% ---------------------- 转动惯量 ----------------------
% 下列惯量来自 SolidWorks 截图。
% SolidWorks 单位为 g*mm^2，MATLAB 模型单位为 kg*m^2。
% 换算关系：1 g*mm^2 = 1e-9 kg*m^2。
%
% 坐标系约定：
%   x 轴：飞镖长轴 / roll 轴
%   y 轴：pitch 轴
%   z 轴：yaw 轴
Ixx_ac = 413276.151e-9;   % 绕 roll 轴转动惯量，kg*m^2，SW: Ixx = 413276.151 g*mm^2
Iyy_ac = 5056825.175e-9;  % 绕 pitch 轴转动惯量，kg*m^2，SW: Iyy = 5056825.175 g*mm^2
Izz_ac = 5062679.784e-9;  % 绕 yaw 轴转动惯量，kg*m^2，SW: Izz = 5062679.784 g*mm^2

% SolidWorks 同时给出的积惯量，当前 v1 只记录，不参与计算。
% v1 假设机体系已经足够接近主惯量轴，因此 B 中只使用 Ixx/Iyy/Izz。
Ixy_ac = -12839.066e-9;   % kg*m^2
Ixz_ac = 15346.476e-9;    % kg*m^2
Iyz_ac = -20107.561e-9;   % kg*m^2

% ---------------------- 几何力臂 ----------------------
r_ac = 0.150;             % roll 力臂，m，约等于翼面到长轴的横向距离/臂展
 a_ac = 0.120;            % pitch/yaw 力臂，m，约等于翼面气动中心到质心的纵向距离

% ---------------------- 气动参数 ----------------------
S_ac = 0.0050;            % 单片翼面参考面积，m^2，占位值
CLalpha_ac = 5.0;         % 升力线斜率，1/rad，占位值，薄翼理论约 2*pi
V_ac = 30.0;              % 标称速度，m/s，后续增益调度可把它作为自变量

% 集总气动增益：
%   k_aero = 动压 * 参考面积 * 升力线斜率
% 它把“舵偏 1 rad 产生多少气动力”压成一个标量。
% 当前纯仿真里模型和控制器共用同一个 k_aero，所以仿真会偏理想。
k_aero = 0.5 * rho_ac * V_ac^2 * S_ac * CLalpha_ac;

% ---------------------- 舵机限位 ----------------------
delta_max_ac = deg2rad(60);   % 舵偏限幅，rad，即每个舵机 +/-60 deg

% ---------------------- 采样周期 ----------------------
Ts = 0.001;                   % 1 kHz 控制周期，s

% ---------------------- LQR 权重 ----------------------
% 状态顺序：x = [d_phi, d_theta, d_psi, p, q, r]
% Q 越大，越希望对应状态更快收敛。
lqr_Q = diag([200, 300, 300, 1, 1, 1]);

% 控制量顺序：u = [delta1, delta2, delta3, delta4]
% R 越大，舵偏越保守；如果仿真持续撞限位，优先增大 R。
lqr_R = diag([3.0, 3.0, 3.0, 3.0]);

%% Step 2: 解析构造连续 A、B 矩阵

% 工作点：零攻角、零标称角速度、长轴对准来流、不自转。
% 在 omega = 0 处，刚体陀螺耦合项 -omega x (I*omega) 的雅可比为零。
%
% 小角度下：
%   d_phi_dot   = p
%   d_theta_dot = q
%   d_psi_dot   = r
%   p_dot = M_x / Ixx
%   q_dot = M_y / Iyy
%   r_dot = M_z / Izz
A = [zeros(3), eye(3);
     zeros(3), zeros(3)];

% X 型四翼面混控几何矩阵 G。
% 舵机编号约定：沿飞行方向看 X 形四角
%   delta1 = 右上翼面
%   delta2 = 左上翼面
%   delta3 = 左下翼面
%   delta4 = 右下翼面
%
% 符号含义：
%   roll  : 四片同向，共模
%   pitch : 两两差动
%   yaw   : 两两差动
%
% 警告：下面只是第一版标准假设，上车前必须按实物逐片核对。
G = [ r_ac,  r_ac,  r_ac,  r_ac;     % roll 力矩分配
      a_ac, -a_ac, -a_ac,  a_ac;     % pitch 力矩分配
      a_ac,  a_ac, -a_ac, -a_ac ];   % yaw 力矩分配

% 气动力矩模型：
%   [M_x; M_y; M_z] = k_aero * G * [delta1; delta2; delta3; delta4]
I_inv = diag([1/Ixx_ac, 1/Iyy_ac, 1/Izz_ac]);
B = [zeros(3,4);
     I_inv * G * k_aero];

% 可控性检查：rank 应为 6。
Co_rank = rank(ctrb(A, B));
fprintf('Controllability rank = %d / 6\n', Co_rank);
if Co_rank ~= 6
    warning('System is not fully controllable. Check G, lever arms, and inertia values.');
endz

%% Step 3: 离散化 + 离散 LQR

% 与 HerKules 流水线一致：连续模型先 ZOH 离散化，再做 dlqr。
[A_d, B_d] = c2d(A, B, Ts);
K_d = dlqr(A_d, B_d, lqr_Q, lqr_R);

fprintf('\nDiscrete gain K_d (4 x 6):\n');
disp(K_d);

% 闭环离散极点全部在单位圆内，则线性离散闭环稳定。
cl_poles = eig(A_d - B_d * K_d);
fprintf('Max closed-loop discrete pole magnitude = %.6f (< 1 is stable)\n', max(abs(cl_poles)));

%% Step 4: 闭环仿真验证

T_sim = 2.0;                       % 仿真时长，s
N = round(T_sim / Ts);             % 仿真步数

% 初始姿态扰动：roll 15 deg，pitch 10 deg，yaw 10 deg，初始角速度为 0。
x0 = [deg2rad(15);
      deg2rad(10);
      deg2rad(10);
      0;
      0;
      0];

x_hist = zeros(6, N+1);
u_hist = zeros(4, N);
x_hist(:,1) = x0;

% 离散闭环迭代：
%   u_raw = -K_d*x
%   u     = clamp(u_raw, +/-delta_max_ac)
%   x_next = A_d*x + B_d*u
for k = 1:N
    u_raw = -K_d * x_hist(:,k);
    u = min(max(u_raw, -delta_max_ac), delta_max_ac);

    u_hist(:,k) = u;
    x_hist(:,k+1) = A_d * x_hist(:,k) + B_d * u;
end

t = (0:N) * Ts;
tu = (0:N-1) * Ts;

figure('Name', 'Dart LQR Closed-loop Simulation', 'Color', 'w');

% 上图：姿态角偏差收敛曲线。
subplot(3,1,1);
plot(t, rad2deg(x_hist(1,:)), 'r', 'LineWidth', 1.5);
hold on
plot(t, rad2deg(x_hist(2,:)), 'g', 'LineWidth', 1.5);
plot(t, rad2deg(x_hist(3,:)), 'b', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('Angle error (deg)');
legend('\Delta\phi roll', '\Delta\theta pitch', '\Delta\psi yaw', 'Location', 'best');
title('Attitude angle errors');
grid on

% 中图：机体角速度。
subplot(3,1,2);
plot(t, rad2deg(x_hist(4,:)), 'r', 'LineWidth', 1.5);
hold on
plot(t, rad2deg(x_hist(5,:)), 'g', 'LineWidth', 1.5);
plot(t, rad2deg(x_hist(6,:)), 'b', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('Angular rate (deg/s)');
legend('p', 'q', 'r', 'Location', 'best');
title('Body angular rates');
grid on

% 下图：四个舵机偏转角和限位线。
subplot(3,1,3);
plot(tu, rad2deg(u_hist(1,:)), 'LineWidth', 1.2);
hold on
plot(tu, rad2deg(u_hist(2,:)), 'LineWidth', 1.2);
plot(tu, rad2deg(u_hist(3,:)), 'LineWidth', 1.2);
plot(tu, rad2deg(u_hist(4,:)), 'LineWidth', 1.2);
yline( rad2deg(delta_max_ac), 'k--', '+limit');
yline(-rad2deg(delta_max_ac), 'k--', '-limit');
ylabel('Servo deflection (deg)');
xlabel('Time (s)');
legend('\delta_1', '\delta_2', '\delta_3', '\delta_4', 'Location', 'best');
title('Servo deflections');
grid on

% 自动验收指标。
final_angle_err_deg = max(abs(rad2deg(x_hist(1:3,end))));
peak_delta_deg = max(abs(rad2deg(u_hist(:))));
limit_delta_deg = rad2deg(delta_max_ac);

fprintf('\n--- Simulation acceptance checks ---\n');
fprintf('Final max attitude error = %.4f deg (should approach 0)\n', final_angle_err_deg);
fprintf('Peak servo deflection    = %.2f deg (limit %.2f deg)\n', peak_delta_deg, limit_delta_deg);

if peak_delta_deg >= limit_delta_deg - 1e-6
    fprintf('Warning: servo deflection reaches limit. Increase R or reduce initial disturbance.\n');
else
    fprintf('Servo saturation check: OK\n');
end

%% Step 5: 导出 C 增益（一键复制到 lqr.c 的 dart_lqr_K 粘贴区）

% C 端实时链路：
%   x = [roll_err, pitch_err, yaw_err, p, q, r]
%   u = -K_d * x，u 为四个舵机目标偏转角，单位 rad。
%
% 旧写法逐个 fprintf 长行，命令行窗口会软换行('{' 单独一行、数字又换行)，框选复制易乱。
% 本写法把 K_d 拼成单一字符串后：
%   1) 直接放进系统剪贴板 → 跑完脚本按 Ctrl+V 即可粘到 lqr.c；
%   2) 写入同目录 dart_lqr_K_paste.txt → 剪贴板不便时打开它复制；
%   3) 仍打印到命令行供核对。
% 注意：lqr.c 的 dart_lqr_K[4][6] 是非 const(需在线观测/调)，故这里只导出 4 行数值，
% 整块覆盖 lqr.c "粘贴区" 的那 4 行即可，不要带 static const 声明头。

n_row = size(K_d, 1);
n_col = size(K_d, 2);
K_lines = strings(n_row, 1);
for row = 1:n_row
    nums = sprintf('%.17gf, ', K_d(row, :));   % 每个数加 f 后缀(C float 字面量)
    nums = nums(1:end-2);                       % 去掉行尾多余的 ', '
    if row < n_row
        K_lines(row) = sprintf('    {%s},', nums);
    else
        K_lines(row) = sprintf('    {%s}', nums);  % 末行不加逗号
    end
end
K_block = strjoin(K_lines, newline);

fprintf('\n==== 复制下面 4 行，整块覆盖 lqr.c 的 dart_lqr_K 粘贴区 ====\n');
fprintf('%s\n', K_block);
fprintf('============================================================\n');

% 一键进剪贴板：跑完直接 Ctrl+V
try
    clipboard('copy', char(K_block));
    fprintf('✓ 已复制到剪贴板，直接 Ctrl+V 粘贴即可。\n');
catch
    fprintf('! clipboard 不可用(无桌面环境?)，请改用下面的 txt 文件。\n');
end

% 落地到文本文件兜底
fid = fopen('dart_lqr_K_paste.txt', 'w');
if fid > 0
    fprintf(fid, '%s\n', K_block);
    fclose(fid);
    fprintf('✓ 同时已写入 dart_lqr_K_paste.txt\n');
end

toc

