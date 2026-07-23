% ========================================================================
% 飞镖 LQI 力矩控制器 + Pitch 保护型零空间分配 — 主脚本
% ========================================================================
% 功能：
%   1. 加载参数配置
%   2. 从几何参数构造 H_tau(V)（舵偏→三轴力矩的完整映射）
%   3. 构造连续/离散增广状态空间模型
%   4. 用 dlqr 计算 LQI 增益 K_lqi（输出三轴力矩，非舵角）
%   5. 实现 Pitch 保护型零空间舵面分配器
%   6. 闭环仿真验证（非线性刚体 + 零空间分配）
%   7. 速度调度：计算 K_lqi(V) 和 H_tau(V) 表
%   8. 多项式拟合导出
%
% 与旧 LQR 脚本的本质区别：
%   - LQI 输出 3×1 力矩 tau (N·m)，而不是 4×1 舵角
%   - H_tau 从几何显式推导，替代集总 G 矩阵
%   - 零空间分配器独立于 LQI 增益
%   - 舵面顺序统一为 [UL, UR, DR, DL]
% ========================================================================

clear; close all; clc; tic;

% ---- 屏幕自适应窗口布局 ----
scr = get(0, 'ScreenSize');  % [left, bottom, width, height]
sw = scr(3); sh = scr(4);
% 左侧输出窗口
pos_log   = [20, 40, 520, sh-120];
% 右侧 2×2 网格
rw = (sw - 580) / 2;  rh = (sh - 160) / 2;
pos_sweep = [560, 60+rh, rw, rh];         % 右下：速度扫描
pos_sim   = [560, 60,     rw, rh];         % 左下：单点仿真
pos_gain  = [560+rw, 60+rh, rw, rh];       % 右下二：增益调度
pos_htau  = [560+rw, 60,     rw, rh];       % 左下二：H_tau 调度

% ---- 创建详细输出窗口 ----
hLogFig = figure('Name', 'LQI 详细输出', 'NumberTitle', 'off', ...
    'MenuBar', 'none', 'ToolBar', 'none', ...
    'Position', pos_log, 'Color', 'w');
hLogText = uicontrol('Style', 'edit', 'Parent', hLogFig, ...
    'Units', 'normalized', 'Position', [0.02, 0.02, 0.96, 0.96], ...
    'Max', 2, 'Min', 0, 'HorizontalAlignment', 'left', ...
    'FontName', 'FixedWidth', 'FontSize', 9, ...
    'BackgroundColor', [1 1 1], 'Enable', 'inactive');
setappdata(hLogFig, 'text', hLogText);
setappdata(hLogFig, 'buf', '');

% ---- Step 0-1: 加载参数 + 构造模型 ----
fprintf('Step 0-1: 加载参数...\n');
param_out = evalc('run(''dart_lqi_parameters.m'')');
lqi_log(param_out);

% H_tau(V) = 每片舵偏 1 rad 产生的三轴力矩 (N·m/rad)
% H_tau(:,j) = q·S·C_Fδ · (r_j × n_j)
H_tau_ref_geom = compute_H_tau(1.0, r_surface, n_surface, S_surface, C_F_delta, rho_ac);

% 在设计速度 V_ref 下的 H_tau（几何推导版）
q_ref = 0.5 * rho_ac * V_ref^2;
k_aero_ref = q_ref * S_surface * C_F_delta;   % 集总气动系数，N/rad
H_tau_geom = k_aero_ref * H_tau_ref_geom;       % 3×4，N·m/rad（几何推导，极性待验证）

% ★ 从旧 LQR B 矩阵提取台架验证的 G 矩阵（极性正确）
%   B_LQR = [zeros(3,4); I_inv_old * G]
%   → G = I_body_old * B_LQR(4:6,:)
%   B_LQR 列序: [UR, UL, DL, DR]（MATLAB 舵号 delta1..4）
%   需换到 LQI 列序: [UL, UR, DR, DL]
try
    B_LQR_at_Vref = LQR_B_Dart(V_ref);          % 6×4, 列序 [UR, UL, DL, DR]
    G_from_B = I_body * B_LQR_at_Vref(4:6, :);  % 3×4, N·m/rad（极性已验证）
    % 列重排: [UR,UL,DL,DR] → [UL,UR,DR,DL]
    G_reorder = [2, 1, 4, 3];  % UR→1, UL→0, DR→3, DL→2
    H_tau_Vref = G_from_B(:, G_reorder);        % 3×4, [UL,UR,DR,DL]

    % 极性对比诊断
    sign_G = sign(G_from_B(:, G_reorder));
    sign_H = sign(H_tau_geom);
    sign_mismatch = (sign_G ~= sign_H) & (sign_G ~= 0);
    if any(sign_mismatch(:))
        lqi_log(sprintf('⚠ H_tau 极性不一致（G=旧LQR验证值, H=几何推导值）:\n'));
        rows = {'Roll','Pitch','Yaw'}; cols = {'UL','UR','DR','DL'};
        for r = 1:3, for c = 1:4
            if sign_mismatch(r,c)
                lqi_log(sprintf('  %s/%s: G=%.4f, H_geom=%.4f → 已采用G\n', ...
                    rows{r}, cols{c}, H_tau_Vref(r,c), H_tau_geom(r,c)));
            end
        end; end
    else
        lqi_log(sprintf('✓ H_tau 极性与旧 G 一致\n'));
    end
catch
    % 回退：若 LQR_B_Dart 不可用，用几何推导值
    lqi_log(sprintf('⚠ 无法调用 LQR_B_Dart(V_ref)，回退到几何推导 H_tau（极性未经台架验证！）\n'));
    H_tau_Vref = H_tau_geom;
end

fprintf('H_tau(V_ref=%.1f m/s) (N·m/rad) — 来源：旧 LQR G 矩阵(极性已验证):\n', V_ref);
lqi_log(sprintf('H_tau(V_ref=%.1f m/s) (N·m/rad):\n', V_ref));
lqi_log(mat2str(H_tau_Vref, 6));
lqi_log(sprintf('\n'));

% 矩阵健康检查
lqi_log(sprintf('rank(H_tau)=%d, rank(H_ry)=%d, cond(H_tau)=%.1f\n', ...
    rank(H_tau_Vref), rank(H_tau_Vref([1,3],:)), cond(H_tau_Vref)));
H_ry = H_tau_Vref([1,3], :);   % Roll + Yaw 行 → 2×4
h_pitch = H_tau_Vref(2, :);    % Pitch 行 → 1×4
% ====== 矩阵秩诊断（分级） ======
H_ry = H_tau_Vref([1,3], :);   % Roll + Yaw 行 → 2×4
h_pitch = H_tau_Vref(2, :);    % Pitch 行 → 1×4

if rank(H_ry) < 2
    error('rank(H_ry)=%d<2！Roll/Yaw 无法独立控制，检查舵面几何。', rank(H_ry));
end

if rank(H_tau_Vref) < 3
    lqi_log(sprintf('注意: rank(H_tau)=%d<3（对称X翼几何，Pitch=1.25*(Roll-Yaw)耦合）\n', rank(H_tau_Vref)));
    lqi_log(sprintf('      零空间Pitch优化无效(h_pitch*N_ry≈0)，这符合"Pitch不主动控制"设计。\n'));
    lqi_log(sprintf('      rank(H_ry)=%d ✓ Roll+Yaw独立可控，分配器正常工作。\n', rank(H_ry)));
else
    lqi_log(sprintf('rank(H_tau)=%d ✓ (满秩，三轴独立可控)\n', rank(H_tau_Vref)));
end
lqi_log(sprintf('rank(H_ry)=%d, cond(H_tau)=%.1f\n', rank(H_ry), cond(H_tau_Vref)));
if rank(H_ry) < 2, warning('rank(H_ry) < 2！'); end

N_ry = null(H_ry);   % Roll/Yaw 零空间，4×2
fprintf('N_ry 零空间维度 = %d (期望 2)\n', size(N_ry,2));

% 构造连续状态空间模型
% x = [e_roll, e_pitch, e_yaw, p, q, r]'
% ẋ = A·x + B_tau·tau  (tau = 三轴力矩)
%
% A 矩阵：运动学 + 刚体动力学
%   eta_dot = omega     (角度误差 = 角速度，在小角度假设下)
%   omega_dot = A_omega_eta·eta + A_omega_rate·omega + inv(I)·tau
%
% 第一版（无可靠气动数据）：
%   A_omega_eta  = zeros(3)
%   A_omega_rate = zeros(3)

A = [zeros(3), eye(3);
     zeros(3), zeros(3)];

% 可选气动项（默认关闭）
if DART_LQI_ENABLE_AERO_A
    nu = V_ref / V_ref;   % = 1.0 at design speed
    A(4,4) = Dp_ref * nu;
    A(5,2) = Cm_pitch_ref * nu^2;
    A(5,5) = Dq_ref * nu;
    A(6,3) = Cm_yaw_ref * nu^2;
    A(6,6) = Dr_ref * nu;
end

% B_tau: 力矩输入矩阵
% tau 直接作用在角加速度上：ω̇ = inv(I)·tau
B_tau = [zeros(3);
         I_inv];    % 6×3

% 可控性检查（原始 6 状态系统）
Co6 = ctrb(A, B_tau);
fprintf('\n--- 可控性检查 ---\n');
fprintf('rank(ctrb(A, B_tau))     = %d / 6 (原始系统)\n', rank(Co6));
if rank(Co6) < 6
    warning('原始系统不完全可控！检查惯量矩阵和 A 矩阵。');
end

% ---- Step 2: LQI 增广 ----
fprintf('\n========== Step 2: LQI 增广 ==========\n');

% C_I: 选择哪些状态被积分（仅角度误差，不含角速度）
C_I = [eye(3), zeros(3,3)];   % 3×6

% 增广系统
A_aug = [A,          zeros(6,3);
         C_I,        zeros(3,3)];   % 9×9

B_aug = [B_tau;
         zeros(3,3)];                % 9×3

% 增广系统可控性检查（9 状态：LQI 需要积分通道也可控）
Co9 = ctrb(A_aug, B_aug);
fprintf('rank(ctrb(A_aug, B_aug)) = %d / 9 (理想力矩输入)\n', rank(Co9));
if rank(Co9) < 9
    warning('增广系统不完全可控（理想力矩）！积分通道可能无效，检查 C_I 和 B_tau。');
end

% ★ 真实舵面可控性检查（用 B_delta = B_tau * H_tau_Vref 替换理想力矩输入）
B_delta = B_tau * H_tau_Vref;   % 6×4：舵偏→状态导数
B_aug_delta = [B_delta;
               zeros(3,4)];      % 9×4
Co9_delta = ctrb(A_aug, B_aug_delta);
fprintf('rank(ctrb(A_aug, B_aug_delta)) = %d / 9 (真实舵面输入，H_tau 3×4)\n', rank(Co9_delta));
if rank(Co9_delta) < 9
    error('真实舵面增广系统不完全可控！rank=%d/9。H_tau 无法产生完整的 9 状态控制。', rank(Co9_delta));
end
lqi_log(sprintf('真实舵面可控性: rank=%d/9 ✓\n', rank(Co9_delta)));

% 零空间有效性检查
lqi_log(sprintf('零空间检查:\n'));
lqi_log(sprintf('  norm(H_ry*N_ry)=%.2e (应≈0)\n', norm(H_ry*N_ry)));
lqi_log(sprintf('  norm(h_pitch*N_ry)=%.2e (%s)\n', norm(h_pitch*N_ry), ...
    iif(norm(h_pitch*N_ry) < 1e-6, 'Pitch零空间不可调→lambda_pitch无效（对称几何正常）', 'Pitch零空间可调→lambda_pitch有效')));

function val = iif(cond, t, f)
    if cond, val = t; else, val = f; end
end

% 离散化增广系统（ss 对象 → c2d，兼容所有 MATLAB 版本）
sysc_aug = ss(A_aug, B_aug, eye(9), zeros(9,3));
sysd_aug = c2d(sysc_aug, Ts);
A_d = sysd_aug.A;
B_d = sysd_aug.B;

% 离散 LQR
[K_lqi, S, cl_poles] = dlqr(A_d, B_d, Q_aug, R_tau);
% dlqr 返回 K 使得 u = -K*x 最优，所以 tau_cmd = -K_lqi * xa
% 不需要手动取反——dlqr 已经处理了符号

s = sprintf('LQI 增益 K_lqi (3×9) at V_ref = %.1f m/s:\n', V_ref);
s = [s sprintf('  列: [e_roll, e_pitch, e_yaw, p, q, r, ∫e_r, ∫e_p, ∫e_y]\n')];
for i = 1:3
    s = [s sprintf('  K(%d,:) = [', i)];
    s = [s sprintf('%9.4f', K_lqi(i,:))];
    s = [s sprintf(']\n')];
end
lqi_log(s);

% 闭环极点检查（全 9 状态，不分类）
cl_poles_d = eig(A_d - B_d * K_lqi);
max_abs_pole = max(abs(cl_poles_d));
fprintf('\n闭环极点: max|λ| = %.6f (全9状态, <1 稳定)\n', max_abs_pole);

% 区分：故意关积分（Q_int≈0→极点≈1）vs 真不稳定（极点≥1）
int_active = (diag(Q_aug(7:9,7:9)) > 1e-5);  % Q_int > 1e-5 视为积分启用
if max_abs_pole >= 1
    error('九状态闭环不稳定！max|λ| = %.6f >= 1。调节 Q/R 或检查模型。', max_abs_pole);
elseif max_abs_pole > 0.999
    if ~all(int_active)
        fprintf('  注意: 存在 ≈1 极点（Q_int 过小/为零导致积分器无反馈），非 instability。\n');
        fprintf('  积分启用状态: roll=%d, pitch=%d, yaw=%d\n', int_active(1), int_active(2), int_active(3));
    else
        warning('闭环极点距单位圆过近: max|λ| = %.6f。积分发散风险。', max_abs_pole);
    end
end

% ---- Step 3: 零空间分配器 ----
fprintf('\n========== Step 3: 零空间分配器 ==========\n');

% 用标称力矩测试分配器
tau_test = [0.1; 0; 0.1];   % roll + yaw，pitch=0
[delta_test, tau_achieved_test, alloc_ok] = allocate_torque_pitch_protected(...
    tau_test, H_tau_Vref, h_pitch, N_ry, delta_max_rad, lambda_pitch, lambda_servo);

lqi_log(sprintf('--- 分配器测试 ---\n'));
lqi_log(sprintf('tau_cmd=[%.2f,%.2f,%.2f] N·m, delta=[%.2f,%.2f,%.2f,%.2f]°, ', ...
    tau_test, rad2deg(delta_test)));
lqi_log(sprintf('tau_ach=[%.4f,%.4f,%.4f], err=[%.1e,%.1e,%.1e], M_pitch=%.4f, ok=%d\n', ...
    tau_achieved_test, tau_test-tau_achieved_test, h_pitch*delta_test, alloc_ok));

% ---- Step 4: 闭环仿真（离散模型 + 条件积分抗饱和） ----
fprintf('\n========== Step 4: 闭环仿真 ==========\n');

N_sim = round(T_sim / Ts);

% 初始状态
x0 = [deg2rad(x0_attitude_deg);   % 角度误差 rad
      deg2rad(x0_rates_deg_s)];   % 角速度 rad/s

% 预计算 6 状态离散模型（V_sim 下的气动项，与 dlqr 设计一致）
A_sim = [zeros(3), eye(3); zeros(3), zeros(3)];
if DART_LQI_ENABLE_AERO_A
    nu_sim = V_sim / V_ref;
    A_sim(4,4) = Dp_ref * nu_sim;
    A_sim(5,2) = Cm_pitch_ref * nu_sim^2;
    A_sim(5,5) = Dq_ref * nu_sim;
    A_sim(6,3) = Cm_yaw_ref * nu_sim^2;
    A_sim(6,6) = Dr_ref * nu_sim;
end
sysc6 = ss(A_sim, B_tau, eye(6), zeros(6,3));
sysd6 = c2d(sysc6, Ts);
A_d6 = sysd6.A;
B_d6 = sysd6.B;

% 仿真速度下的 H_tau（匹配嵌入式 H_tau(V) = (V/V_ref)² * H_tau_Vref）
H_tau_sim = (V_sim / V_ref)^2 * H_tau_Vref;
h_pitch_sim = H_tau_sim(2, :);
N_ry_sim = null(H_tau_sim([1,3], :));  % 对应速度下的零空间

xa_hist = zeros(9, N_sim+1);      % 增广状态历史
x_hist  = zeros(6, N_sim+1);      % 原始状态历史
tau_cmd_hist = zeros(3, N_sim);   % 指令力矩历史
tau_ach_hist = zeros(3, N_sim);   % 实际力矩历史
delta_hist = zeros(4, N_sim);     % 舵角历史
pitch_moment_hist = zeros(1, N_sim);
sat_hist = zeros(1, N_sim);       % 饱和标志
freeze_hist = zeros(3, N_sim);    % 各轴积分冻结标志

xa = [x0; zeros(3,1)];            % 积分初值 = 0
x_hist(:,1) = x0;
xa_hist(:,1) = xa;
 
for k = 1:N_sim
    % ---- LQI 控制律: tau_cmd = -K_lqi * xa ----
    tau_cmd = -K_lqi * xa;
    tau_cmd_hist(:,k) = tau_cmd;

    % ---- 零空间分配: tau_cmd → delta ----
    [delta, tau_achieved, ~] = allocate_torque_pitch_protected(...
        tau_cmd, H_tau_sim, h_pitch_sim, N_ry_sim, delta_max_rad, lambda_pitch, lambda_servo);

    delta_hist(:,k) = delta;
    tau_ach_hist(:,k) = tau_achieved;
    pitch_moment_hist(k) = h_pitch_sim * delta;

    % ---- 饱和检测 ----
    sat_detected = any(abs(delta) >= delta_max_rad - 1e-6);
    sat_hist(k) = sat_detected;

    % ---- 6 状态离散推进（使用实际力矩，与 dlqr 设计一致的 ZOH 模型） ----
    x = xa(1:6);
    x_next = A_d6 * x + B_d6 * tau_achieved;

    % ---- 积分更新：积分分离 + 抗饱和 ----
    integ = xa(7:9);
    freeze = zeros(3,1);   % 1 = 冻结该轴积分

    % ① 积分分离：大误差 → 冻结 + 清零（P/D 主导）；小误差 → 开启积分（消静差）
    for ax = 1:3
        if abs(x(ax)) >= integ_threshold_rad
            freeze(ax) = 1;
            integ(ax) = 0;   % 大误差清零积分
        end
    end

    % ② 抗饱和：舵面饱和 → 全部冻结
    if sat_detected
        freeze(:) = 1;
    end

    freeze_hist(:,k) = freeze;

    % 应用积分更新（冻结的轴保持原值）
    integ_next = integ;
    for ax = 1:3
        if ~freeze(ax)
            integ_next(ax) = integ(ax) + Ts * x(ax);   % ∫e·dt
        end
    end

    % ---- 组装下一拍增广状态 ----
    xa_next = [x_next; integ_next];

    x_hist(:,k+1) = x_next;
    xa_hist(:,k+1) = xa_next;
    xa = xa_next;
end

t = (0:N_sim) * Ts;
tu = (0:N_sim-1) * Ts;

% 验收检查
final_angle_err_deg = max(abs(rad2deg(x_hist(1:3,end))));
peak_delta_deg = max(abs(rad2deg(delta_hist(:))));
rms_pitch_moment = sqrt(mean(pitch_moment_hist.^2));
sat_ratio = sum(sat_hist) / N_sim * 100;

fprintf('\n--- 仿真验收 (V_sim = %.1f m/s) ---\n', V_sim);
fprintf('最终角度误差 = [%.4f, %.4f, %.4f]° (应趋近 0)\n', rad2deg(x_hist(1:3,end)));
fprintf('最大舵偏 = %.2f° (限幅 %.0f°)\n', peak_delta_deg, delta_max_deg);
fprintf('Pitch 力矩 RMS = %.4f N·m\n', rms_pitch_moment);
fprintf('舵面饱和比例 = %.1f%%\n', sat_ratio);

if peak_delta_deg >= delta_max_deg - 1e-6
    fprintf('⚠ 舵面达到限幅！考虑增大 R_tau 或减小初始扰动。\n');
else
    fprintf('舵面限幅检查: OK\n');
end

% ---- Step 4.5: 对比仿真（无零空间 vs 有零空间） ----
fprintf('\n--- 零空间对比 ---\n');

% 无零空间：直接用 H_tau 伪逆（最小 Norm 解，不特殊处理 pitch）
delta_pinv = pinv(H_tau_sim) * tau_cmd_hist(:,1);  % 只用第一拍做比较
pitch_pinv = h_pitch_sim * delta_pinv;
pitch_prot = h_pitch_sim * delta_hist(:,1);

fprintf('第一拍 Pitch 力矩: 伪逆=%.4f N·m vs 零空间保护=%.4f N·m\n', ...
    abs(pitch_pinv), abs(pitch_prot));
if abs(pitch_prot) < abs(pitch_pinv)
    fprintf('✓ 零空间降低了 Pitch 力矩\n');
else
    fprintf('→ 零空间未显著降低 Pitch 力矩（可能第一拍 pitch 力矩不大）\n');
end

% ---- Step 4.5: 全速度区间扫描（1.0:0.1:20.0 m/s） ----
fprintf('\n========== Step 4.5: 全速度扫描 ==========\n');

V_scan = V_schedule;                    % 1.0:0.1:20.0
N_scan = numel(V_scan);
metrics_final_err = zeros(N_scan, 3);   % [roll, pitch, yaw] 最终误差 °
metrics_peak_delta = zeros(N_scan, 1);  % 最大舵偏 °
metrics_sat_ratio  = zeros(N_scan, 1);  % 饱和比例
metrics_rms_pitch  = zeros(N_scan, 1);  % Pitch 力矩 RMS

for iv = 1:N_scan
    Vi = V_scan(iv);
    H_tau_i = (Vi / V_ref)^2 * H_tau_Vref;
    H_ry_i = H_tau_i([1,3], :);
    h_pitch_i = H_tau_i(2, :);
    N_ry_i = null(H_ry_i);

    % 6 状态离散模型（含 V 相关气动项）
    A_i = [zeros(3), eye(3); zeros(3), zeros(3)];
    if DART_LQI_ENABLE_AERO_A
        nu_i = Vi / V_ref;
        A_i(4,4) = Dp_ref * nu_i;
        A_i(5,2) = Cm_pitch_ref * nu_i^2;
        A_i(5,5) = Dq_ref * nu_i;
        A_i(6,3) = Cm_yaw_ref * nu_i^2;
        A_i(6,6) = Dr_ref * nu_i;
    end
    sysc_i = ss(A_i, B_tau, eye(6), zeros(6,3));
    sysd_i = c2d(sysc_i, Ts);
    A_d_i = sysd_i.A;
    B_d_i = sysd_i.B;

    xa = [x0; zeros(3,1)];
    sat_cnt = 0;
    pitch_sq = 0;
    peak_d = 0;

    for k = 1:N_sim
        tau_cmd = -K_lqi * xa;
        [delta, ~, ~] = allocate_torque_pitch_protected(...
            tau_cmd, H_tau_i, h_pitch_i, N_ry_i, delta_max_rad, lambda_pitch, lambda_servo);

        if any(abs(delta) >= delta_max_rad - 1e-6), sat_cnt = sat_cnt + 1; end
        peak_d = max(peak_d, max(abs(delta)));
        pitch_sq = pitch_sq + (h_pitch_i * delta)^2;

        % 离散推进（含该速度下的气动动力学）
        x = xa(1:6);
        x_next = A_d_i * x + B_d_i * (H_tau_i * delta);
        integ = xa(7:9);
        integ_next = integ + Ts * x(1:3);  % 扫速时简化积分（无条件防饱和）
        xa = [x_next; integ_next];
    end

    metrics_final_err(iv, :) = rad2deg(abs(xa(1:3)'));
    metrics_peak_delta(iv)   = rad2deg(peak_d);
    metrics_sat_ratio(iv)    = sat_cnt / N_sim * 100;
    metrics_rms_pitch(iv)    = sqrt(pitch_sq / N_sim);
end

% 绘制速度扫描结果
figure('Name', 'Dart LQI Speed Sweep', 'Color', 'w', 'Position', pos_sweep);

subplot(2,2,1);
plot(V_scan, metrics_final_err(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(V_scan, metrics_final_err(:,2), 'g', 'LineWidth', 1.5);
plot(V_scan, metrics_final_err(:,3), 'b', 'LineWidth', 1.5);
ylabel('最终误差 (°)'); xlabel('速度 (m/s)');
legend('roll', 'pitch', 'yaw'); title('稳态误差 vs 速度');
grid on;

subplot(2,2,2);
plot(V_scan, metrics_peak_delta, 'k', 'LineWidth', 1.5);
yline(delta_max_deg, 'r--', '限幅');
ylabel('最大舵偏 (°)'); xlabel('速度 (m/s)');
title('峰值舵偏 vs 速度');
grid on;

subplot(2,2,3);
plot(V_scan, metrics_sat_ratio, 'LineWidth', 1.5);
ylabel('饱和比例 (%)'); xlabel('速度 (m/s)');
title('舵面饱和率 vs 速度');
grid on;

subplot(2,2,4);
plot(V_scan, metrics_rms_pitch, 'LineWidth', 1.5);
ylabel('Pitch 力矩 RMS (N·m)'); xlabel('速度 (m/s)');
title('Pitch 力矩 vs 速度');
grid on;

fprintf('速度扫描完成: V=%.1f~%.1f, 最大舵偏=%.1f°, 最大饱和率=%.1f%%\n', ...
    V_scan(1), V_scan(end), max(metrics_peak_delta), max(metrics_sat_ratio));

% ---- Step 5: 绘图（单点详细） ----
fprintf('\n========== Step 5: 绘图 (V_sim=%.1f) ==========\n', V_sim);

figure('Name', 'Dart LQI Torque Control', 'Color', 'w', 'Position', pos_sim);

% 图 1: 姿态角误差
subplot(4,1,1);
plot(t, rad2deg(x_hist(1,:)), 'r', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x_hist(2,:)), 'g', 'LineWidth', 1.5);
plot(t, rad2deg(x_hist(3,:)), 'b', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('角度误差 (°)');
legend('e_{roll}', 'e_{pitch}', 'e_{yaw}', 'Location', 'best');
title('姿态角误差收敛');
grid on;

% 图 2: 机体角速度
subplot(4,1,2);
plot(t, rad2deg(x_hist(4,:)), 'r', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x_hist(5,:)), 'g', 'LineWidth', 1.5);
plot(t, rad2deg(x_hist(6,:)), 'b', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('角速度 (°/s)');
legend('p (roll)', 'q (pitch)', 'r (yaw)', 'Location', 'best');
title('机体角速度');
grid on;

% 图 3: 三轴力矩指令 vs 实际
subplot(4,1,3);
plot(tu, tau_cmd_hist(1,:), 'r--', 'LineWidth', 1.0); hold on;
plot(tu, tau_ach_hist(1,:), 'r', 'LineWidth', 1.5);
plot(tu, tau_cmd_hist(2,:), 'g--', 'LineWidth', 1.0);
plot(tu, tau_ach_hist(2,:), 'g', 'LineWidth', 1.5);
plot(tu, tau_cmd_hist(3,:), 'b--', 'LineWidth', 1.0);
plot(tu, tau_ach_hist(3,:), 'b', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('力矩 (N·m)');
legend('Mx cmd', 'Mx ach', 'My cmd', 'My ach', 'Mz cmd', 'Mz ach', 'Location', 'best');
title('三轴力矩：指令(虚线) vs 实际(实线)');
grid on;

% 图 4: 舵面偏转角
subplot(4,1,4);
plot(tu, rad2deg(delta_hist(1,:)), 'LineWidth', 1.2); hold on;
plot(tu, rad2deg(delta_hist(2,:)), 'LineWidth', 1.2);
plot(tu, rad2deg(delta_hist(3,:)), 'LineWidth', 1.2);
plot(tu, rad2deg(delta_hist(4,:)), 'LineWidth', 1.2);
yline( delta_max_deg, 'k--', '+limit');
yline(-delta_max_deg, 'k--', '-limit');
ylabel('舵偏 (°)');
xlabel('时间 (s)');
legend('UL', 'UR', 'DR', 'DL', 'Location', 'best');
title('舵面偏转角');
grid on;

% ---- Step 6: 速度调度 ----
fprintf('\n========== Step 6: 速度调度 K_lqi(V) ==========\n');

V_schedule_N = numel(V_schedule);
K_lqi_schedule = zeros(3, 9, V_schedule_N);
H_tau_schedule = zeros(3, 4, V_schedule_N);
cl_pole_schedule = zeros(1, V_schedule_N);

s_sched = sprintf('  idx    V(m/s)    max|pole|    K(1,1)\n');
for idx = 1:V_schedule_N
    Vi = V_schedule(idx);

    % 构造该速度下的增广模型
    H_tau_i = (Vi / V_ref)^2 * H_tau_Vref;   % 统一缩放，与 C 端一致
    H_tau_schedule(:,:,idx) = H_tau_i;

    B_tau_i = [zeros(3);
               I_inv];  % B_tau 不随速度变（力矩直接作用）

    A_i = [zeros(3), eye(3);
           zeros(3), zeros(3)];

    if DART_LQI_ENABLE_AERO_A
        nu_i = Vi / V_ref;
        A_i(4,4) = Dp_ref * nu_i;
        A_i(5,2) = Cm_pitch_ref * nu_i^2;
        A_i(5,5) = Dq_ref * nu_i;
        A_i(6,3) = Cm_yaw_ref * nu_i^2;
        A_i(6,6) = Dr_ref * nu_i;
    end

    A_aug_i = [A_i, zeros(6,3);
               C_I, zeros(3,3)];
    B_aug_i = [B_tau_i;
               zeros(3,3)];

    sysc_i = ss(A_aug_i, B_aug_i, eye(9), zeros(9,3));
    sysd_i = c2d(sysc_i, Ts);
    A_d_i = sysd_i.A;
    B_d_i = sysd_i.B;
    [K_i, ~, ~] = dlqr(A_d_i, B_d_i, Q_aug, R_tau);
    % dlqr 返回 K 使 u = -K*x 最优，直接保存不需取反
    K_lqi_schedule(:,:,idx) = K_i;

    cl_poles_i = eig(A_d_i - B_d_i * K_i);
    cl_pole_schedule(idx) = max(abs(cl_poles_i));  % 全 9 状态

    if mod(idx, 20) == 0 || idx == 1 || idx == V_schedule_N
        s_sched = [s_sched sprintf('  %3d    %8.3f      %.6f      %.4f\n', ...
            idx, Vi, cl_pole_schedule(idx), K_lqi_schedule(1,1,idx))];
    end
end
s_sched = [s_sched sprintf('所有速度点 max|pole| = %.6f (< 1 全9状态稳定)\n', max(cl_pole_schedule))];
lqi_log(s_sched);

% 绘制 K_lqi(V) 的 roll 力矩行
figure('Name', 'Dart LQI Gain Schedule', 'Color', 'w', 'Position', pos_gain);
subplot(3,1,1);
plot(V_schedule, squeeze(K_lqi_schedule(1,1,:)), 'LineWidth', 1.5); hold on;
plot(V_schedule, squeeze(K_lqi_schedule(1,4,:)), 'LineWidth', 1.5);
plot(V_schedule, squeeze(K_lqi_schedule(1,7,:)), 'LineWidth', 1.5);
ylabel('K_{Mx} (N·m/rad)');
legend('K(1,1) e_{roll}', 'K(1,4) p', 'K(1,7) ∫e_{roll}', 'Location', 'best');
title('Roll 力矩增益 K_{Mx}(V) 调度');
grid on;

subplot(3,1,2);
plot(V_schedule, squeeze(K_lqi_schedule(2,2,:)), 'LineWidth', 1.5); hold on;
plot(V_schedule, squeeze(K_lqi_schedule(2,5,:)), 'LineWidth', 1.5);
ylabel('K_{My} (N·m/rad)');
legend('K(2,2) e_{pitch}', 'K(2,5) q', 'Location', 'best');
title('Pitch 力矩增益 K_{My}(V) 调度（权重低）');
grid on;

subplot(3,1,3);
plot(V_schedule, squeeze(K_lqi_schedule(3,3,:)), 'LineWidth', 1.5); hold on;
plot(V_schedule, squeeze(K_lqi_schedule(3,6,:)), 'LineWidth', 1.5);
plot(V_schedule, squeeze(K_lqi_schedule(3,9,:)), 'LineWidth', 1.5);
xlabel('速度 V (m/s)');
ylabel('K_{Mz} (N·m/rad)');
legend('K(3,3) e_{yaw}', 'K(3,6) r', 'K(3,9) ∫e_{yaw}', 'Location', 'best');
title('Yaw 力矩增益 K_{Mz}(V) 调度');
grid on;

% ---- Step 7: 多项式拟合 K_lqi(V) ----
fprintf('\n========== Step 7: 多项式拟合 ==========\n');

V_fit_offset = V_schedule - V_ref;
[K_poly_coeff, K_poly_max_err] = fit_matrix_schedule(K_lqi_schedule, V_fit_offset, K_poly_order);
[H_poly_coeff, H_poly_max_err] = fit_matrix_schedule(H_tau_schedule, V_fit_offset, K_poly_order);

fprintf('多项式拟合阶数 = %d\n', K_poly_order);
fprintf('K_lqi(V) 拟合最大绝对误差 = %.6e\n', K_poly_max_err);
fprintf('H_tau(V) 拟合最大绝对误差 = %.6e\n', H_poly_max_err);

% ---- Step 8: 绘制 H_tau(V) 调度 ----
figure('Name', 'Dart LQI H_tau Schedule', 'Color', 'w', 'Position', pos_htau);
for ax = 1:3
    subplot(3,1,ax);
    for sv = 1:4
        plot(V_schedule, squeeze(H_tau_schedule(ax,sv,:)), 'LineWidth', 1.2); hold on;
    end
    ylabel(sprintf('H_{%d} (N·m/rad)', ax));
    if ax == 1
        title('H_{tau}(V): 四片舵面→三轴力矩映射');
    end
    if ax == 3
        xlabel('速度 V (m/s)');
    end
    legend('UL', 'UR', 'DR', 'DL', 'Location', 'best');
    grid on;
end

% ---- Step 8: 导出 C 静态表（单矩阵，K 与速度无关，H_tau 解析算） ----
fprintf('\n========== Step 8: 导出 C 静态表 ==========\n');

project_root = fullfile(pwd, '..');
lqi_tool_dir = fullfile(project_root, 'imcalib', 'lqi_tool');
if ~exist(lqi_tool_dir, 'dir'), mkdir(lqi_tool_dir); end

% 零空间符号统一
N_ry_Vref = N_ry;
for col = 1:size(N_ry_Vref, 2)
    [~, fnz] = max(abs(N_ry_Vref(:,col)));
    if N_ry_Vref(fnz, col) < 0, N_ry_Vref(:,col) = -N_ry_Vref(:,col); end
end

% -- lqi_gain_table.h （单矩阵，无速度表） --
fid = fopen(fullfile(lqi_tool_dir, 'lqi_gain_table.h'), 'w');
fprintf(fid, '/*\n');
fprintf(fid, ' * lqi_gain_table.h — MATLAB %s\n', datestr(now));
fprintf(fid, ' * K_lqi 与速度无关(B_tau=I⁻¹不含气动)，存单矩阵。\n');
fprintf(fid, ' * Q=diag([%.2f,%.3f,%.1f,%.1f,%.1f,%.1f,%.2f,%.4f,%.2f])\n', diag(Q_aug));
fprintf(fid, ' * R=diag([%.1f,%.1f,%.1f]) V_ref=%.1f Ts=%.3fs\n', diag(R_tau), V_ref, Ts);
fprintf(fid, ' */\n\n');
fprintf(fid, '#ifndef LQI_GAIN_TABLE_H\n#define LQI_GAIN_TABLE_H\n#include <stdint.h>\n');
fprintf(fid, '#include "lqi_torque.h"  /* LQI_STATE_DIM, LQI_TORQUE_DIM */\n\n');
fprintf(fid, '#define LQI_V_REF %.4ff\n#define LQI_V_MIN %.4ff\n#define LQI_V_MAX %.4ff\n\n', ...
    V_ref, V_schedule(1), V_schedule(end));
fprintf(fid, 'static const float lqi_K[LQI_TORQUE_DIM][LQI_STATE_DIM] = {\n');
for iax = 1:3
    fprintf(fid, '    {');
    fprintf(fid, '%+.8ef, ', K_lqi(iax, 1:8));
    fprintf(fid, '%+.8ef}', K_lqi(iax, 9));
    if iax < 3, fprintf(fid, ',\n'); else, fprintf(fid, '\n'); end
end
fprintf(fid, '};\n\n#endif\n');
fclose(fid);
fprintf('✓ lqi_gain_table.h (K_lqi[3][9] 单矩阵)\n');

% -- lqi_geometry_table.h （单 H_tau_Vref + N_ry） --
fid = fopen(fullfile(lqi_tool_dir, 'lqi_geometry_table.h'), 'w');
fprintf(fid, '/*\n');
fprintf(fid, ' * lqi_geometry_table.h — MATLAB %s\n', datestr(now));
fprintf(fid, ' * H_tau(V) = (V/V_ref)^2 * lqi_H_tau_Vref\n');
fprintf(fid, ' * N_ry: H_ry*N_ry=0 (4x2) ⚠ 占位符\n');
fprintf(fid, ' */\n\n');
fprintf(fid, '#ifndef LQI_GEOMETRY_TABLE_H\n#define LQI_GEOMETRY_TABLE_H\n#include <stdint.h>\n');
fprintf(fid, '#include "lqi_gain_table.h"  /* LQI_V_REF */\n\n');
fprintf(fid, '#define LQI_NRY_DIM 2\n#define LQI_PITCH_ROW 1\n');
fprintf(fid, '#define LQI_DELTA_MAX_DEG %.4ff\n#define LQI_DELTA_MAX_RAD %.8ff\n', ...
    delta_max_deg, delta_max_rad);
fprintf(fid, '#define LQI_LAMBDA_PITCH %.4ff\n#define LQI_LAMBDA_SERVO %.4ff\n', ...
    lambda_pitch, lambda_servo);
fprintf(fid, '#define LQI_GAIN_SCALAR 1.0f\n\n');
fprintf(fid, 'static const float lqi_H_tau_Vref[3][4] = {\n');
for iax = 1:3
    fprintf(fid, '    {');
    fprintf(fid, '%+.8ef, ', H_tau_Vref(iax, 1:3));
    fprintf(fid, '%+.8ef}', H_tau_Vref(iax, 4));
    if iax < 3, fprintf(fid, ',\n'); else, fprintf(fid, '\n'); end
end
fprintf(fid, '};\n\n');
fprintf(fid, 'static const float lqi_N_ry[LQI_SERVO_COUNT][LQI_NRY_DIM] = {\n');
for i = 1:4
    fprintf(fid, '    {%+.8ef, %+.8ef}', N_ry_Vref(i,1), N_ry_Vref(i,2));
    if i < 4, fprintf(fid, ',\n'); else, fprintf(fid, '\n'); end
end
fprintf(fid, '};\n\n#endif\n');
fclose(fid);
fprintf('✓ lqi_geometry_table.h (H_tau_Vref[3][4] + N_ry[4][2])\n');

% ---- 完成 ----
elapsed = toc;
fprintf('\n========== 完成 (%.1f 秒) ==========\n', elapsed);
fprintf('工作区变量: K_lqi(3×9) H_tau_Vref(3×4) K_lqi_schedule(3×9×%d) H_tau_schedule(3×4×%d) K_poly_coeff\n', ...
    V_schedule_N, V_schedule_N);
fprintf('已导出: imcalib/lqi_tool/lqi_gain_table.h, lqi_geometry_table.h\n');
fprintf('⚠ 基于占位符参数，SolidWorks/CFD 数据后重跑本脚本。\n');


% ========================================================================
% ===== 辅助函数 =====
% ========================================================================

function H_ref = compute_H_tau(~, r_surface, n_surface, ~, ~, ~)
% 计算 H_tau 的纯几何部分 r×n（不含动压/面积/舵效）
%
% 输入:
%   r_surface   - 3×4 舵面气动中心位置矩阵 (m, 机体系)
%   n_surface   - 3×4 力方向单位向量（无量纲）
%   ~           - V, S, C_F_delta, rho 保留接口兼容，暂未使用
%
% 输出:
%   H_ref       - 3×4 纯几何矩阵 (m)
%                 使用时: H_tau(V) = 0.5·ρ·V²·S·C_Fδ · H_ref
%
% 推导:
%   舵面 j 偏转 δ_j rad 产生的力矩 τ = r_j × F_j
%   气动力 F_j = q·S·C_Fδ·δ_j · n_j
%   所以 τ = q·S·C_Fδ · (r_j × n_j) · δ_j
%   H_tau(:,j) = q·S·C_Fδ · cross(r_j, n_j)
%   H_ref(:,j) = cross(r_j, n_j)   ← 本函数计算部分
H_ref = zeros(3, 4);
for j = 1:4
    H_ref(:,j) = cross(r_surface(:,j), n_surface(:,j));
end
end


function [delta, tau_achieved, feasible] = allocate_torque_pitch_protected(...
    tau_cmd, H_tau, h_pitch, N_ry, delta_max, lambda_pitch, lambda_servo)
% Pitch 保护型零空间舵面分配器
%
% 输入:
%   tau_cmd     - 3×1 指令力矩 [Mx; My; Mz]，N·m
%   H_tau       - 3×4 满力矩矩阵
%   h_pitch     - 1×4 H_tau 的 pitch 行
%   N_ry        - 4×k Roll/Yaw 零空间（k=2 正常）
%   delta_max   - 单舵限幅，rad
%   lambda_pitch - Pitch 力矩惩罚权重
%   lambda_servo - 舵面动作惩罚权重
%
% 输出:
%   delta       - 4×1 舵面偏转角 [UL;UR;DR;DL]，rad
%   tau_achieved - 3×1 实际力矩，N·m
%   feasible    - 是否在限幅内

H_ry = H_tau([1,3], :);     % 2×4 Roll+Yaw 行
tau_ry = tau_cmd([1,3]);     % 2×1

% Step 1: 最小范数解（满足 Roll/Yaw）
delta0 = pinv(H_ry) * tau_ry;   % 4×1

% Step 2: 在零空间内优化
% 优化变量：eta (2×1)
% delta = delta0 + N_ry * eta
%
% J = λ_pitch·(h_pitch·δ)² + λ_servo·(δ'·δ)
%
% 展开后对 eta 求导 = 0：
% a = h_pitch * N_ry        (1×2)
% b = h_pitch * delta0      (标量)
%
% Hessian: H = λ_pitch·(a'*a) + λ_servo·(N_ry'*N_ry)   (2×2)
% RHS:     g = -(λ_pitch·b·a' + λ_servo·N_ry'*delta0)   (2×1)
%
% 解: eta = H \ g

a = h_pitch * N_ry;           % 1×2
b = h_pitch * delta0;          % 标量

H_hess = lambda_pitch * (a' * a) + lambda_servo * (N_ry' * N_ry);   % 2×2
g = -(lambda_pitch * b * a' + lambda_servo * N_ry' * delta0);       % 2×1

% 解 2×2 线性系统（对称正定）
if cond(H_hess) < 1e12
    eta = H_hess \ g;
else
    eta = [0; 0];   % 病态时不优化
end

delta = delta0 + N_ry * eta;

% Step 3: 限幅
feasible = true;
if any(abs(delta) > delta_max)
    feasible = false;
    % 统一缩放（保持方向）
    scale = delta_max / max(abs(delta));
    delta = delta * scale;
end

% Step 4: 回算实际力矩
tau_achieved = H_tau * delta;
end


function [poly_coeff, max_abs_err] = fit_matrix_schedule(matrix_schedule, V_fit_offset, poly_order)
% 对矩阵调度表做多项式拟合
% 输入:
%   matrix_schedule - rows×cols×N 调度表
%   V_fit_offset    - 1×N 速度偏移（V - V_ref）
%   poly_order      - 多项式阶数
% 输出:
%   poly_coeff      - rows×cols×(poly_order+1) 多项式系数（升幂）
%   max_abs_err     - 最大绝对拟合误差

[row_num, col_num, ~] = size(matrix_schedule);
poly_coeff = zeros(row_num, col_num, poly_order + 1);
matrix_fit = zeros(size(matrix_schedule));

for row = 1:row_num
    for col = 1:col_num
        p = polyfit(V_fit_offset, squeeze(matrix_schedule(row, col, :))', poly_order);
        poly_coeff(row, col, :) = fliplr(p);   % 升幂排列 c0, c1, c2, ...
        matrix_fit(row, col, :) = polyval(p, V_fit_offset);
    end
end

max_abs_err = max(abs(matrix_fit(:) - matrix_schedule(:)));
end


function lqi_log(str)
% 将详细输出追加到 LQI 输出窗口（避免命令行刷屏）
% 通过 hLogFig 的 appdata 共享状态
    hFig = findobj('Type', 'figure', 'Name', 'LQI 详细输出');
    if isempty(hFig), fprintf('%s', str); return; end
    hText = getappdata(hFig, 'text');
    buf   = getappdata(hFig, 'buf');
    buf   = [buf str];
    setappdata(hFig, 'buf', buf);
    set(hText, 'String', buf);
    drawnow;
end
