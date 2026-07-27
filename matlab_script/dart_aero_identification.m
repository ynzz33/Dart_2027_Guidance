% ========================================================================
% 飞镖气动参数离线标定脚本
% ========================================================================
% 功能：从单次飞行遥测数据（Vofa TXT 导出）联合拟合 5 个气动参数：
%         Dp_ref, Dq_ref, Dr_ref（三轴气动阻尼）
%         Cm_pitch_ref（Pitch 气动回正系数）
%         C_F_delta（舵效缩放因子）
%
% 模型：ω̇ = I⁻¹·H_tau(V)·δ + D(V)·ω + Cm(V)·e_pitch
%       阻尼 ∝ V¹，回正/舵效 ∝ V²
%
% 使用：修改下方 TXT_FILE 路径，直接运行本脚本。
%       前半段数据拟合，后半段验证。
%       结果自动绘图，参数块可复制到 dart_lqi_parameters.m。
%
% 输入格式（无表头 TXT，空格/Tab/逗号分隔，12 列）：
%   列 1: 时间戳
%   列 2: V_DART_Lqi (m/s)
%   列 3: Pitch 欧拉角 (°)
%   列 4: Gyro Pitch (°/s)
%   列 5: Gyro Roll  (°/s)
%   列 6: Gyro Yaw   (°/s)
%   列 7: 舵面 UL (°)
%   列 8: 舵面 UR (°)
%   列 9: 舵面 DR (°)
%   列10: 舵面 DL (°)
%   列11: 占位（不使用）
%   列12: 占位（不使用）
% ========================================================================

clear; close all; clc;

%% ==================== 配置区（修改此处） ====================

TXT_FILE = 'flight_data.txt';   % ← 改成你的 TXT 文件路径

% 速度处理
V_USE_FIXED = false;            % true = 使用固定速度（EKF 不可用时）
V_FIXED_VALUE = 6.0;            % 固定速度值 (m/s)

% 数据裁剪：只使用此时间范围内的数据（秒），设 [] 使用全部
T_CROP = [];                    % 例如 [0.5, 1.5] 只取 0.5~1.5 秒之间的数据

% 采样周期（秒），设为 [] 则从时间戳自动检测
Ts_override = [];               % 例如 0.001（1kHz）

% 拟合/验证分割比例
FIT_RATIO = 0.5;                % 前 50% 拟合，后 50% 验证

% 优化参数边界
BOUNDS.Dp   = [-20, 0];         % Roll 阻尼 (1/s)，负值=稳定
BOUNDS.Dq   = [-20, 0];         % Pitch 阻尼
BOUNDS.Dr   = [-20, 0];         % Yaw 阻尼
BOUNDS.Cm   = [-100, 0];        % Pitch 回正 (1/s²)，负值=稳定
BOUNDS.CFd  = [0.1, 50];        % 舵效缩放因子

% 初始猜测
X0.Dp   = -1.0;
X0.Dq   = -1.0;
X0.Dr   = -1.0;
X0.Cm   = -1.0;
X0.CFd  = 5.0;

%% ==================== 物理常量（与 dart_lqi_parameters.m 同步） ====================

% --- 转动惯量 (kg·m²) ---
Ixx = 307131.230e-9;
Iyy = 1580238.787e-9;
Izz = 1589776.981e-9;
Ixy = 15335.950e-9;
Ixz = 2976.203e-9;
Iyz = -4624.159e-9;

I_body = [ Ixx, -Ixy, -Ixz;
          -Ixy,  Iyy, -Iyz;
          -Ixz, -Iyz,  Izz ];
I_inv = inv(I_body);

% --- 参考速度 ---
V_ref = 6.0;   % m/s

% --- H_tau_Vref（来自 lqi_geometry_table.h，N·m/rad @ V_ref） ---
% 列序 [UL, UR, DR, DL]，行序 [Roll力矩, Pitch力矩, Yaw力矩]
H_tau_Vref = [ ...
    -8.34533132e-02, -8.21693641e-02, -8.19216868e-02, -8.32056359e-02;
    +7.04712353e-02, -6.18287647e-02, -6.22135836e-02, +7.00864164e-02;
    +6.71448400e-02, +6.67576983e-02, -6.55423017e-02, -6.51551600e-02 ...
];

% --- C_Fδ 标称值（H_tau_Vref 隐含此值，标定输出的是相对此值的缩放因子） ---
CFd_nominal = 5.0;

%% ==================== Step 1: 加载数据 ====================

fprintf('========== Step 1: 加载数据 ==========\n');

if ~exist(TXT_FILE, 'file')
    error('文件不存在: %s\n请修改 TXT_FILE 变量为实际文件路径。', TXT_FILE);
end

% 自动检测分隔符
fid = fopen(TXT_FILE, 'r');
first_line = fgetl(fid);
fclose(fid);
if contains(first_line, ',')
    delimiter = ',';
elseif contains(first_line, '\t')
    delimiter = '\t';
else
    delimiter = ' ';
end

raw = dlmread(TXT_FILE, delimiter);
fprintf('加载 %s: %d 行 × %d 列\n', TXT_FILE, size(raw, 1), size(raw, 2));

if size(raw, 2) < 10
    error('数据至少需要 10 列（时间戳 + V + 角度 + 3陀螺 + 4舵面），实际 %d 列。', size(raw, 2));
end

% 提取列（不足 12 列时用 0 补齐）
function val = safe_col(data, col)
    if col <= size(data, 2)
        val = data(:, col);
    else
        val = zeros(size(data, 1), 1);
    end
end

t_raw    = safe_col(raw, 1);   % 时间戳
V_raw    = safe_col(raw, 2);   % V_DART_Lqi (m/s)
pitch_raw = safe_col(raw, 3);  % Pitch 欧拉角 (°)
gyro_q_raw = safe_col(raw, 4); % Gyro Pitch (°/s) = q
gyro_p_raw = safe_col(raw, 5); % Gyro Roll  (°/s) = p
gyro_r_raw = safe_col(raw, 6); % Gyro Yaw   (°/s) = r
servo_UL_raw = safe_col(raw, 7);
servo_UR_raw = safe_col(raw, 8);
servo_DR_raw = safe_col(raw, 9);
servo_DL_raw = safe_col(raw, 10);

%% ==================== Step 2: 数据预处理 ====================

fprintf('\n========== Step 2: 数据预处理 ==========\n');

% --- 2a) NaN/Inf 检查 ---
valid_mask = isfinite(t_raw) & isfinite(V_raw) & isfinite(pitch_raw) ...
    & isfinite(gyro_q_raw) & isfinite(gyro_p_raw) & isfinite(gyro_r_raw) ...
    & isfinite(servo_UL_raw) & isfinite(servo_UR_raw) ...
    & isfinite(servo_DR_raw) & isfinite(servo_DL_raw);

n_nan = sum(~valid_mask);
if n_nan > 0
    fprintf('移除 NaN/Inf: %d 行\n', n_nan);
    t_raw = t_raw(valid_mask); V_raw = V_raw(valid_mask);
    pitch_raw = pitch_raw(valid_mask);
    gyro_q_raw = gyro_q_raw(valid_mask); gyro_p_raw = gyro_p_raw(valid_mask); gyro_r_raw = gyro_r_raw(valid_mask);
    servo_UL_raw = servo_UL_raw(valid_mask); servo_UR_raw = servo_UR_raw(valid_mask);
    servo_DR_raw = servo_DR_raw(valid_mask); servo_DL_raw = servo_DL_raw(valid_mask);
end

% --- 2b) 时间戳检查与采样周期 ---
dt_raw = diff(t_raw);
if ~isempty(Ts_override)
    Ts = Ts_override;
    fprintf('采样周期（手动指定）: %.4f ms (%.0f Hz)\n', Ts*1000, 1/Ts);
else
    Ts = median(dt_raw);
    dt_range = max(dt_raw) - min(dt_raw);
    fprintf('采样周期（自动检测）: 中位数 %.4f ms, 范围 [%.4f, %.4f] ms\n', ...
        Ts*1000, min(dt_raw)*1000, max(dt_raw)*1000);
    if dt_range > Ts * 0.5
        fprintf('⚠ 时间戳不均匀（范围 %.2f ms > 中位数一半），检查数据源。\n', dt_range*1000);
    end
end
fprintf('数据长度: %d 点, 约 %.2f 秒\n', length(t_raw), length(t_raw)*Ts);

% --- 2c) 速度处理 ---
if V_USE_FIXED
    V_used = V_FIXED_VALUE * ones(size(V_raw));
    fprintf('使用固定速度: %.1f m/s\n', V_FIXED_VALUE);
else
    V_used = V_raw;
    V_used(V_used < 1.0) = 1.0;   % clamp 下界，避免除零
    V_used(V_used > 20.0) = 20.0;  % clamp 上界
    if all(V_raw == V_raw(1))
        fprintf('注意: V_DART_Lqi 全为 %.1f m/s（常数），等效固定速度。\n', V_raw(1));
    else
        fprintf('速度范围: [%.2f, %.2f] m/s\n', min(V_used), max(V_used));
    end
end

% --- 2d) 单位转换 ---
% 角度/角速度 °→rad
pitch_rad = deg2rad(pitch_raw);
gyro_p = deg2rad(gyro_p_raw);   % p (roll rate)
gyro_q = deg2rad(gyro_q_raw);   % q (pitch rate)
gyro_r = deg2rad(gyro_r_raw);   % r (yaw rate)

% 舵偏 °→rad
servo_UL = deg2rad(servo_UL_raw);
servo_UR = deg2rad(servo_UR_raw);
servo_DR = deg2rad(servo_DR_raw);
servo_DL = deg2rad(servo_DL_raw);

% --- 2e) 时间裁剪 ---
if ~isempty(T_CROP)
    t_start = t_raw(1);
    idx_crop = (t_raw >= t_start + T_CROP(1)) & (t_raw <= t_start + T_CROP(2));
    fprintf('时间裁剪: [%.2f, %.2f] 秒, 保留 %d / %d 点\n', ...
        T_CROP(1), T_CROP(2), sum(idx_crop), length(t_raw));
    t_raw = t_raw(idx_crop); V_used = V_used(idx_crop);
    pitch_rad = pitch_rad(idx_crop);
    gyro_p = gyro_p(idx_crop); gyro_q = gyro_q(idx_crop); gyro_r = gyro_r(idx_crop);
    servo_UL = servo_UL(idx_crop); servo_UR = servo_UR(idx_crop);
    servo_DR = servo_DR(idx_crop); servo_DL = servo_DL(idx_crop);
end

N = length(t_raw);
fprintf('预处理后数据点: %d\n', N);

%% ==================== Step 3: 计算角加速度 ====================

fprintf('\n========== Step 3: 计算角加速度 ==========\n');

% 中心差分（比前向/后向差分噪声减半）
% ω̇[k] = (ω[k+1] - ω[k-1]) / (2·Ts)
alpha_p = zeros(N, 1);  % dp/dt
alpha_q = zeros(N, 1);  % dq/dt
alpha_r = zeros(N, 1);  % dr/dt

for k = 2:N-1
    dt_k = (t_raw(k+1) - t_raw(k-1)) / 2;  % 半跨距（处理非均匀采样）
    if dt_k > 0
        alpha_p(k) = (gyro_p(k+1) - gyro_p(k-1)) / (2 * dt_k);
        alpha_q(k) = (gyro_q(k+1) - gyro_q(k-1)) / (2 * dt_k);
        alpha_r(k) = (gyro_r(k+1) - gyro_r(k-1)) / (2 * dt_k);
    end
end
% 首尾用单侧差分
alpha_p(1) = (gyro_p(2) - gyro_p(1)) / max(t_raw(2)-t_raw(1), Ts);
alpha_q(1) = (gyro_q(2) - gyro_q(1)) / max(t_raw(2)-t_raw(1), Ts);
alpha_r(1) = (gyro_r(2) - gyro_r(1)) / max(t_raw(2)-t_raw(1), Ts);
alpha_p(N) = (gyro_p(N) - gyro_p(N-1)) / max(t_raw(N)-t_raw(N-1), Ts);
alpha_q(N) = (gyro_q(N) - gyro_q(N-1)) / max(t_raw(N)-t_raw(N-1), Ts);
alpha_r(N) = (gyro_r(N) - gyro_r(N-1)) / max(t_raw(N)-t_raw(N-1), Ts);

% 异常值过滤：角加速度超过物理合理范围（5000°/s² ≈ 87 rad/s²）标记但不删除
ALPHA_LIMIT = 87.0;  % rad/s²
outlier = abs(alpha_p) > ALPHA_LIMIT | abs(alpha_q) > ALPHA_LIMIT | abs(alpha_r) > ALPHA_LIMIT;
fprintf('角加速度超限点 (>%.0f rad/s²): %d / %d (%.2f%%)\n', ALPHA_LIMIT, sum(outlier), N, sum(outlier)/N*100);

%% ==================== Step 4: 舵面饱和与状态切换过滤 ====================

fprintf('\n========== Step 4: 质量过滤 ==========\n');

% 舵面饱和：|舵偏| > 55°（留 5° 余量，60° 是硬限幅）
SERVO_SAT_DEG = 55.0;
servo_sat = abs(servo_UL_raw) > SERVO_SAT_DEG | abs(servo_UR_raw) > SERVO_SAT_DEG ...
          | abs(servo_DR_raw) > SERVO_SAT_DEG | abs(servo_DL_raw) > SERVO_SAT_DEG;

% 组合过滤掩码
valid_fit = ~outlier & ~servo_sat;
fprintf('移除舵面饱和点: %d\n', sum(servo_sat));
fprintf('有效数据点: %d / %d (%.1f%%)\n', sum(valid_fit), N, sum(valid_fit)/N*100);

%% ==================== Step 5: 参数辨识 ====================

fprintf('\n========== Step 5: 参数辨识（联合拟合 5 参数） ==========\n');

% 分割拟合/验证集
N_fit = floor(N * FIT_RATIO);
idx_fit = 1:N_fit;
idx_val = (N_fit+1):N;

% 拟合集掩码
fit_mask = valid_fit(idx_fit);

% 提取拟合数据
t_fit   = t_raw(idx_fit);
V_fit   = V_used(idx_fit);
pitch_fit = pitch_rad(idx_fit);
p_fit   = gyro_p(idx_fit);
q_fit   = gyro_q(idx_fit);
r_fit   = gyro_r(idx_fit);
ap_fit  = alpha_p(idx_fit);
aq_fit  = alpha_q(idx_fit);
ar_fit  = alpha_r(idx_fit);
s_fit   = [servo_UL(idx_fit)'; servo_UR(idx_fit)'; servo_DR(idx_fit)'; servo_DL(idx_fit)'];  % 4×N_fit

% 打包优化数据
data.t_raw   = t_fit;
data.V       = V_fit;
data.pitch   = pitch_fit;
data.p       = p_fit;
data.q       = q_fit;
data.r       = r_fit;
data.ap      = ap_fit;
data.aq      = aq_fit;
data.ar      = ar_fit;
data.servo   = s_fit;
data.mask    = fit_mask;
data.I_inv   = I_inv;
data.H_tau_Vref = H_tau_Vref;
data.V_ref   = V_ref;
data.CFd_nominal = CFd_nominal;

% 参数向量: [Dp, Dq, Dr, Cm_pitch, C_F_delta]
x0 = [X0.Dp, X0.Dq, X0.Dr, X0.Cm, X0.CFd];
lb = [BOUNDS.Dp(1), BOUNDS.Dq(1), BOUNDS.Dr(1), BOUNDS.Cm(1), BOUNDS.CFd(1)];
ub = [BOUNDS.Dp(2), BOUNDS.Dq(2), BOUNDS.Dr(2), BOUNDS.Cm(2), BOUNDS.CFd(2)];

fprintf('初始猜测: Dp=%.2f, Dq=%.2f, Dr=%.2f, Cm_pitch=%.2f, C_Fδ=%.2f\n', x0);
fprintf('拟合点数: %d (有效 %d), 验证点数: %d\n', N_fit, sum(fit_mask), N-N_fit);

% 非线性最小二乘
opts = optimoptions('lsqnonlin', ...
    'Display', 'iter', ...
    'Algorithm', 'trust-region-reflective', ...
    'MaxIterations', 200, ...
    'FunctionTolerance', 1e-10, ...
    'StepTolerance', 1e-10);

[x_opt, resnorm, residual, ~, ~, ~, jacobian] = ...
    lsqnonlin(@(x) aero_residual(x, data), x0, lb, ub, opts);

% 参数提取
Dp_fit   = x_opt(1);
Dq_fit   = x_opt(2);
Dr_fit   = x_opt(3);
Cm_fit   = x_opt(4);
CFd_fit  = x_opt(5);

fprintf('\n========== 标定结果 ==========\n');
fprintf('Dp_ref      = %+.4f  (1/s)\n', Dp_fit);
fprintf('Dq_ref      = %+.4f  (1/s)\n', Dq_fit);
fprintf('Dr_ref      = %+.4f  (1/s)\n', Dr_fit);
fprintf('Cm_pitch_ref = %+.4f  (1/s²)\n', Cm_fit);
fprintf('C_F_delta   = %+.4f  (有效缩放因子, 标称=%.1f)\n', CFd_fit, CFd_nominal);

% 拟合优度：RMSE
rmse_fit = sqrt(resnorm / sum(fit_mask));
fprintf('拟合 RMSE (rad/s²): %.4f\n', rmse_fit);

%% ==================== Step 6: 参数可信度 ====================

fprintf('\n========== Step 6: 参数可信度 ==========\n');

% 从 Jacobian 近似计算参数不确定度
if ~isempty(jacobian)
    J = full(jacobian);
    % 残差方差估计
    n_eff = sum(fit_mask);
    sigma2 = resnorm / (n_eff - 5);  % 5 个参数
    % 参数协方差
    try
        Cov = sigma2 * inv(J' * J);
        param_se = sqrt(diag(Cov));
        param_names = {'Dp_ref', 'Dq_ref', 'Dr_ref', 'Cm_pitch_ref', 'C_F_delta'};
        fprintf('参数      估计值        标准误差    相对误差(%%)\n');
        fprintf('--------------------------------------------------\n');
        for i = 1:5
            rel_err = abs(param_se(i) / max(abs(x_opt(i)), 1e-10)) * 100;
            fprintf('%-10s %+12.6f  %12.6f  %8.2f%%\n', ...
                param_names{i}, x_opt(i), param_se(i), rel_err);
        end
    catch
        fprintf('(协方差矩阵奇异，无法计算可信度。可能是数据信息量不足或参数间强相关。)\n');
    end
else
    fprintf('(无 Jacobian 信息，跳过可信度计算。)\n');
end

%% ==================== Step 7: 验证（后半段数据） ====================

fprintf('\n========== Step 7: 验证（后半段数据） ==========\n');

% 用拟合参数对全段数据计算模型预测
[ap_pred_all, aq_pred_all, ar_pred_all] = predict_accel( ...
    x_opt, gyro_p, gyro_q, gyro_r, pitch_rad, ...
    [servo_UL'; servo_UR'; servo_DR'; servo_DL'], ...
    V_used, I_inv, H_tau_Vref, V_ref, CFd_nominal);

% 拟合段残差
res_p_fit = alpha_p(idx_fit) - ap_pred_all(idx_fit);
res_q_fit = alpha_q(idx_fit) - aq_pred_all(idx_fit);
res_r_fit = alpha_r(idx_fit) - ar_pred_all(idx_fit);

% 验证段残差
res_p_val = alpha_p(idx_val) - ap_pred_all(idx_val);
res_q_val = alpha_q(idx_val) - aq_pred_all(idx_val);
res_r_val = alpha_r(idx_val) - ar_pred_all(idx_val);

rmse_fit_val = [...
    sqrt(mean(res_p_fit(fit_mask).^2)), sqrt(mean(res_q_fit(fit_mask).^2)), sqrt(mean(res_r_fit(fit_mask).^2));
    sqrt(mean(res_p_val.^2)), sqrt(mean(res_q_val.^2)), sqrt(mean(res_r_val.^2)) ...
];

fprintf('          Roll RMSE   Pitch RMSE  Yaw RMSE  (rad/s²)\n');
fprintf('拟合段    %10.4f  %10.4f  %10.4f\n', rmse_fit_val(1,1), rmse_fit_val(1,2), rmse_fit_val(1,3));
fprintf('验证段    %10.4f  %10.4f  %10.4f\n', rmse_fit_val(2,1), rmse_fit_val(2,2), rmse_fit_val(2,3));

% 验证段恶化检查
for ax = 1:3
    if rmse_fit_val(2,ax) > rmse_fit_val(1,ax) * 3
        ax_name = {'Roll','Pitch','Yaw'};
        fprintf('⚠ %s 轴验证段 RMSE 恶化 >3×，可能存在过拟合或后半段工况变化。\n', ax_name{ax});
    end
end

%% ==================== Step 8: 绘图 ====================

fprintf('\n========== Step 8: 绘图 ==========\n');

t_plot = t_raw - t_raw(1);  % 相对时间 (s)
t_split = t_raw(N_fit) - t_raw(1);

figure('Name', 'Dart Aero Identification', 'Color', 'w', ...
    'Position', [100, 80, 1200, 800]);

% --- 图 1-3: 三轴角加速度 实测 vs 预测 ---
ax_names = {'Roll (p)', 'Pitch (q)', 'Yaw (r)'};
alpha_meas = {alpha_p, alpha_q, alpha_r};
alpha_pred = {ap_pred_all, aq_pred_all, ar_pred_all};
res_all = {res_p_fit, res_q_fit, res_r_fit};

for ax = 1:3
    subplot(3, 3, (ax-1)*3 + 1);
    hold on;
    plot(t_plot, alpha_meas{ax}, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.5, 'DisplayName', '实测');
    plot(t_plot, alpha_pred{ax}, 'b', 'LineWidth', 1.2, 'DisplayName', '模型预测');
    xline(t_split, 'k--', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('角加速度 (rad/s²)');
    title(sprintf('%s 角加速度: 实测 vs 模型', ax_names{ax}));
    legend('Location', 'best'); grid on;
    hold off;

    % 残差
    subplot(3, 3, (ax-1)*3 + 2);
    plot(t_plot(idx_fit), res_all{ax}(fit_mask), '.', 'MarkerSize', 3, 'Color', [0.2 0.2 0.8]);
    hold on;
    plot(t_plot(idx_val), res_p_val, '.', 'MarkerSize', 3, 'Color', [0.8 0.2 0.2]);
    xline(t_split, 'k--', 'LineWidth', 1.5);
    yline(0, 'k-');
    xlabel('时间 (s)'); ylabel('残差 (rad/s²)');
    title(sprintf('%s 残差 (蓝=拟合段, 红=验证段)', ax_names{ax}));
    grid on;

    % 残差直方图
    subplot(3, 3, (ax-1)*3 + 3);
    histogram(res_all{ax}, 40, 'FaceColor', [0.3 0.5 0.8], 'EdgeColor', 'none');
    xlabel('残差 (rad/s²)'); ylabel('频次');
    title(sprintf('%s 残差分布', ax_names{ax}));
    grid on;
end

sgtitle(sprintf(['飞镖气动参数标定 | Dp=%.2f Dq=%.2f Dr=%.2f Cm=%.2f CFd=%.2f | ' ...
    '拟合RMSE=%.3f rad/s²'], Dp_fit, Dq_fit, Dr_fit, Cm_fit, CFd_fit, rmse_fit));

% --- 图 4: 各轴力矩贡献分解（拟合段取均值） ---
figure('Name', 'Torque Decomposition', 'Color', 'w', ...
    'Position', [150, 100, 1000, 500]);

% 对拟合段逐拍分解力矩贡献
N_plot = min(300, sum(fit_mask));  % 最多画 300 点
idx_plot = find(fit_mask, N_plot);

% 组装拟合段各分量
nu_plot = V_fit(idx_plot) / V_ref;
nu2_plot = nu_plot .^ 2;
s_mat = s_fit(:, idx_plot);  % 4×N

IinvH = I_inv * H_tau_Vref;  % 3×4

ap_servo_plot = zeros(N_plot, 1);
aq_servo_plot = zeros(N_plot, 1);
ar_servo_plot = zeros(N_plot, 1);
ap_damp_plot  = zeros(N_plot, 1);
aq_damp_plot  = zeros(N_plot, 1);
ar_damp_plot  = zeros(N_plot, 1);
aq_restore_plot = zeros(N_plot, 1);

for i = 1:N_plot
    % 舵面项: (CFd/CFd_nominal) * nu² * I⁻¹ * H_tau * δ
    servo_i = (CFd_fit / CFd_nominal) * nu2_plot(i) * IinvH * s_mat(:, i);
    ap_servo_plot(i) = servo_i(1);
    aq_servo_plot(i) = servo_i(2);
    ar_servo_plot(i) = servo_i(3);

    % 阻尼项: D * nu * ω
    ap_damp_plot(i) = Dp_fit * nu_plot(i) * p_fit(idx_plot(i));
    aq_damp_plot(i) = Dq_fit * nu_plot(i) * q_fit(idx_plot(i));
    ar_damp_plot(i) = Dr_fit * nu_plot(i) * r_fit(idx_plot(i));

    % 回正项: Cm * nu² * e_pitch
    aq_restore_plot(i) = Cm_fit * nu2_plot(i) * pitch_fit(idx_plot(i));
end

t_comp = t_plot(idx_fit(idx_plot));

for ax = 1:3
    subplot(1, 3, ax);
    hold on;
    switch ax
        case 1  % Roll
            plot(t_comp, ap_servo_plot, 'r', 'LineWidth', 1.2, 'DisplayName', '舵面');
            plot(t_comp, ap_damp_plot, 'b', 'LineWidth', 1.2, 'DisplayName', '阻尼');
            plot(t_comp, ap_servo_plot + ap_damp_plot, 'k--', 'LineWidth', 1.5, 'DisplayName', '合计');
        case 2  % Pitch
            plot(t_comp, aq_servo_plot, 'r', 'LineWidth', 1.2, 'DisplayName', '舵面');
            plot(t_comp, aq_damp_plot, 'b', 'LineWidth', 1.2, 'DisplayName', '阻尼');
            plot(t_comp, aq_restore_plot, 'g', 'LineWidth', 1.2, 'DisplayName', '回正');
            plot(t_comp, aq_servo_plot + aq_damp_plot + aq_restore_plot, 'k--', 'LineWidth', 1.5, 'DisplayName', '合计');
        case 3  % Yaw
            plot(t_comp, ar_servo_plot, 'r', 'LineWidth', 1.2, 'DisplayName', '舵面');
            plot(t_comp, ar_damp_plot, 'b', 'LineWidth', 1.2, 'DisplayName', '阻尼');
            plot(t_comp, ar_servo_plot + ar_damp_plot, 'k--', 'LineWidth', 1.5, 'DisplayName', '合计');
    end
    xlabel('时间 (s)'); ylabel('角加速度贡献 (rad/s²)');
    title(sprintf('%s 力矩分解', ax_names{ax}));
    legend('Location', 'best'); grid on;
    hold off;
end
sgtitle('角加速度分量分解（拟合段）');

%% ==================== Step 9: 导出参数块 ====================

fprintf('\n========== Step 9: 导出参数块 ==========\n');
fprintf('复制以下内容到 dart_lqi_parameters.m 的对应位置:\n');
fprintf('──────────────────────────────────────────────\n');
fprintf('%% 气动参数（离线标定 %s）\n', datestr(now));
fprintf('Dp_ref   = %+.6f;   %% roll damping, 1/s\n', Dp_fit);
fprintf('Dq_ref   = %+.6f;   %% pitch damping, 1/s\n', Dq_fit);
fprintf('Dr_ref   = %+.6f;   %% yaw damping, 1/s\n', Dr_fit);
fprintf('Cm_pitch_ref = %+.6f;   %% pitch restoring, 1/s²\n', Cm_fit);
fprintf('Cm_yaw_ref   = 0;         %% yaw restoring, 1/s² (飞镖细长体无需标定)\n');
fprintf('C_F_delta = %+.6f;   %% 舵效系数 (标称=%.1f)，1/rad\n', CFd_fit, CFd_nominal);
fprintf('DART_LQI_ENABLE_AERO_A = true;\n');
fprintf('──────────────────────────────────────────────\n');

fprintf('\n========== 完成 ==========\n');
fprintf('工作区变量: Dp_fit, Dq_fit, Dr_fit, Cm_fit, CFd_fit (标定值)\n');
fprintf('            alpha_* (实测角加速度), ap/aq/ar_pred_all (模型预测)\n');

%% ==================== 辅助函数 ====================

function res = aero_residual(x, data)
% 残差函数：ω̇_measured - ω̇_model(params)
% x = [Dp, Dq, Dr, Cm_pitch, C_F_delta]
%
% 模型: ω̇ = I⁻¹·H_tau(V)·δ + D(V)·ω + Cm(V)·e_pitch
%   H_tau(V) = (CFd/CFd_nominal) * (V/V_ref)² * H_tau_Vref
%   阻尼 D(V) = D_ref * (V/V_ref)
%   回正 Cm(V) = Cm_ref * (V/V_ref)²

Dp   = x(1);
Dq   = x(2);
Dr   = x(3);
Cm   = x(4);
CFd  = x(5);

% 解包
V    = data.V;
p    = data.p;
q    = data.q;
r    = data.r;
pitch = data.pitch;
servo = data.servo;    % 4×N
ap   = data.ap;
aq   = data.aq;
ar   = data.ar;
mask = data.mask;
I_inv = data.I_inv;
H_tau_Vref = data.H_tau_Vref;
V_ref = data.V_ref;
CFd_nom = data.CFd_nominal;

% 有效数据点
V_e    = V(mask);
p_e    = p(mask);
q_e    = q(mask);
r_e    = r(mask);
pitch_e = pitch(mask);
servo_e = servo(:, mask);  % 4×N_eff
ap_e   = ap(mask);
aq_e   = aq(mask);
ar_e   = ar(mask);

N_eff = sum(mask);
res = zeros(3 * N_eff, 1);

% 预计算 I⁻¹ * H_tau_Vref (3×4)，每拍只需缩放
IinvH = I_inv * H_tau_Vref;    % 3×4
CFd_scale = CFd / CFd_nom;

for i = 1:N_eff
    nu   = V_e(i) / V_ref;
    nu2  = nu * nu;

    % --- 舵面贡献: (CFd/CFd_nom) * nu² * I⁻¹ * H_tau_Vref * δ ---
    tau_servo = CFd_scale * nu2 * IinvH * servo_e(:, i);  % 3×1

    % --- 阻尼贡献: D_ref * nu * ω ---
    damp_p = Dp * nu * p_e(i);
    damp_q = Dq * nu * q_e(i);
    damp_r = Dr * nu * r_e(i);

    % --- 回正贡献: Cm * nu² * e_pitch（仅 pitch，= 0 for others） ---
    restore_q = Cm * nu2 * pitch_e(i);

    % --- 模型预测角加速度 ---
    ap_model = tau_servo(1) + damp_p;                     % Roll: 无回正
    aq_model = tau_servo(2) + damp_q + restore_q;          % Pitch: 含回正
    ar_model = tau_servo(3) + damp_r;                     % Yaw: 无回正

    % --- 残差: 实测 - 预测 ---
    base = 3 * (i-1);
    res(base+1) = ap_e(i) - ap_model;
    res(base+2) = aq_e(i) - aq_model;
    res(base+3) = ar_e(i) - ar_model;
end
end

function [ap, aq, ar] = predict_accel(x, p, q, r, pitch, servo, V, I_inv, H_tau_Vref, V_ref, CFd_nom)
% 用给定参数计算全段角加速度预测
% x = [Dp, Dq, Dr, Cm_pitch, C_F_delta]
% servo: 4×N

Dp  = x(1); Dq  = x(2); Dr  = x(3);
Cm  = x(4); CFd = x(5);

N = length(p);
ap = zeros(N, 1); aq = zeros(N, 1); ar = zeros(N, 1);

IinvH = I_inv * H_tau_Vref;
CFd_scale = CFd / CFd_nom;

for i = 1:N
    nu  = V(i) / V_ref;
    nu2 = nu * nu;

    tau_servo = CFd_scale * nu2 * IinvH * servo(:, i);

    ap(i) = tau_servo(1) + Dp * nu * p(i);
    aq(i) = tau_servo(2) + Dq * nu * q(i) + Cm * nu2 * pitch(i);
    ar(i) = tau_servo(3) + Dr * nu * r(i);
end
end
