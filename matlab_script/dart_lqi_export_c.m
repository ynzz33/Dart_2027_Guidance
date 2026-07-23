% ========================================================================
% ⛔ 已废弃！请勿运行本脚本！
% ========================================================================
% 本脚本生成旧格式的多速点 K 表 + H 表，与当前 C 端不兼容。
% 当前 C 端使用单矩阵格式（lqi_K[3][9] + lqi_H_tau_Vref[3][4]）。
%
% 请改用主脚本的导出功能：
%   运行 dart_lqi_parameters.m → 自动链式运行
%   或 dart_attitude_lqi_torque_pitch_protected.m
%   主脚本内部 Step 8 已包含完整的单矩阵 C 头文件导出。
%
% 本文件保留仅作旧格式参考。若再次运行将覆盖当前正确的单矩阵头文件！
% ========================================================================
error('此脚本已废弃。请运行 dart_lqi_parameters.m（自动链式调用主脚本）。');
% ========================================================================

fprintf('\n========== 导出 C 静态表 ==========\n');

% ---- 路径配置 ----
% 工程项目根目录（相对于本脚本所在位置向上两级）
project_root = fullfile(pwd, '..');
lqi_tool_dir = fullfile(project_root, 'imcalib', 'lqi_tool');

% 确保目录存在
if ~exist(lqi_tool_dir, 'dir')
    mkdir(lqi_tool_dir);
    fprintf('创建目录: %s\n', lqi_tool_dir);
end

% 选择导出的速度点（采样 N_speed 个点做线性插值表）
N_speed_export = 20;   % 嵌入式表点数
V_export_idx = round(linspace(1, numel(V_schedule), N_speed_export));
V_export = V_schedule(V_export_idx);

% 提取对应增益
K_export = K_lqi_schedule(:,:,V_export_idx);   % 3×9×N
H_export = H_tau_schedule(:,:,V_export_idx);   % 3×4×N

% 导出时计算 V_ref 处的零空间（嵌入式当前用此速度）
k_aero_ref = 0.5 * rho_ac * V_ref^2 * S_surface * C_F_delta;
H_tau_Vref = k_aero_ref * H_tau_ref;
H_ry_Vref = H_tau_Vref([1,3], :);    % Roll+Yaw 行
N_ry_Vref = null(H_ry_Vref);          % 4×2 零空间

% 确保 N_ry 两列符号一致（null 的输出符号可能随 MATLAB 版本变化）
% 约定：使第一行非零元素的符号为正
for col = 1:size(N_ry_Vref, 2)
    [~, first_nonzero] = max(abs(N_ry_Vref(:,col)));
    if N_ry_Vref(first_nonzero, col) < 0
        N_ry_Vref(:,col) = -N_ry_Vref(:,col);
    end
end

% ---- 生成 lqi_gain_table.h ----
gain_h_path = fullfile(lqi_tool_dir, 'lqi_gain_table.h');
fid_gain = fopen(gain_h_path, 'w');
if fid_gain < 0
    error('无法写入文件: %s', gain_h_path);
end

fprintf(fid_gain, '/* ========================================================================\n');
fprintf(fid_gain, ' * lqi_gain_table.h — LQI 增益调度表（MATLAB 自动生成）\n');
fprintf(fid_gain, ' * ========================================================================\n');
fprintf(fid_gain, ' * 生成时间: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid_gain, ' * 脚本:     matlab_script/dart_lqi_export_c.m\n');
fprintf(fid_gain, ' *\n');
fprintf(fid_gain, ' * 状态顺序: xa = [e_roll, e_pitch, e_yaw, p, q, r, I_roll, I_pitch, I_yaw]\n');
fprintf(fid_gain, ' *            单位: rad, rad, rad, rad/s, rad/s, rad/s, rad·s, rad·s, rad·s\n');
fprintf(fid_gain, ' * 控制输出: tau = [Mx, My, Mz]  (N·m)\n');
fprintf(fid_gain, ' *            Mx = roll 力矩, My = pitch 力矩, Mz = yaw 力矩\n');
fprintf(fid_gain, ' *\n');
fprintf(fid_gain, ' * 控制律:   tau = -K_lqi * xa\n');
fprintf(fid_gain, ' *\n');
fprintf(fid_gain, ' * LQR 权重（dlqr, Ts=%.3fs）:\n', Ts);
fprintf(fid_gain, ' *   Q = diag([%.3f, %.4f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f])\n', ...
    diag(Q_aug));
fprintf(fid_gain, ' *   R = diag([%.1f, %.1f, %.1f])\n', diag(R_tau));
fprintf(fid_gain, ' *\n');
fprintf(fid_gain, ' * 速度范围: %.1f ~ %.1f m/s (V_ref = %.1f m/s)\n', ...
    V_schedule(1), V_schedule(end), V_ref);
fprintf(fid_gain, ' * 表长度:   %d 个速度点\n', N_speed_export);
fprintf(fid_gain, ' *\n');
fprintf(fid_gain, ' * ⚠ 警告: 本表基于占位符气动参数生成，\n');
fprintf(fid_gain, ' *          所有数值仅用于算法逻辑验证，\n');
fprintf(fid_gain, ' *          上台架前须用真实参数重新生成！\n');
fprintf(fid_gain, ' * ======================================================================== */\n\n');

fprintf(fid_gain, '#ifndef LQI_GAIN_TABLE_H\n');
fprintf(fid_gain, '#define LQI_GAIN_TABLE_H\n\n');
fprintf(fid_gain, '#include <stdint.h>\n\n');

fprintf(fid_gain, '/* 速度表（m/s）*/\n');
fprintf(fid_gain, '#define LQI_SPEED_TABLE_SIZE %d\n', N_speed_export);
fprintf(fid_gain, '#define LQI_V_REF %.4ff\n', V_ref);
fprintf(fid_gain, '#define LQI_V_MIN %.4ff\n', V_export(1));
fprintf(fid_gain, '#define LQI_V_MAX %.4ff\n', V_export(end));
fprintf(fid_gain, '#define LQI_STATE_DIM  9\n');
fprintf(fid_gain, '#define LQI_TORQUE_DIM 3\n\n');

fprintf(fid_gain, 'static const float lqi_speed_table[LQI_SPEED_TABLE_SIZE] = {\n    ');
fprintf(fid_gain, '%.4ff, ', V_export(1:end-1));
fprintf(fid_gain, '%.4ff\n};\n\n', V_export(end));

fprintf(fid_gain, '/*\n');
fprintf(fid_gain, ' * K_lqi 增益表: [速度索引][力矩轴][状态]\n');
fprintf(fid_gain, ' *   力矩轴: 0=Mx(roll), 1=My(pitch), 2=Mz(yaw)\n');
fprintf(fid_gain, ' *   状态:   0=e_roll, 1=e_pitch, 2=e_yaw, 3=p, 4=q, 5=r, 6=I_roll, 7=I_pitch, 8=I_yaw\n');
fprintf(fid_gain, ' *   单位:   角度误差→N·m/rad, 角速度→N·m/(rad/s), 积分→N·m/(rad·s)\n');
fprintf(fid_gain, ' */\n');
fprintf(fid_gain, 'static const float lqi_K_table[LQI_SPEED_TABLE_SIZE][LQI_TORQUE_DIM][LQI_STATE_DIM] = {\n');

for iv = 1:N_speed_export
    fprintf(fid_gain, '    /* V = %.4f m/s */\n', V_export(iv));
    fprintf(fid_gain, '    {\n');
    for iax = 1:3
        fprintf(fid_gain, '        {');
        fprintf(fid_gain, '%+.8ef, ', K_export(iax, 1:8, iv));
        fprintf(fid_gain, '%+.8ef}', K_export(iax, 9, iv));
        if iax < 3
            fprintf(fid_gain, ',\n');
        else
            fprintf(fid_gain, '\n');
        end
    end
    fprintf(fid_gain, '    }');
    if iv < N_speed_export
        fprintf(fid_gain, ',');
    end
    fprintf(fid_gain, '\n');
end
fprintf(fid_gain, '};\n\n');

fprintf(fid_gain, '#endif /* LQI_GAIN_TABLE_H */\n');
fclose(fid_gain);
fprintf('✓ 已生成: %s\n', gain_h_path);

% ---- 生成 lqi_geometry_table.h ----
geom_h_path = fullfile(lqi_tool_dir, 'lqi_geometry_table.h');
fid_geom = fopen(geom_h_path, 'w');
if fid_geom < 0
    error('无法写入文件: %s', geom_h_path);
end

fprintf(fid_geom, '/* ========================================================================\n');
fprintf(fid_geom, ' * lqi_geometry_table.h — 力矩几何表 + 零空间（MATLAB 自动生成）\n');
fprintf(fid_geom, ' * ========================================================================\n');
fprintf(fid_geom, ' * 生成时间: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid_geom, ' *\n');
fprintf(fid_geom, ' * H_tau: 舵偏 → 三轴力矩 (N·m/rad)\n');
fprintf(fid_geom, ' *   行: 0=Mx(roll), 1=My(pitch), 2=Mz(yaw)\n');
fprintf(fid_geom, ' *   列: [UL, UR, DR, DL]\n');
fprintf(fid_geom, ' *\n');
fprintf(fid_geom, ' * N_ry:  Roll/Yaw 零空间 (4×2)\n');
fprintf(fid_geom, ' *   H_ry * N_ry = 0\n');
fprintf(fid_geom, ' *   行: [UL, UR, DR, DL]\n');
fprintf(fid_geom, ' *   列: 两个零空间方向\n');
fprintf(fid_geom, ' *\n');
fprintf(fid_geom, ' * ⚠ 占位符参数：所有数值需 SolidWorks/CFD 数据后重新生成！\n');
fprintf(fid_geom, ' * ======================================================================== */\n\n');

fprintf(fid_geom, '#ifndef LQI_GEOMETRY_TABLE_H\n');
fprintf(fid_geom, '#define LQI_GEOMETRY_TABLE_H\n\n');
fprintf(fid_geom, '#include <stdint.h>\n\n');

fprintf(fid_geom, '/* 舵面数量 */\n');
fprintf(fid_geom, '#define LQI_SERVO_COUNT 4\n');
fprintf(fid_geom, '#define LQI_NRY_DIM     2   /* Roll/Yaw 零空间维度 */\n\n');

fprintf(fid_geom, '/*\n');
fprintf(fid_geom, ' * H_tau_ref: 力矩几何部分（r×n，不含动压/面积/舵效）\n');
fprintf(fid_geom, ' * 单位: m（位置叉乘力臂）\n');
fprintf(fid_geom, ' * 实际 H_tau(V,k) = 0.5*rho*V²*S*C_Fδ * H_tau_ref\n');
fprintf(fid_geom, ' * （嵌入式可预乘后存入下表）\n');
fprintf(fid_geom, ' */\n');
fprintf(fid_geom, 'static const float lqi_H_ref[3][4] = {\n');
for iax = 1:3
    fprintf(fid_geom, '    {');
    fprintf(fid_geom, '%+.8ef, ', H_tau_ref(iax, 1:3));
    fprintf(fid_geom, '%+.8ef}', H_tau_ref(iax, 4));
    if iax < 3
        fprintf(fid_geom, ',\n');
    else
        fprintf(fid_geom, '\n');
    end
end
fprintf(fid_geom, '};\n\n');

fprintf(fid_geom, '/*\n');
fprintf(fid_geom, ' * H_tau 调度表（已含动压缩放，可直接使用）\n');
fprintf(fid_geom, ' * [速度索引][力矩轴][舵面]\n');
fprintf(fid_geom, ' */\n');
fprintf(fid_geom, 'static const float lqi_H_table[LQI_SPEED_TABLE_SIZE][3][4] = {\n');
for iv = 1:N_speed_export
    fprintf(fid_geom, '    /* V = %.4f m/s */\n', V_export(iv));
    fprintf(fid_geom, '    {\n');
    for iax = 1:3
        fprintf(fid_geom, '        {');
        fprintf(fid_geom, '%+.8ef, ', H_export(iax, 1:3, iv));
        fprintf(fid_geom, '%+.8ef}', H_export(iax, 4, iv));
        if iax < 3
            fprintf(fid_geom, ',\n');
        else
            fprintf(fid_geom, '\n');
        end
    end
    fprintf(fid_geom, '    }');
    if iv < N_speed_export
        fprintf(fid_geom, ',');
    end
    fprintf(fid_geom, '\n');
end
fprintf(fid_geom, '};\n\n');

fprintf(fid_geom, '/*\n');
fprintf(fid_geom, ' * N_ry: Roll/Yaw 零空间 (4×2)，在 V_ref 处计算\n');
fprintf(fid_geom, ' * H_ry * N_ry = 0\n');
fprintf(fid_geom, ' * 嵌入式当前固定使用此零空间（单速度模式）\n');
fprintf(fid_geom, ' */\n');
fprintf(fid_geom, 'static const float lqi_N_ry[LQI_SERVO_COUNT][LQI_NRY_DIM] = {\n');
for i = 1:4
    fprintf(fid_geom, '    {%+.8ef, %+.8ef}', N_ry_Vref(i,1), N_ry_Vref(i,2));
    if i < 4
        fprintf(fid_geom, ',\n');
    else
        fprintf(fid_geom, '\n');
    end
end
fprintf(fid_geom, '};\n\n');

fprintf(fid_geom, '/* Pitch 行（从 H_tau 提取，方便快速计算 pitch 力矩）*/\n');
fprintf(fid_geom, '#define LQI_PITCH_ROW 1  /* H_tau 第 1 行 = pitch 力矩 */\n\n');

fprintf(fid_geom, '/* 舵面限幅 */\n');
fprintf(fid_geom, '#define LQI_DELTA_MAX_DEG %.4ff\n', delta_max_deg);
fprintf(fid_geom, '#define LQI_DELTA_MAX_RAD %.8ff\n\n', delta_max_rad);

fprintf(fid_geom, '/* 零空间优化权重 */\n');
fprintf(fid_geom, '#define LQI_LAMBDA_PITCH %.4ff\n', lambda_pitch);
fprintf(fid_geom, '#define LQI_LAMBDA_SERVO %.4ff\n\n', lambda_servo);

fprintf(fid_geom, '/* 全局增益旋钮（占位符补偿，默认 1.0）*/\n');
fprintf(fid_geom, '#define LQI_GAIN_SCALAR 1.0f\n\n');

fprintf(fid_geom, '#endif /* LQI_GEOMETRY_TABLE_H */\n');
fclose(fid_geom);
fprintf('✓ 已生成: %s\n', geom_h_path);

% ---- 打印导出摘要 ----
fprintf('\n--- 导出摘要 ---\n');
fprintf('速度点: %d 个 (%.1f ~ %.1f m/s)\n', N_speed_export, V_export(1), V_export(end));
fprintf('K_lqi 表: %d × %d × %d = %d 个 float\n', ...
    N_speed_export, 3, 9, N_speed_export*3*9);
fprintf('H_tau 表: %d × %d × %d = %d 个 float\n', ...
    N_speed_export, 3, 4, N_speed_export*3*4);
fprintf('零空间 N_ry: 4×2 = 8 个 float\n');
fprintf('\n已导出文件:\n');
fprintf('  %s\n', gain_h_path);
fprintf('  %s\n', geom_h_path);
fprintf('\n嵌入端使用方法:\n');
fprintf('  1. lqi_torque.c  #include "lqi_gain_table.h" + "lqi_geometry_table.h"\n');
fprintf('  2. 查 K 表: LQI_InterpolateK(velocity, K_out_3x9)\n');
fprintf('  3. torqu_cmd = -K * xa\n');
fprintf('  4. Torque_Allocate_PitchProtected(torque_cmd, H_tau, delta_out)\n');
fprintf('\n⚠ 提醒: 所有数值基于占位符气动参数。\n');
fprintf('         获取 SolidWorks 数据后重新运行本脚本。\n');
