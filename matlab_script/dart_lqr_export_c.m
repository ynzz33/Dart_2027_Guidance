function header_path = dart_lqr_export_c(result)
% 求 K 主程序通过数值检查后调用，直接覆盖固定的 3×6 力矩增益附件。
% 输出目录固定为本文件旁的 export/，不写 imcalib/ 或旧 LQI 参数表。

K = single(result.design.K_tau);
output_dir = fullfile(fileparts(mfilename('fullpath')),'export');
if ~isfolder(output_dir)
    mkdir(output_dir);
end
header_path = fullfile(output_dir,'dart_lqr_torque_gain_table.h');
fid = fopen(header_path,'w','n','UTF-8');
if fid < 0
    error('DartLQR:ExportOpen', '无法写入导出附件：%s', header_path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid,'/* Pure LQR torque gain: 6 states -> 3 torques, no integral.\n');
fprintf(fid,' * State: [e_roll,e_pitch,e_yaw,p,q,r], measured - target.\n');
fprintf(fid,' * Units: angle rad, rate rad/s, torque N*m.\n');
fprintf(fid,' * Torque rows: [roll,pitch,yaw]; tau = -K_tau*x.\n');
fprintf(fid,' * Pitch angle column is zero; pitch rate damping is retained.\n');
fprintf(fid,' * Existing inertia values/signs retained; physical axis mapping needs bench confirmation.\n');
fprintf(fid,' * Ideal torque model only: no servo polarity or effectiveness in K.\n');
fprintf(fid,' * Handoff attachment: existing 3x9 MCU LQI must be adapted before use.\n');
fprintf(fid,' * Q active states [e_roll,e_yaw,p,q,r]: %.9g %.9g %.9g %.9g %.9g\n',diag(result.cfg.Q));
fprintf(fid,' * R torque: %.9g %.9g %.9g\n',diag(result.cfg.R_tau));
fprintf(fid,' */\n\n');
fprintf(fid,'#ifndef DART_LQR_TORQUE_GAIN_TABLE_H\n#define DART_LQR_TORQUE_GAIN_TABLE_H\n\n');
fprintf(fid,'#define DART_LQR_STATE_DIM 6\n#define DART_LQR_TORQUE_DIM 3\n');
fprintf(fid,'#define DART_LQR_DESIGN_TS_S %.9ef\n\n',result.cfg.Ts);
fprintf(fid,'static const float dart_lqr_K_tau[DART_LQR_TORQUE_DIM][DART_LQR_STATE_DIM] = {\n');
for axis = 1:3
    fprintf(fid,'    {');
    fprintf(fid,'%+.9ef, ',K(axis,1:5));
    fprintf(fid,'%+.9ef}',K(axis,6));
    if axis < 3
        fprintf(fid,',\n');
    else
        fprintf(fid,'\n');
    end
end
fprintf(fid,'};\n\n#endif /* DART_LQR_TORQUE_GAIN_TABLE_H */\n');
delete(cleanup);
fprintf('已自动覆盖增益表：%s\n',header_path);
end
