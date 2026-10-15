function cfg = dart_lqr_parameters()
% 无积分 LQR 配置：六态输入，输出 [roll,pitch,yaw] 三轴力矩 N·m。
% 惯量沿用原参数矩阵，未重新解释 CAD/ENU 轴映射，物理方向仍待台架。

cfg = struct();

% 原 SolidWorks 数值及交叉项符号原样保留，单位 kg·m²。
cfg.I_body_kgm2 = [160428.06e-9, 20714.52e-9, -46.58e-9;
                  20714.52e-9, 951713.64e-9, 67.56e-9;
                  -46.58e-9, 67.56e-9, 957346.58e-9];

% 有效状态 [e_roll,e_yaw,p,q,r]；pitch 不追角度，只保留速率阻尼。
% 权重沿用原配置对应值；力矩模型中不包含舵面或气动参数。
cfg.Q = diag([50.0, 1000.0, 1.0, 0.50, 1.00]);
cfg.R_tau = diag([5.0, 5.0, 5.0]);
cfg.Ts = 0.001;
end
