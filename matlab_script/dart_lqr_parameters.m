function varargout = dart_lqr_parameters()
% 日常调参入口：改完本文件直接运行，自动求 K、检查并覆盖代码内增益表。
% cfg=dart_lqr_parameters() 仅供程序读取配置，不触发求解或写表。
% 无积分 LQR：六态输入，输出 [roll,pitch,yaw] 三轴力矩 N·m。
% 惯量沿用原参数矩阵，未重新解释 CAD/ENU 轴映射，物理方向仍待台架。

varargout = {};
cfg = struct();

% 原 SolidWorks 数值及交叉项符号原样保留，单位 kg·m²。
cfg.I_body_kgm2 = [160428.06e-9, 20714.52e-9, -46.58e-9;
                  20714.52e-9, 951713.64e-9, 67.56e-9;
                  -46.58e-9, 67.56e-9, 957346.58e-9];

% 完整六态 [e_roll,e_pitch,e_yaw,p,q,r]，三轴都参与角度及角速度反馈。
% pitch 角度权重恢复原配置值，其余权重不变；不包含舵面或气动参数。
cfg.Q = diag([50.0, 0.01, 1000.0, 1.0, 0.50, 1.00]);
cfg.R_tau = diag([5.0, 5.0, 5.0]);
cfg.Ts = 0.001;

% 直接运行时执行完整流程；有返回值调用只读参数，避免求解器回读时递归。
if nargout == 0
    dart_attitude_lqr(cfg);
else
    varargout{1} = cfg;
end
end
