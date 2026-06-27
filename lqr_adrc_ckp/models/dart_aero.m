function [F_body, M_body, info] = dart_aero(x, delta, wind_ned, params)
% =========================================================================
% DART_AERO  机体系「气动力 + 气动力矩」—— 飞镖与空气相互作用的全部物理
% -------------------------------------------------------------------------
% 【这个函数干什么】给定状态、舵偏、风，算出空气作用在飞镖上的 3 个力 + 3 个力矩。
%   它是 dart_6dof 的“供货商”：6DOF 拿到这里给的力/力矩，才能算加速度。
%
% 【三个核心气动概念（看懂这三个，整段就懂了）】
%   ① 相对气流：飞镖感受到的风 = 自己的飞行速度 − 大气的风速。无风时就是飞行速度本身；
%      有侧风时，相对气流方向会偏，于是产生侧滑角 β、攻角 α。
%   ② 攻角 α / 侧滑 β：相对气流相对机头的“上下偏角/左右偏角”。
%      α = atan2(w_r, u_r)（气流偏上/下）；β = asin(v_r/V)（气流偏左/右）。
%      它们是产生升力/侧力、以及静稳定力矩的根源。
%   ③ 动压 q̄ = ½·ρ·V²：空气的“冲击压强”，正比于速度平方。所有气动力/力矩都 ∝ q̄。
%      → 速度从 60 掉到 20，动压变成 1/9，舵面效率也变 1/9（这就是为什么要“动压调度”）。
%
% 【力与力矩怎么算】都用工程上常见的“系数 × 动压 × 参考量”形式：
%     力   F = q̄·S·C_力        （S=参考面积）
%     力矩 M = q̄·S·d·C_力矩    （d=参考长度/直径）
%   其中各 C 系数 = 静稳定项 + 阻尼项 + 舵效项 的线性叠加（小扰动近似）。
%
% 【入参】
%   x        - 12×1 状态 [pN pE pD; u v w; φ θ ψ; p q r]
%   delta    - 4×1 物理舵面偏角 (rad)
%   wind_ned - 3×1 NED 风速 (m/s)
%   params   - 参数结构体（气动系数、几何、空气密度等）
% 【返回】
%   F_body - 3×1 气动力（机体系, N，**不含重力**，重力在 dart_6dof 里加）
%   M_body - 3×1 气动力矩（机体系, N·m）
%   info   - 诊断结构体: V(空速) / alpha / beta / qbar / u_ctrl(等效控制量)
% =========================================================================

    % ---- 拆状态 ----
    uvw = x(4:6);
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);            % 三轴角速度（用于阻尼项）

    rho = params.env.rho;                       % 空气密度
    S = params.body.S; d = params.body.d;       % 参考面积 / 参考长度

    % ---- ① 相对气流（机体系）= 机体速度 − 风（风要先从 NED 转到机体系）----
    R_n2b = dcm_ned2body(phi, theta, psi);
    Vb = uvw - R_n2b * wind_ned;                % 飞镖真正“迎面吃到”的气流
    ur = Vb(1); vr = Vb(2); wr = Vb(3);
    V = sqrt(ur^2 + vr^2 + wr^2);               % 空速（相对气流速率）
    Vsafe = max(V, 1.0);                        % 防止低速/静止时除以 0

    % ---- ② 攻角 α、侧滑 β、③ 动压 q̄ ----
    alpha = atan2(wr, ur);                      % 攻角：气流相对机头偏上(+)/偏下(-)
    beta  = asin( min(max(vr/Vsafe, -1), 1) );  % 侧滑：气流相对机头偏右(+)/偏左(-)，clamp 防 asin 越界
    qbar  = 0.5 * rho * V^2;                    % 动压

    % ---- X 构型混控：4 个物理舵偏 → 等效的 [滚转;俯仰;偏航] 控制量 ----
    %   E 是几何混控矩阵；u_ctrl(1/2/3) 分别是“等效副翼/升降/方向舵”偏角。
    u_ctrl = params.alloc.E * delta;            % 3×1

    % ---- 气动力（机体系，小扰动线性近似）----
    Fx = -qbar * S * params.aero.Cd0;           % 轴向阻力：永远顶着机头方向往后拖（-X）
    Fy =  qbar * S * params.aero.CYb * beta;    % 侧力：由侧滑 β 产生（CYb<0）
    Fz = -qbar * S * params.aero.CLa * alpha;   % 升力：由攻角 α 产生（α>0 → 力朝 -Z 即向上）
    F_body = [Fx; Fy; Fz];

    % ---- 气动力矩系数（每个轴 = 静稳定 + 阻尼 + 舵效）----
    % dd = d/(2V) 是把“角速度”无量纲化的标准因子（阻尼项要用）。
    dd = d / (2*Vsafe);
    % 滚转 Cl：滚转阻尼 Clp·(p·dd) + 副翼舵效 Clda·等效副翼
    Cl = params.aero.Clp*(p*dd) + params.aero.Clda*u_ctrl(1);
    % 俯仰 Cm：静稳定 Cma·α（α↑→低头，Cma<0=稳定）+ 俯仰阻尼 Cmq + 升降舵效 Cmde
    Cm = params.aero.Cma*alpha  + params.aero.Cmq*(q*dd) + params.aero.Cmde*u_ctrl(2);
    % 偏航 Cn：方向稳定 Cnb·β（Cnb>0=风向标稳定）+ 偏航阻尼 Cnr + 方向舵效 Cndr（很小）
    Cn = params.aero.Cnb*beta   + params.aero.Cnr*(r*dd) + params.aero.Cndr*u_ctrl(3);
    M_body = qbar*S*d * [Cl; Cm; Cn];           % 系数 × q̄·S·d = 实际力矩 (N·m)

    % ---- 诊断输出（画图/调试用，不影响动力学）----
    info.V = V; info.alpha = alpha; info.beta = beta;
    info.qbar = qbar; info.u_ctrl = u_ctrl;
end
