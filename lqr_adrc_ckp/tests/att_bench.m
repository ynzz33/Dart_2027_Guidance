function rec = att_bench(params, Q, ref_fun, T, omega_max, opts)
% =========================================================================
% ATT_BENCH  姿态控制台闭环仿真（M3/M4 共用的“试验台”）
% -------------------------------------------------------------------------
% 【它模拟什么】把完整控制链 + 飞镖转动动力学跑起来，速度固定（动压恒定），
%   只演化“转动 + 姿态角”，不算掉高减速 —— 这样能干净地测姿态环性能。
%   完整每步（1ms）流程：
%     陀螺测角速度(+噪声+低通) → 外环LQR(姿态差→期望角速度,带限幅)
%       → 内环LADRC(期望角速度→期望力矩,带抗饱和) → X构型分配(力矩→4舵偏)
%       → 二阶舵机(限幅/限速) → 6DOF转动方程(力矩→角加速度→角速度→姿态角)
%
% 【入参】
%   params    参数结构体（dart_params 生成）
%   Q         外环 LQR 状态权重 (3x3)；R 取 params.lqr.R
%   ref_fun   参考指令函数句柄：ref = ref_fun(t)，返回 [φ;θ;ψ]_目标 (rad)
%   T         仿真时长 (s)
%   omega_max 角速度指令限幅 (3x1, rad/s)；传 [] 或省略=不限幅
%   opts      可选设置结构体：
%               opts.V        飞行速度 (m/s)，默认 50
%               opts.wind_fun NED 风速函数句柄 wind=wind_fun(t)，默认无风
% 【返回】rec：时间序列 t / att(姿态角) / ref / pqr(角速度) / cmd(角速度指令)
%                       / tau(LADRC力矩) / delta(4舵偏) / wind(NED风速)
% =========================================================================
    if nargin < 5; omega_max = []; end
    if nargin < 6 || isempty(opts); opts = struct; end
    if ~isfield(opts,'V');        opts.V = 50;                 end  % 飞行速度 (m/s)
    if ~isfield(opts,'wind_fun'); opts.wind_fun = @(t)[0;0;0]; end  % NED 风速
    if ~isfield(opts,'dist_fun'); opts.dist_fun = @(t)[0;0;0]; end  % 外部扰动力矩 (N·m)

    Ts = params.sim.Ts;  N = round(T/Ts);
    V  = opts.V;  qbar = 0.5*params.env.rho*V^2;  I = params.body.I;
    S  = params.body.S;  d = params.body.d;
    K  = lqr_outer_design(params, Q);             % 外环增益（R 用 params 默认）

    % 控制力矩效率矩阵（随动压变化）：tau_ctrl = B_eff*delta，用于抗饱和反馈。
    % 注意：低速时 qbar 小 → B_eff 小 → 同样力矩需要更大舵偏（分配会自动放大），
    %       这正是“动压调度”的体现；只要不饱和，力矩就能精确实现、性能几乎不随速度变。
    B_eff = diag(qbar*S*d*[params.aero.Clda; params.aero.Cmde; params.aero.Cndr]) * params.alloc.E;

    att = zeros(3,1);  pqr = zeros(3,1);  st_l = [];  st_s = [];
    a_lpf = exp(-2*pi*params.sensor.lpf_fc*Ts);   % 陀螺一阶低通系数
    wf = zeros(3,1);  tau_act = zeros(3,1);        % 滤波后角速度 / 上一步实际力矩

    rec.t=(0:N-1)'*Ts; rec.att=zeros(N,3); rec.ref=zeros(N,3); rec.pqr=zeros(N,3);
    rec.cmd=zeros(N,3); rec.tau=zeros(N,3); rec.delta=zeros(N,4); rec.wind=zeros(N,3);

    for k = 1:N
        t   = (k-1)*Ts;
        ref = ref_fun(t);
        wind = opts.wind_fun(t);

        % --- 传感器：陀螺噪声 + 一阶低通（姿态角理想可测）---
        wm = pqr + params.sensor.gyro_noise*randn(3,1);
        wf = a_lpf*wf + (1-a_lpf)*wm;

        % --- 外环 LQR：姿态角误差 → 期望角速度（限幅防大误差过激）---
        omega_cmd = K*(ref - att);
        if ~isempty(omega_max)
            omega_cmd = max(min(omega_cmd, omega_max), -omega_max);
        end

        % --- 内环 LADRC（传上一步实际力矩做抗饱和）→ 分配 → 舵机 ---
        [tau, st_l] = ladrc_inner(omega_cmd, wf, st_l, params, tau_act);
        dcmd = control_alloc(tau, qbar, params);
        [delta, st_s] = actuator_2nd(dcmd, st_s, params);
        tau_act = B_eff * delta;                   % 实际控制力矩（经分配/饱和/舵机）

        % --- 气动力矩（伪状态：体轴速度 [V;0;0]，叠加 NED 风扰）---
        % 有风时相对气流方向改变 → 攻角α/侧滑β 变化 → 额外气动扰动力矩，由控制器抵抗。
        xps = [0;0;0; V;0;0; att; pqr];
        [~, M] = dart_aero(xps, delta, wind, params);
        M = M + opts.dist_fun(t);                  % 叠加外部扰动力矩（模拟常值风净效应/质量不对称/未建模动态）

        % --- 6DOF 转动 + 姿态运动学（前向欧拉 @1000Hz）---
        dpqr = I \ (M - cross(pqr, I*pqr));        % ω̇ = I⁻¹(M − ω×Iω)，含陀螺耦合
        pqr  = pqr + Ts*dpqr;
        cth = cos(att(2)); cth = sign(cth+(cth==0))*max(abs(cth),1e-4); tth = sin(att(2))/cth;
        sp = sin(att(1)); cp = cos(att(1));
        H = [1 sp*tth cp*tth; 0 cp -sp; 0 sp/cth cp/cth];   % 欧拉角速率变换矩阵
        att = att + Ts*(H*pqr);

        rec.att(k,:)=att'; rec.ref(k,:)=ref'; rec.pqr(k,:)=pqr';
        rec.cmd(k,:)=omega_cmd'; rec.tau(k,:)=tau'; rec.delta(k,:)=delta'; rec.wind(k,:)=wind';
    end
end
