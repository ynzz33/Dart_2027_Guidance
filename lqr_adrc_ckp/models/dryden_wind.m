function wind = dryden_wind(t, V, params, seed)
% =========================================================================
% DRYDEN_WIND  Dryden 紊流风速序列（NED，一阶近似）
% -------------------------------------------------------------------------
% 【什么是 Dryden 紊流】MIL-F-8785C 规范用一组成形滤波器把“白噪声”整形成有空间相关性的
%   随机风速，模拟真实大气湍流（不是纯随机抖动，而是有一定持续性/相关时间的起伏）。
% 【本实现】用一阶 Markov（指数相关）近似 Dryden：相关时间 τ = L/V，
%     w[k] = a·w[k-1] + b·randn，其中 a=exp(-Ts/τ)，b=σ·√(1-a²)
%   生成的序列 RMS≈σ、相关时间≈τ。纵向(u)分量这是 Dryden 的精确形式；
%   横/垂(v,w)分量 Dryden 原为二阶，这里用一阶近似，对姿态控制鲁棒性验证足够。
%   （需要更高保真时可换成完整二阶传递函数，接口不变。）
%
% 【入参】t(N) 时间向量；V 飞行速度(m/s)；params(用 params.wind.L / .sigma)；seed 可选随机种子
% 【返回】wind(N×3) NED 三分量湍流风速 (m/s)；常值侧风请在外部叠加 params.wind.steady
% =========================================================================
    if nargin >= 4 && ~isempty(seed); rng(seed); end
    N  = numel(t);  Ts = t(2)-t(1);
    L  = params.wind.L(:);                 % [Lu;Lv;Lw] 尺度长度
    sig = params.wind.sigma(:);            % 强度（标量或 3x1）
    if isscalar(sig); sig = sig*ones(3,1); end

    wind = zeros(N,3);
    for i = 1:3                             % 三个方向各自成形
        tau = L(i)/max(V,1);                % 相关时间（V 越大、湍流“扫过”越快）
        a   = exp(-Ts/tau);
        b   = sig(i)*sqrt(1-a^2);
        wi  = 0;
        for k = 1:N
            wi = a*wi + b*randn;            % 一阶递推：有色噪声
            wind(k,i) = wi;
        end
    end
end
