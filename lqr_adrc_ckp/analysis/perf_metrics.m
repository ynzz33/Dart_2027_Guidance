function m = perf_metrics(t, y, t_step, y0, yf, tol_frac)
% =========================================================================
% PERF_METRICS  阶跃响应「性能指标」自动计算：超调 / 调节时间 / 上升时间 / 稳态误差
% -------------------------------------------------------------------------
% 【这个函数干什么】给它一段阶跃响应曲线（比如“滚转角从 0 阶跃到 10°”的时间历程），
%   它自动量出控制工程最关心的 4 个指标，用来判定是否达标（本项目要求超调<20%、ts<0.5s、无静差）。
%   （自己写而不用 stepinfo，是为了不依赖工具箱、且能精确控制阶跃起点/方向/容差。）
%
% 【4 个指标的通俗含义】
%   - 超调 overshoot(%)：冲过头的最大幅度，占阶跃幅度的百分比。越小越稳，0 表示没冲过头。
%   - 调节时间 settling(s)：从阶跃时刻起，多久之后曲线“永久”进入 ±tol 误差带不再出来。越小越快。
%   - 上升时间 rise(s)：从 10% 爬到 90% 所用时间。衡量“反应快慢”。
%   - 稳态误差 ess：最终稳住的值与目标值之差。理想为 0（无静差）。
%
% 【入参】
%   t(N)      时间向量
%   y(N)      响应信号
%   t_step    阶跃发生时刻 (s)
%   y0        阶跃前的稳态值
%   yf        阶跃目标终值
%   tol_frac  调节时间的误差带（相对阶跃幅度），默认 0.02 即 ±2%
% 【返回】结构体 m：.overshoot(%) .settling(s,相对t_step) .rise(s) .ess(与 y 同单位)
% =========================================================================
    if nargin < 6 || isempty(tol_frac); tol_frac = 0.02; end
    idx = t >= t_step;                           % 只看阶跃之后的部分
    tt = t(idx);  yy = y(idx);
    span = yf - y0;                              % 阶跃幅度（带符号，支持上升/下降阶跃）

    % --- 超调（按阶跃方向归一化，无超调记 0）---
    if span >= 0
        peak = max(yy);   m.overshoot = max(0, (peak - yf)/max(abs(span),eps))*100;
    else
        peak = min(yy);   m.overshoot = max(0, (yf - peak)/max(abs(span),eps))*100;
    end

    % --- 稳态误差（末 5% 样本均值 − 目标）---
    nlast = max(1, round(0.05*numel(yy)));
    m.ess = mean(yy(end-nlast+1:end)) - yf;

    % --- 调节时间（±tol 带，最后一次越带后回带时刻）---
    tol = tol_frac*abs(span);
    out = abs(yy - yf) > tol;
    if ~any(out)
        m.settling = 0;
    else
        k = find(out, 1, 'last');
        if k < numel(tt);  m.settling = tt(k+1) - t_step;
        else               m.settling = NaN;          % 仿真结束仍未进带
        end
    end

    % --- 上升时间（10%→90%）---
    if span ~= 0
        y10 = y0 + 0.1*span;  y90 = y0 + 0.9*span;
        if span > 0
            i10 = find(yy >= y10, 1, 'first');  i90 = find(yy >= y90, 1, 'first');
        else
            i10 = find(yy <= y10, 1, 'first');  i90 = find(yy <= y90, 1, 'first');
        end
        if ~isempty(i10) && ~isempty(i90) && i90 >= i10
            m.rise = tt(i90) - tt(i10);
        else
            m.rise = NaN;
        end
    else
        m.rise = NaN;
    end
end
