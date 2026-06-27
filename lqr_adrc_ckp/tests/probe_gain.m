% PROBE_GAIN  内环带宽 wc × 外环增益 K 联合扫描（隔离单通道阶跃 @V=50）
%   闭环近似二阶：ωn=√(wc·K), ζ=0.5·√(wc/K)；初始角加速度需求 ≈ wc·K·θ_step。
%   目的：找到使 滚转/俯仰阶跃 OS<20% 且 ts<0.5s 的 (wc,K) 工作点。
%   小角度阶跃（无动力飞镖为小角度姿态修正/增稳，非大机动）。
clear; clc;
if ~exist('projroot','var'); projroot=fileparts(fileparts(mfilename('fullpath'))); end
addpath(genpath(projroot));
params = dart_params();
% params.aero.Clda = 0.20;  params.aero.Cmde = -2.0;   % 增强舵效对比（先测“当前提纲舵效”能否达标）

rollDeg = 10;  pitchDeg = 5;
omax = deg2rad([200;120;120]);
wcs = [12 16 20 25 30];
Ks  = [6 8 10 12 14];

fprintf('=== wc x K sweep | aero Clda=%.2f Cmde=%.2f | roll=%ddeg pitch=%ddeg ===\n',...
        params.aero.Clda, params.aero.Cmde, rollDeg, pitchDeg);
fprintf('wc   K  | ROLL  OS%%    ts   ess  md | PITCH OS%%    ts   ess  md\n');
for wc = wcs
  p = params;  p.adrc.wc = wc;
  for Kv = Ks
    Q = diag([Kv^2, Kv^2, Kv^2*0.1]);
    rng(0); rr = att_bench(p, Q, @(t)[deg2rad(rollDeg)*(t>=0.2);0;0], 1.5, omax);
    mr = perf_metrics(rr.t, rad2deg(rr.att(:,1)), 0.2, 0, rollDeg);
    rng(0); rp = att_bench(p, Q, @(t)[0;deg2rad(pitchDeg)*(t>=0.2);0], 1.5, omax);
    mp = perf_metrics(rp.t, rad2deg(rp.att(:,2)), 0.2, 0, pitchDeg);
    flag = '';
    if mr.overshoot<20 && mr.settling<0.5 && mp.overshoot<20 && mp.settling<0.5; flag=' <== OK'; end
    fprintf('%2d %4.1f | %5.1f %5.3f %5.2f %4.1f | %5.1f %5.3f %5.2f %4.1f%s\n',...
      wc, Kv, mr.overshoot, mr.settling, mr.ess, max(abs(rad2deg(rr.delta(:)))),...
              mp.overshoot, mp.settling, mp.ess, max(abs(rad2deg(rp.delta(:)))), flag);
  end
end
