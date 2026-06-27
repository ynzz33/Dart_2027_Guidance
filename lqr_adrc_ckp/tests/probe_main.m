% PROBE_MAIN  诊断 main 舵饱和来源 —— 扫描风强度 + 定位是哪个轴/哪个舵在饱和
clear; clc;
projroot = fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(projroot));
params = dart_params(); Ts=params.sim.Ts; T=4; N=round(T/Ts); tvec=(0:N-1)'*Ts;
K=lqr_outer_design(params); rho=params.env.rho; S=params.body.S; d=params.body.d;
E=params.alloc.E; Ceff=[params.aero.Clda;params.aero.Cmde;params.aero.Cndr];
omax=params.lqr.omega_max; a_lpf=exp(-2*pi*params.sensor.lpf_fc*Ts);
clamp01=@(x)max(0,min(1,x)); ss=@(x)3*clamp01(x).^2-2*clamp01(x).^3;
gn=params.sensor.gyro_noise;
phiCmd=@(t)deg2rad(8)*(ss((t-1)/0.5)-ss((t-3)/0.5));

% [紊流σ, 侧风m/s, 陀螺噪声]
configs=[0 0 0; 0 2.5 0; 1.0 0 0; 0.5 1.5 0.01; 0.3 1.0 0.01];
fprintf('  σ   side noise | nsat  后半  maxd | 力矩RMS(L,M,N) | z2末值(L,M,N) | maxδ各舵\n');
for c=1:size(configs,1)
  sig=configs(c,1); side=configs(c,2); nz=configs(c,3);
  p=params; p.wind.sigma=sig; wg=dryden_wind(tvec,50,p,7)+[0,side,0];
  x=[0;0;0;60;0;0;deg2rad([1.5;-1;1]);0;0;0];
  st_l=[];st_s=[];wf=zeros(3,1);tau_act=zeros(3,1);phi_prev=phiCmd(-Ts);
  nsat=0;nsat2=0;maxd=zeros(1,4);sumtau=zeros(1,3);z2end=zeros(1,3); rng(0);
  for k=1:N
    t=tvec(k); att=x(7:9); pqr=x(10:12);
    wm=pqr+nz*randn(3,1); wf=a_lpf*wf+(1-a_lpf)*wm;
    Rb2n=dcm_ned2body(att(1),att(2),att(3))'; Vned=Rb2n*x(4:6);
    gamma=atan2(-Vned(3),hypot(Vned(1),Vned(2)));
    ref=[phiCmd(t);gamma;0];
    off=[(phiCmd(t)-phi_prev)/Ts;0;0]; phi_prev=phiCmd(t);
    oc=K*(ref-att)+off; oc=max(min(oc,omax),-omax);
    [tau,st_l]=ladrc_inner(oc,wf,st_l,params,tau_act);
    V=norm(x(4:6)); qb=0.5*rho*V^2;
    [dc,ai]=control_alloc(tau,qb,params);
    [delta,st_s]=actuator_2nd(dc,st_s,params);
    Beff=diag(qb*S*d*Ceff)*E; tau_act=Beff*delta;
    nsat=nsat+ai.sat; if t>=2, nsat2=nsat2+ai.sat; end
    maxd=max(maxd,abs(rad2deg(delta))'); sumtau=sumtau+tau'.^2;
    x=rk4_step(x,delta,wg(k,:)',params,Ts);
  end
  z2end=st_l.z2';
  fprintf('%4.1f %4.1f %5.2f | %4d %4d %5.1f | %.3f %.3f %.3f | %+.3f %+.3f %+.3f | %.1f %.1f %.1f %.1f\n', ...
    sig,side,nz,nsat,nsat2,max(maxd), sqrt(sumtau/N), z2end, maxd);
end
