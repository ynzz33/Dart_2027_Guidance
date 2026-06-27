function plot_results(R, params, projroot)
% =========================================================================
% PLOT_RESULTS  把 main_dart_sim 跑出来的时间序列画成一张“综合体检报告”大图
% -------------------------------------------------------------------------
% 【入参】
%   R        main_dart_sim 记录的结构体（含 t/pos/V/att/ref/pqr/cmd/tau/delta/
%            wind/z2/alpha/beta/qbar/sat 等时间序列）
%   params   参数结构体（取舵机限幅画参考线）
%   projroot 项目根目录（图存到 <projroot>/analysis/figs/）
% 【输出】9 宫格 PNG：MAIN_overview.png
%   每个子图都对应信号链的一个环节，连起来看就能讲清楚“系统是怎么工作的”。
% =========================================================================
    if nargin < 3 || isempty(projroot); projroot = pwd; end
    t = R.t;
    fig = figure('Position',[30 30 1280 860],'Visible','off');

    % (1) 空速衰减：无动力飞镖靠阻力减速，动压随之下降——动压调度据此放大舵偏
    subplot(3,3,1);
      plot(t, R.V,'LineWidth',1.6); grid on;
      xlabel('时间 (s)'); ylabel('空速 V (m/s)');
      title(sprintf('① 空速自然衰减 %.0f→%.0f m/s', R.V(1), R.V(end)));

    % (2) 姿态角跟踪：实线=实际，虚线=参考目标。看跟踪精度 + 偏航增稳
    subplot(3,3,2);
      plot(t, rad2deg(R.att(:,1)),'b','LineWidth',1.5); hold on;
      plot(t, rad2deg(R.att(:,2)),'m','LineWidth',1.5);
      plot(t, rad2deg(R.att(:,3)),'g','LineWidth',1.5);
      plot(t, rad2deg(R.ref(:,1)),'b--'); plot(t, rad2deg(R.ref(:,2)),'m--');
      grid on; xlabel('时间 (s)'); ylabel('姿态角 (deg)');
      title('② 姿态跟踪(实=实际,虚=参考)');
      legend('\phi','\theta','\psi','\phi_{ref}','\theta_{ref}','Location','best');

    % (3) 角速度：内环 LADRC 的被控量。实线=实测，虚线=外环给的指令
    subplot(3,3,3);
      plot(t, rad2deg(R.pqr(:,1)),'b','LineWidth',1.3); hold on;
      plot(t, rad2deg(R.pqr(:,2)),'m','LineWidth',1.3);
      plot(t, rad2deg(R.pqr(:,3)),'g','LineWidth',1.3);
      plot(t, rad2deg(R.cmd(:,1)),'b--'); plot(t, rad2deg(R.cmd(:,2)),'m--');
      grid on; xlabel('时间 (s)'); ylabel('角速度 (deg/s)');
      title('③ 角速度(实=实测,虚=指令)');
      legend('p','q','r','p_c','q_c','Location','best');

    % (4) LADRC 期望力矩：内环输出，待分配到舵面
    subplot(3,3,4);
      plot(t, R.tau,'LineWidth',1.3); grid on;
      xlabel('时间 (s)'); ylabel('力矩 (N·m)');
      title('④ LADRC 期望力矩');
      legend('L_c(滚转)','M_c(俯仰)','N_c(偏航)','Location','best');

    % (5) 4 舵面偏角 + 限幅线：看是否饱和
    subplot(3,3,5);
      plot(t, rad2deg(R.delta),'LineWidth',1.0); grid on; hold on;
      yline( rad2deg(params.servo.pos_max),'r--');
      yline(-rad2deg(params.servo.pos_max),'r--');
      xlabel('时间 (s)'); ylabel('舵偏 (deg)');
      title(sprintf('⑤ 4 舵面偏角(红线=±%.0f°)', rad2deg(params.servo.pos_max)));
      legend('\delta_1','\delta_2','\delta_3','\delta_4','Location','best');

    % (6) LESO 估计的“总扰动”z2：LADRC 的核心——把风扰/耦合实时估出来对消
    subplot(3,3,6);
      plot(t, R.z2,'LineWidth',1.3); grid on;
      xlabel('时间 (s)'); ylabel('z_2 (扰动估计)');
      title('⑥ LESO 总扰动估计 z_2');
      legend('滚转','俯仰','偏航','Location','best');

    % (7) 风速(NED)：Dryden 紊流的随机起伏 + 常值东向侧风
    subplot(3,3,7);
      plot(t, R.wind,'LineWidth',1.0); grid on;
      xlabel('时间 (s)'); ylabel('风速 (m/s)');
      title('⑦ 风扰(Dryden紊流+常值侧风)');
      legend('北','东','地','Location','best');

    % (8) 攻角 α / 侧滑 β：风把它们顶起来，气动静稳定 + 控制把它们压回
    subplot(3,3,8);
      plot(t, rad2deg(R.alpha),'LineWidth',1.3); hold on;
      plot(t, rad2deg(R.beta),'LineWidth',1.3); grid on;
      xlabel('时间 (s)'); ylabel('角度 (deg)');
      title('⑧ 攻角 \alpha / 侧滑 \beta');
      legend('\alpha','\beta','Location','best');

    % (9) 3D 弹道：高度=-pD，看整体飞行轨迹（受重力下坠 + 侧风偏移）
    subplot(3,3,9);
      plot3(R.pos(:,1), R.pos(:,2), -R.pos(:,3),'LineWidth',1.6); grid on;
      xlabel('北 p_N (m)'); ylabel('东 p_E (m)'); zlabel('高度 -p_D (m)');
      title('⑨ 3D 弹道'); view(45,20);

    sgtitle('无动力 X 构型飞镖 6DOF 自由飞行：LQR前馈 + LADRC + X分配 + 二阶舵机 综合报告');
    outdir = fullfile(projroot,'analysis','figs');
    if ~exist(outdir,'dir'); mkdir(outdir); end
    saveas(fig, fullfile(outdir,'MAIN_overview.png'));
end
