function log = dart_lqr_log_check(source,sample_period_s,crop_s)
% 读取当前固件 Vofa 24 通道，不辨识气动参数、不补造速度或指令力矩。
% source 为数字矩阵或文件路径：24 列必须显式给日志采样周期；25 列首列是秒时间戳。
% crop_s 为相对首个有效样本的 [起点,终点] 秒；不传则不裁剪。
% 返回结构体时不绘图；无返回值调用时显示曲线。
% 示例：log=dart_lqr_log_check('vofa24.csv',0.01,[0.5,1.5]);
% 带时间列：log=dart_lqr_log_check('vofa25.csv',[],[0.5,1.5]);

log = struct();
removed_by_crop = 0;

if nargin < 2
    sample_period_s = [];
end
if nargin < 3
    crop_s = [];
end
if isnumeric(source)
    raw = source;
else
    raw = readmatrix(source);
end
if ~isnumeric(raw) || ~isreal(raw) || ~ismatrix(raw) || ~ismember(size(raw,2),[24,25])
    error('DartLQRLog:Columns','输入必须是当前 24 通道，或首列为秒时间戳的 25 列数字表。');
end
if size(raw,2) == 25
    if ~isempty(sample_period_s)
        error('DartLQRLog:TimeColumn','已有秒时间戳时采样周期参数应传 []，不覆盖实际时间。');
    end
    time_s = raw(:,1);
    channels = raw(:,2:25);
else
    if ~isnumeric(sample_period_s) || ~isreal(sample_period_s) || ~isscalar(sample_period_s) || ~isfinite(sample_period_s) || sample_period_s <= 0
        error('DartLQRLog:SamplePeriod','无时间列时必须显式给出日志采样周期（秒），不能用控制周期代替。');
    end
    time_s = (0:size(raw,1)-1).' * sample_period_s;
    channels = raw;
end

% 所有列用同一个掩码处理；保留源行号供追查，不对反馈加滤波。
valid = isfinite(time_s) & all(isfinite(channels),2);
removed_nonfinite = sum(~valid);
source_rows = find(valid);
time_s = time_s(valid);
channels = channels(valid,:);
if numel(time_s) < 2
    error('DartLQRLog:TooFewSamples','至少需要两个有效样本。');
end
if any(diff(time_s) <= 0)
    error('DartLQRLog:TimeOrder','时间戳必须严格递增；重复或倒序数据不能用于时间分析。');
end
% 裁剪边界允许原时间戳一个 ULP 的舍入量，避免扣除大起点时丢掉端点。
time_tolerance_s = eps(max(abs(time_s)));
time_s = time_s - time_s(1);
if ~isempty(crop_s)
    if ~isnumeric(crop_s) || ~isreal(crop_s) || numel(crop_s) ~= 2 || any(~isfinite(crop_s(:))) || crop_s(1) > crop_s(2)
        error('DartLQRLog:Crop','裁剪范围必须是有限的 [起点,终点] 秒，且起点不大于终点。');
    end
    selected = time_s >= crop_s(1) - time_tolerance_s & time_s <= crop_s(2) + time_tolerance_s;
    removed_by_crop = sum(~selected);
    time_s = time_s(selected);
    channels = channels(selected,:);
    source_rows = source_rows(selected);
end
if numel(time_s) < 2
    error('DartLQRLog:TooFewSamples','裁剪后至少需要两个有效样本。');
end

% TotalControl.c 当前发送顺序：前三路为 pitch/roll/yaw，陀螺在 16..18。
log.time_s = time_s;
log.source_rows = source_rows;
log.channels = channels;
log.angle_deg = channels(:,[2,1,3]).';
log.error_rad = channels(:,6:8).';
log.rate_rad_s = deg2rad(channels(:,[17,16,18]).');
log.x = [log.error_rad; log.rate_rad_s];
log.torque_achieved_Nm = channels(:,9:11).';
log.servo_deg = channels(:,12:15).';
log.packed_channels = channels(:,22:24).';
log.quality.removed_nonfinite = removed_nonfinite;
log.quality.removed_by_crop = removed_by_crop;
log.quality.median_sample_period_s = median(diff(time_s));

% 22..24 为复合打包值，不猜测距离范围、不据此自动判定飞行阶段。
fprintf('日志有效样本 %d；移除非有限行 %d，裁剪行 %d。\n',numel(time_s),removed_nonfinite,removed_by_crop);
fprintf('力矩为当前固件分配后估计值；当前布局未发送速度和指令力矩。\n');
if nargout == 0
    fig = figure('Name','Dart Vofa 24 Channels','NumberTitle','off','Color','w','Position',[100,80,1000,800]);
    tiledlayout(fig,4,1);
    nexttile;
    plot(time_s,log.angle_deg.','LineWidth',1.1);
    ylabel('姿态 (deg)');
    legend('roll','pitch','yaw','Location','best');
    grid on;
    nexttile;
    plot(time_s,rad2deg(log.rate_rad_s).','LineWidth',1.1);
    ylabel('角速度 (deg/s)');
    legend('roll','pitch','yaw','Location','best');
    grid on;
    nexttile;
    plot(time_s,log.torque_achieved_Nm.','LineWidth',1.1);
    ylabel('分配后力矩 (N·m)');
    legend('roll','pitch','yaw','Location','best');
    grid on;
    nexttile;
    plot(time_s,log.servo_deg.','LineWidth',1.1);
    ylabel('舵角 (deg)');
    xlabel('相对日志起点时间 (s)');
    legend('UL','UR','DR','DL','Location','best');
    grid on;
end
end
