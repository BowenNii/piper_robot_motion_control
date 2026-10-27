%% CSV Action离线执行分析：原始记录只读；不连接ROS、不发送指令
% 直接运行此脚本。run_name可切换后续已归档的实验；不读取易丢失的/tmp文件。
% actual是模拟系统反馈；这些图不能用于宣称真机定位精度。
clear; clc;
script_dir=fileparts(mfilename('fullpath'));
archive_root=fileparts(fileparts(fileparts(fileparts(script_dir))));
repo_dir=fileparts(fileparts(archive_root));
addpath(fullfile(repo_dir,'MATLAB')); setup_piper_paths();
experiment_dir=fullfile(archive_root,'experiments','project1','2026-10-24_offline_csv_action');
run_name='run01';
run_dir=fullfile(experiment_dir,run_name);
reference_csv=fullfile(repo_dir,'experiments','project1','2026-10-18_ik_trajectory', ...
    'data_csv','trajectory.csv');
figure_dir=fullfile(run_dir,'figures'); analysis_dir=fullfile(run_dir,'analysis');
cfg.show_figures=usejava('desktop'); cfg.font='Noto Sans CJK SC';
[r,joint_stats,timing_stats,execution_stats]=analyze_csv_action_run(run_dir,reference_csv);
% 只覆盖本脚本生成的analysis文件与PNG，不覆盖run01中的三个原始文件。
save_analysis_results(analysis_dir, ...
    struct('joint_stats',joint_stats,'timing_stats',timing_stats,'execution_stats',execution_stats), ...
    struct('r',r,'cfg',cfg,'joint_stats',joint_stats,'timing_stats',timing_stats, ...
    'execution_stats',execution_stats,'matlab_version',version),struct('overwrite',true));

%% 图1：目标关节角与模拟反馈角；两条线几乎重合时结合图2看误差
f=new_plot('离线Action六轴角度对比',cfg);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for j=1:6
    nexttile;
    plot(r.action_time,r.q_desired(:,j)*180/pi,'b-','LineWidth',1.5); hold on;
    plot(r.action_time,r.q_actual(:,j)*180/pi,'--','Color',[.9 .35 .05],'LineWidth',1.2);
    grid on; xlim([0,r.meta.duration_s]);
    xlabel('目标轨迹时间 / s'); ylabel('关节角 / °'); title(sprintf('J%d',j));
    if j==1, legend('控制器目标','模拟反馈','Location','best'); end
end
sgtitle('图1  六轴目标与模拟反馈（不是实机测量）');
save_plot(f,figure_dir,'matlab_01_joint_tracking.png',cfg);

%% 图2：按同一Action反馈计算desired-actual，不拿不同接收时刻的两种日志相减
f=new_plot('离线Action六轴误差',cfg);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for j=1:6
    nexttile;
    plot(r.action_time,r.error(:,j),'LineWidth',1.2); hold on; yline(0,'k:');
    [peak,index]=max(abs(r.error(:,j)));
    plot(r.action_time(index),r.error(index,j),'ro','MarkerFaceColor','r');
    low=min(r.error(:,j)); high=max(r.error(:,j));
    margin=max(0.10*(high-low),1e-12); ylim([low-margin,high+margin]);
    title(sprintf('J%d  最大|误差| = %.3g rad',j,peak));
    grid on; xlim([0,r.meta.duration_s]); xlabel('目标轨迹时间 / s'); ylabel('目标 - 模拟反馈 / rad');
end
sgtitle('图2  模拟控制器关节误差（未设置真机合格阈值）');
save_plot(f,figure_dir,'matlab_02_joint_error.png',cfg);

%% 图3：记录时间质量；Action反馈约20Hz不代表控制器只以20Hz执行
f=new_plot('离线Action记录时间间隔',cfg);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(r.interval_time_est,r.dt_receive_ms,'LineWidth',1.1); hold on;
plot(r.interval_time_est,r.dt_header_ms,'--','LineWidth',1.0);
yline(r.gap_screen_ms,'r:','20ms记录筛查线（非安全限值）');
xline(0,'k--','估计开始'); xline(r.meta.duration_s,'k--','估计结束');
xlim([min(r.state_time_est)-0.05,max(r.state_time_est)+0.05]);
grid on; ylabel('间隔 / ms'); xlabel('相对估计执行起点的接收时间 / s');
title('全部状态记录：包含启动前和结束后；执行边界仅为估计');
legend('接收时间差','消息时间戳差','Location','best');
nexttile;
mask=r.execution_intervals;
plot(r.interval_time_est(mask),r.dt_receive_ms(mask),'LineWidth',1.1); hold on;
plot(r.interval_time_est(mask),r.dt_header_ms(mask),'--','LineWidth',1.0);
yline(5,'k:','名义5ms（200Hz）'); xlim([0,r.meta.duration_s]);
grid on; ylabel('间隔 / ms'); xlabel('相对估计执行起点的接收时间 / s');
title(sprintf('仅执行窗口：最大接收间隔 %.3f ms',max(r.dt_receive_ms(mask))));
legend('接收时间差','消息时间戳差','Location','best');
nexttile;
plot(r.action_time(2:end),r.action_receive_dt_ms,'LineWidth',1.1); hold on;
plot(r.action_time(2:end),r.action_desired_dt_ms,'--','LineWidth',1.0);
yline(50,'k:','50ms（约20Hz反馈）'); xlim([0,r.meta.duration_s]);
grid on; ylabel('间隔 / ms'); xlabel('目标轨迹时间 / s');
title('Action反馈间隔；不是控制器更新周期，也不是原CSV采样间隔');
legend('接收时间差','目标时间差','Location','best');
sgtitle('图3  接收时间与消息时间分开统计（不是网络单向延迟）');
save_plot(f,figure_dir,'matlab_03_record_timing.png',cfg);

%% 终端汇总；最后一次Action反馈未必正好位于10秒，终点另用状态日志检查
disp(joint_stats); disp(timing_stats); disp(execution_stats);
fprintf('全程已记录Action样本的最大关节偏差：%.9g rad\n',max(abs(r.error),[],'all'));
fprintf('最后状态与CSV终点的最大偏差：%.9g rad\n',max(abs(r.endpoint_error)));
fprintf('Action反馈目标时间覆盖：%.6f～%.6f s（不外推缺失端点）\n', ...
    r.action_time(1),r.action_time(end));
fprintf('结果目录：%s\n',run_dir);

function f=new_plot(name,cfg)
visible='off'; if cfg.show_figures, visible='on'; end
f=figure('Name',name,'Color','w','Visible',visible,'Position',[80,60,1200,900], ...
    'Renderer','painters'); % 二维曲线不依赖硬件OpenGL，避免批处理显卡兼容问题。
set(f,'DefaultAxesFontName',cfg.font,'DefaultTextFontName',cfg.font);
set(f,'DefaultAxesFontSize',11,'DefaultTextInterpreter','none','DefaultLegendInterpreter','none');
end

function save_plot(f,folder,name,cfg)
if ~isfolder(folder), mkdir(folder); end
drawnow;
exportgraphics(f,fullfile(folder,name),'Resolution',150);
if ~cfg.show_figures, close(f); end
end
