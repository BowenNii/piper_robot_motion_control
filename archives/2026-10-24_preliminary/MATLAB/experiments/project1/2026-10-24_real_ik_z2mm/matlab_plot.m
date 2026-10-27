%% 真机小幅IK实验：只读记录；不连接ROS、不执行运动
% 修改run_name/execution_name即可分析后续同类实验。
% 图中末端效果均为编码器+URDF的FK结果，不是外部实测定位精度。
clear; clc;
script_dir=fileparts(mfilename('fullpath'));
archive_root=fileparts(fileparts(fileparts(fileparts(script_dir))));
repo_dir=fileparts(fileparts(archive_root));
addpath(fullfile(repo_dir,'MATLAB')); setup_piper_paths();
run_name='run01'; execution_name='execution_2zwqntrv';
run_dir=fullfile(archive_root,'experiments','project1','2026-10-24_real_ik_z2mm',run_name);
execution_dir=fullfile(run_dir,execution_name);
urdf_path=fullfile(repo_dir,'models','urdf','piper_description.urdf');
analysis_dir=fullfile(execution_dir,'matlab_analysis');
figure_dir=fullfile(analysis_dir,'figures');
cfg=struct('tail_window_s',0.5,'legacy_tail_s',2, ...
    'show_figures',usejava('desktop'),'font','Noto Sans CJK SC');
[r,joint_stats,pose_stats,timing_stats]=analyze_real_ik_run(run_dir,execution_dir,urdf_path,cfg);
analysis_info=table(r.alignment,r.offset_s,r.alignment_note,r.status_bad_count, ...
    cfg.tail_window_s,r.endpoint_stable, ...
    'VariableNames',{'time_alignment','receive_origin_offset_s','alignment_note', ...
    'bad_status_rows','endpoint_window_s','endpoint_joint_stable'});
save_analysis_results(analysis_dir, ...
    struct('joint_stats',joint_stats,'pose_stats',pose_stats,'timing_stats',timing_stats, ...
    'plan_stats',r.plan_stats,'analysis_info',analysis_info),struct('r',r,'cfg',cfg,'joint_stats',joint_stats, ...
    'pose_stats',pose_stats,'timing_stats',timing_stats,'matlab_version',version), ...
    struct('overwrite',true)); % 仅覆盖本脚本生成的结果；不修改任何原始记录。
time_label='相对指令起点时间 / s';
if r.alignment=="legacy_tail_estimate", time_label='相对估计指令起点时间 / s'; end
T=r.plan.t(end); bound=[min(r.t_feedback),max(r.t_feedback)];

%% 图1：六轴计划角与真机反馈；先看变化方向、整体跟随及终点稳定性
f=new_plot('真机IK六轴跟随',cfg); tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for j=1:6
    nexttile;plot(r.plan.t,r.plan.q_rad(:,j)*180/pi,'b-','LineWidth',1.5);hold on;
    plot(r.t_feedback,r.q_feedback(:,j)*180/pi,'Color',[.9 .35 .05],'LineWidth',1.0);
    xline(T,'k:');grid on;xlim(bound);xlabel(time_label);ylabel('关节角 / °');title(sprintf('J%d',j));
    if j==1, legend('计划关节角','编码器反馈角','Location','best');end
end
sgtitle({'图1  六轴计划与真机反馈',char(r.alignment_note)});
save_plot(f,figure_dir,'matlab_01_joint_tracking.png',cfg);

%% 图2：反馈角减最近已发送目标；峰值/RMS受旧日志估计时间对齐影响
f=new_plot('真机关节跟随偏差',cfg);tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for j=1:6
    nexttile;plot(r.t_feedback,r.joint_error(:,j),'LineWidth',1.1);hold on;
    yline(0,'k:');xline(T,'k:');
    title(sprintf('J%d  运动段最大|偏差| %.3g rad',j,joint_stats.peak_abs_error_rad_aligned(j)));
    grid on;xlim(bound);xlabel(time_label);ylabel('反馈 - 最近已发目标 / rad');
end
sgtitle({'图2  关节跟随偏差，不是IK求解残差',char(r.alignment_note)});
save_plot(f,figure_dir,'matlab_02_joint_error.png',cfg);

%% 图3：XYZ模型末端位移及姿态漂移；位置共用计划起点，不能各自归零掩盖初始偏差
f=new_plot('模型末端位移与姿态变化',cfg);tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
axes_names={'X','Y','Z'};
for j=1:3
    nexttile;plot(r.plan.t,r.plan_delta_mm(:,j),'b-','LineWidth',1.5);hold on;
    plot(r.t_feedback,r.feedback_delta_mm(:,j),'Color',[.9 .35 .05],'LineWidth',1.0);
    xline(T,'k:');grid on;xlim(bound);xlabel(time_label);ylabel('相对计划起点 / mm');
    title(sprintf('%s轴：计划变化 %.3f mm，反馈FK变化 %.3f mm', ...
        axes_names{j},r.planned_displacement_mm(j),r.actual_displacement_mm(j)));
    if j==1,legend('笛卡尔计划','反馈角FK','Location','best');end
end
nexttile;plot(r.t_feedback,r.orientation_drift_deg,'LineWidth',1.1);hold on;
yline(0,'k:');xline(T,'k:');grid on;xlim(bound);xlabel(time_label);ylabel('相对初始反馈姿态 / °');
title(sprintf('计划姿态不变；终态漂移 %.4f°',pose_stats.endpoint_orientation_drift_deg));
sgtitle('图3  编码器+URDF回算末端效果（非外部实测）');
save_plot(f,figure_dir,'matlab_03_cartesian_displacement.png',cfg);

%% 图4：位姿跟随误差；不是MATLAB/C++交叉验算误差，不用1e-9等数值容差判真机
f=new_plot('模型末端跟随偏差',cfg);tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile;plot(r.t_feedback,r.p_error_mm,'LineWidth',1.2);hold on;xline(T,'k:');
grid on;xlim(bound);xlabel(time_label);ylabel('位置偏差范数 / mm');
title(sprintf('终点均值角FK对计划终点：%.4f mm',pose_stats.endpoint_position_error_mm));
nexttile;plot(r.t_feedback,r.rotation_error_deg,'LineWidth',1.2);hold on;xline(T,'k:');
grid on;xlim(bound);xlabel(time_label);ylabel('SO(3)姿态偏差 / °');
title(sprintf('终点均值角FK对计划姿态：%.4f°',pose_stats.endpoint_orientation_error_deg));
sgtitle({'图4  反馈角FK与最近已发目标角FK比较',char(r.alignment_note)});
save_plot(f,figure_dir,'matlab_04_cartesian_error.png',cfg);

%% 图5：反馈记录间隔、发送间隔、发送迟到；不等同于CAN六帧跨度
f=new_plot('真机实验记录时间质量',cfg);tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile;plot(r.interval_time,r.dt_receive_ms,'LineWidth',1.0);hold on;
plot(r.interval_time,r.dt_header_ms,'--','LineWidth',1.0);yline(20,'r:','20ms记录筛查线');
xline(0,'k:');xline(T,'k:');grid on;xlim(bound);xlabel(time_label);ylabel('间隔 / ms');
title('接收间隔与消息时间戳间隔：含启动阶段');legend('接收时间差','消息时间戳差','Location','best');
nexttile;plot(r.C.target_time_s(2:end),diff(r.C.send_elapsed_s)*1000,'LineWidth',1.0);hold on;
yline(20,'k:','计划20ms');grid on;xlim([0,T]);xlabel('计划指令时间 / s');ylabel('发送间隔 / ms');
title('实际发送周期（不是电机底层控制周期）');
nexttile;plot(r.C.target_time_s,r.C.lateness_s*1000,'LineWidth',1.0);hold on;
yline(20,'r:','程序终止线20ms');grid on;xlim([0,T]);xlabel('计划指令时间 / s');ylabel('发送前调度迟到 / ms');
title(sprintf('状态异常记录 %d 条；重复关节时间戳 %d 条',r.status_bad_count,r.header_repeat_count));
sgtitle('图5  主机记录与发送时间质量（不测单向通信延迟）');
save_plot(f,figure_dir,'matlab_05_timing.png',cfg);

%% 数值汇总：先看终点效果，再结合曲线，不自动宣称末端精度合格
disp(joint_stats);disp(pose_stats);disp(timing_stats);
fprintf('\n时间对齐：%s\n',r.alignment_note);
fprintf('计划XYZ变化：[%+.6f %+.6f %+.6f] mm\n',r.planned_displacement_mm);
fprintf('反馈FK的XYZ变化（首尾0.5秒均值角）：[%+.6f %+.6f %+.6f] mm\n',r.actual_displacement_mm);
fprintf('终点位置偏差：%.6f mm；姿态偏差：%.6f deg\n', ...
    pose_stats.endpoint_position_error_mm,pose_stats.endpoint_orientation_error_deg);
fprintf('终点平均角最大偏差：%.9g rad；末0.5秒最大关节波动范围：%.9g rad\n', ...
    r.final_joint_max_rad,max(r.final_joint_range));
fprintf('状态异常记录：%d；末段关节波动是否在程序自设0.001rad内：%d\n', ...
    r.status_bad_count,r.endpoint_stable);
fprintf('各关节记录中不同角度值的数量：');fprintf('%d ',joint_stats.observed_unique_positions);fprintf('\n');
fprintf('反馈有台阶/保持段；不能将记录跳变直接当作物理瞬时速度或判断编码器分辨率。\n');
fprintf('结论须结合本次2mm任务误差预算；没有外部测量，不能宣称真实末端定位精度。\n');
fprintf('结果与5张PNG：%s\n',analysis_dir);

function f=new_plot(name,cfg)
visible='off';if cfg.show_figures,visible='on';end
f=figure('Name',name,'Color','w','Visible',visible,'Position',[80,60,1400,950],'Renderer','painters');
set(f,'DefaultAxesFontName',cfg.font,'DefaultTextFontName',cfg.font,'DefaultAxesFontSize',11, ...
    'DefaultTextInterpreter','none','DefaultLegendInterpreter','none');
end

function save_plot(f,folder,name,cfg)
if ~isfolder(folder),mkdir(folder);end
drawnow;exportgraphics(f,fullfile(folder,name),'Resolution',150);
if ~cfg.show_figures,close(f);end
end
