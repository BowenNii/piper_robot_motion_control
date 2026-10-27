%% 60秒终点IK真机实验：计划与反馈分析，不连接ROS，不发送控制指令
% 正式入口：复核计划并分析真实执行记录；旧独立预览入口已归档。
clear;clc;
script_dir=fileparts(mfilename('fullpath'));
repo_dir=fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(fullfile(repo_dir,'MATLAB'));setup_piper_paths;
run_name='run02'; % 切换回run01时，执行目录为execution_mwd2hy56。
execution_name='execution_joaafelg';
run_dir=fullfile(repo_dir,'experiments','project1','2026-10-24_zero_to_pose',run_name);
execution_dir=fullfile(run_dir,execution_name);
urdf=fullfile(repo_dir,'models','urdf','piper_description.urdf');
out=fullfile(execution_dir,'matlab_analysis');
figures=fullfile(out,'figures');
cfg=struct('tail_window_s',.5,'show_figures',usejava('desktop'),'font','Noto Sans CJK SC');
[r,joint_stats,pose_stats,timing_stats]=analyze_real_ik_run(run_dir,execution_dir,urdf,cfg);
assert(r.plan.path_mode==1 && r.meta.duration_s==60,'不是本次60秒终点IK实验');
analysis_info=table(string(execution_name),r.alignment,r.offset_s,r.alignment_note, ...
    r.status_bad_count,r.endpoint_stable,cfg.tail_window_s, ...
    'VariableNames',{'execution_name','time_alignment','receive_origin_offset_s', ...
    'alignment_note','bad_status_rows','endpoint_joint_stable','endpoint_window_s'});
save_analysis_results(out,struct('joint_stats',joint_stats,'pose_stats',pose_stats, ...
    'timing_stats',timing_stats,'plan_stats',r.plan_stats,'analysis_info',analysis_info), ...
    struct('r',r,'cfg',cfg,'joint_stats',joint_stats,'pose_stats',pose_stats, ...
    'timing_stats',timing_stats,'analysis_info',analysis_info,'matlab_version',version), ...
    struct('overwrite',true)); % 只覆盖本脚本派生结果，不改动原CSV/JSON。
T=r.plan.t(end);bound=[min(r.t_feedback),max(r.t_feedback)];
time_label='相对指令起点的接收时间 / s';
if r.alignment=="legacy_tail_estimate",time_label='相对估计起点的接收时间 / s';end

%% 图1：六轴跟随；同向、重合程度、台阶/保持、终点是否稳定
f=new_plot('60秒真机IK：图1 六轴跟随',cfg);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for j=1:6
    nexttile;plot(r.plan.t,r.plan.q_deg(:,j),'b-','LineWidth',1.5);hold on;
    plot(r.t_feedback,r.q_feedback(:,j)*180/pi,'Color',[.9 .35 .05],'LineWidth',1.0);
    xline(0,'k:');xline(T,'k:');grid on;xlim(bound);xlabel(time_label);ylabel('角度 / °');
    title(sprintf('J%d：末段均值偏差 %+.4f°',j,joint_stats.final_mean_minus_goal_rad(j)*180/pi));
    if j==1,legend('计划角（按计划时间）','编码器反馈角（按接收时间）','Location','best');end
end
sgtitle({'图1  零位→目标T：60秒终点IK后的关节轨迹跟随',execution_name});
save_plot(f,figures,'matlab_01_joint_tracking.png',cfg);

%% 图2：反馈角减最近已发目标角，不是DLS数值残差
f=new_plot('60秒真机IK：图2 关节偏差',cfg);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for j=1:6
    nexttile;plot(r.t_feedback,r.joint_error(:,j),'LineWidth',1.05);hold on;
    yline(0,'k:');xline(T,'k:');grid on;xlim(bound);xlabel(time_label);ylabel('反馈 - 最近已发目标 / rad');
    title(sprintf('J%d：运动段最大|偏差| %.4g rad',j,joint_stats.peak_abs_error_rad_aligned(j)));
end
sgtitle({'图2  真机关节跟随偏差（不是IK残差）',char(r.alignment_note)});
save_plot(f,figures,'matlab_02_joint_error.png',cfg);

%% 图3：本轮位置、姿态都会变化；不能沿用2mm实验“姿态不变”的结论
f=new_plot('60秒真机IK：图3 末端位姿变化',cfg);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');axes_names={'X','Y','Z'};
for j=1:3
    nexttile;plot(r.plan.t,r.plan_delta_mm(:,j),'b-','LineWidth',1.5);hold on;
    plot(r.t_feedback,r.feedback_delta_mm(:,j),'Color',[.9 .35 .05],'LineWidth',1.0);
    xline(T,'k:');grid on;xlim(bound);xlabel(time_label);ylabel('相对同一计划起点 / mm');
    title(sprintf('%s：计划变化 %.3f mm，反馈FK变化 %.3f mm', ...
        axes_names{j},r.planned_displacement_mm(j),r.actual_displacement_mm(j)));
    if j==1,legend('关节路径对应的计划FK','反馈角FK','Location','best');end
end
nexttile;plot(r.plan.t,r.plan_orientation_change_deg,'b-','LineWidth',1.5);hold on;
plot(r.t_feedback,r.feedback_orientation_change_deg,'Color',[.9 .35 .05],'LineWidth',1.0);
xline(T,'k:');grid on;xlim(bound);xlabel(time_label);ylabel('相对同一计划起点的SO(3)角 / °');
title('计划与反馈的姿态变化（不是姿态误差，也不是RPY分量）');
sgtitle('图3  模型末端位姿变化：此关节路径不要求末端走直线（非外部实测）');
save_plot(f,figures,'matlab_03_cartesian_motion.png',cfg);

%% 图4：位置距离与SO(3)姿态误差；不能用1e-10的IK容差判断真机是否合格
f=new_plot('60秒真机IK：图4 末端位姿偏差',cfg);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile;plot(r.t_feedback,r.p_error_mm,'LineWidth',1.1);hold on;xline(T,'k:');
grid on;xlim(bound);xlabel(time_label);ylabel('位置偏差范数 / mm');
title(sprintf('末0.5秒均值角FK对目标T：位置偏差 %.4f mm',pose_stats.endpoint_position_error_mm));
nexttile;plot(r.t_feedback,r.rotation_error_deg,'LineWidth',1.1);hold on;xline(T,'k:');
grid on;xlim(bound);xlabel(time_label);ylabel('SO(3)姿态偏差 / °');
title(sprintf('末0.5秒均值角FK对目标T：姿态偏差 %.4f°',pose_stats.endpoint_orientation_error_deg));
sgtitle('图4  反馈角FK与最近已发目标角FK比较：不是外部实测定位精度');
save_plot(f,figures,'matlab_04_cartesian_error.png',cfg);

%% 图5：把启动记录与运动窗口分开，不把消息间隔当CAN六帧跨度
f=new_plot('60秒真机IK：图5 时间质量',cfg);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
nexttile;plot(r.interval_time,r.dt_receive_ms,'LineWidth',1.0);hold on;
plot(r.interval_time,r.dt_header_ms,'--');xline(0,'k:');xline(T,'k:');
grid on;xlim(bound);xlabel(time_label);ylabel('间隔 / ms');title('全部反馈记录：包含运动前后');
legend('接收间隔','消息时间戳间隔','Location','best');
inside=r.execution_mask(1:end-1)&r.execution_mask(2:end);
nexttile;plot(r.interval_time(inside),r.dt_receive_ms(inside),'LineWidth',1.0);hold on;
plot(r.interval_time(inside),r.dt_header_ms(inside),'--');yline(5,'k:','名义5ms');
grid on;xlim([0,T]);xlabel(time_label);ylabel('间隔 / ms');title('仅运动窗口：观察反馈记录是否有明显间断');
nexttile;plot(r.C.target_time_s(2:end),diff(r.C.send_elapsed_s)*1000);hold on;yline(20,'k:','计划20ms');
grid on;xlim([0,T]);xlabel('计划指令时间 / s');ylabel('发送间隔 / ms');title('主机实际发送间隔，不是电机控制周期');
nexttile;plot(r.C.target_time_s,r.C.lateness_s*1000);hold on;yline(20,'r:','程序终止线20ms');
grid on;xlim([0,T]);xlabel('计划指令时间 / s');ylabel('调度迟到 / ms');
title(sprintf('状态异常 %d 条；重复反馈时间戳 %d 条',r.status_bad_count,r.header_repeat_count));
sgtitle('图5  60秒实验记录/发送质量，不测单向通信延迟');
save_plot(f,figures,'matlab_05_timing.png',cfg);

disp(joint_stats);disp(pose_stats);disp(timing_stats);
fprintf('\n本轮记录：%s\n对齐方法：%s\n',execution_name,r.alignment_note);
fprintf('终点最大关节偏差（最后样本）：%.9g rad\n',max(abs(joint_stats.last_minus_goal_rad)));
fprintf('末0.5秒最大关节波动范围：%.9g rad\n',max(r.final_joint_range));
fprintf('反馈FK对目标T：位置 %.6f mm，姿态 %.6f deg\n', ...
    pose_stats.endpoint_position_error_mm,pose_stats.endpoint_orientation_error_deg);
fprintf('运动段最大模型位置偏差 %.6f mm，最大姿态偏差 %.6f deg\n', ...
    pose_stats.peak_position_error_mm_aligned,pose_stats.peak_orientation_error_deg_aligned);
fprintf('状态异常记录 %d 条。末端结果为模型估计，不是外部独立精度测量。\n',r.status_bad_count);
fprintf('已保存5张PNG、统计CSV与MAT：%s\n',out);

function f=new_plot(name,cfg)
visible='off';if cfg.show_figures,visible='on';end
f=figure('Name',name,'Color','w','Visible',visible,'Position',[80,60,1450,1000],'Renderer','painters');
set(f,'DefaultAxesFontName',cfg.font,'DefaultTextFontName',cfg.font,'DefaultAxesFontSize',11, ...
    'DefaultTextInterpreter','none','DefaultLegendInterpreter','none');
end

function save_plot(f,folder,name,cfg)
if ~isfolder(folder),mkdir(folder);end
drawnow;exportgraphics(f,fullfile(folder,name),'Resolution',150);
if ~cfg.show_figures,close(f);end
end
