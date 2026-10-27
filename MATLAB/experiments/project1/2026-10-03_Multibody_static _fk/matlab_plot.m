%% 实验2：多位形静态FK一致性；各位形独立统计，不是连续轨迹
clear; clc; close all;
script_dir=fileparts(mfilename('fullpath'));
repo_dir=fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(fullfile(repo_dir,'MATLAB')); setup_piper_paths;
root_dir=fullfile(repo_dir,'experiments','project1','2026-10-03_Multibody_static _fk');
data_dir=fullfile(root_dir,'data_csv');
num_poses=10; pairing_window_ms=2;
plot_each_pose=true; show_figures=usejava('desktop');
pose_labels=string(compose('pose%02d',(1:num_poses)'));
q_names=compose('q%d_rad',1:6);
feedback_names={'feedback_x_m','feedback_y_m','feedback_z_m'};
poe_names={'poe_x_m','poe_y_m','poe_z_m'};

%% 1. 读取与独立统计；只核对位置误差列，不新增独立POE验算
data_all=cell(num_poses,1); rows=struct([]);
cfg_static=struct('frame_window_s',pairing_window_ms/1000, ...
    'position_recheck_tolerance_m',1e-11);
for k=1:num_poses
    d=load_feedback_csv(fullfile(data_dir,sprintf('pose%02d.csv',k)));
    row=summarize_static_pose(d,pose_labels(k),cfg_static);
    if k==1, rows=row; else, rows(k)=row; end
    data_all{k}=d.table; % 保留原MAT归档中的原始表
    if ~d.timing.ok, warning('%s：时间或跨度异常，请查看质量报告',pose_labels(k)); end
end
summary=struct2table(rows);
% 下面的数组仅供本实验专项总览绘图使用，不重复计算统计量。
sample_count=summary.sample_count;
q_mean_deg=summary{:,cellstr(compose('q%d_mean_deg',1:6))};
q_range_deg=summary{:,cellstr(compose('q%d_range_deg',1:6))};
feedback_mean_mm=summary{:,{'feedback_x_mean_mm','feedback_y_mean_mm','feedback_z_mean_mm'}};
poe_mean_mm=summary{:,{'poe_x_mean_mm','poe_y_mean_mm','poe_z_mean_mm'}};
position_stats_mm=summary{:,{'position_mean_mm','position_rms_mm','position_max_mm'}};
rotation_stats_deg=summary{:,{'rotation_mean_deg','rotation_rms_deg','rotation_max_deg'}};
span_max_ms=summary.frame_span_max_ms;
span_over_window_count=summary.span_over_window_count;
axis_names={'x','y','z'};

%% 2. 保存统计；明确允许重新运行覆盖本脚本生成的CSV/MAT，不保存图片
save_analysis_results(root_dir,struct('summary',summary), ...
    struct('summary',summary,'data_all',{data_all},'pairing_window_ms',pairing_window_ms), ...
    struct('overwrite',true));

%% 3. 图形设置（不修改 MATLAB 的全局默认字体）
font_name = 'Noto Sans CJK SC';
visibility = 'off';
if show_figures
    visibility = 'on';
end
x = 1:num_poses;

%% 4. 总览：各位形的六轴实际关节角
f = figure('Name','各位形实际关节角','Color','w', ...
    'Position',[100 100 1200 800], 'Visible',visibility);
tiledlayout(3,2,'TileSpacing','compact');
for j = 1:6
    nexttile;
    plot(x, q_mean_deg(:,j), 'o', 'LineWidth',1.3, 'MarkerSize',6);
    grid on; xticks(x); xticklabels(pose_labels); xtickangle(35);
    xlabel('位形编号（非连续时间）'); ylabel('角度 / deg'); title(sprintf('J%d',j));
end
sgtitle('十个位形的实际关节角均值');
format_plot(f, '01_joint_angles_overview', font_name);

%% 5. 总览：模型一致性偏差
f = figure('Name','各位形模型一致性偏差','Color','w', ...
    'Position',[100 100 1200 720], 'Visible',visibility);
tiledlayout(2,1,'TileSpacing','compact');
nexttile; bar(x, position_stats_mm); grid on;
ylim([0,max(1e-6,1.15*max(position_stats_mm,[],'all'))]);
xticks(x); xticklabels(pose_labels); ylabel('位置偏差 / mm');
title('位置偏差：均值、RMS、最大值'); legend('均值','RMS','最大值','Location','eastoutside');
nexttile; bar(x, rotation_stats_deg); grid on;
ylim([0,max(1e-6,1.15*max(rotation_stats_deg,[],'all'))]);
xticks(x); xticklabels(pose_labels); ylabel('姿态偏差 / deg');
title('姿态偏差：均值、RMS、最大值'); legend('均值','RMS','最大值','Location','eastoutside');
sgtitle('POE 与CAN反馈的模型一致性（不是独立定位精度）');
format_plot(f, '02_model_errors_overview', font_name);

%% 6. 总览：末端 XYZ 与带符号位置偏差
f = figure('Name','末端位置对比','Color','w', ...
    'Position',[100 100 1200 850], 'Visible',visibility);
tiledlayout(3,2,'TileSpacing','compact');
for j = 1:3
    nexttile;
    plot(x, feedback_mean_mm(:,j),'bo',x,poe_mean_mm(:,j),'rx','LineWidth',1.2);
    grid on; xticks(x); xticklabels(pose_labels); xtickangle(35);
    ylabel(sprintf('%s / mm',upper(axis_names{j})));
    title(sprintf('%s 坐标均值',upper(axis_names{j}))); legend('CAN反馈','POE计算','Location','best');
    nexttile;
    bar(x,poe_mean_mm(:,j)-feedback_mean_mm(:,j)); grid on;
    xticks(x); xticklabels(pose_labels); xtickangle(35);
    ylabel('POE - 反馈 / mm'); title(sprintf('%s 方向带符号偏差',upper(axis_names{j})));
end
sgtitle('末端位置与方向偏差（各位形独立统计）');
format_plot(f, '03_position_comparison_overview', font_name);

%% 7. 总览：采样数量和六帧接收跨度
f = figure('Name','采样情况','Color','w', ...
    'Position',[100 100 1200 700], 'Visible',visibility);
tiledlayout(2,1,'TileSpacing','compact');
nexttile; bar(x,sample_count); grid on;
ylim([0,1.1*max(sample_count)]);
xticks(x); xticklabels(pose_labels); ylabel('完整反馈组数'); title('各位形有效数据数量');
nexttile; bar(x,span_max_ms); hold on; yline(pairing_window_ms,'r--','配对窗口 2 ms');
ylim([0,1.1*max([pairing_window_ms;span_max_ms])]);
grid on; xticks(x); xticklabels(pose_labels); ylabel('接收时间跨度 / ms');
title('各位形六帧最大接收跨度（不是同步采样证明）');
format_plot(f, '04_sampling_overview', font_name);

%% 8. 每个位形详细图：实际角度、XYZ、位置和姿态偏差
if plot_each_pose
    for k = 1:num_poses
        D = data_all{k};
        f = figure('Name',char(pose_labels(k)), 'Color','w', ...
            'Position',[100 100 1350 950], 'Visible',visibility);
        tiledlayout(3,2,'TileSpacing','compact');
        nexttile;
        plot(D.t_s,D{:,cellstr(q_names)}*180/pi,'LineWidth',1.2);
        grid on; xlabel('相对时间 / s'); ylabel('角度 / deg'); title('实际关节角反馈');
        legend('J1','J2','J3','J4','J5','J6','Location','eastoutside');
        for j = 1:3
            nexttile;
            plot(D.t_s,D{:,feedback_names{j}}*1000,'b-', ...
                D.t_s,D{:,poe_names{j}}*1000,'r--','LineWidth',1.2);
            grid on; xlabel('相对时间 / s'); ylabel('位置 / mm');
            title(sprintf('%s 坐标',upper(axis_names{j}))); legend('CAN反馈','POE计算','Location','best');
        end
        nexttile;
        plot(D.t_s,D.position_error_m*1000,'LineWidth',1.2);
        ylim([0,max(1e-6,1.2*max(D.position_error_m*1000))]);
        grid on; xlabel('相对时间 / s'); ylabel('位置偏差 / mm'); title('位置模型一致性偏差');
        nexttile;
        plot(D.t_s,D.rotation_error_rad*180/pi,'LineWidth',1.2);
        ylim([0,max(1e-6,1.2*max(D.rotation_error_rad*180/pi))]);
        grid on; xlabel('相对时间 / s'); ylabel('姿态偏差 / deg'); title('姿态模型一致性偏差');
        sgtitle(sprintf('%s：停稳后静态反馈，不是运动轨迹',pose_labels(k)));
        format_plot(f, sprintf('pose%02d_detail',k), font_name);
    end
end

%% 9. 命令窗口结果与解释
disp(summary(:,{'pose','sample_count','position_mean_mm','position_rms_mm', ...
    'position_max_mm','rotation_mean_deg','rotation_max_deg','frame_span_max_ms'}));
fprintf('位形数：%d；完整反馈总数：%d\n', num_poses,sum(sample_count));
fprintf('各位形平均位置偏差范围：%.9f ~ %.9f mm\n', ...
    min(position_stats_mm(:,1)),max(position_stats_mm(:,1)));
fprintf('全部样本最大姿态偏差：%.9f deg\n',max(rotation_stats_deg(:,3)));
fprintf('六帧最大接收跨度：%.9f ms；超窗口反馈组：%d\n', ...
    max(span_max_ms),sum(span_over_window_count));
fprintf('单个位形内各关节最大变化范围：%.9f deg\n',max(q_range_deg,[],'all'));
fprintf('统计表已保存：%s\n',root_dir);
fprintf('图形仅显示，不自动保存 PNG 或 FIG。\n');
fprintf(['说明：十个位形覆盖的是本次局部测试范围，不代表整个工作空间。\n' ...
    '厂家位姿反馈不是外部测量，本次不是实际定位精度或动态跟踪验证。\n']);

%% 本实验专项格式：位形编号不是时间，不调用时间曲线函数
function format_plot(f,basename,font_name)
apply_plot_style(f,struct('font_name',font_name,'font_size',10));
if endsWith(basename,'overview')
    set(findall(f,'Type','axes'),'XLim',[0.5,10.5]);
end
drawnow;
end
