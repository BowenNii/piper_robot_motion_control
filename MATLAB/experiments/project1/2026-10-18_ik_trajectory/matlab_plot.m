%% 时间参数化IK离线验证：C++导出 -> MATLAB独立验算 -> 统计与绘图
% 直接运行本文件，不复制到命令窗口。只读CSV/URDF，不连接机械臂。
% s仍是路径进度；本版按t绘图，复核数值速度/加速度，不控制真机。
clear; clc;
script_dir=fileparts(mfilename('fullpath'));
repo_dir=fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(fullfile(repo_dir,'MATLAB'));
setup_piper_paths();
root_dir=fullfile(repo_dir,'experiments','project1','2026-10-18_ik_trajectory');
csv_path=fullfile(root_dir,'data_csv','trajectory.csv');
figure_dir=fullfile(root_dir,'figures');
analysis_dir=fullfile(root_dir,'analysis');
urdf_path=fullfile(repo_dir,'models','urdf','piper_description.urdf');

%% 1. 参数和独立验算；读取/验证细节封装在experiment_analysis中
cfg.position_tolerance_m=1e-9;  % MATLAB与C++ FK交叉验算容差，不是IK求解容差
cfg.rotation_tolerance_rad=1e-8;
cfg.show_figures=usejava('desktop');
cfg.font='Noto Sans CJK SC';
[r,summary,points]=analyze_ik_trajectory(csv_path,urdf_path,cfg);
assert(r.timed,'请先用新版C++导出含t_s的时间轨迹CSV');
% 只覆盖本脚本生成的统计和PNG，不覆盖输入CSV或照片。
save_analysis_results(analysis_dir,struct('summary',summary,'point_checks',points), ...
    struct('result',r,'summary',summary,'cfg',cfg,'csv_path',csv_path, ...
    'urdf_path',urdf_path,'matlab_version',version),struct('overwrite',true));

%% 2. 六轴关节角：检查整体变化与连续性，不要求中间角度线性插值
f=new_figure('连续路径IK六轴关节角',cfg);
tiledlayout(3,2,'TileSpacing','compact');
for j=1:6
    nexttile;
    plot(r.t,r.q_deg(:,j),'LineWidth',1.3); hold on;
    plot(0,r.q_start(j)*180/pi,'go','MarkerFaceColor','g');
    plot(r.config.duration_s,r.q_goal(j)*180/pi,'rx','LineWidth',1.5);
    % 给端点标记留白，避免红叉/绿点被坐标轴边界裁掉。
    xlim([-0.02,1.02]*r.config.duration_s);
    angles=[r.q_deg(:,j);r.q_start(j)*180/pi;r.q_goal(j)*180/pi];
    margin=max(0.05,0.06*(max(angles)-min(angles)));
    ylim([min(angles)-margin,max(angles)+margin]);
    xlabel('时间 / s'); ylabel('角度 / °'); title(sprintf('J%d',j));
    if j==1, legend('IK解','起点参考角','终点参考角','Location','best'); end
end
sgtitle('六轴关节角随时间变化（五次时间缩放，离线IK）');
finish_figure(f,figure_dir,'matlab_01_六轴关节角.png',cfg);

%% 3. 相邻增量：每轴有符号变化及每步六轴最大绝对变化
f=new_figure('相邻关节解增量',cfg); tiledlayout(2,1);
nexttile;
% 第0点没有路径前驱；CSV第一行0只用于起始检查，图中不把它当运动增量。
plot(r.t(2:end),r.delta_q_rad(2:end,:),'LineWidth',1.1);
xlabel('时间 / s'); ylabel('相邻角度增量 / rad');
legend(compose('J%d',1:6),'Location','eastoutside'); title('六轴有符号增量，不是关节速度');
nexttile;
plot(r.t(2:end),r.max_abs_delta_q_rad(2:end),'LineWidth',1.3); hold on;
yline(r.config.max_sample_step_rad,'r--','相邻解上限');
ylim([0,r.config.max_sample_step_rad*1.15]);
xlabel('时间 / s'); ylabel('最大绝对增量 / rad'); title('每一步取六轴最大值');
finish_figure(f,figure_dir,'matlab_02_相邻关节增量.png',cfg);

%% 4. 回代残差：与目标位姿比较，不涉及CAN和真实测量
f=new_figure('FK回代位姿残差',cfg); tiledlayout(2,1);
nexttile;
plot(r.t,r.position_error_m*1000,'LineWidth',1.3); hold on;
yline(r.config.position_tolerance_m*1000,'r--','位置容差');
xlabel('时间 / s'); ylabel('位置残差 / mm'); title('MATLAB POE回代与目标位置');
nexttile;
plot(r.t,r.rotation_error_rad*180/pi,'LineWidth',1.3); hold on;
yline(r.config.orientation_tolerance_rad*180/pi,'r--','姿态容差');
xlabel('时间 / s'); ylabel('相对旋转角 / °'); title('MATLAB POE回代与目标姿态');
finish_figure(f,figure_dir,'matlab_03_回代位姿残差.png',cfg);

%% 5. 奇异性与求解代价：没有设置奇异值验收线，迭代次数不是耗时
f=new_figure('加权奇异值与IK迭代次数',cfg); tiledlayout(2,1);
nexttile;
plot(r.t,r.weighted_sigma_min,'LineWidth',1.3);
xlabel('时间 / s'); ylabel('最小加权奇异值'); title('A = D J_b，仅记录，不据此判定安全');
nexttile; stairs(r.t,r.iterations,'LineWidth',1.3);
ylim([-0.1,max(1,max(r.iterations))+0.5]);
xlabel('时间 / s'); ylabel('IK更新次数'); title('每个目标使用上一点成功解作为初值');
finish_figure(f,figure_dir,'matlab_04_奇异值与迭代次数.png',cfg);

%% 6. 独立POE交叉验算：微小非零值可能仅为浮点舍入
f=new_figure('MATLAB与C++交叉验算',cfg); tiledlayout(2,1);
nexttile; plot(r.t,r.cross_position_m,'LineWidth',1.3);
xlabel('时间 / s'); ylabel('位置差 / m'); title('MATLAB POE与C++ POE');
nexttile; plot(r.t,r.cross_rotation_rad,'LineWidth',1.3);
xlabel('时间 / s'); ylabel('相对旋转角 / rad'); title('不是与目标的残差，也不是真机精度');
finish_figure(f,figure_dir,'matlab_05_MATLAB_CPP交叉验算.png',cfg);

%% 7. 数值速度和加速度：放大纵轴显示细节，上限在每幅图内标注。
values={r.dq_rad_s,r.ddq_rad_s2};
limits=[r.config.offline_velocity_limit_rad_s,r.config.offline_acceleration_limit_rad_s2];
names={'关节速度','关节加速度'}; units={'rad/s','rad/s²'};
files={'matlab_06_六轴关节速度.png','matlab_07_六轴关节加速度.png'};
for k=1:2
    f=new_figure(names{k},cfg); tiledlayout(3,2,'TileSpacing','compact');
    for j=1:6
        nexttile; plot(r.t,values{k}(:,j),'LineWidth',1.3); hold on;
        yline(limits(k),'r--'); yline(-limits(k),'r--');
        peak=max(abs(values{k}(:,j)));
        ylim([-1,1]*max(1.2*peak,1e-6));
        text(0.03,0.92,sprintf('自设上限 ±%.2g；峰值 %.4g',limits(k),peak), ...
            'Units','normalized','FontSize',9);
        xlabel('时间 / s'); ylabel(units{k}); title(sprintf('J%d',j));
    end
    sgtitle([names{k},'（数值差分估计，纵轴放大显示运动细节）']);
    finish_figure(f,figure_dir,files{k},cfg);
end
disp(summary);
fprintf('完成：%d点；图像：%s\n统计：%s\n',summary.samples,figure_dir,analysis_dir);

function f=new_figure(name,cfg)
% 只封装本脚本重复的图窗外观；数学验算仍由算法库负责。
visibility='off'; if cfg.show_figures, visibility='on'; end
f=figure('Name',name,'Color','w','Visible',visibility,'Position',[60,60,1100,760]);
end
function finish_figure(f,folder,name,cfg)
apply_plot_style(f,struct('font_name',cfg.font,'font_size',11));
save_experiment_figure(f,folder,name,struct('resolution',150,'overwrite',true));
if ~cfg.show_figures, close(f); end
end
