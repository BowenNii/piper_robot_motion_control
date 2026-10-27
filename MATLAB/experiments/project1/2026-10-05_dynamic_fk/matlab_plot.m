%% PiPER 动态 FK：五组分析、独立 MATLAB POE 验算与绘图
% 输入 data_csv/run01.csv～run05.csv（C++ 导出，m/rad/s）。
% 输出 analysis/ 两张统计表及 MAT 文件，figures/ 下 PNG。
% 在 MATLAB 中打开并运行本文件。不要复制到命令窗口运行。
% 只读 CSV/URDF，不连接 CAN，不修改日志、输入 CSV 或已有照片。
% 重复运行覆盖本脚本生成的同名输出；report.md 不自动覆盖。csv_path
clear; clc;
script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(fullfile(repo_dir,'MATLAB')); 
setup_piper_paths();
root_dir = fullfile(repo_dir,'experiments','project1','2026-10-05_dynamic_fk');
data_dir = fullfile(root_dir,'data_csv');
figure_dir = fullfile(root_dir,'figures');
analysis_dir = fullfile(root_dir,'analysis');

%% 1. 分析参数（不是机械臂控制参数）
cfg.change_threshold_deg = 0.0005; % 半个0.001°量化单位，运动变化检测阈值
cfg.motion_padding_s = 0.05;           % 运动阶段缓冲：变化包络前后各扩展50ms
cfg.activity_gap_s = 0.20;                 % 运动分段阈值：变化间隔超过此值，提示可能存在多次运动
cfg.frame_window_s = 0.002;           %六帧配对窗口阈值，实时性校验
cfg.endpoint_window_s = 1.0;           % 稳态统计窗口：最后1s均值代表终态反馈
cfg.static_range_deg = 0.002;          % 静止波动阈值：终态角度范围检查，不等同物理静止
cfg.cpp_position_tolerance_m = 1e-9;
cfg.cpp_rotation_tolerance_rad = 1e-8;
cfg.show_figures = usejava('desktop'); % 环境自适应显示：桌面图形界面显示图窗，批处理模式自动隐藏。
cfg.font = 'Noto Sans CJK SC';
cfg.limit_tolerance_rad = pi/180*0.001; % 保留原实验0.001deg限位比较余量
cfg.static_baseline_s = 0.5;           % 静态基线至少0.5s才标记为充足
run_names = string(compose('run%02d',(1:5)'));
run_labels = ["小幅单轴预检";"单轴去程";"单轴回零";"多轴去程";"多轴回零"];
% 目标来自操作记录；没有实时指令轨迹，不能据此计算动态跟踪误差。
targets_rad = [2*pi/180,0,0,0,0,0;0.3,0,0,0,0,0;zeros(1,6); ...
    0.3,0.3,-0.3,0.3,0.3,0.5;zeros(1,6)];
urdf_path = fullfile(repo_dir,'models','urdf','piper_description.urdf');
model = make_piper_model(urdf_path);
%% 2. 检查输入、计算；全部成功后再写文件
results = cell(5,1); %run01～run05 五组数据
run_rows=struct([]); %保存每组实验的汇总指标，例如采样数量、位置误差均值、最大误差等。
phase_rows=struct([]);%保存的是各阶段的汇总指标，例如运动前静止段、运动段、运动后静止段。
for k=1:5
    csv_path=fullfile(data_dir,run_names(k)+".csv");
    % 完整位姿模式：通用模块检查列、数值、时间及矩阵结构。
    d=load_feedback_csv(csv_path,struct('require_transforms',true,'strict_time',true));
    r=rmfield(d,{'table','T_can','T_cpp','timing'});%将 d 中除这四个字段外的内容放入 r
    r.name=run_names(k); 
    r.label=run_labels(k); 
    n=numel(r.t);%给当前结果添加实验名称、说明，并取得样本数 n
    % 核心业务仍在这里：用q独立回算MATLAB POE，指定允许的位置和旋转偏差,再与C++比较。
    cross_cfg=struct('position_tolerance_m',cfg.cpp_position_tolerance_m, ...
        'rotation_tolerance_rad',cfg.cpp_rotation_tolerance_rad);
    cross=validate_fk_samples(model.S_list,model.M,r.q_rad,d.T_cpp,cross_cfg);
    assert(cross.ok,'%s MATLAB/C++交叉验算超出容差',r.name);
    %cross.T_matlab 是 4×4×n 数组，每个样本对应一张位姿矩阵。
    % 取所有矩阵第4列的前三个元素整理为 3×n,转置为 n×3
    r.p_matlab=reshape(cross.T_matlab(1:3,4,:),3,n)';
    r.R_matlab=cross.T_matlab(1:3,1:3,:); 
    r.R_cpp=d.T_cpp(1:3,1:3,:);
    r.cpp_position_mm=cross.position_error_m*1000;
    r.cpp_rotation_deg=cross.rotation_error_rad*180/pi;
    r.position_mm=zeros(n,1); 
    r.rotation_deg=zeros(n,1);
    r.cpp_can_rotation_deg=zeros(n,1); 
    r.signed_position_mm=zeros(n,3);
    for j=1:n
    %取第 j 个样本的 MATLAB 位姿与 CAN 反馈位姿，计算：
    %p：位置偏差的大小，m。
    % a：相对旋转角，rad。
    % dp：XYZ 有符号位置差，MATLAB位置 − CAN位置，m。
        [p,a,dp]=pose_error(cross.T_matlab(:,:,j),d.T_can(:,:,j));
        r.position_mm(j)=p*1000; 
        r.rotation_deg(j)=a*180/pi;
        r.signed_position_mm(j,:)=dp'*1000;
        [~,a]=pose_error(d.T_cpp(:,:,j),d.T_can(:,:,j));
        r.cpp_can_rotation_deg(j)=a*180/pi;
    end
    % 每行的C++位置减去CAN反馈位置，得到N×3的位置差，与CAN旋转偏差，从°转回rad
    % 每行计算 sqrt(dx²+dy²+dz²)，得到N×1距离，单位m
    % 与CSV原有误差列比较，所有样本中的最大绝对差必须小于容差
    assert(max(abs(vecnorm(r.p_cpp-r.p_feedback,2,2)-d.table.position_error_m))<1e-12, ...
        '%s CSV位置偏差列复核失败',r.name);
    assert(max(abs(r.cpp_can_rotation_deg*pi/180-d.table.rotation_error_rad))<1e-8, ...
        '%s CSV姿态偏差列复核失败',r.name);
    %把当前实验结果、目标关节角、模型和分析参数送入汇总函数.
    [r,s,phase_stats]=summarize_dynamic_run(r,targets_rad(k,:),model,cfg);
    if k==1
        run_rows=s; phase_rows=phase_stats;
    else
        run_rows(k)=s; phase_rows=[phase_rows,phase_stats]; %#ok<AGROW>
    end
    results{k}=r;
    fprintf('%s：%d组，变化区间 %.3f～%.3fs，位置峰值 %.6fmm，姿态峰值 %.6f°\n', ...
        r.name,n,r.motion_bounds,max(r.position_mm),max(r.rotation_deg));
end
run_summary=struct2table(run_rows); 
phase_summary=struct2table(phase_rows);
assert(all(arrayfun(@(k) sum(phase_summary.samples(phase_summary.run==run_names(k))) ...
    ==run_summary.samples(k),1:5)),'阶段分配存在遗漏或重复');
matlab_version=version;
save_analysis_results(analysis_dir, ...
    struct('run_summary',run_summary,'phase_summary',phase_summary), ...
    struct('results',{results},'run_summary',run_summary,'phase_summary',phase_summary, ...
    'cfg',cfg,'model',model,'targets_rad',targets_rad,'urdf_path',urdf_path, ...
    'matlab_version',matlab_version),struct('overwrite',true));

%% 3. 五组总览、交叉验算和单轴运动
f=new_figure('五组动态FK总览',cfg); tiledlayout(2,2,'TileSpacing','compact');
nexttile; h=bar([run_summary.position_mean_mm,run_summary.position_rms_mm,run_summary.position_max_mm]);
xticks(1:5); xticklabels(run_names); ylabel('位置偏差 / mm'); grid on;
legend(h,{'均值','RMS','最大值'},'Location','northwest'); title('MATLAB POE与CAN位置');
nexttile; h=bar([run_summary.rotation_mean_deg,run_summary.rotation_rms_deg,run_summary.rotation_max_deg]);
xticks(1:5); xticklabels(run_names); ylabel('姿态偏差 / °'); grid on;
legend(h,{'均值','RMS','最大值'},'Location','northwest'); title('相对旋转角');
nexttile; bar(run_summary.motion_duration_s); xticks(1:5); xticklabels(run_names);
ylabel('角度变化包络 / s'); title('量化反馈变化时长（不含缓冲）'); grid on;
nexttile; bar(run_summary.frame_span_max_ms); yline(2,'r--','配对窗口');
ylim([0,max(2.2,1.1*max(run_summary.frame_span_max_ms))]); % 阈值线和标签留白
xticks(1:5); xticklabels(run_names); ylabel('六帧最大跨度 / ms'); grid on;
finish_figure(f,figure_dir,'matlab_01_五组动态FK总览.png',cfg);
f=new_figure('MATLAB与C++ POE交叉验算',cfg); tiledlayout(2,1);
nexttile; bar(run_summary.cpp_position_max_mm); xticks(1:5); xticklabels(run_names);
ylabel('最大位置差 / mm'); grid on; title('MATLAB独立POE对比CSV中的C++ POE');
nexttile; bar(run_summary.cpp_rotation_max_deg); xticks(1:5); xticklabels(run_names);
ylabel('最大相对旋转角 / °'); grid on; title('数值舍入级差异；不是物理定位精度');
finish_figure(f,figure_dir,'matlab_02_CPP交叉验算.png',cfg);
f=new_figure('单轴去程与回零',cfg); tiledlayout(3,2,'TileSpacing','compact');
for k=2:3
    r=results{k}; x=r.t-r.motion_bounds(1);
    curves={r.q_deg(:,1),r.position_mm,r.rotation_deg};
    labels={'J1 / °','位置偏差 / mm','姿态偏差 / °'};
    for j=1:3
        nexttile((j-1)*2+k-1); plot(x,curves{j},'LineWidth',1.1); grid on;
        xlim([-1,diff(r.motion_bounds)+1]); ylabel(labels{j}); xlabel('距反馈开始变化 / s');
        title(r.name+" "+r.label,'Interpreter','none');
        xline(0,'k--','HandleVisibility','off');
        xline(diff(r.motion_bounds),'k--','HandleVisibility','off');
    end
end
finish_figure(f,figure_dir,'matlab_03_单轴去程与回零.png',cfg);

%% 4. run04/run05重点图：全程保留，偏差另给运动区间放大
for k=4:5
    r=results{k}; prefix="matlab_"+r.name+"_";
    % 通用时序图交给plotting；阈值/终态目标/对比来源仍由主脚本明确。
    ix=find(r.phase==2); plot_cfg=struct('visible','off','font_name',cfg.font);
    if cfg.show_figures, plot_cfg.visible='on'; end
    plot_cfg.bounds_s=[]; if ~isempty(ix), plot_cfg.bounds_s=[r.t(ix(1)),r.t(ix(end))]; end
    plot_cfg.target_rad=r.target_rad; plot_cfg.title=r.name+" "+r.label;
    f=plot_joint_angles(r.t,r.q_rad,plot_cfg);
    finish_figure(f,figure_dir,prefix+"六关节角度.png",cfg);
    plot_cfg.title=r.name+"：C++与MATLAB曲线基本重合";
    plot_cfg.legend_location='southeast';
    f=plot_position_comparison(r.t,{r.p_feedback,r.p_cpp,r.p_matlab}, ...
        {'CAN反馈','C++ POE','MATLAB POE'},plot_cfg);
    finish_figure(f,figure_dir,prefix+"末端XYZ对比.png",cfg);
    f=new_figure(r.name+" 模型偏差",cfg); tiledlayout(2,2,'TileSpacing','compact');
    for j=1:4
        nexttile;
        if j<=2, y=r.position_mm; unit='位置偏差 / mm'; else, y=r.rotation_deg; unit='姿态偏差 / °'; end
        plot(r.t,y,'LineWidth',1.1); mark_motion_phases(gca,plot_cfg.bounds_s); grid on;
        ylabel(unit); xlabel('相对时间 / s');
        if mod(j,2)==0
            xlim([max(0,r.motion_bounds(1)-0.5),min(r.t(end),r.motion_bounds(2)+0.5)]);
            title('运动区间放大');
        else
            title('完整采集过程');
        end
    end
    sgtitle(r.name+"：MATLAB POE与CAN位姿偏差",'Interpreter','none');
    finish_figure(f,figure_dir,prefix+"模型偏差.png",cfg);
    f=new_figure(r.name+" 帧跨度与带符号位置偏差",cfg); tiledlayout(2,1);
    nexttile; plot(r.t,r.span_ms,'LineWidth',0.8); mark_motion_phases(gca,plot_cfg.bounds_s); grid on;
    yline(2,'r--','2ms窗口','HandleVisibility','off'); ylabel('六帧跨度 / ms'); xlabel('相对时间 / s');
    ylim([0,max(2.2,1.1*max(r.span_ms))]);
    nexttile; h=plot(r.t,r.signed_position_mm,'LineWidth',1); mark_motion_phases(gca,plot_cfg.bounds_s); grid on;
    ylabel('MATLAB POE - CAN / mm'); xlabel('相对时间 / s'); legend(h,{'dX','dY','dZ'},'Location','northeast');
    finish_figure(f,figure_dir,prefix+"帧跨度与位置分量.png",cfg);
end
disp(run_summary(:,{'run','samples','motion_start_s','motion_end_s', ...
    'position_max_mm','rotation_max_deg','cpp_position_max_mm','cpp_rotation_max_deg'}));
fprintf('完成：统计结果 %s\n图片 %s\n',analysis_dir,figure_dir);

%创建统一规格的图窗
function f=new_figure(name,cfg)
visibility='off'; if cfg.show_figures, visibility='on'; end
f=figure('Name',name,'Color','w','Visible',visibility,'Position',[60,60,1120,780]);
end

%统一格式 + 导出保存 + 收尾
function finish_figure(f,folder,name,cfg)
apply_plot_style(f,struct('font_name',cfg.font,'font_size',11));
save_experiment_figure(f,folder,name,struct('resolution',150,'overwrite',true));
if ~cfg.show_figures, close(f); end
end
