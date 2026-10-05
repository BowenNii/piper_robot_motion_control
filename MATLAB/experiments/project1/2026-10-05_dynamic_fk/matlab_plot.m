%% PiPER 动态 FK：五组分析、独立 MATLAB POE 验算与绘图
% 输入 data_csv/run01.csv～run05.csv（C++ 导出，m/rad/s）。
% 输出 analysis/ 两张统计表及 MAT 文件，figures/ 下 PNG。
% 在 MATLAB 中打开并运行本文件。不要复制到命令窗口运行。
% 只读 CSV/URDF，不连接 CAN，不修改日志、输入 CSV 或已有照片。
% 重复运行覆盖本脚本生成的同名输出；report.md 不自动覆盖。
clear; clc;
script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(fullfile(repo_dir,'MATLAB')); setup_piper_paths();
root_dir = fullfile(repo_dir,'experiments','project1','2026-10-05_dynamic_fk');
data_dir = fullfile(root_dir,'data_csv');
figure_dir = fullfile(root_dir,'figures');
analysis_dir = fullfile(root_dir,'analysis');

%% 1. 分析参数（不是机械臂控制参数）
cfg.change_threshold_deg = 0.0005; % 半个0.001°量化单位，检出角度变化
cfg.motion_padding_s = 0.05;       % 变化包络前后各扩展50ms
cfg.activity_gap_s = 0.20;         % 变化间隔超过此值，提示可能存在多次运动
cfg.frame_window_s = 0.002;
cfg.endpoint_window_s = 1.0;       % 最后1s均值代表终态反馈
cfg.static_range_deg = 0.002;      % 终态角度范围检查，不等同物理静止
cfg.cpp_position_tolerance_m = 1e-9;
cfg.cpp_rotation_tolerance_rad = 1e-8;
cfg.show_figures = usejava('desktop'); % batch仅保存，桌面模式显示图窗
cfg.font = 'Noto Sans CJK SC';
run_names = string(compose('run%02d',(1:5)'));
run_labels = ["小幅单轴预检";"单轴去程";"单轴回零";"多轴去程";"多轴回零"];
% 目标来自操作记录；没有实时指令轨迹，不能据此计算动态跟踪误差。
targets_rad = [2*pi/180,0,0,0,0,0;0.3,0,0,0,0,0;zeros(1,6); ...
    0.3,0.3,-0.3,0.3,0.3,0.5;zeros(1,6)];
urdf_path = fullfile(repo_dir,'models','urdf','piper_description.urdf');
model = make_piper_model(urdf_path);
% 最小自检：全静态与边界运动；空阶段不能被统计成零偏差。
test_cfg = cfg; test_cfg.motion_padding_s=0;
[mask,bounds] = split_phases((0:0.1:1)',zeros(11,6),test_cfg);
assert(all(mask==1) && all(isnan(bounds)));
[mask,bounds] = split_phases((0:0.1:1)',[(0:10)',zeros(11,5)],test_cfg);
assert(all(mask==2) && isequal(bounds,[0,1]));
assert(rotation_angle(eye(3))==0 && abs(rotation_angle(rot_z(pi/2))-pi/2)<1e-12);

%% 2. 检查输入、计算；全部成功后再写文件
results = cell(5,1); run_rows=struct([]); phase_rows=struct([]);
for k=1:5
    csv_path=fullfile(data_dir,run_names(k)+".csv");
    assert(isfile(csv_path),'找不到数据文件：%s',csv_path);
    d=readtable(csv_path,'VariableNamingRule','preserve');
    q_cols=string(compose('q%d_rad',1:6));
    r_cols=["poe_r11","poe_r12","poe_r13","poe_r21","poe_r22","poe_r23", ...
        "poe_r31","poe_r32","poe_r33"];
    required=["t_s","timestamp_s","frame_span_s",q_cols, ...
        "feedback_x_m","feedback_y_m","feedback_z_m", ...
        "feedback_roll_rad","feedback_pitch_rad","feedback_yaw_rad", ...
        "poe_x_m","poe_y_m","poe_z_m",r_cols,"position_error_m","rotation_error_rad"];
    assert(all(ismember(required,string(d.Properties.VariableNames))),'%s 缺少必需列',run_names(k));
    values=d{:,cellstr(required)};
    assert(isnumeric(values) && height(d)>=2 && all(isfinite(values),'all'), ...
        '%s 数据不足或包含非有限数值',run_names(k));
    assert(all(diff(d.timestamp_s)>0) && all(diff(d.t_s)>0), ...
        '%s 时间必须严格递增；不自动排序或删除重复样本',run_names(k));
    assert(max(abs(d.t_s-(d.timestamp_s-d.timestamp_s(1))))<1e-6,'%s 相对时间不一致',run_names(k));
    assert(all(d.frame_span_s>=0) && all(d.position_error_m>=0) && ...
        all(d.rotation_error_rad>=0 & d.rotation_error_rad<=pi),'%s 跨度或偏差列非法',run_names(k));
    r.name=run_names(k); r.label=run_labels(k); r.source=csv_path;
    r.t=d.t_s-d.t_s(1); r.timestamp_s=d.timestamp_s;
    r.q_rad=d{:,cellstr(q_cols)}; r.q_deg=r.q_rad*180/pi;
    r.p_feedback=d{:,{'feedback_x_m','feedback_y_m','feedback_z_m'}};
    r.p_cpp=d{:,{'poe_x_m','poe_y_m','poe_z_m'}};
    r.rpy_feedback=d{:,{'feedback_roll_rad','feedback_pitch_rad','feedback_yaw_rad'}};
    r.span_ms=d.frame_span_s*1000;
    n=height(d); r.p_matlab=zeros(n,3); r.R_matlab=zeros(3,3,n);
    r.R_cpp=zeros(3,3,n); r.rotation_deg=zeros(n,1);
    r.cpp_rotation_deg=zeros(n,1); r.cpp_can_rotation_deg=zeros(n,1);
    R_rows=d{:,cellstr(r_cols)};
    for j=1:n
        % 从q独立计算MATLAB POE，不使用CSV中的POE值作为计算输入。
        T=forward_poe_space(model.S_list,r.q_rad(j,:)',model.M);
        Rc=reshape(R_rows(j,:),3,3)'; % CSV按行存储矩阵元素
        assert(norm(Rc'*Rc-eye(3),'fro')<1e-8 && abs(det(Rc)-1)<1e-8, ...
            '%s 第%d行C++旋转矩阵非法',r.name,j);
        a=r.rpy_feedback(j,:); Rf=rot_z(a(3))*rot_y(a(2))*rot_x(a(1));
        r.p_matlab(j,:)=T(1:3,4)'; r.R_matlab(:,:,j)=T(1:3,1:3); r.R_cpp(:,:,j)=Rc;
        r.rotation_deg(j)=rotation_angle(Rf'*T(1:3,1:3))*180/pi;
        r.cpp_rotation_deg(j)=rotation_angle(Rc'*T(1:3,1:3))*180/pi;
        r.cpp_can_rotation_deg(j)=rotation_angle(Rf'*Rc)*180/pi;
    end
    r.position_mm=vecnorm(r.p_matlab-r.p_feedback,2,2)*1000;
    r.signed_position_mm=(r.p_matlab-r.p_feedback)*1000;
    r.cpp_position_mm=vecnorm(r.p_matlab-r.p_cpp,2,2)*1000;
    assert(max(abs(vecnorm(r.p_cpp-r.p_feedback,2,2)-d.position_error_m))<1e-12, ...
        '%s CSV位置偏差列复核失败',r.name);
    assert(max(abs(r.cpp_can_rotation_deg*pi/180-d.rotation_error_rad))<1e-8, ...
        '%s CSV姿态偏差列复核失败',r.name);
    assert(max(r.cpp_position_mm)<cfg.cpp_position_tolerance_m*1000 && ...
        max(r.cpp_rotation_deg)*pi/180<cfg.cpp_rotation_tolerance_rad, ...
        '%s MATLAB与C++交叉验算超出容差',r.name);
    [r.phase,r.motion_bounds,r.activity_segments]=split_phases(r.t,r.q_deg,cfg);
    r.target_rad=targets_rad(k,:); endpoint=r.t>=r.t(end)-cfg.endpoint_window_s;
    r.endpoint_rad=mean(r.q_rad(endpoint,:),1);
    r.endpoint_range_deg=max(r.q_deg(endpoint,:),[],1)-min(r.q_deg(endpoint,:),[],1);
    r.endpoint_error_deg=(r.endpoint_rad-r.target_rad)*180/pi;
    dt=diff(r.t); r.median_dt_ms=median(dt)*1000; r.max_dt_ms=max(dt)*1000;
    r.large_gap_count=nnz(dt>3*median(dt)); r.over_window_count=nnz(d.frame_span_s>cfg.frame_window_s);
    r.limit_violation_count=nnz(any(r.q_rad<model.q_min'-pi/180*0.001 | ...
        r.q_rad>model.q_max'+pi/180*0.001,2));
    r.limit_violation_indices=find(any(r.q_rad<model.q_min'-pi/180*0.001 | ...
        r.q_rad>model.q_max'+pi/180*0.001,2));
    if r.activity_segments~=1 || r.large_gap_count>0 || r.over_window_count>0 || ...
            r.limit_violation_count>0 || any(r.endpoint_range_deg>cfg.static_range_deg)
        warning('%s：活动段%d，长采样间隔%d，超配对窗口%d，URDF边界越界样本%d，终态范围%.6f°', ...
            r.name,r.activity_segments,r.large_gap_count,r.over_window_count, ...
            r.limit_violation_count,max(r.endpoint_range_deg));
    end
    s=error_stats(r,true(n,1)); s.run=r.name; s.label=r.label;
    s.samples=n; s.duration_s=r.t(end); s.motion_start_s=r.motion_bounds(1);
    s.motion_end_s=r.motion_bounds(2); s.motion_duration_s=diff(r.motion_bounds);
    s.activity_segments=r.activity_segments; s.median_dt_ms=r.median_dt_ms;
    s.max_dt_ms=r.max_dt_ms; s.large_gap_count=r.large_gap_count;
    s.over_window_count=r.over_window_count; s.limit_violation_count=r.limit_violation_count;
    s.cpp_position_max_mm=max(r.cpp_position_mm); s.cpp_rotation_max_deg=max(r.cpp_rotation_deg);
    s.endpoint_max_abs_error_deg=max(abs(r.endpoint_error_deg));
    s.endpoint_max_range_deg=max(r.endpoint_range_deg);
    if k==1, run_rows=s; else, run_rows(k)=s; end
    phase_names=["pre_static","motion_buffered","post_static"];
    for p=1:3
        selected=r.phase==p; s_phase=error_stats(r,selected);
        s_phase.run=r.name; s_phase.phase=phase_names(p); s_phase.samples=nnz(selected);
        indices=find(selected);
        if isempty(indices)
            s_phase.start_s=NaN; s_phase.end_s=NaN; s_phase.duration_s=NaN;
        else
            s_phase.start_s=r.t(indices(1)); s_phase.end_s=r.t(indices(end));
            s_phase.duration_s=s_phase.end_s-s_phase.start_s;
        end
        s_phase.sufficient_static_baseline=p~=2 && ~isempty(indices) && s_phase.duration_s>=0.5;
        if isempty(phase_rows), phase_rows=s_phase; else, phase_rows(end+1)=s_phase; end %#ok<SAGROW>
    end
    results{k}=r;
    fprintf('%s：%d组，变化区间 %.3f～%.3fs，位置峰值 %.6fmm，姿态峰值 %.6f°\n', ...
        r.name,n,r.motion_bounds,max(r.position_mm),max(r.rotation_deg));
end
run_summary=struct2table(run_rows); phase_summary=struct2table(phase_rows);
assert(all(arrayfun(@(k) sum(phase_summary.samples(phase_summary.run==run_names(k))) ...
    ==run_summary.samples(k),1:5)),'阶段分配存在遗漏或重复');
if ~isfolder(analysis_dir), mkdir(analysis_dir); end
if ~isfolder(figure_dir), mkdir(figure_dir); end
writetable(run_summary,fullfile(analysis_dir,'run_summary.csv'));
writetable(phase_summary,fullfile(analysis_dir,'phase_summary.csv'));
matlab_version=version;
save(fullfile(analysis_dir,'analysis_results.mat'),'results','run_summary','phase_summary', ...
    'cfg','model','targets_rad','urdf_path','matlab_version');

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
    f=new_figure(r.name+" 六关节角度",cfg); tiledlayout(3,2,'TileSpacing','compact');
    for j=1:6
        nexttile; plot(r.t,r.q_deg(:,j),'LineWidth',1.1); grid on;
        yline(r.target_rad(j)*180/pi,'r:','终态目标','HandleVisibility','off');
        mark_phases(r); title(sprintf('J%d',j)); ylabel('角度 / °'); xlabel('相对时间 / s');
    end
    sgtitle(r.name+" "+r.label+"：虚线为缓冲运动阶段边界",'Interpreter','none');
    finish_figure(f,figure_dir,prefix+"六关节角度.png",cfg);
    f=new_figure(r.name+" 末端XYZ对比",cfg); tiledlayout(3,1,'TileSpacing','compact');
    axis_names={'X','Y','Z'};
    for j=1:3
        nexttile; h1=plot(r.t,r.p_feedback(:,j)*1000,'b-','LineWidth',1.1); hold on;
        h2=plot(r.t,r.p_cpp(:,j)*1000,'r--','LineWidth',1.1);
        h3=plot(r.t,r.p_matlab(:,j)*1000,'k:','LineWidth',1.1);
        mark_phases(r); grid on; ylabel([axis_names{j},' / mm']); xlabel('相对时间 / s');
        legend([h1,h2,h3],{'CAN反馈','C++ POE','MATLAB POE'},'Location','southeast');
    end
    sgtitle(r.name+"：C++与MATLAB曲线基本重合",'Interpreter','none');
    finish_figure(f,figure_dir,prefix+"末端XYZ对比.png",cfg);
    f=new_figure(r.name+" 模型偏差",cfg); tiledlayout(2,2,'TileSpacing','compact');
    for j=1:4
        nexttile;
        if j<=2, y=r.position_mm; unit='位置偏差 / mm'; else, y=r.rotation_deg; unit='姿态偏差 / °'; end
        plot(r.t,y,'LineWidth',1.1); mark_phases(r); grid on;
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
    nexttile; plot(r.t,r.span_ms,'LineWidth',0.8); mark_phases(r); grid on;
    yline(2,'r--','2ms窗口','HandleVisibility','off'); ylabel('六帧跨度 / ms'); xlabel('相对时间 / s');
    nexttile; h=plot(r.t,r.signed_position_mm,'LineWidth',1); mark_phases(r); grid on;
    ylabel('MATLAB POE - CAN / mm'); xlabel('相对时间 / s'); legend(h,{'dX','dY','dZ'},'Location','northeast');
    finish_figure(f,figure_dir,prefix+"帧跨度与位置分量.png",cfg);
end
disp(run_summary(:,{'run','samples','motion_start_s','motion_end_s', ...
    'position_max_mm','rotation_max_deg','cpp_position_max_mm','cpp_rotation_max_deg'}));
fprintf('完成：统计结果 %s\n图片 %s\n',analysis_dir,figure_dir);

%% 局部函数
function angle=rotation_angle(R)
% atan2避免acos(trace)在接近单位矩阵时丢失精度；不是相减RPY。
v=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
angle=atan2(norm(v),max(-1,min(1,(trace(R)-1)/2)));
end

function [phase,bounds,segments]=split_phases(t,q_deg,cfg)
changed=find(any(abs(diff(q_deg,1,1))>cfg.change_threshold_deg,2));
phase=ones(size(t)); bounds=[NaN,NaN]; segments=0;
if isempty(changed), return; end
bounds=[t(changed(1)),t(changed(end)+1)];
segments=1+nnz(diff(t(changed))>cfg.activity_gap_s);
phase(t>=bounds(1)-cfg.motion_padding_s & t<=bounds(2)+cfg.motion_padding_s)=2;
phase(t>bounds(2)+cfg.motion_padding_s)=3;
end

function s=error_stats(r,mask)
% 空阶段写NaN，不能写成“偏差为0”；峰值时间用该次采集相对时间。
names={'position_mean_mm','position_rms_mm','position_max_mm','position_peak_t_s', ...
    'rotation_mean_deg','rotation_rms_deg','rotation_max_deg','rotation_peak_t_s','frame_span_max_ms'};
for j=1:numel(names), s.(names{j})=NaN; end
if ~any(mask), return; end
indices=find(mask); p=r.position_mm(mask); a=r.rotation_deg(mask);
s.position_mean_mm=mean(p); s.position_rms_mm=sqrt(mean(p.^2));
[s.position_max_mm,ip]=max(p); s.position_peak_t_s=r.t(indices(ip));
s.rotation_mean_deg=mean(a); s.rotation_rms_deg=sqrt(mean(a.^2));
[s.rotation_max_deg,ia]=max(a); s.rotation_peak_t_s=r.t(indices(ia));
s.frame_span_max_ms=max(r.span_ms(mask));
end

function f=new_figure(name,cfg)
visibility='off'; if cfg.show_figures, visibility='on'; end
f=figure('Name',name,'Color','w','Visible',visibility,'Position',[60,60,1120,780]);
end

function mark_phases(r)
if any(isnan(r.motion_bounds)), return; end
ix=find(r.phase==2);
xline(r.t(ix(1)),'k--','HandleVisibility','off');
xline(r.t(ix(end)),'k--','HandleVisibility','off');
end

function finish_figure(f,folder,name,cfg)
set(findall(f,'-property','FontName'),'FontName',cfg.font);
set(findall(f,'Type','axes'),'FontSize',11);
drawnow;
exportgraphics(f,fullfile(folder,name),'Resolution',150);
if ~cfg.show_figures, close(f); end
end
