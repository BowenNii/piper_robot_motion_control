%TEST_EXPERIMENT_ANALYSIS 人工数据与边界测试；输出只放临时目录。
clear;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths;
folder=tempname; mkdir(folder);
N=3; t=[0;.01;.02];
D=table(t,100+t,ones(N,1)*.001, ...
    'VariableNames',{'t_s','timestamp_s','frame_span_s'});
for k=1:6, D.(sprintf('q%d_rad',k))=zeros(N,1); end
for name={'feedback_x_m','feedback_y_m','feedback_z_m','poe_x_m','poe_y_m','poe_z_m', ...
        'feedback_roll_rad','feedback_pitch_rad','feedback_yaw_rad','position_error_m','rotation_error_rad'}
    D.(name{1})=zeros(N,1);
end
for i=1:3
    for j=1:3, D.(sprintf('poe_r%d%d',i,j))=repmat(double(i==j),N,1); end
end
csv_path=fullfile(folder,'input.csv'); writetable(D,csv_path);
d=load_feedback_csv(csv_path,struct('require_transforms',true,'strict_time',true));
assert(isequal(d.q_rad,zeros(N,6)) && isequal(d.T_cpp,repmat(eye(4),1,1,N)));
s=summarize_static_pose(d,'pose01');
assert(s.sample_count==3 && s.position_max_mm==0 && s.q1_range_deg==0);

% 非对称旋转验证行/列顺序；RPY次序不能互换。
a=[.1,.2,.3]; p=[1,2,3]; T=feedback_rpy_to_transforms(p,a);
R=rot_z(.3)*rot_y(.2)*rot_x(.1);
assert(norm(T(1:3,1:3)-R,'fro')<1e-12 && isequal(T(1:3,4),p'));
Tc=cpp_columns_to_transforms(p,reshape(R',1,9));
assert(norm(T-Tc,'fro')<1e-12);
expect_error(@()cpp_columns_to_transforms(p,zeros(1,9)));
expect_error(@()feedback_rpy_to_transforms(p,[0,0]));

% 单位/输出保存与防覆盖；MAT变量名必须可追溯。
tables=struct('summary',struct2table(s)); variables=struct('summary',s,'cfg',struct('test',true));
paths=save_analysis_results(fullfile(folder,'out'),tables,variables);
assert(isfile(paths.summary) && isfile(paths.mat));
saved=load(paths.mat); assert(isequal(saved.summary,s) && saved.cfg.test);
expect_error(@()save_analysis_results(fullfile(folder,'out'),tables,variables));
save_analysis_results(fullfile(folder,'out'),tables,variables,struct('overwrite',true));
expect_error(@()save_analysis_results(folder,tables,variables,struct('mat_name','../bad.mat')));

% 全静态：无运动区间，空阶段不能返回0；样本总数不遗漏。
d.name="test"; d.label="全静态"; d.cpp_position_mm=zeros(N,1); d.cpp_rotation_deg=zeros(N,1);
model=struct('q_min',-ones(6,1),'q_max',ones(6,1));
cfg=struct('change_threshold_deg',.0005,'motion_padding_s',.05, ...
    'activity_gap_s',.2,'endpoint_window_s',1,'frame_window_s',.002, ...
    'static_range_deg',.002,'limit_tolerance_rad',0,'static_baseline_s',.5);
[r,s,ph]=summarize_dynamic_run(d,zeros(1,6),model,cfg);
assert(all(r.phase==1) && all(isnan(r.motion_bounds)) && isnan(s.motion_duration_s));
assert(sum([ph.samples])==N && isnan(ph(2).position_mean_mm));

% 运动贯穿全程：不能伪造前后静态基线；恰好检查用户原split_phases。
d.q_rad(:,1)=[0;.01;.02]; d.q_deg=d.q_rad*180/pi;
cfg.motion_padding_s=0;
[r,~,ph]=summarize_dynamic_run(d,zeros(1,6),model,cfg);
assert(all(r.phase==2) && all([ph.sufficient_static_baseline]==false));

% 缺列、NaN、时间倒退、严格模式重复时间、负跨度必须明确拒绝。
bad=D; bad.q1_rad(2)=NaN; writetable(bad,csv_path);
expect_error(@()load_feedback_csv(csv_path));
bad=removevars(D,'q1_rad'); writetable(bad,csv_path);
expect_error(@()load_feedback_csv(csv_path));
bad=D; bad.t_s=[0;.02;.01]; writetable(bad,csv_path);
expect_error(@()load_feedback_csv(csv_path));
bad=D; bad.t_s=[0;0;.02]; bad.timestamp_s=100+bad.t_s; writetable(bad,csv_path);
expect_error(@()load_feedback_csv(csv_path,struct('strict_time',true)));
bad=D; bad.frame_span_s(1)=-1; writetable(bad,csv_path);
expect_error(@()load_feedback_csv(csv_path));

fprintf('All experiment analysis tests passed. Temporary outputs: %s\n',folder);

function expect_error(action)
failed=false; try, action(); catch, failed=true; end
assert(failed,'无效输入应被拒绝');
end
