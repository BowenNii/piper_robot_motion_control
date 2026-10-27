function [r,summary,points]=analyze_ik_trajectory(csv_path,urdf_path,cfg)
%ANALYZE_IK_TRAJECTORY 检查离线路径CSV，并独立重算MATLAB POE/增量/奇异值。
% 输入：C++导出的trajectory.csv、URDF路径、交叉验算容差cfg（可省略）。
% 输出：r绘图数据；summary单行统计table；points逐点复核table。
% 不修改输入、不删异常行、不写文件。兼容旧几何CSV和新增t_s的时间轨迹CSV。
% 这里验算已求得关节解，不重新运行MATLAB IK，也不是CAN/真机精度实验。
if nargin<3, cfg=struct; end
if ~isfield(cfg,'position_tolerance_m'), cfg.position_tolerance_m=1e-9; end
if ~isfield(cfg,'rotation_tolerance_rad'), cfg.rotation_tolerance_rad=1e-8; end
assert(isfile(csv_path),'找不到CSV：%s',csv_path);
D=readtable(csv_path,'VariableNamingRule','preserve');
r.path_mode=0;
if ismember('path_mode',D.Properties.VariableNames)
    assert(all(D.path_mode==D.path_mode(1)) && ismember(D.path_mode(1),[0,1]));
    r.path_mode=D.path_mode(1);
end
q_cols=cellstr(compose('q%d_rad',1:6));
delta_cols=cellstr(compose('delta_q%d_rad',1:6));
% 配置跟随C++数据保存，列出需要读取的实验配置名称，避免C++用一套参数、MATLAB却误用另一套参数。
settings={'position_tolerance_m','orientation_tolerance_rad','max_sample_step_rad', ...
    'rotation_weight','dls_lambda','ik_max_joint_step_rad','max_iterations','sample_count'};
r.timed=ismember('t_s',D.Properties.VariableNames);
timed_cols={};
if r.timed
    settings=[settings,{'duration_s','sample_period_s', ...
        'offline_velocity_limit_rad_s','offline_acceleration_limit_rad_s2'}];
    timed_cols=[{'t_s','ds_dt','d2s_dt2'},cellstr(compose('dq%d_rad_s',1:6)), ...
        cellstr(compose('ddq%d_rad_s2',1:6))];
end
required=[{'index','s','iterations','position_error_m','rotation_error_rad', ...
    'weighted_sigma_min','max_abs_delta_q_rad'},q_cols,delta_cols,settings];
for prefix={'target','poe'}
    required=[required,cellstr(compose('%s_%s_m',prefix{1},["x","y","z"]))]; %#ok<AGROW>
    for row=1:3
        for col=1:3, required{end+1}=sprintf('%s_r%d%d',prefix{1},row,col); end %#ok<AGROW>
    end
end
metadata_prefix={'q_min','q_max','q_start','q_goal'};
metadata_cols={};
for k=1:4
    metadata_cols=[metadata_cols,cellstr(compose('%s%d_rad',metadata_prefix{k},1:6))]; %#ok<AGROW>
end
required=[required,metadata_cols,timed_cols];
quality=check_feedback_data(D,required);
assert(quality.ok,'%s',strjoin(quality.messages,'；'));
N=height(D);
assert(N>=2 && isequal(D.index,(0:N-1)'),'index必须从0开始连续编号，至少两个点');
assert(all(diff(D.s)>0) && abs(D.s(1))<1e-12 && abs(D.s(end)-1)<1e-12, ...
    '路径s必须单调覆盖0～1');
if ~r.timed
    assert(max(abs(D.s-linspace(0,1,N)'))<1e-12,'旧几何路径必须均匀采样s');
end
fixed_cols=[settings,metadata_cols];%包含所有同一次求解中应该固定不变的列：
fixed_values=D{:,fixed_cols};
assert(all(all(fixed_values==fixed_values(1,:))),'同一CSV各行配置必须一致');
for k=1:numel(settings), r.config.(settings{k})=D.(settings{k})(1); end
c=r.config;
assert(c.sample_count==N && c.max_iterations>=0 && c.max_iterations==fix(c.max_iterations));
assert(c.position_tolerance_m>0 && c.orientation_tolerance_rad>0 ...
    && c.max_sample_step_rad>0 && c.rotation_weight>0 && c.dls_lambda>=0 ...
    && c.ik_max_joint_step_rad>0,'配置范围不合法');
assert(all(D.iterations>=0 & D.iterations==fix(D.iterations) ...
    & D.iterations<=c.max_iterations),'迭代次数不合法');
assert(all(D.position_error_m>=0 & D.rotation_error_rad>=0 ...
    & D.rotation_error_rad<=pi & D.weighted_sigma_min>=0 & D.max_abs_delta_q_rad>=0));

% 1. 读取关节解与模型，核对上下限，禁止以放宽限位掩盖不一致。
model=make_piper_model(urdf_path);
r.q_rad=D{:,q_cols}; r.q_deg=r.q_rad*180/pi; r.s=D.s; r.iterations=D.iterations;
if r.timed
    % 独立复算时间缩放和差分，不只信任CSV里已保存的导数。
    assert(N>=4 && c.duration_s>0 && c.sample_period_s>0 ...
        && c.offline_velocity_limit_rad_s>0 && c.offline_acceleration_limit_rad_s2>0);
    r.t=D.t_s; dt=c.sample_period_s;
    assert(max(abs(r.t-linspace(0,c.duration_s,N)'))<1e-12 ...
        && abs(dt-c.duration_s/(N-1))<1e-12,'时间必须等间隔覆盖0～T');
    u=r.t/c.duration_s;
    expected_s=10*u.^3-15*u.^4+6*u.^5;
    expected_ds=30*u.^2.*(1-u).^2/c.duration_s;
    expected_dds=60*u.*(1-u).*(1-2*u)/c.duration_s^2;
    assert(max(abs(r.s-expected_s))<1e-12 && max(abs(D.ds_dt-expected_ds))<1e-12 ...
        && max(abs(D.d2s_dt2-expected_dds))<1e-12,'五次时间缩放不一致');
    [r.dq_rad_s,r.ddq_rad_s2]=sample_derivatives(r.q_rad,dt);
    assert(max(abs(r.dq_rad_s-D{:,cellstr(compose('dq%d_rad_s',1:6))}),[],'all')<1e-10 ...
        && max(abs(r.ddq_rad_s2-D{:,cellstr(compose('ddq%d_rad_s2',1:6))}),[],'all')<1e-8, ...
        'CSV速度或加速度与关节角差分不一致');
    assert(max(abs(r.dq_rad_s),[],'all')<=c.offline_velocity_limit_rad_s ...
        && max(abs(r.ddq_rad_s2),[],'all')<=c.offline_acceleration_limit_rad_s2, ...
        '超过自设离线速度或加速度上限');
end
r.q_start=D{1,cellstr(compose('q_start%d_rad',1:6))};
r.q_goal=D{1,cellstr(compose('q_goal%d_rad',1:6))};
assert(max(abs(D{1,cellstr(compose('q_min%d_rad',1:6))}-model.q_min'))<1e-12 ...
    && max(abs(D{1,cellstr(compose('q_max%d_rad',1:6))}-model.q_max'))<1e-12, ...
    'C++与URDF限位不一致');%基准一致性校验：C++ 限位 vs URDF 限位
limits=check_joint_limits([r.q_start;r.q_goal;r.q_rad],model.q_min,model.q_max);%全轨迹关节越界检查
assert(limits.ok,'存在关节限位异常');

% 2. 复核每轴相邻增量。第一行按“初始解相对q_start”计算，默认是0。
continuity=check_joint_continuity([r.q_start;r.q_rad],c.max_sample_step_rad);
assert(continuity.ok,'相邻关节解超过增量阈值');
r.delta_q_rad=continuity.delta_rad;
r.max_abs_delta_q_rad=max(abs(r.delta_q_rad),[],2);
assert(max(abs(r.delta_q_rad-D{:,delta_cols}),[],'all')<1e-12, ...
    'CSV六轴delta_q与关节角不一致');
assert(max(abs(r.max_abs_delta_q_rad-D.max_abs_delta_q_rad))<1e-12);

% 3. 还原C++目标和FK位姿；通过关节解独立计算MATLAB POE。
r.T_target=read_pose(D,'target'); r.T_cpp=read_pose(D,'poe');
cross=validate_fk_samples(model.S_list,model.M,r.q_rad,r.T_cpp,cfg);
assert(cross.ok,'MATLAB/C++ POE交叉验算超出容差');
r.T_matlab=cross.T_matlab;
r.cross_position_m=cross.position_error_m;
r.cross_rotation_rad=cross.rotation_error_rad;
r.position_error_m=zeros(N,1); 
r.rotation_error_rad=zeros(N,1);
r.weighted_sigma_min=zeros(N,1);
T0=forward_poe_space(model.S_list,r.q_start',model.M);
T1=forward_poe_space(model.S_list,r.q_goal',model.M);
for k=1:N
    % 目标必须对应声明的SE(3)插值，不能把任意位姿序列当作本次路径。
    if r.path_mode==1
        expected_q=r.q_start+r.s(k)*(r.q_goal-r.q_start);
        assert(max(abs(r.q_rad(k,:)-expected_q))<1e-12,'关节五次路径不一致');
        expected=forward_poe_space(model.S_list,expected_q',model.M);
    else
        expected=interpolate_se3(T0,T1,r.s(k));
    end
    [p,a]=pose_error(expected,r.T_target(:,:,k));
    assert(p<cfg.position_tolerance_m && a<cfg.rotation_tolerance_rad, ...
        '目标路径与配置不一致：第%d行',k);
    [p,a]=pose_error(r.T_matlab(:,:,k),r.T_target(:,:,k));
    r.position_error_m(k)=p; r.rotation_error_rad(k)=a;
    A=adjoint_inverse(r.T_matlab(:,:,k))*jacobian_space(model.S_list,r.q_rad(k,:)');
    A(1:3,:)=A(1:3,:)*c.rotation_weight;
    singular_values=svd(A);
    r.weighted_sigma_min(k)=singular_values(end);
end
assert(all(r.position_error_m<=c.position_tolerance_m ...
    & r.rotation_error_rad<=c.orientation_tolerance_rad),'回代残差超出配置容差');
assert(max(abs(r.position_error_m-D.position_error_m))<1e-12 ...
    && max(abs(r.rotation_error_rad-D.rotation_error_rad))<1e-10, ...
    'CSV残差列与MATLAB重算结果不一致');
assert(max(abs(r.weighted_sigma_min-D.weighted_sigma_min))<1e-10, ...
    'CSV奇异值列与MATLAB计算不一致');

% 4. 汇总：奇异值只记录；时间轨迹另外汇总数值导数峰值。
summary=table(N,mean(r.position_error_m),sqrt(mean(r.position_error_m.^2)), ...
    max(r.position_error_m),mean(r.rotation_error_rad), ...
    sqrt(mean(r.rotation_error_rad.^2)),max(r.rotation_error_rad), ...
    max(r.max_abs_delta_q_rad),min(r.weighted_sigma_min), ...
    sum(r.iterations),max(r.iterations),max(r.cross_position_m),max(r.cross_rotation_rad), ...
    c.position_tolerance_m,c.orientation_tolerance_rad,c.max_sample_step_rad, ...
    'VariableNames',{'samples','position_mean_m','position_rms_m','position_max_m', ...
    'rotation_mean_rad','rotation_rms_rad','rotation_max_rad','max_abs_delta_q_rad', ...
    'min_weighted_sigma','total_iterations','max_iterations_per_sample', ...
    'cross_position_max_m','cross_rotation_max_rad','position_tolerance_m', ...
    'orientation_tolerance_rad','max_sample_step_rad'});
points=table(D.index,r.s,r.position_error_m,r.rotation_error_rad, ...
    r.max_abs_delta_q_rad,r.weighted_sigma_min,r.iterations, ...
    r.cross_position_m,r.cross_rotation_rad,'VariableNames', ...
    {'index','s','position_error_m','rotation_error_rad','max_abs_delta_q_rad', ...
    'weighted_sigma_min','iterations','cross_position_m','cross_rotation_rad'});
if r.timed
    summary.duration_s=c.duration_s;
    summary.sample_period_s=c.sample_period_s;
    summary.max_velocity_rad_s=max(abs(r.dq_rad_s),[],'all');
    summary.max_acceleration_rad_s2=max(abs(r.ddq_rad_s2),[],'all');
    summary.endpoint_velocity_max_rad_s=max(abs(r.dq_rad_s([1,end],:)),[],'all');
    summary.endpoint_acceleration_max_rad_s2=max(abs(r.ddq_rad_s2([1,end],:)),[],'all');
    points.t_s=r.t;
end
end

function [v,a]=sample_derivatives(q,dt)
% 内点中心差分；首尾为单边估计，不强制填0，便于发现数值误差。
v=zeros(size(q)); a=v;
v(2:end-1,:)=(q(3:end,:)-q(1:end-2,:))/(2*dt);
a(2:end-1,:)=(q(3:end,:)-2*q(2:end-1,:)+q(1:end-2,:))/dt^2;
v(1,:)=(-11*q(1,:)+18*q(2,:)-9*q(3,:)+2*q(4,:))/(6*dt);
v(end,:)=(11*q(end,:)-18*q(end-1,:)+9*q(end-2,:)-2*q(end-3,:))/(6*dt);
a(1,:)=(2*q(1,:)-5*q(2,:)+4*q(3,:)-q(4,:))/dt^2;
a(end,:)=(2*q(end,:)-5*q(end-1,:)+4*q(end-2,:)-q(end-3,:))/dt^2;
end

function T=read_pose(D,prefix)
% 复用通用位姿还原函数，旋转元素按行排列。
p=D{:,cellstr(compose('%s_%s_m',prefix,["x","y","z"]))};
cols=cell(1,9);
for row=1:3, for col=1:3, cols{(row-1)*3+col}=sprintf('%s_r%d%d',prefix,row,col); end, end
T=cpp_columns_to_transforms(p,D{:,cols});
end
