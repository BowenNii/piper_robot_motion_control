function [r,joint_stats,pose_stats,timing_stats]=analyze_real_ik_run(run_dir,execution_dir,urdf_path,cfg)
%ANALYZE_REAL_IK_RUN 真机MOVE J记录：关节跟随、末端FK及终点稳定性。
% 输入：run_dir含trajectory.csv/manifest.json；execution_dir含commands、feedback、
% status.csv和summary.json；urdf_path为模型。只读输入，不连接ROS。
% cfg.tail_window_s默认0.5；cfg.legacy_tail_s默认2（旧工具终点观察时长）。
% 新日志按保存的单调时钟原点对齐。旧日志仅用记录尾部倒推起点，含未知
% 清理/记录延迟；不按关节曲线拟合时移，不将估计误差解释成真实跟踪延迟。
% 终点统计不依赖起点时间估计；末端全部由编码器+URDF推算，非外部测量。
if nargin<4, cfg=struct; end
if ~isfield(cfg,'tail_window_s'), cfg.tail_window_s=0.5; end
if ~isfield(cfg,'legacy_tail_s'), cfg.legacy_tail_s=2; end
validateattributes(cfg.tail_window_s,{'numeric'},{'scalar','positive','finite'});
validateattributes(cfg.legacy_tail_s,{'numeric'},{'scalar','nonnegative','finite'});
meta=jsondecode(fileread(fullfile(run_dir,'manifest.json')));
execution=jsondecode(fileread(fullfile(execution_dir,'summary.json')));
assert(any(strcmp(meta.kind,{'real_move_j_relative_ik_v1','real_move_j_endpoint_ik_v1'})), ...
    '不是本工具的真机IK实验');
assert(isfield(execution,'simulation_only') && isequal(execution.simulation_only,false), ...
    '这里仅分析真机记录，不能混用离线Action日志');
assert(execution.success && execution.sent_points==meta.sample_count, ...
    '本脚本用于完整成功记录；中断记录需单独分析，不能按完整轨迹处理');
assert(isfield(execution.cleanup,'gate_close') && ~isfield(execution.cleanup,'gate_close_error'), ...
    '记录中没有确认门控关闭');
[plan,plan_stats]=analyze_ik_trajectory(fullfile(run_dir,'trajectory.csv'),urdf_path);
J=readtable(fullfile(execution_dir,'feedback.csv'),'VariableNamingRule','preserve');
C=readtable(fullfile(execution_dir,'commands.csv'),'VariableNamingRule','preserve');
S=readtable(fullfile(execution_dir,'status.csv'),'VariableNamingRule','preserve');
qcols=cellstr(compose('q%d_rad',1:6));
check_numeric(J,[{'receive_elapsed_s','header_stamp_s'},qcols]);
check_numeric(C,[{'index','target_time_s','send_elapsed_s','lateness_s'},qcols]);
check_numeric(S,{'receive_elapsed_s','ctrl_mode','arm_status','mode_feedback','motion_status','err_status'});
assert(height(J)>=5 && height(S)>=2 && height(C)==meta.sample_count);
assert(all(diff(J.receive_elapsed_s)>0) && all(diff(J.header_stamp_s)>=0), ...
    '反馈接收时间必须递增、消息时间戳不得回退（重复时间戳单独统计）');
assert(all(diff(S.receive_elapsed_s)>0) && all(diff(C.send_elapsed_s)>0));
assert(isequal(C.index,(0:height(C)-1)') && ...
    max(abs(C.target_time_s-plan.t))<1e-12,'发送点编号/目标时间与计划不一致');
assert(max(abs(C{:,qcols}-plan.q_rad),[],'all')<1e-12,'实发指令关节角与计划CSV不一致');
assert(all(C.lateness_s>=0) && max(C.lateness_s)<.02,'发送记录存在跨周期迟到');
assert(max(abs(J{end,qcols}-execution.q_final_rad(:)'))<1e-12,'最终反馈与summary不一致');
assert(max(abs(plan.q_start-meta.q_start_rad(:)'))<1e-12 && ...
    max(abs(plan.q_rad(end,:)-meta.q_goal_rad(:)'))<1e-12,'清单与CSV实际首末关节解不一致');

% 1. 时间对齐：绝不直接将不同采样频率的两份CSV按行相减。
if isfield(execution,'trajectory_start_receive_elapsed_s')
    offset=execution.trajectory_start_receive_elapsed_s;
    assert(isscalar(offset) && isfinite(offset));
    alignment="recorded_monotonic_origin";
    note="已记录同机单调时钟原点；接收时刻比较仍包含传输/排队影响";
else
    offset=J.receive_elapsed_s(end)-execution.command_elapsed_s-cfg.legacy_tail_s;
    alignment="legacy_tail_estimate";
    note=sprintf('旧记录按尾部减%.6g秒发送和%.6g秒观察估计；含未知清理延迟，不能测跟踪/通信延迟', ...
        execution.command_elapsed_s,cfg.legacy_tail_s);
    warning('piper:EstimatedAlignment','%s',note);
end
r=struct('plan',plan,'plan_stats',plan_stats,'meta',meta,'execution',execution, ...
    'J',J,'C',C,'S',S,'alignment',alignment,'alignment_note',note,'offset_s',offset);
r.t_feedback=J.receive_elapsed_s-offset;
r.t_status=S.receive_elapsed_s-offset;
r.q_feedback=J{:,qcols};
assert(r.t_feedback(1)<0 && r.t_feedback(end)>plan.t(end),'反馈没有覆盖运动前后');
% 比较最近已发布的位置目标（零阶保持），不是假定驱动跟踪原CSV的连续插值。
query=max(C.send_elapsed_s(1),min(C.send_elapsed_s(end),r.t_feedback));
r.q_sent_at_feedback=interp1(C.send_elapsed_s,C{:,qcols},query,'previous');
r.joint_error=r.q_feedback-r.q_sent_at_feedback;
r.execution_mask=r.t_feedback>=0 & r.t_feedback<=plan.t(end);
assert(any(r.execution_mask));
tail=J.receive_elapsed_s>=J.receive_elapsed_s(end)-cfg.tail_window_s;
assert(sum(tail)>=5,'终点窗口样本不足');
r.tail_mask=tail;
% 初始窗口取实际记录开头0.5秒，必须在估计/已知发送开始前。
initial=J.receive_elapsed_s<=J.receive_elapsed_s(1)+cfg.tail_window_s;
assert(sum(initial)>=5 && all(r.t_feedback(initial)<0),'起点静止窗口不足');
r.initial_mask=initial;
r.q_initial_mean=mean(r.q_feedback(initial,:),1);
r.q_final_mean=mean(r.q_feedback(tail,:),1);
r.final_joint_range=max(r.q_feedback(tail,:),[],1)-min(r.q_feedback(tail,:),[],1);
assert(max(max(r.q_feedback(initial,:),[],1)-min(r.q_feedback(initial,:),[],1))<=.001, ...
    '初始位置窗口不稳定');
assert(max(abs(r.q_initial_mean-plan.q_start))<=.001,'初始反馈与计划起点差过大');
errors=r.joint_error(r.execution_mask,:);
[peak,idx]=max(abs(errors),[],1); times=r.t_feedback(r.execution_mask);
joint_stats=table((1:6)',peak',sqrt(mean(errors.^2,1))',times(idx), ...
    (r.q_final_mean-plan.q_rad(end,:))',r.final_joint_range', ...
    (r.q_feedback(end,:)-plan.q_rad(end,:))', ...
    'VariableNames',{'joint','peak_abs_error_rad_aligned','rms_error_rad_aligned', ...
    'peak_time_s_aligned','final_mean_minus_goal_rad','final_window_range_rad','last_minus_goal_rad'});
joint_stats.time_alignment=repmat(alignment,6,1);
joint_stats.observed_unique_positions=zeros(6,1);
joint_stats.max_adjacent_feedback_change_rad=max(abs(diff(r.q_feedback)),[],1)';
for j=1:6, joint_stats.observed_unique_positions(j)=numel(unique(r.q_feedback(:,j))); end

% 2. 同一个POE模型回算计划与反馈，位置统一相对于计划起点，保留初始偏差。
model=make_piper_model(urdf_path); N=height(J);
if strcmp(meta.kind,'real_move_j_endpoint_ik_v1')
    desired=forward_poe_space(model.S_list,meta.q_reference_rad(:),model.M);
    [ep,er]=pose_error(plan.T_target(:,:,end),desired);
    assert(plan.path_mode==1 && ep<1e-9 && er<1e-8,'终点IK位姿与用户参考目标不一致');
end
r.T_feedback=zeros(4,4,N); r.p_feedback=zeros(N,3);
r.p_error_mm=zeros(N,1);r.rotation_error_deg=zeros(N,1);r.orientation_drift_deg=zeros(N,1);
r.feedback_orientation_change_deg=zeros(N,1);
r.plan_orientation_change_deg=zeros(numel(plan.t),1);
for k=1:numel(plan.t)
    [~,a]=pose_error(plan.T_target(:,:,k),plan.T_target(:,:,1));
    r.plan_orientation_change_deg(k)=a*180/pi;
end
T_initial=forward_poe_space(model.S_list,r.q_initial_mean',model.M);
T_final=forward_poe_space(model.S_list,r.q_final_mean',model.M);
for k=1:N
    T=forward_poe_space(model.S_list,r.q_feedback(k,:)',model.M);
    r.T_feedback(:,:,k)=T;r.p_feedback(k,:)=T(1:3,4)';
    target=forward_poe_space(model.S_list,r.q_sent_at_feedback(k,:)',model.M);
    [p,a]=pose_error(T,target);r.p_error_mm(k)=p*1000;r.rotation_error_deg(k)=a*180/pi;
    [~,a]=pose_error(T,T_initial);r.orientation_drift_deg(k)=a*180/pi;
    [~,a]=pose_error(T,plan.T_target(:,:,1));r.feedback_orientation_change_deg(k)=a*180/pi;
end
p0=plan.T_target(1:3,4,1)';
r.plan_delta_mm=(reshape(plan.T_target(1:3,4,:),3,[])'-p0)*1000;
r.feedback_delta_mm=(r.p_feedback-p0)*1000;
r.actual_displacement_mm=(T_final(1:3,4)-T_initial(1:3,4))'*1000;
r.planned_displacement_mm=(plan.T_target(1:3,4,end)-plan.T_target(1:3,4,1))'*1000;
[final_p,final_a]=pose_error(T_final,plan.T_target(:,:,end));
[~,drift]=pose_error(T_final,T_initial);
final_range_mm=(max(r.p_feedback(tail,:),[],1)-min(r.p_feedback(tail,:),[],1))*1000;
pose_stats=table(r.planned_displacement_mm(1),r.planned_displacement_mm(2),r.planned_displacement_mm(3), ...
    r.actual_displacement_mm(1),r.actual_displacement_mm(2),r.actual_displacement_mm(3), ...
    final_p*1000,final_a*180/pi,drift*180/pi,final_range_mm(1),final_range_mm(2),final_range_mm(3), ...
    max(r.p_error_mm(r.execution_mask)),max(r.rotation_error_deg(r.execution_mask)), ...
    'VariableNames',{'planned_dx_mm','planned_dy_mm','planned_dz_mm','feedback_fk_dx_mm', ...
    'feedback_fk_dy_mm','feedback_fk_dz_mm','endpoint_position_error_mm','endpoint_orientation_error_deg', ...
    'endpoint_orientation_drift_deg','tail_x_range_mm','tail_y_range_mm','tail_z_range_mm', ...
    'peak_position_error_mm_aligned','peak_orientation_error_deg_aligned'});
pose_stats.time_alignment=alignment;

% 3. 记录质量与状态单独统计，不将消息间隔叫作CAN六帧跨度。
r.dt_receive_ms=diff(J.receive_elapsed_s)*1000;
r.dt_header_ms=diff(J.header_stamp_s)*1000;
r.interval_time=r.t_feedback(2:end);
inside=r.execution_mask(1:end-1)&r.execution_mask(2:end);
series={r.dt_receive_ms,r.dt_header_ms,r.dt_receive_ms(inside),r.dt_header_ms(inside), ...
    diff(C.send_elapsed_s)*1000,C.lateness_s*1000};
stream=["feedback_receive_all";"feedback_header_all";"feedback_receive_motion_aligned"; ...
    "feedback_header_motion_aligned";"command_send_interval";"command_lateness"];
count=zeros(6,1);median_ms=count;max_ms=count;over_20ms_count=count;
for k=1:6
    x=series{k};assert(~isempty(x)); count(k)=numel(x);median_ms(k)=median(x);
    max_ms(k)=max(x);over_20ms_count(k)=sum(x>20);
end
timing_stats=table(stream,count,median_ms,max_ms,over_20ms_count);
angle_limit=read_flag(S,'any_angle_limit'); comm_error=read_flag(S,'any_communication_error');
r.bad_status=S.ctrl_mode~=1|S.mode_feedback~=1|S.arm_status~=0|S.err_status~=0|angle_limit|comm_error;
r.status_bad_count=sum(r.bad_status);
r.header_repeat_count=sum(diff(J.header_stamp_s)==0);
r.final_joint_max_rad=max(abs(r.q_final_mean-plan.q_rad(end,:)));
r.endpoint_stable=max(r.final_joint_range)<=.001;
r.final_last_error_recheck=abs(max(abs(joint_stats.last_minus_goal_rad))-execution.final_joint_error_max_rad);
assert(r.final_last_error_recheck<1e-12,'summary终点关节误差复核失败');
end

function check_numeric(T,cols)
assert(all(ismember(cols,T.Properties.VariableNames)),'CSV缺少必要列');
values=T{:,cols};
assert(isnumeric(values) && all(isfinite(values),'all'),'CSV必要数值列包含无效数据');
end

function b=read_flag(T,name)
assert(ismember(name,T.Properties.VariableNames),'状态CSV缺少布尔列');
v=T.(name);
if islogical(v), b=v;
elseif isnumeric(v), assert(all(v==0|v==1)); b=logical(v);
else
    v=lower(string(v));assert(all(v=="true"|v=="false"),'状态布尔列无效');b=v=="true";
end
end
