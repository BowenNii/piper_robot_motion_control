function [r,joint_stats,timing_stats,execution_stats]=analyze_csv_action_run(run_dir,reference_csv)
%ANALYZE_CSV_ACTION_RUN 读取离线Action记录，核对误差列并统计关节偏差和时间间隔。
% 输入：run_dir含action_feedback.csv、joint_states.csv、summary.json；
% reference_csv是原始C++时间轨迹。只读输入，不连接ROS，不修改CSV。
% 输出：r保存绘图数据；三个table分别统计六轴误差、时间质量、执行结果。
% 关节单位rad；时间s。actual是GenericSystem模拟状态，不是真机测量。
% Action轨迹时钟与本程序单调接收时钟不同：用median(接收时刻-目标时间)
% 估计起点偏移；含反馈传输延迟，不是精确同步，不据此测量通信延迟。
% 时间统计的执行窗口采用上述估计；不插值/外推补齐Action首尾未记录样本。
run_dir=char(run_dir); reference_csv=char(reference_csv);
A=readtable(fullfile(run_dir,'action_feedback.csv'),'VariableNamingRule','preserve');
J=readtable(fullfile(run_dir,'joint_states.csv'),'VariableNamingRule','preserve');
C=readtable(reference_csv,'VariableNamingRule','preserve');
meta=jsondecode(fileread(fullfile(run_dir,'summary.json')));
desired=cellstr(compose('desired_q%d',1:6));
actual=cellstr(compose('actual_q%d',1:6));
errors=cellstr(compose('error_q%d',1:6));
joints=cellstr(compose('joint%d',1:6)); qcols=cellstr(compose('q%d_rad',1:6));
check_table(A,[{'receive_elapsed_s','desired_time_s'},desired,actual,errors]);
check_table(J,[{'receive_elapsed_s','header_stamp_s'},joints]);
check_table(C,[{'t_s'},qcols]);
assert(height(A)>=2 && height(J)>=2 && height(C)>=2,'每份记录至少需要两行');
assert(isfield(meta,'simulation_only') && isequal(meta.simulation_only,true), ...
    '本分析仅适用于simulation_only的离线模拟记录');
assert(isfield(meta,'duration_s') && isfinite(meta.duration_s) && meta.duration_s>0, ...
    'summary缺少有效轨迹时长');
assert(all(diff(A.receive_elapsed_s)>0) && all(diff(A.desired_time_s)>0), ...
    'Action接收时间/目标时间必须严格递增');
assert(all(diff(J.receive_elapsed_s)>0) && all(diff(J.header_stamp_s)>0), ...
    '关节状态接收时间/消息时间必须严格递增');
assert(C.t_s(1)==0 && all(diff(C.t_s)>0) && abs(C.t_s(end)-meta.duration_s)<1e-9, ...
    '原CSV的时间定义与执行时长不一致');
assert(isfield(meta,'sample_count') && height(C)==meta.sample_count,'原CSV点数与记录不一致');
assert(A.desired_time_s(1)>=0 && A.desired_time_s(end)<=meta.duration_s+0.1, ...
    'Action目标时间超出记录时长');

r=struct('A',A,'J',J,'C',C,'meta',meta,'run_dir',run_dir,'reference_csv',reference_csv);
r.q_desired=A{:,desired}; r.q_actual=A{:,actual};
r.error=r.q_desired-r.q_actual;
r.error_column_difference=max(abs(r.error-A{:,errors}),[],'all');
assert(r.error_column_difference<1e-12,'记录的error列与desired-actual不一致');
r.q_state=J{:,joints};
r.action_time=A.desired_time_s;
r.receive_offset_est_s=median(A.receive_elapsed_s-A.desired_time_s);
r.state_time_est=J.receive_elapsed_s-r.receive_offset_est_s;
r.dt_receive_ms=diff(J.receive_elapsed_s)*1000;
r.dt_header_ms=diff(J.header_stamp_s)*1000;
r.interval_time_est=r.state_time_est(2:end);
% 一个时间间隔只有两端都在执行窗口内，才纳入执行阶段统计。
r.execution_intervals=r.state_time_est(1:end-1)>=0 & r.state_time_est(2:end)<=meta.duration_s;
assert(any(r.execution_intervals),'执行窗口内没有有效关节状态间隔');
r.action_receive_dt_ms=diff(A.receive_elapsed_s)*1000;
r.action_desired_dt_ms=diff(A.desired_time_s)*1000;
[peak,index]=max(abs(r.error),[],1);
r.peak_time=r.action_time(index);
r.endpoint_error=r.q_state(end,:)-C{end,qcols};
joint_stats=table((1:6)',peak',sqrt(mean(r.error.^2,1))', ...
    r.peak_time(:),r.error(end,:)',r.endpoint_error', ...
    'VariableNames',{'joint','max_abs_error_rad','rms_error_rad','peak_target_time_s', ...
    'last_action_error_rad','final_state_minus_csv_goal_rad'});
% 20ms只作为本次状态记录的间隔筛查线（名义5ms的4倍），不是硬件安全阈值。
r.gap_screen_ms=20;
series={r.dt_receive_ms,r.dt_header_ms, ...
    r.dt_receive_ms(r.execution_intervals),r.dt_header_ms(r.execution_intervals), ...
    r.action_receive_dt_ms,r.action_desired_dt_ms};
stream=["state_receive_all";"state_header_all";"state_receive_execution_est"; ...
    "state_header_execution_est";"action_receive";"action_target_time"];
count=zeros(6,1); median_ms=count; max_ms=count; over_20ms_count=count;
for k=1:6
    v=series{k}; count(k)=numel(v); median_ms(k)=median(v); max_ms(k)=max(v);
    if k<=4, over_20ms_count(k)=sum(v>r.gap_screen_ms); else, over_20ms_count(k)=NaN; end
end
timing_stats=table(stream,count,median_ms,max_ms,over_20ms_count);
execution_stats=table(logical(meta.success),meta.duration_s,meta.action_elapsed_s, ...
    height(A),height(J),A.desired_time_s(1),A.desired_time_s(end), ...
    max(abs(r.endpoint_error)),r.receive_offset_est_s,r.error_column_difference, ...
    'VariableNames',{'action_success','planned_duration_s','action_elapsed_s', ...
    'action_feedback_rows','joint_state_rows','first_action_target_time_s', ...
    'last_action_target_time_s','final_state_error_max_rad', ...
    'receive_offset_est_s','error_column_recheck_max_rad'});
end

function check_table(T,required)
assert(all(ismember(required,T.Properties.VariableNames)),'CSV缺少必要列');
values=T{:,required};
assert(isnumeric(values) && all(isfinite(values),'all'),'CSV必要列必须为有限数值');
end
