%TEST_VALIDATION 验证模块回归测试；人工构造输入，不改实验日志/CSV/图片。
clear;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths;

% 格式：缺列、非数值、非有限值、空数据均不能通过。
d=table([0;0.005],[0;1],'VariableNames',{'t_s','q1_rad'});
assert(check_feedback_data(d,{'t_s','q1_rad'}).ok);
assert(~check_feedback_data(d,{'missing'}).ok);
d.q1_rad(2)=NaN;
r=check_feedback_data(d,{'q1_rad'}); assert(isequal(r.invalid_rows,2));
assert(~check_feedback_data(d([],:),{'t_s'}).ok);
d.q1_rad={'a';'b'}; assert(~check_feedback_data(d,{'q1_rad'}).ok);
assert(~check_feedback_data(eye(3),{'t_s'}).ok);

% 时间：严格递增、间隔异常、负跨度和超窗口；输出原始行号。
r=check_sample_timing([0;.005;.01],ones(3,1)*.001);
assert(r.ok && abs(r.median_dt_s-.005)<1e-12);
r=check_sample_timing([0;0;.005;.1],[0;-.001;.003;0],struct('max_gap_s',.02));
assert(isequal(r.nonincreasing_rows,2) && isequal(r.gap_rows,4));
assert(isequal(r.span_rows,[2;3]));
assert(check_sample_timing(0,0).ok);
expect_error(@()check_sample_timing([0;NaN],[0;0]));

% 关节：边界允许，超限报告，不自动改关节角。
r=check_joint_limits([0,1;-.1,1.2],[0,-1],[1,1]);
assert(~r.ok && isequal(r.rows,[2;2]) && isequal(r.joints,[1;2]));
assert(norm(r.excess_rad-[.1;.2])<1e-12);
assert(check_joint_limits([0,1],[0,-1],[1,1]).ok);
assert(check_joint_limits(-.0001,0,1,.001).ok);
expect_error(@()check_joint_limits(0,1,-1));

% SE3/位姿偏差：平移、零角、pi角，以及非法反射矩阵。
assert(check_se3(eye(4)).ok);
reflection=diag([-1,1,1,1]); assert(~check_se3(reflection).ok);
bad=eye(4); bad(4,1)=.1; assert(~check_se3(bad).ok);
T=eye(4); T(1,4)=.1;
[p,a,dp]=pose_error(T,eye(4));
assert(abs(p-.1)<1e-12 && a==0 && norm(dp-[.1;0;0])<1e-12);
T=diag([1,-1,-1,1]); [p,a]=pose_error(T,eye(4));
assert(p==0 && abs(a-pi)<1e-12);
expect_error(@()pose_error(reflection,eye(4)));

% 一轴机器人：不依赖IK求解器返回标志，直接验证FK回代。
S=[0;0;1;0;0;0]; M=eye(4); M(1,4)=1;
target=forward_poe_space(S,.2,M);
r=check_ik_solution(S,M,.2,target,-1,1); assert(r.ok);
assert(~check_ik_solution(S,M,.3,target,-1,1).ok);
assert(~check_ik_solution(S,M,.2,target,-.1,.1).ok);
cfg=struct('position_tolerance_m',1e-10,'rotation_tolerance_rad',1e-10);
ref=cat(3,M,target);
r=validate_fk_samples(S,M,[0;.2],ref,cfg);
assert(r.ok && max(r.position_error_m)<1e-12);
ref(1,4,2)=ref(1,4,2)+.01;
assert(~validate_fk_samples(S,M,[0;.2],ref,cfg).ok);
assert(check_target_reached(target,target,1e-6,1e-6).reached);
assert(~check_target_reached(M,target,1e-6,1e-6).reached);

% 连续性/奇异性：2pi不自动wrap，欠驱动任务明确报告。
r=check_joint_continuity([0,0;.1,.1;.2,2*pi],.5);
assert(isequal(r.rows,3) && isequal(r.joints,2));
assert(check_joint_continuity([0,0],.1).ok);
r=check_jacobian_condition(diag([2,1]),.01);
assert(~r.near_singular && r.condition_number==2);
assert(check_jacobian_condition(diag([1,0]),.01).near_singular);
r=check_jacobian_condition([1;0],.01);
assert(r.near_singular && ~r.task_full_row_rank);

% 静态窗口与统计：单点不证明稳定；空选区不是零误差。
assert(check_static_stability(zeros(2,6),zeros(2,3),0,0).stable);
assert(~check_static_stability(zeros(1,6),zeros(1,3),0,0).stable);
assert(~check_static_stability([0;1],zeros(2,3),.1,0).stable);
s=summarize_errors([1;2;3],[true;false;true]);
assert(s.count==2 && s.mean==2 && abs(s.rms-sqrt(5))<1e-12 && s.max_row==3);
s=summarize_errors([1;2],false(2,1)); assert(s.count==0 && isnan(s.mean));
expect_error(@()summarize_errors([1;NaN]));
expect_error(@()summarize_errors([1;2],[1;0]));
disp('All validation tests passed.');

function expect_error(action)
failed=false;
try, action(); catch, failed=true; end
assert(failed,'无效输入应被拒绝');
end
