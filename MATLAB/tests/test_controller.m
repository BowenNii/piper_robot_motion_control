%TEST_CONTROLLER 控制律、奇异分区和一步关节安全约束测试。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
root=fullfile(fileparts(mfilename('fullpath')),'..','algorithm_library');
addpath(fullfile(root,'math'),fullfile(root,'control'));
tau=joint_pd([1;2],[0;0],[0;1],[0;0],[2;3],[1;1]);
assert(norm(tau-[2;3])<1e-12);
id=@(q,dq,ddq) ddq+2*q+dq;
tau=computed_torque([1;1],[0;0],[0;0], ...
    [0;0],[0;0],[2;2],[0;0],id);
assert(norm(tau-[2;2])<1e-12);
assert(norm(gravity_compensation([1;2],id)-[2;4])<1e-12);
assert(norm(joint_impedance([1;1],[0;0],[0;0],[0;0], ...
    [2;2],[1;1],[3;3])-[5;5])<1e-12);

T=eye(4); T_des=T; T_des(1,4)=0.1;
[tau,wrench]=cartesian_impedance(T_des,zeros(6,1),T, ...
    zeros(6,1),eye(6),eye(6),zeros(6));
assert(norm(tau-wrench)<1e-12 && abs(tau(4)-0.1)<1e-12);
[delta,integral]=force_controller(ones(6,1),zeros(6,1), ...
    eye(6),eye(6),zeros(6,1),0.1,0.05*ones(6,1));
assert(norm(integral-0.05*ones(6,1))<1e-12);
assert(norm(delta-1.05*ones(6,1))<1e-12);

config=struct('q_dot_max',0.5,'q_current',zeros(6,1), ...
    'q_min',-ones(6,1),'q_max',ones(6,1),'dt',0.1, ...
    'q_dot_previous',zeros(6,1),'q_ddot_max',2);
r=cartesian_velocity_controller(eye(6),ones(6,1),config);
assert(strcmp(r.region,'safe') && r.constraint_active);
assert(norm(r.q_dot_command-0.2*ones(6,1))<1e-12);
assert(norm(r.v_achieved-r.q_dot_command)<1e-12);
config.q_current=0.99*ones(6,1);
r=cartesian_velocity_controller(eye(6),ones(6,1),config);
assert(all(r.q_dot_command<=0.1+1e-12));
critical=cartesian_velocity_controller(diag([1,1,1,1,1,0]), ...
    ones(6,1));
assert(strcmp(critical.region,'critical') && ...
    all(isfinite(critical.q_dot_command)));
config.q_current=ones(6,1);
config.q_dot_previous=ones(6,1);
config.q_ddot_max=0.1;
caught=false;
try
    cartesian_velocity_controller(eye(6),ones(6,1),config);
catch
    caught=true;
end
assert(caught,'互相冲突的安全约束必须明确报错');
disp('All controller tests passed.');
