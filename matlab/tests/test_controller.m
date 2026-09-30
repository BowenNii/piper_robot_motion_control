% MATLAB控制函数的最小数值验证（不连接真机）。
matlab_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(matlab_root, 'algorithm_library', 'math'));
addpath(fullfile(matlab_root, 'algorithm_library', 'controller'));

v = (1:6)';
r = cartesian_velocity_controller(eye(6), v);
assert(strcmp(r.region, 'safe') && r.speed_scale == 1);
assert(norm(r.q_dot_command-v) < 1e-12);

r = cartesian_velocity_controller(diag([1 1 1 1 1 0.05]), v);
assert(strcmp(r.region, 'damped') && r.speed_scale > 0.1 && r.speed_scale < 1);
assert(all(isfinite(r.q_dot_command)));

r = cartesian_velocity_controller(diag([1 1 1 1 1 0]), v);
assert(strcmp(r.region, 'critical') && r.speed_scale == 0.1);
assert(all(isfinite(r.q_dot_command)));

q = zeros(6,1); q_des = ones(6,1); dq = q;
kp = 2*ones(6,1); kd = 3*ones(6,1);
assert(norm(joint_pd(q_des,q,q,dq,kp,kd)-2*ones(6,1)) < 1e-12);

id = @(q,dq,ddq) 4*q + 3*dq + 2*ddq;
assert(norm(gravity_compensation(q_des,id)-4*ones(6,1)) < 1e-12);
assert(norm(computed_torque(q_des,q,ones(6,1),q,dq,kp,kd,id) ...
            -6*ones(6,1)) < 1e-12);
assert(norm(joint_impedance(q_des,q,q,dq,kp,kd,ones(6,1)) ...
            -3*ones(6,1)) < 1e-12);

T = eye(4); T_des = T; T_des(1,4) = 0.1;
[tau,wrench] = cartesian_impedance(T_des,zeros(6,1),T,zeros(6,1), ...
                                   eye(6),eye(6),zeros(6));
expected = [0;0;0;0.1;0;0];
assert(norm(tau-expected) < 1e-12 && norm(wrench-expected) < 1e-12);

wrench_des = zeros(6,1); wrench_des(6) = 1;
[delta_pose, integral_next] = force_controller(wrench_des,zeros(6,1), ...
    0.01*eye(6),eye(6),zeros(6,1),0.1,0.05*ones(6,1));
assert(abs(integral_next(6)-0.05) < 1e-12);
assert(abs(delta_pose(6)-0.06) < 1e-12);

disp('All MATLAB controller tests passed.');
