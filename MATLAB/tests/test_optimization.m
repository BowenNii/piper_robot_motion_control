%TEST_OPTIMIZATION QP、关节轨迹约束与逐次凸化的离线测试。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
root=fullfile(fileparts(mfilename('fullpath')),'..','algorithm_library');
addpath(fullfile(root,'optimization'));

% min (x-1)^2+(y-2)^2，约束x+y<=2、x>=0、y>=0。
problem=struct('H',2*eye(2),'f',[-2;-4], ...
    'A',[1,1],'b',2,'lb',[0;0]);
qp=solve_qp(problem);
assert(qp.success && qp.max_violation<1e-8);
assert(norm(qp.x-[0.5;1.5])<1e-6);

% 轨迹：固定首末位置，检查位置、速度、加速度硬约束。
q_ref=[0,0.1,0.5,0.9,1,1,1; ...
       0,-0.1,-0.5,-0.9,-1,-1,-1];
limits=struct('q_min',[-1;-1],'q_max',[1;1], ...
    'dq_max',[0.5;0.5],'ddq_max',[1;1]);
trajectory=trajectory_optimizer(q_ref,0.5,limits);
assert(trajectory.success && trajectory.qp.max_violation<1e-7);
assert(norm(trajectory.q(:,1)-q_ref(:,1))<1e-8);
assert(norm(trajectory.q(:,end)-q_ref(:,end))<1e-8);
assert(all(abs(trajectory.dq(:))<=0.5+1e-8));
assert(all(abs(trajectory.ddq(:))<=1+1e-8));
assert(all(abs(trajectory.q(:))<=1+1e-8));

% 逐次凸化：min(x-2)^2，非线性约束x^2<=1，最优x=1。
problem=struct('H',2,'f',-4,'x0',0, ...
    'nonlinear_ineq',@(x) deal(x.^2-1,2*x));
solution=sequential_convexification(problem);
assert(solution.success && abs(solution.x-1)<1e-5);
assert(solution.max_violation<1e-7);

disp('All optimization tests passed.');
