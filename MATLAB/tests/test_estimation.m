%TEST_ESTIMATION MATLAB estimation 算法库的确定性回归测试。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
library_dir = fullfile(fileparts(mfilename('fullpath')), '..', ...
    'algorithm_library', 'estimation');
addpath(library_dir);

dt = 0.005;
cutoff_hz = 10;
alpha = 1 - exp(-2*pi*cutoff_hz*dt);
assert(abs(low_pass_filter(1, 0, dt, cutoff_hz) - alpha) < 1e-12);
assert(norm(low_pass_filter([1; -2], [0; 0], dt, cutoff_hz) ...
    - alpha*[1; -2]) < 1e-12);

dq = estimate_joint_velocity([0.1; -0.2], [0; 0], 0.1);
assert(norm(dq - [1; -2]) < 1e-12);

% 一维测量：先验 x=0、P=1，测量 z=1、R=1，后验均值/方差均为 0.5。
[x, P, innovation, K] = kalman_filter(0, 1, 1, 1, 0, 1, 1);
assert(abs(x - 0.5) < 1e-12);
assert(abs(P - 0.5) < 1e-12);
assert(abs(innovation - 1) < 1e-12);
assert(abs(K - 0.5) < 1e-12);

% 测量缺失时仅预测，不进行测量更新。
[x, P, innovation, K] = kalman_filter(2, 3, [], 1, 0.2, [], []);
assert(abs(x - 2) < 1e-12 && abs(P - 3.2) < 1e-12);
assert(isempty(innovation) && isempty(K));

noise = struct('sigma_q', 0.01, 'sigma_dq', 0.1, 'sigma_acc', 1);
state0 = joint_state_estimator_step([0; 0], [1; 2], [], 0.1, noise);
assert(norm(state0.q) < 1e-12);
assert(norm(state0.dq - [1; 2]) < 1e-12);

state1 = joint_state_estimator_step([0.1; 0.2], [1; 2], ...
    state0, 0.1, noise);
assert(norm(state1.q - [0.1; 0.2]) < 1e-12);
assert(norm(state1.dq - [1; 2]) < 1e-12);
assert(isequal(size(state1.K), [4, 4]));

% 关节速度反馈不可用时，退化为仅用关节角测量更新。
state2 = joint_state_estimator_step([0.2; 0.4], [], ...
    state1, 0.1, noise);
assert(isequal(size(state2.K), [4, 2]));
assert(all(isfinite(state2.x)) && all(isfinite(state2.P(:))));
assert(norm(state2.P - state2.P', 'fro') < 1e-12);
assert(min(eig(state2.P)) > -1e-12);

disp('All estimation tests passed.');
