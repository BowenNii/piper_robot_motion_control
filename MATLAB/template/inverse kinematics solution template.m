%%物体POE + DLS逆解
clear; clc; close all;

%% 1. 加载机器人参数
robot = newrobot_body_params();

%% 2. 构造目标位姿（此处用正解生成，实际使用可直接赋值目标位姿）
q_true = [pi/6; pi/2];
T_target = fk_body_poe(q_true, robot);

%% 3. 逆运动学求解
q0 = zeros(2,1);  % 零位初始猜测

% 基础调用（使用全部默认参数）
% [q_sol, info] = ik_dls_body(T_target, q0, robot);

% 自定义参数调用
[q_sol, info] = ik_dls_body(T_target, q0, robot, ...
    'Lambda', 0.05, ...
    'MaxIter', 3000, ...
    'Tol', 1e-8, ...
    'Step', 0.1, ...
    'WeightPos', 10);

%% 4. 结果验证
T_sol = fk_body_poe(q_sol, robot);
[pos_err, rot_err, total_err] = calc_pose_error(T_sol, T_target);

%% 5. 结果打印
fprintf('==================== 关节角结果 ====================\n');
fprintf('真实关节角 (rad):\n'); disp(q_true');
fprintf('求解关节角 (rad):\n'); disp(q_sol');

fprintf('\n==================== 位姿误差 ====================\n');
fprintf('位置误差: %.10f m\n', pos_err);
fprintf('旋转误差: %.10f rad\n', rot_err);
fprintf('总旋量误差: %.10f\n', total_err);

fprintf('\n==================== 求解状态 ====================\n');
fprintf('迭代次数: %d\n', info.iterations);
fprintf('是否收敛: %s\n', string(info.converged));
fprintf('最终残差: %.10f\n', info.final_error);