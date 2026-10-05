%% UR5e 运动轨迹可视化
clear; clc; close all;

%% 1. 加载机器人参数 + 求解目标位姿逆解
robot = ur5e_body_params();

% 目标关节角（生成目标位姿）
q_target = [0.2; -0.3; 0.5; -0.1; 0.4; 0.3];
T_target = fk_body_poe(q_target, robot);

% DLS逆解求解
q0 = zeros(6,1);
[q_sol, ~] = ik_dls_body(T_target, q0, robot, 'Tol', 1e-8);

%% 2. 生成关节插值轨迹【修正部分】
step_num = 100;                  % 动画总帧数
alpha = linspace(0, 1, step_num);% 0~1 插值系数
q_traj = q0 + (q_sol - q0) * alpha; % 6×step_num 关节轨迹

%% 3. 初始化绘图
figure('Color','w');
points0 = get_joint_points(q0, robot);
ax_lim = [-1 1 -1 1 -0.1 1.2]; % UR5e 适配坐标轴范围
h = plot_robot_frame(points0, ax_lim);

% 预分配末端轨迹存储
end_traj = zeros(3, step_num);
end_traj(:,1) = points0(:,end);
h.traj = plot3(end_traj(1,1), end_traj(2,1), end_traj(3,1), ...
               'm--', 'LineWidth', 1.5);

%% 4. 动画循环
for k = 2:step_num
    q_curr = q_traj(:,k);
    points_curr = get_joint_points(q_curr, robot);
    
    % 更新机器人姿态
    update_robot_frame(h, points_curr);
    
    % 更新末端轨迹
    end_traj(:,k) = points_curr(:,end);
    set(h.traj, 'XData', end_traj(1,1:k), ...
                'YData', end_traj(2,1:k), ...
                'ZData', end_traj(3,1:k));
    
    drawnow;
    pause(0.02); % 控制动画速度
end

%% 5. 最终标注
title('机器人运动轨迹（末端轨迹：紫色虚线）');
disp('动画播放完成。');