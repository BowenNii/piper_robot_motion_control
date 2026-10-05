clear; clc; close all;
%% ========= 模型参数 =========
I = 0.1;       % kg·m^2
b = 0.1;       % N·m·s/rad
mgl = 1.0;     % N·m

%% ========= 仿真参数 =========
dt = 0.001;
t_end_b = 2;   % b问总仿真2秒
T_step = 1.0;  % 阶跃发生时刻1s
theta_d1 = -pi/2;
theta_d2 = 0;

Kp = 10;       % N·m/rad，题目指定
Kd = 2;        % N·m·s/rad，调优得到

% ========= 动力学匿名函数 =========
dynamics = @(theta, thetadot, tau) (tau - b*thetadot - mgl*sin(theta)) / I;

% ========= simulate接口，传入dynamics函数句柄 =========
%simulate(-pi/2, 0, tau_pd_func, t_end_b, dt);
simulate = @(theta0, thetadot0, tau_func, t_end, dt) ...
    euler_integrator(theta0, thetadot0, tau_func, t_end, dt, dynamics);

%% ========= 1.轨迹生成函数：输入t，输出θd,θddot_d,θdddot_d =========
function [theta_d, thetadot_d, thetaddot_d] = traj_gen(t, T_step, thd1, thd2)
    if t < T_step
        theta_d = thd1;
    else
        theta_d = thd2;
    end
    thetadot_d = 0;
    thetaddot_d = 0;
end

%% ========= 2.PD控制函数 =========
function tau = pd_controller(t, th, thd, Kp, Kd, T_step, thd1, thd2)
    [theta_d, thetadot_d, ~] = traj_gen(t, T_step, thd1, thd2);
    tau = Kp*(theta_d - th) + Kd*(thetadot_d - thd);
end

%% ========= 组装tau_func函数句柄 =========
tau_pd_func = @(t, th, thd) pd_controller(t, th, thd, Kp, Kd, T_step, theta_d1, theta_d2);

%% ========= 欧拉积分函数  =========
function [t, theta, thetadot] = euler_integrator(theta0, thetadot0, tau_func, t_end, dt, dynamics)
    n = round(t_end/dt);
    t = zeros(n+1,1);
    theta = zeros(n+1,1);
    thetadot = zeros(n+1,1);
    theta(1) = theta0;
    thetadot(1) = thetadot0;
    for k = 1:n
        tau = tau_func(t(k), theta(k), thetadot(k));
        thetaddot = dynamics(theta(k), thetadot(k), tau);
        theta(k+1) = theta(k) + thetadot(k)*dt;
        thetadot(k+1) = thetadot(k) + thetaddot*dt;
        t(k+1) = t(k)+dt;
    end
end

%% ========= 调用仿真器 =========
[t_b, th_b, thd_b] = simulate(-pi/2, 0, tau_pd_func, t_end_b, dt);

%% ========= 绘图 =========
figure;
plot(t_b, th_b, 'b-','LineWidth',1.5); hold on;
% 绘制期望轨迹参考线
theta_des = zeros(size(t_b));
for i = 1:length(t_b)
    if t_b(i) < T_step
        theta_des(i) = theta_d1;
    else
        theta_des(i) = theta_d2;
    end
end
plot(t_b, theta_des, 'r--','LineWidth',1.5);
grid on;
xlabel('时间 (s)');
ylabel('\theta (rad)');
legend('实际关节角度','期望阶跃轨迹','Location','best');
title('PD控制阶跃响应，K_p=10，K_d=2');

