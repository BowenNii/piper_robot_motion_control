clear; clc; close all;
I = 0.1;       % 转动惯量 kg·m^2
b = 0.1;       % 粘性阻尼 N·m·s/rad
mgl = 1.0;     % 重力矩系数 N·m

% ===== 仿真参数 =====
dt = 0.001;    % 时间步长 1 ms
t_end = 5;     % 总仿真时长

% ===== 动力学函数：输入状态和力矩，输出角加速度 =====
dynamics = @(theta, thetadot, tau) ...
    (tau - b*thetadot - mgl*sin(theta)) / I;

% ===== 一阶欧拉积分器 =====
simulate = @(theta0, thetadot0, tau_func, t_end, dt) ...
    euler_integrator(theta0, thetadot0, tau_func, t_end, dt, dynamics);

function [t, theta, thetadot] = euler_integrator(theta0, thetadot0, tau_func, t_end, dt, dynamics)
    n = round(t_end/dt);%四舍五入取整
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

% ===== 测试用例1：初始θ=-π/2，恒定力矩0.5 N·m =====
tau_const1 = @(t, th, thd) 0.5;
[t1, th1, thd1] = simulate(-pi/2, 0, tau_const1, t_end, dt);

% ===== 测试用例2：初始θ=-π/4，力矩0 =====
tau_zero = @(t, th, thd) 0;
[t2, th2, thd2] = simulate(-pi/4, 0, tau_zero, t_end, dt);

% ===== 绘图 =====
figure;
plot(t1, th1, 'b-', 'LineWidth',1.5); hold on;
plot(t2, th2, 'r--', 'LineWidth',1.5);
grid on;
xlabel('时间 (s)');
ylabel('\theta (rad)');
legend('测试1: \tau=0.5 Nm, \theta_0=-\pi/2', ...
       '测试2: \tau=0, \theta_0=-\pi/4','Location','best');
title('开环仿真位置‑时间曲线');
