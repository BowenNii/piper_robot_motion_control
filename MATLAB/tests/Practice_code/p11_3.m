clear; clc; close all;
syms c1 c2 t real

% 系统参数
wn = 4;          % 自然频率
kx = 0.2;        % 阻尼比
wd = wn*sqrt(1-kx^2);  % 阻尼固有频率

% 位移表达式
theta = (c1*cos(wd*t) + c2*sin(wd*t)) * exp(-kx*wn*t);

% 求速度表达式
dtheta = diff(theta, t);

% 给定初始条件（这里使用示例：初始位移1，初始速度0）
theta0 = 1;
dtheta0 = 0;

% 用初始条件求 c1, c2
eq1 = subs(theta, t, 0) == theta0;
eq2 = subs(dtheta, t, 0) == dtheta0;
sol = solve([eq1, eq2], [c1, c2]);

% 提取数值解
c1_val = double(sol.c1);
c2_val = double(sol.c2);

% 将常数代入位移表达式，得到具体的符号表达式（只含 t）
theta_subs = subs(theta, {c1, c2}, {c1_val, c2_val});

% 将符号表达式转换为可数值计算的函数句柄
theta_func = matlabFunction(theta_subs, 'Vars', t);

% 定义时间向量
t_vec = linspace(0, 10, 500);  % 0到5秒，500个点

% 计算位移
theta_vals = theta_func(t_vec);

% 绘图
figure;
plot(t_vec, theta_vals, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('时间 t (s)');
ylabel('位移 \theta(t)');
title('阻尼振动响应');