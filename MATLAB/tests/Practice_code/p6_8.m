clear;clc;close all;

J_func = @(q) deal(...
    [2*q(1), 0;
     0, 2*q(2)], ...
    [q(1)^2 - 4;
     q(2)^2 - 9] ...
);


q0        = [1;1];          % 初始迭代值 [x0;y0]
x_target  = zeros(2,1);    % 目标残差等于0
lambda_min= 1e-4;
lambda0   = 0.2;
w0        = 0.05;
q_min     = [-10;-10];
q_max     = [10;10];
max_iter  = 50;
tol       = 1e-6;

[q_sol, err_hist, lambda_hist] = dls_ik_adaptive(J_func,q0,x_target,lambda_min,lambda0,w0,q_min,q_max,max_iter,tol);

fprintf('求解结果 x = %.6f , y = %.6f\n', q_sol(1), q_sol(2));
fprintf('方程组残差 f1=%.6f, f2=%.6f\n', q_sol(1)^2-4, q_sol(2)^2-9);

figure('Color','w');
subplot(2,1,1);
plot(err_hist,'b-o');
xlabel('迭代步数');ylabel('||e||');
title('DLS迭代误差收敛曲线');grid on;

subplot(2,1,2);
plot(lambda_hist,'r-o');
xlabel('迭代步数');ylabel('\lambda');
title('自适应阻尼变化');grid on;