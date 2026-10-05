clear;clc;close all;
L1 = 1; L2 = 1;

J_func = @(q) deal(...
    [ -L1*sin(q(1))-L2*sin(q(1)+q(2)),   -L2*sin(q(1)+q(2));
       L1*cos(q(1))+L2*cos(q(1)+q(2)),    L2*cos(q(1)+q(2)) ], ...
    [ L1*cos(q(1))+L2*cos(q(1)+q(2));
      L1*sin(q(1))+L2*sin(q(1)+q(2)) ]);

q0 = deg2rad([0;pi/6]);
x_target = [0.366;1.366];

lambda_min = 0.01;
lambda0 = 0.2;
w0 = 0.15;

% 关节限位：q1[-170°,170°], q2[-160°,160°]
q_min = deg2rad([-170; -160]);
q_max = deg2rad([170; 160]);

max_iter = 150;
tol = 1e-4;

[q_sol, err_hist, lambda_hist] = dls_ik_adaptive(J_func,q0,x_target,lambda_min,lambda0,w0,q_min,q_max,max_iter,tol);

fprintf('q1=%.2f ° , q2=%.2f °\n',rad2deg(q_sol(1)),rad2deg(q_sol(2)));
[~,x_final] = J_func(q_sol);
fprintf('末端 x=%.4f y=%.4f 误差||dx||=%.4e\n',x_final(1),x_final(2),norm(x_target-x_final));

%% 绘制收敛曲线
figure('Color','w');
subplot(2,1,1);
plot(err_hist,'b-o','LineWidth',1.2);
xlabel('迭代步数');
ylabel('位置误差 ||dx||');
title('自适应DLS收敛误差');
grid on;

subplot(2,1,2);
plot(lambda_hist,'r-o','LineWidth',1.2);
xlabel('迭代步数');
ylabel('阻尼系数 \lambda');
title('迭代过程自适应λ变化');
grid on;
