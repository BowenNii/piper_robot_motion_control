clear;clc;close all;
syms theta2 real
M_sym = [8+6*sin(theta2)^2, 2*cos(theta2);
         2*cos(theta2),      6];
M_val = double(subs(M_sym,theta2,pi/4));

[V,D] = eig(M_val);
% eig输出：从小到大排序
lambda_small = D(1,1);
u_small = V(:,1);

lambda_large = D(2,2);
u_large = V(:,2);

%% 生成椭球边界
theta_ell = linspace(0,2*pi,200);
ddq_circle = [cos(theta_ell); sin(theta_ell)];
tau_ell = M_val * ddq_circle;

figure('Color','w');
plot(tau_ell(1,:), tau_ell(2,:),'b-','LineWidth',1.5); hold on;
axis equal; grid on;
xlabel('\tau_1'); ylabel('\tau_2');
title('关节空间力矩椭球 \theta_2=\pi/4');

% 长主轴（大特征值）
plot([0, lambda_large*u_large(1)], [0, lambda_large*u_large(2)],'r--','LineWidth',1.5);
% 短主轴（小特征值）
plot([0, lambda_small*u_small(1)],[0, lambda_small*u_small(2)],'g--','LineWidth',1.5);

legend({'力矩椭球','长主轴 \lambda_1','短主轴 \lambda_2'});
