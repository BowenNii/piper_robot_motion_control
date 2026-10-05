clear;clc;close all;
opengl software; 

L1 = 1; L2 = 1;

q1 = [-10; 20];
q1_rad= deg2rad(q1);

q2 = [60; 60];
q2_rad= deg2rad(q2);

q3 = [135; 90];
q3_rad= deg2rad(q3);

q4 = [190; 160];
q4_rad= deg2rad(q4);

q_set = {q1, q2, q3, q4};
q_rad_set = {q1_rad, q2_rad, q3_rad, q4_rad};

figure('Color','w','Position',[100,100,1000,800]);

for idx = 1:4
    q_deg = q_set{idx};
    q = q_rad_set{idx};
    th1 = q(1);
    th2 = q(2);

    c1 = cos(th1); s1 = sin(th1);
    c12 = cos(th1+th2); s12 = sin(th1+th2);

    % 平面2R雅可比 2×2
    J = [ -L1*s1 - L2*s12,   -L2*s12;
           L1*c1 + L2*c12,    L2*c12 ];

    % 操作度
    w = sqrt(det(J*J'));
    fprintf('第%d组：q1=%g°, q2=%g°  操作度 w=%.4f\n',...
        idx, q_deg(1), q_deg(2), w);

    % SVD分解椭球
    [U,S,~] = svd(J);
    sigma1 = S(1,1);
    sigma2 = S(2,2);

    % 末端位置
    x_end = L1*c1 + L2*c12;
    y_end = L1*s1 + L2*s12;

    %% 子图绘制
    subplot(2,2,idx); hold on; axis equal; grid on;
    xlim([-2.2,2.2]); ylim([-2.2,2.2]);

    % 机械臂连杆
    p0 = [0,0];
    p1 = [L1*c1, L1*s1];
    p2 = [x_end, y_end];
    plot([p0(1),p1(1),p2(1)],[p0(2),p1(2),p2(2)],'b-o','LineWidth',2,'MarkerSize',6);

    % 椭球轮廓点
    theta_ell = linspace(0,2*pi,200);
    ell_pts = U * S * [cos(theta_ell); sin(theta_ell)];
    xe = ell_pts(1,:) + x_end;
    ye = ell_pts(2,:) + y_end;
    plot(xe,ye,'r-','LineWidth',1.4);

    % 椭球主轴
    u1 = U(:,1)*sigma1;
    u2 = U(:,2)*sigma2;
    plot([x_end, x_end+u1(1)],[y_end, y_end+u1(2)],'g--','LineWidth',1.3);
    plot([x_end, x_end+u2(1)],[y_end, y_end+u2(2)],'m--','LineWidth',1.3);

    xlabel('x'); ylabel('y');
    title(sprintf('q1=%g°, q2=%g°, w=%.4f',q_deg(1),q_deg(2),w));
    hold off;
end
