clear;clc;close all;
syms theta1 theta2 thetad1 thetad2 thetadd1 thetadd2 real
syms L1 L2 m1 m2 g real

% ========== 1.参数赋值 ==========
L1_val = 1;
L2_val = 1;
m1_val = 2;
m2_val = 2;
g_val  = 10;

% ========== 2.体雅可比 Jb1 Jb2 ==========
%连杆1 体雅可比 Jb1 (6×1)
Jb1 = [0;
       0;
       1;
       0;
       L1;
       0];

%连杆2 体雅可比 Jb2 (6×2)
Jb2 = [0,          1;
       sin(theta2),0;
       cos(theta2),0;
      -L2*sin(theta2),0;
       L1*cos(theta2),L2;
      -L1*sin(theta2),0];

%连杆、关节速度
thetad = [thetad1; thetad2];
Vb1 = Jb1 * thetad1;
Vb2 = Jb2 * thetad;

% ========== 3.本体坐标系惯量矩阵 Gb1 Gb2 ==========
I1 = [4,0,0;
      0,4,0;
      0,0,0];
I2 = [0,0,0;
      0,4,0;
      0,0,4];

Gb1 = [I1, zeros(3,3);
       zeros(3,3), m1*eye(3)];

Gb2 = [I2, zeros(3,3);
       zeros(3,3), m2*eye(3)];

% ==========4.总动能 K ==========
K1 = 1/2 * Vb1.' * Gb1 * Vb1;
K2 = 1/2 * Vb2.' * Gb2 * Vb2;
K  = simplify(K1 + K2)

% ==========5.势能P ==========
P = m2*g*L2*(1 - cos(theta2));

% ==========6.拉格朗日 L = K‑P ==========
Lagr = simplify(K - P);

% ==========7.欧拉‑拉格朗日方程 ==========
% ∂L/∂thetad
dL_dthetad = [diff(Lagr,thetad1);
              diff(Lagr,thetad2)];

% d/dt (∂L/∂thetad)
dLdt = diff(dL_dthetad,theta1)*thetad1 ...
     + diff(dL_dthetad,theta2)*thetad2 ...
     + diff(dL_dthetad,thetad1)*thetadd1 ...
     + diff(dL_dthetad,thetad2)*thetadd2;

% ∂L/∂theta
dL_dtheta = [diff(Lagr,theta1);
             diff(Lagr,theta2)];

tau = simplify(dLdt - dL_dtheta)

% ==========代入数值参数 ==========
tau_num = subs(tau,{L1,L2,m1,m2,g},{L1_val,L2_val,m1_val,m2_val,g_val});

%% ======== (a)代入 theta1=pi/4 theta2=pi/4，所有速度加速度为0 ========
tau_a = subs(tau_num,...
    {theta1,theta2,thetad1,thetad2,thetadd1,thetadd2},...
    {pi/4,pi/4,0,0,0,0});
disp('===== 8.4(a)结果 tau =====');
double(tau_a)

%% ======== (b)提取质量矩阵M(q) ========
%质量矩阵：只保留角加速度thetadd1,thetadd2项，其余置0
M_col1 = subs(tau_num,{thetadd1,thetadd2,thetad1,thetad2},{1,0,0,0});
M_col2 = subs(tau_num,{thetadd1,thetadd2,thetad1,thetad2},{0,1,0,0});
M = simplify([M_col1,M_col2])

%代入 theta2=pi/4
M_eval = subs(M,theta2,pi/4);
disp('=====8.4(b) M矩阵(theta2=pi/4)=====');
double(M_eval)

%% ======== 绘图：固定theta1=pi/4，theta2扫描[-pi,pi] ========
theta2_series = linspace(-pi,pi,200);
M11_series = zeros(size(theta2_series));
M12_series = zeros(size(theta2_series));
M22_series = zeros(size(theta2_series));

for i = 1:length(theta2_series)
    Mt = double(subs(M,theta2,theta2_series(i)));
    M11_series(i) = Mt(1,1);
    M12_series(i) = Mt(1,2);
    M22_series(i) = Mt(2,2);
end

figure('Color','w');
subplot(3,1,1);
plot(theta2_series,M11_series,'LineWidth',1.5);
title('M_{11}(\theta_2)');xlabel('\theta_2 [rad]');grid on;

subplot(3,1,2);
plot(theta2_series,M12_series,'LineWidth',1.5);
title('M_{12}(\theta_2)');xlabel('\theta_2 [rad]');grid on;

subplot(3,1,3);
plot(theta2_series,M22_series,'LineWidth',1.5);
title('M_{22}(\theta_2)');xlabel('\theta_2 [rad]');grid on;
