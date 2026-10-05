clear; clc; close all;

%% --------参数--------
g = 9.81;
L1 = 1; L2 = 1;
r1 = 0.5; r2 = 0.5;
m1 = 3; m2 = 2;
I1 = 2; I2 = 1;

% PID增益
Kp = diag([80,60]);
Kd = diag([20,15]);
Ki = diag([0,0]);

%力矩饱和
tau1_max = 100;
tau2_max = 20;
tau_max = [tau1_max; tau2_max];

%粘性摩擦系数 f问
b = [1;1];

tspan = [0,4];
%初始状态 [q1 q2 dq1 dq2]
x0 = [-pi/2, 0, 0, 0]';       

opts = odeset('RelTol',1e-6);

%% --------d问 PID，无饱和，无摩擦--------
[t_d, x_d] = ode45(@(t,x) odefun_2R(t,x,g,L1,L2,r1,r2,m1,m2,I1,I2,Kp,Kd,Ki,[0;0],false,false), tspan, x0, opts);

%% --------e问 PID + 力矩饱和，无摩擦--------
[t_e, x_e] = ode45(@(t,x) odefun_2R(t,x,g,L1,L2,r1,r2,m1,m2,I1,I2,Kp,Kd,Ki,tau_max,true,false), tspan, x0, opts);

%% --------f问 PID +力矩饱和 +粘性摩擦--------
[t_f, x_f] = ode45(@(t,x) odefun_2R(t,x,g,L1,L2,r1,r2,m1,m2,I1,I2,Kp,Kd,Ki,tau_max,true,true,b), tspan, x0, opts);

%% 绘图位置
figure('Name','q(t)');
subplot(2,1,1);
plot(t_d,x_d(:,1),'r', t_e,x_e(:,1),'g', t_f,x_f(:,1),'b'); hold on;
%参考轨迹
qr1 = @(t) (t<1)*(-pi/2) + (t>=1)*0;
fplot(qr1,[0,4],'k--');
ylabel('\theta_1'); grid on;
legend('d:PID','e:sat','f:sat+friction','ref');

subplot(2,1,2);
plot(t_d,x_d(:,2),'r', t_e,x_e(:,2),'g', t_f,x_f(:,2),'b'); hold on;
qr2 = @(t) (t<1)*0 + (t>=1)*(-pi/2);
fplot(qr2,[0,4],'k--');
ylabel('\theta_2'); xlabel('t(s)'); grid on;

%% 子函数：2R动力学+PID控制器 ODE
function dxdt = odefun_2R(t,x,g,L1,L2,r1,r2,m1,m2,I1,I2,Kp,Kd,Ki,tau_max,use_sat,use_fric,b)
q1 = x(1);
q2 = x(2);
dq1 = x(3);
dq2 = x(4);
q = [q1;q2];
dq = [dq1;dq2];

%====参考轨迹====
if t < 1
    qr = [-pi/2; 0];
else
    qr = [0; -pi/2];
end
dqr = [0;0];

%====2R质量矩阵M(q)====
M11 = I1 + I2 + m1*r1^2 + m2*(L1^2 + r2^2 + 2*L1*r2*cos(q2));
M12 = I2 + m2*(r2^2 + L1*r2*cos(q2));
M21 = M12;
M22 = I2 + m2*r2^2;
M = [M11, M12; M21, M22];

%====科氏离心C(q,dq)====
h = -m2*L1*r2*sin(q2);
C = [ h*dq2, h*(dq1+dq2);
     -h*dq1, 0 ];

%====重力项g(q)====
g1 = (m1*r1 + m2*L1)*g*cos(q1) + m2*r2*g*cos(q1+q2);
g2 = m2*r2*g*cos(q1+q2);
G = [g1;g2];

%====PID控制律====
e = qr - q;
edot = dqr - dq;
tau = Kp*e + Kd*edot;

%====力矩饱和 e,f问====
if use_sat
    tau(1) = clamp(tau(1), -tau_max(1), tau_max(1));
    tau(2) = clamp(tau(2), -tau_max(2), tau_max(2));
end

%====粘性摩擦 f问：tau = tau - b*dq====
if use_fric
    tau = tau - b.*dq;
end

%====正向动力学 M*ddq = tau - C*dq - G ====
ddq = M \ ( tau - C*dq - G );

dxdt = [dq1; dq2; ddq(1); ddq(2)];
end

%% 辅助饱和函数
function y = clamp(x,lo,hi)
    y = max(lo, min(x, hi));
end
