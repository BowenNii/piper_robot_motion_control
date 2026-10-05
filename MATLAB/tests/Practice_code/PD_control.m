%% 闭环仿真示例：计算力矩控制(PD+动力学前馈)
clear; clc; 
syms q1 q2 qd1 qd2 real
syms L1 L2 m1 m2 g real
syms I1 I2 real

q   = [q1; q2];
qd  = [qd1; qd2];

% ========== 连杆1 质心位置 ==========
pc1 = [ L1/2*cos(q1);
        L1/2*sin(q1);
        0 ];

% ========== 连杆2 质心位置 ==========
pc2 = [ L1*cos(q1) + L2/2*cos(q1+q2);
        L1*sin(q1) + L2/2*sin(q1+q2);
        0 ];

% 质心速度，对时间求导
pc1_dot = jacobian(pc1,q)*qd;
pc2_dot = jacobian(pc2,q)*qd;

% ========== 总动能 T ==========
% 平动动能 + 转动动能(绕z轴)
T1 = 0.5*m1*(pc1_dot.'*pc1_dot) + 0.5*I1*qd1^2;
w2 = qd1 + qd2;
T2 = 0.5*m2*(pc2_dot.'*pc2_dot) + 0.5*I2*(w2)^2;
T = simplify(T1 + T2);

% ========== 总势能V ==========
V1 = m1*g * pc1(2);
V2 = m2*g * pc2(2);
V = simplify(V1 + V2);

Lagr = T - V;

%% 拉格朗日方程 d/dt(∂Lagr/∂qdot) − ∂Lagr/∂q = τ
syms qdd1 qdd2 real
qdd = [qdd1;qdd2];

% ∂Lagr/∂q̇
dL_dqd = jacobian(Lagr, qd).';

% d/dt (dL_dqd)
ddt_dLdqd = jacobian(dL_dqd,q)*qd + jacobian(dL_dqd,qd)*qdd;
dL_dq = jacobian(Lagr,q).';

tau_eq = simplify( ddt_dLdqd - dL_dq );

%% 分离 M(q), C(q,q̇), G(q)
M = jacobian(tau_eq, qdd);
M = simplify(M);

% G(q): qd=0,qdd=0
G = subs(tau_eq, {qdd1,qdd2,qd1,qd2},{0,0,0,0});
G = simplify(G);

Cqd = simplify( tau_eq - M*qdd - G );
C = jacobian(Cqd, qd);
C = simplify(C);

%% 打印符号形式
disp('====质量矩阵 M(q)===='); disp(M);
disp('====科氏离心矩阵 C(q,qd)===='); disp(C);
disp('====重力力矩 G(q)===='); disp(G);

%% ==============================
%% 代入数值：必须给定 q1,q2,qd1,qd2
%% ==============================
% 连杆参数（大写L）
L1_val = 0.5;
L2_val = 0.5;
m1_val = 1;
m2_val = 1;
g_val  = 9.81;
I1_val = 0.01;
I2_val = 0.01;

% 某一时刻关节状态
q1_val  = pi/4;
q2_val  = pi/6;
qd1_val = 0.5;
qd2_val = 0.3;

M_num = double(subs(M, ...
    {L1,L2,m1,m2,g,I1,I2,q1,q2}, ...
    {L1_val,L2_val,m1_val,m2_val,g_val,I1_val,I2_val,q1_val,q2_val}));

C_num = double(subs(C, ...
    {L1,L2,m1,m2,g,I1,I2,q1,q2,qd1,qd2}, ...
    {L1_val,L2_val,m1_val,m2_val,g_val,I1_val,I2_val,q1_val,q2_val,qd1_val,qd2_val}));

G_num = double(subs(G, ...
    {L1,L2,m1,m2,g,I1,I2,q1,q2}, ...
    {L1_val,L2_val,m1_val,m2_val,g_val,I1_val,I2_val,q1_val,q2_val}));

disp('------------------------------');
disp('M数值矩阵：'); disp(M_num);
disp('C数值矩阵：'); disp(C_num);
disp('G重力力矩：'); disp(G_num);

%% ==============================
%% 计算前馈力矩 tau_ff = M*qdd + C*qd + G
%% ==============================
qdd_d = [0.2; 0.1];
qd_d  = [qd1_val; qd2_val];

tau_ff = M_num * qdd_d + C_num * qd_d + G_num;
disp('------------------------------');
disp('前馈力矩 tau_ff：'); disp(tau_ff);

%% ==============================
%% 闭环仿真示例：计算力矩控制(PD+动力学前馈)
%% ==============================
% 控制器参数
Kp = diag([80,80]);
Kd = diag([10,10]);

% 当前实际关节
q_act  = [0.1;0.2];
qd_act = [0;0];

% 期望
q_des  = [q1_val; q2_val];
qd_des = [qd1_val; qd2_val];
qdd_des= qdd_d;

% 在当前期望点处求取M,C,G
M_d = double(subs(M,{L1,L2,m1,m2,I1,I2,q1,q2},{0.5,0.5,1,1,0.01,0.01,q_des(1),q_des(2)}));
C_d = double(subs(C,{L1,L2,m1,m2,I1,I2,q1,q2,qd1,qd2},{0.5,0.5,1,1,0.01,0.01,q_des(1),q_des(2),qd_des(1),qd_des(2)}));
G_d = double(subs(G,{L1,L2,m1,m2,g,q1,q2},{0.5,0.5,1,1,9.81,q_des(1),q_des(2)}));

tau_ff_sim = M_d*qdd_des + C_d*qd_des + G_d;
e    = q_des  - q_act;
edot = qd_des - qd_act;
tau_fb = Kp*e + Kd*edot;
tau_total = tau_ff_sim + tau_fb;

disp('------------------------------');
disp('闭环总输出力矩 tau_total：'); disp(tau_total);
