clear;clc;close all;
syms L theta1 theta2 theta3 real

M=[1,0,0,0;
    0,1,0,(sqrt(2)-2)*L;
    0,0,1,0;
    0,0,0,1];

w1=[0;0;1];p1=[0;0;0];
w2=[1;0;0];p2=[0;0;L];
w3=[1;0;0];p3=[0;L;L];

S1 = [w1;-skew_sym(w1)*p1];
S2 = [w2;-skew_sym(w2)*p2];
S3 = [w3;-skew_sym(w3)*p3];

T1=MatrixExp6_sym(S1,theta1);
T2=MatrixExp6_sym(S2,theta2);
T3=MatrixExp6_sym(S3,theta3);

T = T1*T2*T3;

px = T(1,4);
py = T(2,4);
pz = T(3,4);

px_des = M(1,4);
py_des = M(2,4);
pz_des = M(3,4);

eq1 = px == px_des;
eq2 = py == py_des;
eq3 = pz == pz_des;

sol = solve([eq1,eq2,eq3],[theta1,theta2,theta3],'Real',true);

theta1_sol = double(sol.theta1)
theta2_sol = double(sol.theta2)
theta3_sol = double(sol.theta3)
