clear;clc;close all;
syms L theta1 theta2 theta3 real
%两种答案验证
% theta1=0;theta2=pi/3;theta3=7;
%theta1=2*atan(6/5);theta2=-pi/3;theta3=7;
w1=[0;0;1];p1=[0;0;0];
w2=[0;1/2;sqrt(3)/2];p2=[0;0;0];
w3=[0;0;0];v3=[0;1;0];

w=[w1,w2,w3];p=[p1,p2,v3];
S=twist_wp(w,p);

T1=MatrixExp6_sym(S(:,1),theta1);
T2=MatrixExp6_sym(S(:,2),theta2);
T3=MatrixExp6_sym(S(:,3),theta3);
p0=[0;1;0;1];

T=T1*T2*T3*p0
px = T(1,:);
py = T(2,:);
pz = T(3,:);

eq1=px==-6;
eq2=py==5;
eq3=pz==sqrt(3);
sol=solve([eq1,eq2,eq3],[theta1,theta2,theta3]);

theta1=sol.theta1
theta2=sol.theta2
theta3=sol.theta3




