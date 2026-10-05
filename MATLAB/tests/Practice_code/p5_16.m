clear;clc;close all;
syms L theta1 theta2 theta3 theta4 theta5 theta6 real

w1=[0;0;0];
v1=[0;0;1];
w2=[0;0;1];
p2=[0;0;0];
w3=[0;0;0];
v3=[0;1;0];
w4=[0;1;0];
p4=[0;0;0];
w5=[0;0;1];
p5=[0;L;0];
w6=[1;0;0];
p6=[0;L;0];
w=[w1,w2,w3,w4,w5,w6];
p=[v1,p2,v3,p4,p5,p6];
theta=[theta1;theta2;theta3;theta4;theta5;theta6];
%a
S=twist_wp(w,p);
Js=JacobianSpace_sym(S,theta);
Js=simplify(Js(:,1:3))
%b
M=[1,0,0,0;
       0,1,0,L;
       0,0,1,0;
       0,0,0,1];
M_inv=pinv(M)
B=Ad_sym(M_inv)*S;
Jb=JacobianBody_sym(B,theta);
Jb=simplify(Jb(:,5:6))