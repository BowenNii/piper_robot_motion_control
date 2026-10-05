clear;clc;close all;
syms theta1 theta2 theta3 theta4 theta5 theta6 real

w1=[0;0;0];v1=[0;0;1];
w2=[1;0;0];p2=[0;0;0];
w3=[0;0;1];p3=[0;1;0];
w4=[0;0;1];p4=[-1;1;0];
w5=[sqrt(2)/2;-sqrt(2)/2;0];p5=[0;1;0];
w6=[0;0;0];v6=[0;1;0];
theta=[theta1;theta2;theta3;theta4;theta5;theta6];

w=[w1,w2,w3,w4,w5,w6];p=[v1,p2,p3,p4,p5,v6];
s=twist_wp(w,p);
Js=JacobianSpace_sym(s,theta);

Js_col4=simplify(Js(:,1:4));

Js_num=subs(Js,{theta1;theta2;theta3;theta4;theta5;theta6}, {0;0;0;0;0;0})
det(Js_num);%det=0

Fs1=[0;1;-1;1;0;0];Fs2=[1;-1;0;1;0;-1];
tao1=Js_num'*Fs1;
tao2=Js_num'*Fs2;


