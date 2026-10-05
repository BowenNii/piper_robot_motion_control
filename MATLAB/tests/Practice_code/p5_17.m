clear;clc;close all;
syms L theta1 theta2 theta3 theta4 theta5 theta6 real

w1=[0;0;0];v1=[0;0;1];
w2=[1;0;0];p2=[0;0;0];
w3=[0;0;1];p3=[0;L;0];
w4=[1;0;0];p4=[L;L;0];
w5=[sqrt(2)/2;sqrt(2)/2;0];p5=[0;L;0];
w6=[0;0;0];v6=[0;1;0];
theta=[theta1;theta2;theta3;theta4;theta5;theta6];

w=[w1,w2,w3,w4,w5,w6];p=[v1,p2,p3,p4,p5,v6];
s=twist_wp(w,p);
Js=JacobianSpace_sym(s,theta);
Js(1:6,1:3);%a问

Js_0=subs(Js,{theta1;theta2;theta3;theta4;theta5;theta6},[0;0;0;0;0;0]);

Vs=Js_0*[1;0;1;-1;2;0]%b问

det(Js_0);%c问，det=0,奇异



