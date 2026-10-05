clear;clc;close all;
syms L theta1 theta2 theta3 real

w1=[0;0;0];v1=[0;1;0];
w2=[0;0;1];p2=[0;0;0];
w3=[0;0;1];p3=[-L;L;0];
w=[w1,w2,w3];p=[v1,p2,p3];
theta=[theta1;theta2;theta3];

s=twist_wp(w,p);
Js=JacobianSpace_sym(s,theta);

Js_3_3=Js(3:5,1:3);
eq=det(Js_3_3)==0;

[sol]=solve(eq,[theta1,theta2,theta3],'ReturnConditions',true);

theta_val=[0;0;0];
Js_num=subs(Js,{theta1,theta2,theta3},{theta_val(1),theta_val(2),theta_val(3)})

Tbs=[0,0,-1,0;
        0,1,0,-5*L;
        1,0,0,L;
        0,0,0,1];
Fb=[0;0;0;10;0;10];
Fs=Ad_sym(Tbs)'*Fb;
tao=Js_num'*Fs;




