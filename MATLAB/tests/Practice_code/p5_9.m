clear;clc;close all;
syms theta1 theta2 theta3 

w1=[0;0;1];
w2=[1/sqrt(2);0;1/sqrt(2)];
w3=[1;0;0];

p=[0;0;0];
s1=[w1;p];
s2=[w2;p];
s3=[w3;p];

s=[s1,s2,s3];
q=[theta1,theta2,theta3];

Js=JacobianSpace_sym(s,q);

Js=simplify(Js);
Js=Js(1:3,1:3)

sq=det(Js)==0
sol = solve(sq,theta2)







