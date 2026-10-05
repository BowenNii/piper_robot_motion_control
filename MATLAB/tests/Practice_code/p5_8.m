clear;clc;close all;
syms theta1 theta2 theta3 L

%物体坐标系
w1=[0;1;0];
w2=[0;0;0];
w3=[0;0;1];
p1=[0;0;0];
p3=[0;-L;0];

v1=-skew_sym(w1)*p1;
v3=-skew_sym(w3)*p3;
v2=[0;1;0];

B1=[w1;v1];
B2=[w2;v2];
B3=[w3;v3];

T3=MatrixExp6_sym(-B3,theta3);
T2=MatrixExp6_sym(-B2,theta2);

Jb3=B3;
Jb2=Ad_sym(T3)*B2;
Jb1=Ad_sym(T3)*Ad_sym(T2)*B1;

Jb=simplify([Jb1,Jb2,Jb3])









