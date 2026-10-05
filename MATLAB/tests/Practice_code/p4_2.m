clear;clc;close all;
L0=1;L1=1;L2=1;
theta1=0;theta2=pi/2;theta3=-pi/2;theta4=1;
M=[1,0,0,0;
      0,1,0,L1+L2;
      0,0,1,L0;
      0,0,0,1];
% 空间坐标系
% w1=[0;0;1];w2=[0;0;1];w3=[0;0;1];w4=[0;0;0];
% p1=[0;0;0];p2=[0;L1;0];p3=[0;L1+L2;0];
% v4=[0;0;1];
% w=[w1,w2,w3,w4];p=[p1,p2,p3,v4];
% S=twist_wp(w,p)
% theta=[theta1;theta2;theta3;theta4];
% Fk_Space=FkinSpace(S,theta,M);
% round(Fk_Space,3)

%物体坐标系
 w1=[0;0;1];w2=[0;0;1];w3=[0;0;1];w4=[0;0;0];
 v4=[0;0;1];p3=[0;0;0];p2=[0;L1;0];p1=[0;L1+L2;0];
 w=[w1,w2,w3,w4];p=[p1,p2,p3,v4];
 B=twist_wp(w,p)
 theta=[theta1;theta2;theta3;theta4];
 Fk_Body=FkinBody(B,theta,M);
 Fk_Body=round(Fk_Body,3);