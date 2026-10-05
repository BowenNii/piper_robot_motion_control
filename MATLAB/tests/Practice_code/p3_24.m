clear;clc;close all;
L1=10;L2=5;L3=5;L4=3;
theta1=pi;theta2=pi/2;theta3=-pi;
w1=[0;0;1];p1=[0;0;0];h1=2/(2*pi);
w2=[0;1;0];p2=[0;0;L1];
w3=[1;0;0];p3=[0;L2;L1-L3];
M=[0,-1,0,0;
      1,0,0,L2+L4;
      0,0,1,L1-L3;
      0,0,0,1];
S1=twist_wp(w1,p1,h1);T1=MatrixExp6(S1,theta1);
S2=twist_wp(w2,p2);T2=MatrixExp6(S2,theta2);
S3=twist_wp(w3,p3);T3=MatrixExp6(S3,theta3);
T=T1*T2*T3*M
