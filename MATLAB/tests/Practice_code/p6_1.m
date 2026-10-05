clear;clc;close all;
syms theta1 theta2 theta3  real
L1 = 3; L2 = 2;L3 = 1;

eq1=L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)==4;
eq2=L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)==2;
eq3=theta1+theta2+theta3==0;

sol=solve([eq1,eq2,eq3],[theta1,theta2,theta3],"Real",true);
sol.theta1;
sol.theta2;
sol.theta3;