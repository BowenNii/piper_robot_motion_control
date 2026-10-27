clear;clc;close all;
syms x real
a=x+1;b=x-1;
f1=10*a^3 - 15*a^4 + 6*a^5;
f2=10*b^3 - 15*b^4 + 6*b^5;
simplify(f1-f2)