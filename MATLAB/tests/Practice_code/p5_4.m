clear;clc;
syms f1x f1y f1z f2x f2y f2z L real

f1=[0;0;0;f1x;f1y;f1z];f2=[0;0;0;f2x;f2y;f2z];
T10=[1,0,0,0;
         0,1,0,L;
         0,0,1,0;
         0,0,0,1];
T20=[0,-1,0,L;
         1,0,0,0;
         0,0,1,0;
         0,0,0,1];
F=Ad_sym(T10)'*f1+Ad_sym(T20)'*f2
