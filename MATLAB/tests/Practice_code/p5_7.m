clear;clc;

w1=[0;0;1];p1=[0;0;0];
w2=[1;0;0];p2=[0;0;2];
w3=[0;0;0];v3=[0;1;0];
w=[w1,w2,w3];p=[p1,p2,v3];
theta=[pi/2;pi/2;1];
s=twist_wp(w,p);
Js=JacobianSpace_sym(s,theta)

wb1=[0;1;0];pb1=[0;0;3];
wb2=[-1;0;0];pb2=[0;0;3];
wb3=[0;0;0];vb3=[0;0;1];
wb=[wb1,wb2,wb3];pb=[pb1,pb2,vb3];
sb=twist_wp(wb,pb);
Jb=JacobianBody_sym(sb,theta);
Jb_sim=round(Jb,2);

