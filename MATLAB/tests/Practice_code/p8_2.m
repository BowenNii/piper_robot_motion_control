clear; clc; close all;
r_sphere=0.1;
r_cylinder=0.02;h_cylinder=0.2;
dens=7500;
m_sphere=pi*r_sphere^3*4/3*dens
Ixx_sphere=m_sphere*2*r_sphere^2/5;
Iyy_sphere=m_sphere*2*r_sphere^2/5;
Izz_sphere=m_sphere*2*r_sphere^2/5;
I_sphere=[Ixx_sphere,0,0;
                 0,Iyy_sphere,0;
                 0,0,Izz_sphere]
m_cylinder=pi*r_cylinder^2*h_cylinder*dens
Ixx_cylinder=m_cylinder*(3*r_cylinder^2+h_cylinder^2)/12;
Iyy_cylinder=m_cylinder*(3*r_cylinder^2+h_cylinder^2)/12;
Izz_cylinder=m_cylinder*r_cylinder^2/2;
I_cylinder=[Ixx_cylinder,0,0;
                 0,Iyy_cylinder,0;
                 0,0,Izz_cylinder]

q=[0;0;0.2];
I=I_cylinder+2*m_sphere*(q'*q*eye(3)-q*q')+2*I_sphere;
m=m_sphere*2+m_cylinder;
Gb=[I,zeros(3,3);
         zeros(3,3),m*eye(3)] 