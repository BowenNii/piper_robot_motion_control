%TOPP_RA 离散TOPP-RA示例；算法见topp_ra_parameterize.m。
%   在MATLAB命令行运行：run('.../trajectory/TOPP_RA.m')。
%   本示例只覆盖关节速度/加速度，不含力矩或碰撞约束。
clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));

N=100;
s=linspace(0,1,N);
q0=[0.2;0.3];
q1=[pi/2;pi/4];
delta=q1-q0;
q_path=zeros(2,N);
dq_ds=zeros(2,N);
ddq_ds=zeros(2,N);
for k=1:N
    u=(s(k)-0.5)/0.12;
    e=exp(-u^2);
    scale=1-0.7*e;
    scale_s=1.4*u*e/0.12;
    scale_ss=1.4*e*(1-2*u^2)/0.12^2;
    q_path(:,k)=q0+delta*s(k)*scale;
    dq_ds(:,k)=delta*(scale+s(k)*scale_s);
    ddq_ds(:,k)=delta*(2*scale_s+s(k)*scale_ss);
end

result=topp_ra_parameterize(s,q_path,dq_ds,ddq_ds, ...
    [1.5;1.5],[4;4]);
fprintf('执行时间 %.3f s，最大速度/加速度约束残差 %.3g / %.3g\n', ...
    result.t(end),result.max_velocity_violation, ...
    result.max_acceleration_violation);

if usejava('desktop')
    figure('Color','w');
    subplot(3,1,1); plot(result.t,result.q'); grid on;
    ylabel('q [rad]'); title('TOPP-RA关节轨迹');
    subplot(3,1,2); plot(result.t,result.dq'); grid on;
    ylabel('dq [rad/s]');
    subplot(3,1,3); plot(result.t(1:end-1),result.ddq_segment'); grid on;
    xlabel('t [s]'); ylabel('ddq [rad/s^2]');
end
