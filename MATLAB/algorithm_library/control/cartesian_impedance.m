function [tau, wrench_cmd] = cartesian_impedance(T_des, V_des, T, V, J_body, K, D, wrench_ff)
%CARTESIAN_IMPEDANCE 笛卡尔弹簧-阻尼控制，输出关节力矩N*m。
%   e=Log(inv(T)*T_des)=[旋转误差;平移误差]，在当前末端系表达。
%   V和V_des必须均在当前末端系表达；J_body也须为当前末端系Jacobian。
%   wrench排列为[力矩;力]，tau=J_body'*wrench_cmd。
if nargin < 8 || isempty(wrench_ff)
    wrench_ff = zeros(6,1);
end
assert(isequal(size(T),[4,4]) && isequal(size(T_des),[4,4]), '位姿须为4x4');
assert(size(J_body,1) == 6 && isequal(size(K),[6,6]) && ...
       isequal(size(D),[6,6]), 'Jacobian或增益维度错误');
assert(all([numel(V_des),numel(V),numel(wrench_ff)] == 6), 'Twist或Wrench须为6维');
e = se3_log(se3_inverse(T)*T_des);
wrench_cmd = K*e + D*(V_des(:)-V(:)) + wrench_ff(:);
tau = J_body'*wrench_cmd;
end
