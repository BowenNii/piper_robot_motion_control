function tau = joint_impedance(q_des, dq_des, q, dq, stiffness, damping, tau_ff)
%JOINT_IMPEDANCE 关节弹簧-阻尼控制：tau=K.*e+D.*de+tau_ff。
%   tau_ff可取重力补偿力矩；输出为N*m，仅用于模型仿真。
q = q(:);
if nargin < 7 || isempty(tau_ff)
    tau_ff = zeros(size(q));
end
tau_ff = tau_ff(:);
assert(numel(tau_ff) == numel(q), 'tau_ff维度错误');
tau = joint_pd(q_des, dq_des, q, dq, stiffness, damping) + tau_ff;
end
