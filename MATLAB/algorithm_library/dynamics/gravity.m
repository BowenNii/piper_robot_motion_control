function tau_g = gravity(q, model)
%GRAVITY 仅计算模型重力项g(q)，不含摩擦和外部Wrench。
%   输入：q [rad]、model（见rnea.m）。输出：tau_g [N*m]。
%   示例：tau_g=gravity(zeros(6,1),piper_dynamics_model());
q=q(:);
tau_g=rnea(q,zeros(size(q)),zeros(size(q)),model);
end
