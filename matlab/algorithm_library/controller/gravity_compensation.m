function tau_g = gravity_compensation(q, inverse_dynamics_fn)
%GRAVITY_COMPENSATION 零速度、零加速度下的逆动力学重力力矩。
%   inverse_dynamics_fn签名：tau=fn(q,dq,ddq)，输出单位N*m。
q = q(:);
assert(isa(inverse_dynamics_fn, 'function_handle'), '需要逆动力学函数句柄');
tau_g = inverse_dynamics_fn(q, zeros(size(q)), zeros(size(q)));
tau_g = tau_g(:);
assert(numel(tau_g) == numel(q), '逆动力学输出维度错误');
end
