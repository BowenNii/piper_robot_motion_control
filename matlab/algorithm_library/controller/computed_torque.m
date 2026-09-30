function tau = computed_torque(q_des, dq_des, ddq_des, q, dq, kp, kd, inverse_dynamics_fn)
%COMPUTED_TORQUE 完整计算力矩控制：tau=ID(q,dq,ddq_cmd)。
%   ddq_cmd=ddq_des+kp.*(q_des-q)+kd.*(dq_des-dq)。
%   此处kp、kd是加速度反馈增益，不是joint_pd的力矩增益。
q_des = q_des(:); dq_des = dq_des(:); ddq_des = ddq_des(:);
q = q(:); dq = dq(:); kp = kp(:); kd = kd(:);
n = numel(q);
assert(all([numel(q_des),numel(dq_des),numel(ddq_des), ...
            numel(dq),numel(kp),numel(kd)] == n), '关节状态和增益维度不一致');
assert(isa(inverse_dynamics_fn, 'function_handle'), '需要逆动力学函数句柄');
ddq_cmd = ddq_des + kp.*(q_des-q) + kd.*(dq_des-dq);
tau = inverse_dynamics_fn(q, dq, ddq_cmd);
tau = tau(:);
assert(numel(tau) == n, '逆动力学输出维度错误');
end
