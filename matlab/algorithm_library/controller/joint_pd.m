function tau = joint_pd(q_des, dq_des, q, dq, kp, kd)
%JOINT_PD 关节PD：tau=kp.*(q_des-q)+kd.*(dq_des-dq)。
%   各输入均为n维列向量；增益单位应与输出力矩N*m相匹配。
q_des = q_des(:); dq_des = dq_des(:);
q = q(:); dq = dq(:); kp = kp(:); kd = kd(:);
n = numel(q);
assert(all([numel(q_des),numel(dq_des),numel(dq),numel(kp),numel(kd)] == n), ...
       '关节状态和增益维度不一致');
tau = kp.*(q_des-q) + kd.*(dq_des-dq);
end
