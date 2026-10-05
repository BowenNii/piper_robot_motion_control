function limited = limit_joint_velocity(q_dot,q_dot_max)
% 关节速度 rad/s、同维限值 -> 对称逐关节限幅；不代替硬件安全。
q_dot=q_dot(:); q_dot_max=q_dot_max(:);
assert(numel(q_dot)==numel(q_dot_max) && all(isfinite(q_dot)) && ...
    all(isfinite(q_dot_max)),'速度和限值必须是同维有限向量');
limited=clamp(q_dot,-abs(q_dot_max),abs(q_dot_max));
end
