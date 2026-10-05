function dq = estimate_joint_velocity(q, q_prev, dt)
%ESTIMATE_JOINT_VELOCITY 关节角一阶后向差分，输出关节角速度。
%   dq = ESTIMATE_JOINT_VELOCITY(q,q_prev,dt)
%   输入：q、q_prev 同尺寸关节角[rad]；dt 实际时间间隔[s]。
%   输出：dq=(q-q_prev)/dt [rad/s]。
%   用法：dq_raw=estimate_joint_velocity(q_now,q_last,t_now-t_last);
%   注意：输入关节角须已处理零位与可能的角度跳变；差分会放大噪声。
assert(isequal(size(q),size(q_prev)), 'q与q_prev尺寸必须一致');
assert(all(isfinite(q(:))) && all(isfinite(q_prev(:))), '输入必须有限');
assert(isscalar(dt) && isfinite(dt) && dt>0, 'dt必须为正数');
dq=(q-q_prev)/dt;
end
