function [delta_pose, integral_next] = force_controller(wrench_des, wrench_meas, Kp, Ki, integral_prev, dt, integral_limit)
%FORCE_CONTROLLER 离线力控PI原型：力误差 -> 末端位姿修正量。
%   wrench=[力矩;力]，delta_pose=[旋转修正(rad);平移修正(m)]。
%   Kp、Ki及误差符号取决于接触方向和被控对象；禁止直接用于真机。
if nargin < 7 || isempty(integral_limit)
    integral_limit = inf(6,1);
end
assert(all([numel(wrench_des),numel(wrench_meas),numel(integral_prev), ...
            numel(integral_limit)] == 6), 'Wrench和积分状态须为6维');
assert(isequal(size(Kp),[6,6]) && isequal(size(Ki),[6,6]), '增益须为6x6');
assert(isscalar(dt) && isfinite(dt) && dt > 0, 'dt必须为正数');
lim = integral_limit(:);
assert(all(lim >= 0), '积分限幅必须非负');
e = wrench_des(:)-wrench_meas(:);
integral_next = max(-lim, min(lim, integral_prev(:)+dt*e));
delta_pose = Kp*e + Ki*integral_next;
end
