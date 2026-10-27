function report = check_target_reached(T_feedback,T_target,position_tolerance_m,rotation_tolerance_rad)
%CHECK_TARGET_REACHED 判断单个位姿反馈是否处于目标容差内。
% 输入：同坐标系/同参考点的4×4位姿；显式设置位置m、姿态rad容差。
% 输出：reached、position_error_m、rotation_error_rad。
% 只判断该条反馈；不能由此证明停稳、持续到位或外部实际定位精度。
% 停稳检查应对一个时间窗口调用check_static_stability，再检查终点。
validateattributes(position_tolerance_m,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(rotation_tolerance_rad,{'numeric'},{'real','finite','scalar','nonnegative'});
[p,r]=pose_error(T_feedback,T_target);
report=struct('reached',p<=position_tolerance_m && r<=rotation_tolerance_rad, ...
    'position_error_m',p,'rotation_error_rad',r);
end
