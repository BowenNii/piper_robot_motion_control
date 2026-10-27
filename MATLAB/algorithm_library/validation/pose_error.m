function [position_m, rotation_rad, delta_p_m] = pose_error(T_actual,T_reference)
%POSE_ERROR 两个位姿的平移距离与SO(3)最短旋转角，不直接相减RPY。
% 输入：两个有效4×4 SE(3)，在相同基坐标系、相同末端参考点下，平移m。
% 输出：position_m标量；rotation_rad标量[0,pi]；
%       delta_p_m 3×1，actual-reference，有符号且在基坐标系表达。
% 用法：[ep,er,dp]=pose_error(T_matlab,T_cpp);
assert(check_se3(T_actual).ok && check_se3(T_reference).ok, ...
    '输入必须是有效SE(3)，请先用check_se3检查');
delta_p_m=T_actual(1:3,4)-T_reference(1:3,4);
position_m=norm(delta_p_m);
R=T_reference(1:3,1:3)'*T_actual(1:3,1:3);
% atan2比接近零角时的acos(trace)更稳定；pi附近仍能返回pi。
v=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
rotation_rad=atan2(norm(v),max(-1,min(1,(trace(R)-1)/2)));
end
