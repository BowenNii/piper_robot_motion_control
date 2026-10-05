function R = rot_axis_angle(axis,theta)
% 轴方向 3×1、有符号转角 rad -> 3×3 旋转。
axis=axis(:);
validateattributes(axis,{'numeric'},{'real','finite','numel',3});
validateattributes(theta,{'numeric'},{'real','finite','scalar'});
assert(norm(axis)>1e-12,'旋转轴不能为零');
R=so3_exp(axis/norm(axis)*theta);
end
