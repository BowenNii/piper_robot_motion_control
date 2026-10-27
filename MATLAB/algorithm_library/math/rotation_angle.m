function angle=rotation_angle(R)
% atan2避免acos(trace)在接近单位矩阵时丢失精度；不是相减RPY。
v=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
angle=atan2(norm(v),max(-1,min(1,(trace(R)-1)/2)));
end