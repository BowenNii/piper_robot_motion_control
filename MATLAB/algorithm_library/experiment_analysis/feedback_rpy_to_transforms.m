function transforms=feedback_rpy_to_transforms(position_m,rpy_rad)
%把 CSV 里摊平的rpy数据，还原成带时间维度的 4×4 齐次变换矩阵序列，同时逐点校验每个矩阵的数学合法性
% 输入：position_m N×3米，rpy_rad N×3弧度，列为roll/pitch/yaw。
% 输出：4×4×N；旋转约定R=Rz(yaw)*Ry(pitch)*Rx(roll)。
% 输入必须已解码且使用相同基坐标系/末端参考点，不做单位猜测。
validateattributes(position_m,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(rpy_rad,{'numeric'},{'real','finite','2d','nonempty'});
assert(size(position_m,2)==3 && isequal(size(position_m),size(rpy_rad)), ...
    '位置与RPY必须是相同尺寸N×3');
N=size(position_m,1); transforms=repmat(eye(4),1,1,N);
for k=1:N
    a=rpy_rad(k,:);
    transforms(1:3,1:3,k)=rot_z(a(3))*rot_y(a(2))*rot_x(a(1));
    transforms(1:3,4,k)=position_m(k,:)';
end
end
 