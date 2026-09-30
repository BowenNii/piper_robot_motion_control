function T = make_transform(R, p)
%MAKE_TRANSFORM 由旋转R和平移p组成4x4齐次变换矩阵。
T = [R, p(:);
     0, 0, 0, 1];
end
