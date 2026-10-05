function T = se3_from_rt(R, p)
%SE3_FROM_RT 由旋转R和平移p组成4x4齐次变换矩阵。
validateattributes(R,{'numeric'},{'real','finite','size',[3,3]});
validateattributes(p,{'numeric'},{'real','finite','vector','numel',3});
T = [R, p(:);
     0, 0, 0, 1];
end
