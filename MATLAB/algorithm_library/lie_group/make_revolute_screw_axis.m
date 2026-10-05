function S = make_revolute_screw_axis(omega,point)
% 转动轴方向、轴上一点（同空间系，m）-> 6×1 [omega;v]。
omega=omega(:); point=point(:);
validateattributes(omega,{'numeric'},{'real','finite','numel',3});
validateattributes(point,{'numeric'},{'real','finite','numel',3});
assert(norm(omega)>1e-12,'关节轴方向不能为零');
omega=omega/norm(omega);
S=[omega;-cross(omega,point)];
end
