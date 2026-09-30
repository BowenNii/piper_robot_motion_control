function Ad = adjoint_se3(T)
%ADJOINT_SE3 SE(3)大伴随；旋量排列固定为[角速度;线速度]。
R = T(1:3,1:3);
p = T(1:3,4);
Z = zeros(3,3);
Ad = [R,          Z;
      skew(p)*R,  R];
end
