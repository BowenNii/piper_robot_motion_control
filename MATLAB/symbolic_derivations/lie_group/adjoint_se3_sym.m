function AdT = adjoint_se3_sym(T)
%ADJOINT_SE3_SYM 位姿T -> 6×6符号伴随矩阵；旋量排列[omega;v]。
%   Ad_T=[R,0;skew(p)*R,R]，可变换空间/物体旋量。
assert(isequal(size(T),[4,4]), 'T必须为4×4位姿');
T=sym(T);
R=T(1:3,1:3);
p=T(1:3,4);
AdT=[R,sym(zeros(3));skew_sym(p)*R,R];
end
