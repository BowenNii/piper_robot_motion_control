function result = se3_inverse_sym(T)
% 合法符号 SE(3) 位姿 -> 解析逆；调用者保证旋转正交。
assert(isequal(size(T),[4,4]),'T必须为4×4');
R=sym(T(1:3,1:3)); result=se3_from_rt_sym(R.',-R.'*sym(T(1:3,4)));
end
