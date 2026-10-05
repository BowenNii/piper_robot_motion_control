function T = se3_from_rt_sym(R,p)
% 符号旋转 3×3、平移 3×1 -> 符号 4×4 位姿。
assert(isequal(size(R),[3,3]) && numel(p)==3,'输入维度无效');
T=[sym(R),sym(p(:));sym([0,0,0,1])];
end
