function w = vee(R)
%VEE 3×3反对称矩阵提取向量；是SKEW的逆操作。
%   输入：R 3×3反对称矩阵；输出：w=[R(3,2);R(1,3);R(2,1)]。
assert(isequal(size(R),[3,3]), 'R必须为3×3矩阵');
w=[R(3,2);R(1,3);R(2,1)];
end
