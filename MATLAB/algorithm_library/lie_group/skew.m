function W = skew(w)
%SKEW 三维向量转3×3反对称矩阵，使得skew(w)*v=cross(w,v)。
%   输入：w 3维向量；输出：W 3×3矩阵。
assert(numel(w)==3, 'w必须是3维向量');
w=w(:);
W=[0,-w(3),w(2);
   w(3),0,-w(1);
   -w(2),w(1),0];
end
