function W = skew_sym(w)
%SKEW_SYM 3维符号向量 -> 3×3反对称矩阵，skew_sym(w)*v=cross(w,v)。
assert(numel(w)==3, 'w必须为3维向量');
w=sym(w(:));
W=[sym(0),-w(3),w(2);w(3),sym(0),-w(1);-w(2),w(1),sym(0)];
end
