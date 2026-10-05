function w = vee_sym(W)
%VEE_SYM 3×3反对称矩阵 -> 3维符号向量，SKEW_SYM的逆操作。
assert(isequal(size(W),[3,3]), 'W必须为3×3矩阵');
W=sym(W);
w=[W(3,2);W(1,3);W(2,1)];
end
