function H = twist_hat_sym(xi)
% 6×1 符号 [omega;v] -> 4×4 se(3) 矩阵。
assert(numel(xi)==6,'xi必须有6个元素'); xi=sym(xi(:));
H=[skew_sym(xi(1:3)),xi(4:6);sym(zeros(1,4))];
end
