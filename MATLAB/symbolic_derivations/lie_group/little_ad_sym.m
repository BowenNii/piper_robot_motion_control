function A = little_ad_sym(xi)
% 6×1 符号旋量 -> 6×6 小伴随，与数值 little_ad 一致。
assert(numel(xi)==6,'xi必须有6个元素'); xi=sym(xi(:));
W=skew_sym(xi(1:3)); A=[W,sym(zeros(3));skew_sym(xi(4:6)),W];
end
