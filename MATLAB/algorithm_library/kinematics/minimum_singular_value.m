function value = minimum_singular_value(J)
% 与 C++ 一致：返回实际 SVD 最后一个奇异值。
sigma=singular_values(J); value=sigma(end);
end
