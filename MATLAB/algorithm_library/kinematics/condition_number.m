function value = condition_number(J)
% 奇异值比；sigma_min<=1e-12 时返回 Inf，与 C++ 一致。
sigma=singular_values(J);
if sigma(end)<=1e-12, value=Inf; else, value=sigma(1)/sigma(end); end
end
