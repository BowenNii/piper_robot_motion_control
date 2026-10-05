function [J_pinv,lambda,sigma_min] = adaptive_dls_pseudoinverse(J,lambda_max,sigma_safe)
% 旧接口兼容，算法已改为 C++ 的平方根阻尼律。
[J_pinv,lambda,sigma_min]=dls_adaptive(J,lambda_max,sigma_safe);
end
