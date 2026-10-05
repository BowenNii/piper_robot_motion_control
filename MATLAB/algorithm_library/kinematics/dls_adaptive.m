function [J_pinv,lambda,sigma_min] = dls_adaptive(J,lambda_max,sigma_threshold)
% Jacobian、最大阻尼>=0、阈值>0 -> 伪逆、实际阻尼、最小奇异值。
% 与 C++ 同一平方根阻尼律；不补欠驱动任务方向。
validateattributes(lambda_max,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(sigma_threshold,{'numeric'},{'real','finite','scalar','positive'});
sigma_min=minimum_singular_value(J);
r=min(1,sigma_min/sigma_threshold);
lambda=lambda_max*sqrt(max(0,1-r^2));
J_pinv=dls_pseudoinverse(J,lambda);
end
