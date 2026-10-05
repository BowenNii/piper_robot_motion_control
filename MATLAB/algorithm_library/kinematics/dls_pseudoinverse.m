function J_pinv = dls_pseudoinverse(J,lambda)
% m×n Jacobian、非负阻尼 -> n×m DLS 伪逆；lambda=0 用截断 SVD。
validateattributes(J,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(lambda,{'numeric'},{'real','finite','scalar','nonnegative'});
if lambda==0, J_pinv=pseudoinverse_svd(J,1e-12); return; end
[U,S,V]=svd(J,'econ'); sigma=diag(S); k=numel(sigma);
gain=sigma./(sigma.^2+lambda^2);
J_pinv=V(:,1:k)*diag(gain)*U(:,1:k)';
end
