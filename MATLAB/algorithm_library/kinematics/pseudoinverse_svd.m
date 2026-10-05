function J_pinv = pseudoinverse_svd(J,tolerance)
% m×n 有限实矩阵、绝对奇异值阈值 -> n×m 伪逆（与 C++ 一致）。
if nargin<2, tolerance=1e-12; end
validateattributes(J,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(tolerance,{'numeric'},{'real','finite','scalar','nonnegative'});
[U,S,V]=svd(J,'econ'); sigma=diag(S); k=numel(sigma);
gain=zeros(k,1); keep=sigma>tolerance; gain(keep)=1./sigma(keep);
J_pinv=V(:,1:k)*diag(gain)*U(:,1:k)';
end
