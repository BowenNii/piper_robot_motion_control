function result = trim_small(matrix,eps)
% 矩阵中 abs(element)<eps 的元素置零，默认1e-10；仅用于显示/整理。
if nargin<2, eps=1e-10; end
validateattributes(matrix,{'numeric'},{'real','finite'});
validateattributes(eps,{'numeric'},{'real','finite','scalar','nonnegative'});
result=matrix; result(abs(result)<eps)=0;
end
