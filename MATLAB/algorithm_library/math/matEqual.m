function flag = matEqual(A,B,tol)
% matEqual 判断矩阵/向量数值近似相等
% A,B: 数值矩阵/向量 double
% tol:容差，一般取1e-10 ~ 1e-8
flag = false;
if ~isequal(size(A), size(B))
    return;
end
flag = all( abs(A - B) < tol , 'all' );
end