function v = vee(mat)
%VEE 3x3反对称矩阵 -> 3维列向量，是 skew 的逆运算。
v = [mat(3,2); mat(1,3); mat(2,1)];
end
