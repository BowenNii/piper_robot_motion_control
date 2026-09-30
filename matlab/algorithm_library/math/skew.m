function mat = skew(v)
%SKEW 3维向量 -> 3x3反对称矩阵，满足 skew(v)*u = cross(v,u)。
mat = [    0, -v(3),  v(2);
         v(3),     0, -v(1);
        -v(2),  v(1),     0];
end
