function R = roty_sym(theta)
%ROTY_SYM 绕Y轴符号旋转矩阵
theta = sym(theta);
R = sym([
     cos(theta), 0, sin(theta);
             0, 1,         0;
    -sin(theta), 0, cos(theta)
]);
end
