function R = rotz_sym(theta)
%ROTZ_SYM 绕Z轴符号旋转矩阵
theta = sym(theta);
R = sym([
    cos(theta), -sin(theta), 0;
    sin(theta),  cos(theta), 0;
            0,          0, 1
]);
end
