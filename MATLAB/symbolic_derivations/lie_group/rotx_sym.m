function R = rotx_sym(theta)
%ROTX_SYM 绕X轴旋转的符号旋转矩阵
%   R = rotx_sym(theta)
% theta: 旋转角，可以是sym符号变量，也可以是double数值
% 返回3×3旋转矩阵(sym类型)

theta = sym(theta);   % 强制转为sym，兼容数值输入

R = sym([
    1,          0,           0;
    0,  cos(theta), -sin(theta);
    0,  sin(theta),  cos(theta)
]);

end
