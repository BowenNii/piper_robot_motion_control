function R = so3_exp_sym(w,theta)
%SO3_EXP_SYM 符号SO(3)指数映射，输出3×3旋转矩阵。
%   R=SO3_EXP_SYM(phi)：phi为积分旋转向量[rad]；
%   R=SO3_EXP_SYM(w,theta)：w为单位轴，theta为有符号转角[rad]。
%   第二种形式更适合直接推导关节变量；符号轴的单位性由调用者保证。
assert(numel(w)==3, '旋转向量/轴必须为3维');
w=sym(w(:));
if nargin<2
    phi=w;
    if isequal(simplify(phi),sym(zeros(3,1)))
        R=sym(eye(3));
        return;
    end
    angle=sqrt(phi.'*phi);
    W=skew_sym(phi);
    R=sym(eye(3))+(sin(angle)/angle)*W+ ...
        ((1-cos(angle))/angle^2)*(W*W);
else
    theta=sym(theta);
    W=skew_sym(w);
    R=sym(eye(3))+sin(theta)*W+(1-cos(theta))*(W*W);
end
end
