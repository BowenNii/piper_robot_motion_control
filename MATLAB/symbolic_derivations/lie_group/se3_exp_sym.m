function T = se3_exp_sym(xi,theta)
%SE3_EXP_SYM 符号SE(3)指数映射，旋量排列[omega;v]。
%   T=SE3_EXP_SYM(xi)：xi=[phi;rho]为积分旋量；
%   T=SE3_EXP_SYM(S,theta)：S为单位旋量，theta为关节量[rad或m]。
%   后一种形式直接实现T=exp([S]*theta)，支持移动关节S=[0;v]。
assert(numel(xi)==6, 'xi/S必须为6维旋量');
xi=sym(xi(:));
if nargin<2
    phi=xi(1:3);
    rho=xi(4:6);
    if isequal(simplify(phi),sym(zeros(3,1)))
        T=[sym(eye(3)),rho;sym(zeros(1,3)),sym(1)];
        return;
    end
    angle=sqrt(phi.'*phi);
    W=skew_sym(phi);
    A=(1-cos(angle))/angle^2;
    B=(angle-sin(angle))/angle^3;
    R=so3_exp_sym(phi);
    V=sym(eye(3))+A*W+B*W^2;
    p=V*rho;
else
    theta=sym(theta);
    w=xi(1:3); v=xi(4:6);
    if isequal(simplify(w),sym(zeros(3,1)))
        T=[sym(eye(3)),theta*v;sym(zeros(1,3)),sym(1)];
        return;
    end
    W=skew_sym(w);
    R=so3_exp_sym(w,theta);
    G=theta*sym(eye(3))+(1-cos(theta))*W+(theta-sin(theta))*W^2;
    p=G*v;
end
T=[R,p;sym(zeros(1,3)),sym(1)];
end
