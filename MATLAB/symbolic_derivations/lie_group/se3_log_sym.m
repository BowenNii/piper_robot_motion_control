function xi = se3_log_sym(T)
%SE3_LOG_SYM SE(3)主值对数，返回积分旋量xi=[phi;rho]。
%   输入：T 4×4符号位姿；输出：xi 6×1，使exp([xi])=T（主值分支）。
%   纯平移返回[0;p]；一般转动采用V^-1*p，约定与数值se3_log一致。
assert(isequal(size(T),[4,4]), 'T必须为4×4位姿');
T=sym(T);
R=T(1:3,1:3);
p=T(1:3,4);
[phi,theta]=so3_log_sym(R);
if isequal(theta,sym(0))
    xi=[sym(zeros(3,1));p];
    return;
end
W=skew_sym(phi);
C=(1-theta/(2*tan(theta/2)))/theta^2;
V_inverse=sym(eye(3))-W/2+C*W^2;
xi=[phi;simplify(V_inverse*p)];
end
