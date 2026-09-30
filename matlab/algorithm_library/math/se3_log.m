function xi = se3_log(T)
%SE3_LOG SE(3) -> 主值积分旋量xi=[phi;rho]。
phi = so3_log(T(1:3,1:3));
p = T(1:3,4);
theta = norm(phi);
Phi = skew(phi);
if theta < 1e-6
    C = 1/12 + theta^2/720 + theta^4/30240;
else
    C = (1 - theta/(2*tan(theta/2)))/theta^2;
end
V_inv = eye(3) - Phi/2 + C*Phi^2;
xi = [phi; V_inv*p];
end
