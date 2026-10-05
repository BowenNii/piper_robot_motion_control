function T = se3_exp(xi)
%SE3_EXP 积分旋量xi=[phi;rho] -> SE(3)，T=exp(hat(xi))。
validateattributes(xi,{'numeric'},{'real','finite','vector','numel',6}); xi=xi(:);
phi = xi(1:3);
rho = xi(4:6);
theta = norm(phi);
Phi = skew(phi);
if theta < 1e-6
    A = 1/2 - theta^2/24 + theta^4/720;
    B = 1/6 - theta^2/120 + theta^4/5040;
else
    A = (1-cos(theta))/theta^2;
    B = (theta-sin(theta))/theta^3;
end
V = eye(3) + A*Phi + B*Phi^2;
T = se3_from_rt(so3_exp(phi), V*rho);
end
