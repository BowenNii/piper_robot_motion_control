function R = so3_exp(phi)
%SO3_EXP 旋转向量phi(rad) -> SO(3)，R=exp(skew(phi))。
validateattributes(phi,{'numeric'},{'real','finite','vector','numel',3}); phi=phi(:);
theta = norm(phi);
Phi = skew(phi);
if theta < 1e-6
    A = 1 - theta^2/6 + theta^4/120;
    B = 1/2 - theta^2/24 + theta^4/720;
else
    A = sin(theta)/theta;
    B = (1-cos(theta))/theta^2;
end
R = eye(3) + A*Phi + B*Phi^2;
end
