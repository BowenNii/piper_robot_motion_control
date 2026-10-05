function V = twist_from_body_jacobian(J,dq)
% 6×n Jacobian、n×1 关节速度 -> 同系 6×1 Twist [omega;v]。
validateattributes(J,{'numeric'},{'real','finite','2d','nrows',6,'nonempty'});
dq=dq(:); assert(numel(dq)==size(J,2) && all(isfinite(dq)),'dq维度或数值无效');
V=J*dq;
end
