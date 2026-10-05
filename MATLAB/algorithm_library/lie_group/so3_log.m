function phi = so3_log(R)
validateattributes(R,{'numeric'},{'real','finite','size',[3,3]});
%SO3_LOG SO(3) -> 主值旋转向量，角度范围[0,pi]。
s = vee(R-R');
sin_theta = norm(s)/2;
cos_theta = max(-1, min(1, (trace(R)-1)/2));
theta = atan2(sin_theta, cos_theta);

if theta < 1e-6
    phi = s/2;
elseif pi-theta < 1e-6
    % 对称部分+I约为2*u*u'；先消除微小反对称项，再取最长列。
    C = (R+R')/2+eye(3);
    [~, idx] = max(sum(C.^2,1));
    axis = C(:,idx)/norm(C(:,idx));
    if dot(axis,s) < 0
        axis = -axis;
    end
    phi = theta*axis;
else
    phi = theta/(2*sin(theta))*s;
end
end
