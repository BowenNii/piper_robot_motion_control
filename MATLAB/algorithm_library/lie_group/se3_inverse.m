function T_inv = se3_inverse(T)
validateattributes(T,{'numeric'},{'real','finite','size',[4,4]});
%SE3_INVERSE SE(3)解析逆：R_inv=R'，p_inv=-R'*p。
R = T(1:3,1:3);
p = T(1:3,4);
T_inv = se3_from_rt(R', -R'*p);
end
