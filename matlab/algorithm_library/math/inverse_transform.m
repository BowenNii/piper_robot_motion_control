function T_inv = inverse_transform(T)
%INVERSE_TRANSFORM SE(3)解析逆：R_inv=R'，p_inv=-R'*p。
R = T(1:3,1:3);
p = T(1:3,4);
T_inv = make_transform(R', -R'*p);
end
