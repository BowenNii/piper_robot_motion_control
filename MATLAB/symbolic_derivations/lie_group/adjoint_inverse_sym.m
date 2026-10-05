function A = adjoint_inverse_sym(T)
% 符号 Ad(T^-1)，旋量顺序 [omega;v]。
A=adjoint_sym(se3_inverse_sym(T));
end
