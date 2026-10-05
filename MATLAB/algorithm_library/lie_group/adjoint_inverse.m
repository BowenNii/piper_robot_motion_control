function A = adjoint_inverse(T)
% 4×4 T_ab -> 6×6 Ad(T_ab^-1)。
A=adjoint(se3_inverse(T));
end
