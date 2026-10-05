function Js = JacobianSpace_sym(S_list,theta)
%JACOBIANSPACE_SYM 空间雅可比J_s(q)，输入S_list 6×n、theta n维。
%   第i列=Ad_(exp([S_1]q_1)...exp([S_(i-1)]q_(i-1)))*S_i。
assert(size(S_list,1)==6 && size(S_list,2)==numel(theta), ...
    'S_list必须为6×n且theta有n项');
S_list=sym(S_list); theta=sym(theta(:));
n=numel(theta);
Js=sym(zeros(6,n));
T=sym(eye(4));
for i=1:n
    Js(:,i)=adjoint_se3_sym(T)*S_list(:,i);
    T=T*se3_exp_sym(S_list(:,i),theta(i));
end
end
