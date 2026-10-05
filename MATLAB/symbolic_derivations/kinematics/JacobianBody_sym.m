function Jb = JacobianBody_sym(B_list,theta)
%JACOBIANBODY_SYM 物体雅可比J_b(q)，输入B_list 6×n、theta n维。
%   第i列=Ad_(exp(-[B_n]q_n)...exp(-[B_(i+1)]q_(i+1)))*B_i。
assert(size(B_list,1)==6 && size(B_list,2)==numel(theta), ...
    'B_list必须为6×n且theta有n项');
B_list=sym(B_list); theta=sym(theta(:));
n=numel(theta);
Jb=sym(zeros(6,n));
T=sym(eye(4));
for i=n:-1:1
    Jb(:,i)=adjoint_se3_sym(T)*B_list(:,i);
    T=T*se3_exp_sym(B_list(:,i),-theta(i));
end
end
