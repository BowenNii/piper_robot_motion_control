function J = jacobian_space(S_list,q)
%JACOBIAN_SPACE 计算空间雅可比，V_s=J_s(q)*dq。
%   输入：S_list 6×n空间旋量；q n×1关节量。
%   输出：J 6×n空间雅可比，列顺序与关节顺序一致。
%   第i列=Ad_(exp([S1]q1)...exp([S_(i-1)]q_(i-1)))*S_i。
assert(size(S_list,1)==6 && size(S_list,2)==numel(q) && ...
    all(isfinite(S_list(:))) && all(isfinite(q(:))), '输入无效');
q=q(:);
n=numel(q);
J=zeros(6,n);
T=eye(4);
for i=1:n
    J(:,i)=adjoint(T)*S_list(:,i);
    T=T*se3_exp(S_list(:,i)*q(i));
end
end
