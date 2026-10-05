function J = jacobian_body(B_list,q)
%JACOBIAN_BODY 计算物体雅可比，V_b=J_b(q)*dq。
%   输入：B_list 6×n物体旋量；q n×1关节量。
%   输出：J 6×n物体雅可比。
%   第i列=Ad_(exp(-[B_n]q_n)...exp(-[B_(i+1)]q_(i+1)))*B_i。
assert(size(B_list,1)==6 && size(B_list,2)==numel(q) && ...
    all(isfinite(B_list(:))) && all(isfinite(q(:))), '输入无效');
q=q(:);
n=numel(q);
J=zeros(6,n);
T=eye(4);
for i=n:-1:1
    J(:,i)=adjoint(T)*B_list(:,i);
    T=T*se3_exp(-B_list(:,i)*q(i));
end
end
