function [A, Xup] = chain_transforms(model, q)
%CHAIN_TRANSFORMS RNEA/CRBA共用的关节轴和父子运动变换。
%   A(:,i)是关节i在零位link_i坐标系表达的运动子空间。
%   Xup(:,:,i)=Ad_(T_parent,child(q_i)^-1)，把父速度变到子坐标系。
n = numel(q);
A = zeros(6,n);
Xup = zeros(6,6,n);
M0i = eye(4);
for i = 1:n
    M0i = M0i*model.Mlist(:,:,i);
    A(:,i) = adjoint_se3(inverse_transform(M0i))*model.Slist(:,i);
    Xup(:,:,i) = adjoint_se3( ...
        se3_exp(-A(:,i)*q(i))*inverse_transform(model.Mlist(:,:,i)));
end
end
