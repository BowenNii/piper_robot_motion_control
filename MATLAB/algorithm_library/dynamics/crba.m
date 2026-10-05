function Mq = crba(q, model)
%CRBA 复合刚体法计算串联机械臂关节惯量矩阵。
%   输入：q n维关节角[rad]，model字段约定见rnea.m。
%   输出：Mq n*n，对称；tau_inertia=Mq*ddq（dq=0、gravity=0时）。
%   示例：Mq=crba(q,model);
q=q(:);
n=numel(q);
assert(size(model.Slist,1)==6 && size(model.Slist,2)==n && ...
       size(model.Mlist,1)==4 && size(model.Mlist,2)==4 && ...
       size(model.Mlist,3)==n+1 && size(model.Glist,1)==6 && ...
       size(model.Glist,2)==6 && size(model.Glist,3)==n, 'model维度错误');
[A,Xup]=chain_transforms(model,q);
Ic=model.Glist;
Mq=zeros(n,n);
for i=n:-1:1
    F=Ic(:,:,i)*A(:,i);
    Mq(i,i)=A(:,i)'*F;
    j=i;
    while j>1
        F=Xup(:,:,j)'*F;
        j=j-1;
        Mq(i,j)=A(:,j)'*F;
        Mq(j,i)=Mq(i,j);
    end
    if i>1
        Ic(:,:,i-1)=Ic(:,:,i-1)+Xup(:,:,i)'*Ic(:,:,i)*Xup(:,:,i);
    end
end
end
