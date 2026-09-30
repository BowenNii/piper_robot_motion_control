function tau = rnea(q, dq, ddq, model, Ftip)
%RNEA 串联开链递归牛顿-欧拉逆动力学。
%   tau = RNEA(q,dq,ddq,model,Ftip)
%   输入：q [rad]、dq [rad/s]、ddq [rad/s^2]，均为n维；
%         model.Slist 6xn，零位空间关节轴[omega;v]；
%         model.Mlist 4x4x(n+1)，前n页为零位父link->子link，
%           第n+1页为link_n->工具坐标系的固定变换；
%         model.Glist 6x6xn，link_i坐标系中的空间惯量；
%         model.gravity 3维，基坐标系的重力加速度[m/s^2]；
%         Ftip 可选，工具坐标系外加Wrench[力矩;力]，单位[N*m;N]。
%   输出：tau n维关节力矩[N*m]。不含摩擦、电机转子惯量。
%   示例：tau=rnea(q,dq,ddq,model);  %% model见piper_dynamics_model
q=q(:); dq=dq(:); ddq=ddq(:);
n=numel(q);
assert(numel(dq)==n && numel(ddq)==n, 'q/dq/ddq维度不一致');
assert(size(model.Slist,1)==6 && size(model.Slist,2)==n && ...
       size(model.Mlist,1)==4 && size(model.Mlist,2)==4 && ...
       size(model.Mlist,3)==n+1 && size(model.Glist,1)==6 && ...
       size(model.Glist,2)==6 && size(model.Glist,3)==n && ...
       numel(model.gravity)==3, 'model维度不符合串联开链约定');
if nargin<5 || isempty(Ftip)
    Ftip=zeros(6,1);
end
assert(numel(Ftip)==6, 'Ftip必须为6维Wrench');
Ftip=Ftip(:);

[A,Xup]=chain_transforms(model,q);
V=zeros(6,n+1);
Vdot=zeros(6,n+1);
Vdot(:,1)=[zeros(3,1);-model.gravity(:)];
for i=1:n
    V(:,i+1)=Xup(:,:,i)*V(:,i)+A(:,i)*dq(i);
    Vdot(:,i+1)=Xup(:,:,i)*Vdot(:,i)+A(:,i)*ddq(i)+ ...
        spatial_motion_cross(V(:,i+1))*(A(:,i)*dq(i));
end

tau=zeros(n,1);
Fchild=Ftip;
Xchild=adjoint_se3(inverse_transform(model.Mlist(:,:,n+1)));
for i=n:-1:1
    G=model.Glist(:,:,i);
    F=Xchild'*Fchild+G*Vdot(:,i+1)+ ...
        spatial_force_cross(V(:,i+1))*(G*V(:,i+1));
    tau(i)=A(:,i)'*F;
    Fchild=F;
    Xchild=Xup(:,:,i);
end
end
