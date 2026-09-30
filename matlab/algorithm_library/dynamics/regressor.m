function Y = regressor(q, dq, ddq, model)
%REGRESSOR 逐个惯性基参数调用RNEA，构造离线动力学回归矩阵。
%   输入：q,dq,ddq n维；model字段见rnea.m，Ftip视为零。
%   输出：Y n*(10n)，满足tau=Y*pi（无摩擦、无外力）。
%   每个连杆的pi_i顺序：[m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]；
%   h=m*c [kg*m]，I为连杆原点处的3x3转动惯量[kg*m^2]，
%   不是质心惯量。该实现用于验证和小批量辨识，不做实时控制。
%   示例：Y=regressor(q,dq,ddq,model);
q=q(:); dq=dq(:); ddq=ddq(:);
n=numel(q);
assert(numel(dq)==n && numel(ddq)==n, '关节状态维度错误');
Y=zeros(n,10*n);
basis_model=model;
basis_model.Glist=zeros(6,6,n);
for i=1:n
    for k=1:10
        G=zeros(6,6);
        if k==1
            G(4:6,4:6)=eye(3); % m
        elseif k<=4
            h=zeros(3,1); h(k-1)=1;
            H=skew(h);
            G(1:3,4:6)=H;      % hx,hy,hz
            G(4:6,1:3)=H';
        else
            switch k
                case 5, G(1,1)=1; % Ixx
                case 6, G(2,2)=1; % Iyy
                case 7, G(3,3)=1; % Izz
                case 8, G(1,2)=1; G(2,1)=1; % Ixy
                case 9, G(1,3)=1; G(3,1)=1; % Ixz
                case 10, G(2,3)=1; G(3,2)=1; % Iyz
            end
        end
        basis_model.Glist(:,:,i)=G;
        Y(:,10*(i-1)+k)=rnea(q,dq,ddq,basis_model);
        basis_model.Glist(:,:,i)=zeros(6,6);
    end
end
end
