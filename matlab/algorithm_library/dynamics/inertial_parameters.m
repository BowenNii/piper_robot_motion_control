function pi_vec = inertial_parameters(model)
%INERTIAL_PARAMETERS 从Glist提取与regressor配套的10n惯性参数列向量。
%   每连杆顺序[m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]。
%   示例：pi_vec=inertial_parameters(model); tau=regressor(q,dq,ddq,model)*pi_vec;
n=size(model.Glist,3);
pi_vec=zeros(10*n,1);
for i=1:n
    G=model.Glist(:,:,i);
    m=trace(G(4:6,4:6))/3;
    h=vee(G(1:3,4:6));
    I=G(1:3,1:3);
    pi_vec(10*(i-1)+(1:10))=[m;h;I(1,1);I(2,2);I(3,3); ...
                             I(1,2);I(1,3);I(2,3)];
end
end
