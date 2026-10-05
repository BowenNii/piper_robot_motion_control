function [points,T_end] = get_joint_points(q,model)
%GET_JOINT_POINTS 用串联链零位变换求各关节原点及工具点的实际位置。
%   [points,T_end]=GET_JOINT_POINTS(q,model)
%   输入：q n×1关节角[rad]；model需有Mlist、Slist（见make_piper_model）。
%   输出：points 3×(n+1)，前n列为各关节坐标系原点，末列为工具原点；
%         T_end 4×4工具位姿，可用于绘制真实末端姿态。
%   与POE旋量中的轴上最近点不同，关节原点来自URDF的joint/origin。
q=q(:);
n=numel(q);
assert(n>0 && all(isfinite(q)) && isfield(model,'Mlist') && ...
    isfield(model,'Slist') && isequal(size(model.Mlist),[4,4,n+1]) && ...
    isequal(size(model.Slist),[6,n]), 'q或model维度无效');
points=zeros(3,n+1);
T=eye(4);
T_home=eye(4);
for i=1:n
    M_i=model.Mlist(:,:,i);
    T_home=T_home*M_i;
    joint_axis=adjoint(se3_inverse(T_home))*model.Slist(:,i);
    T=T*M_i;
    points(:,i)=T(1:3,4);
    T=T*se3_exp(joint_axis*q(i));
end
T_end=T*model.Mlist(:,:,n+1);
points(:,n+1)=T_end(1:3,4);
end
