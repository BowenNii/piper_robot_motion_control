%TEST_KINEMATICS POE、Jacobian、奇异抑制和IK的回归测试。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
root=fullfile(fileparts(mfilename('fullpath')),'..','algorithm_library');
addpath(fullfile(root,'math'),fullfile(root,'kinematics'));

% 平面3R机械臂：关节轴分别穿过(0,0,0)、(1,0,0)、(2,0,0)。
S=[0, 0, 0;
   0, 0, 0;
   1, 1, 1;
   0, 0, 0;
   0,-1,-2;
   0, 0, 0];
axis_directions=repmat([0;0;2],1,3);
axis_points=[0,1,2;0,0,0;0,0,0];
assert(norm(make_revolute_screw_axes(axis_directions,axis_points)-S,'fro')<1e-12);
assert(norm(make_revolute_screw_axes([0;0;1],[1;0;0])-[0;0;1;0;-1;0])<1e-12);
try
    make_revolute_screw_axes([0;0;0],[1;0;0]);
    error('零旋转轴未被拒绝');
catch err
    assert(contains(err.message,'不能为零向量'));
end
M=make_transform(eye(3),[3;0;0]);
B=adjoint_se3(inverse_transform(M))*S;
q=[0.3;-0.6;0.4];
T_space=forward_poe_space(S,q,M);
T_body=forward_poe_body(B,q,M);
assert(norm(T_space-T_body,'fro')<1e-12);
assert(norm(forward_poe_space(S,zeros(3,1),M)-M,'fro')<1e-12);

% 改进DH次序与显式矩阵乘积一致；不冒充PiPER尚未确认的DH参数表。
alpha=-pi/2; a=0.2; d=0.1; theta=0.3;
Tx=make_transform(eye(3),[a;0;0]);
Tz=make_transform(eye(3),[0;0;d]);
Rx=make_transform([1,0,0;0,cos(alpha),-sin(alpha); ...
    0,sin(alpha),cos(alpha)],zeros(3,1));
Rz=make_transform([cos(theta),-sin(theta),0; ...
    sin(theta),cos(theta),0;0,0,1],zeros(3,1));
assert(norm(mdh_transform(alpha,a,d,theta)-Rx*Tx*Rz*Tz,'fro')<1e-12);
assert(norm(forward_mdh([alpha,a,d,0],theta)- ...
    mdh_transform(alpha,a,d,theta),'fro')<1e-12);

J_s=jacobian_space(S,q);
J_b=jacobian_body(B,q);
assert(norm(J_s-adjoint_se3(T_space)*J_b,'fro')<1e-11);

% 独立的有限差分验证：Log(T(q)^-1*T(q+h*e_i))/h ≈ J_b(:,i)。
h=1e-6;
for i=1:3
    q_perturbed=q;
    q_perturbed(i)=q_perturbed(i)+h;
    T_perturbed=forward_poe_space(S,q_perturbed,M);
    body_fd=se3_log(inverse_transform(T_space)*T_perturbed)/h;
    assert(norm(body_fd-J_b(:,i))<1e-6);
end

J=diag([3,2,1]);
sigma=jacobian_singular_values(J);
assert(norm(sigma-[3;2;1])<1e-12);
assert(abs(jacobian_condition_number(J)-3)<1e-12);
assert(abs(manipulability(J)-6)<1e-12);
assert(norm(pseudoinverse_svd(J)-diag([1/3,1/2,1]),'fro')<1e-12);
lambda=0.1;
assert(norm(dls_pseudoinverse(J,lambda)- ...
    diag([3/(9+lambda^2),2/(4+lambda^2),1/(1+lambda^2)]),'fro')<1e-12);

J_singular=diag([1,0.1,0]);
assert(isinf(jacobian_condition_number(J_singular)));
assert(all(isfinite(dls_pseudoinverse(J_singular,0.1)),'all'));
[J_adapt,lambda_adapt,sigma_min]=adaptive_dls_pseudoinverse( ...
    J_singular,0.2,0.5);
assert(abs(lambda_adapt-0.2)<1e-12 && sigma_min==0);
assert(all(isfinite(J_adapt),'all'));
[J_adapt,lambda_adapt]=adaptive_dls_pseudoinverse(J,0.2,0.5);
assert(lambda_adapt==0 && norm(J_adapt-pseudoinverse_svd(J),'fro')<1e-12);

q_target=[0.4;-0.5;0.3];
T_target=forward_poe_space(S,q_target,M);
for method={'newton','dls'}
    opts=struct('method',method{1},'lambda',0.01, ...
        'max_iterations',100,'step_max',0.2);
    result=solve_ik_poe(S,M,T_target,[0.2;-0.3;0.1],opts);
    assert(result.success, ['IK求解失败：',method{1}]);
    assert(norm(forward_poe_space(S,result.q,M)-T_target,'fro')<1e-5);
end
failed=solve_ik_poe(S,M,T_target,zeros(3,1), ...
    struct('max_iterations',0));
assert(~failed.success && failed.iterations==0);

disp('All kinematics tests passed.');
