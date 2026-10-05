%TEST_CPP_ALIGNMENT 新接口、数学约定与 C++ 已知输出回归。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..')); setup_piper_paths;
c=piper_constants(); assert(c.kEpsilon==1e-12 && c.kDof==6);
assert(abs(deg_to_rad(180)-pi)<1e-15);
assert(norm(rot_axis_angle([0;0;2],0.3)-rot_z(0.3),'fro')<1e-12);
assert(norm(make_revolute_screw_axis([0;0;2],[1;0;0])-[0;0;1;0;-1;0])<1e-12);
assert(norm(make_prismatic_screw_axis([2;0;0])-[0;0;0;1;0;0])<1e-12);
T=se3_exp([.1;-.2;.3;.4;-.1;.2]);
assert(norm(adjoint_inverse(T)*adjoint(T)-eye(6),'fro')<1e-12);
assert(norm(twist_hat([1;2;3;4;5;6])-twist_skew([1;2;3;4;5;6]),'fro')<1e-12);
for J={randn(2,4),randn(4,2),zeros(3),diag([2,1e-13])}
    A=J{1}; P=pseudoinverse_svd(A,1e-12);
    assert(norm(A*P*A-A,'fro')<1e-10);
    assert(isequal(size(dls_pseudoinverse(A,.1)),fliplr(size(A))));
end
assert(isinf(condition_number(diag([2,1e-13]))));
[~,lambda]=dls_adaptive(diag([1,.025]),.05,.05);
assert(abs(lambda-.05*sqrt(.75))<1e-15);
assert(norm(limit_joint_velocity([2;-3],[1;-2])-[1;-2])<1e-15);
[y,state]=low_pass_filter(1,.2,0); assert(y==.2 && state==y);
assert(norm(interpolate_se3(eye(4),T,0)-eye(4),'fro')<1e-12);
assert(norm(interpolate_se3(eye(4),T,1)-T,'fro')<1e-12);
r=solve_cartesian_velocity(eye(6),ones(6,1));
assert(r.region==0 && norm(r.q_dot_command-ones(6,1))<1e-12);
r=solve_cartesian_velocity(diag([1,1,1,1,1,0]),ones(6,1));
assert(r.region==2 && r.speed_scale==.1 && all(isfinite(r.q_dot_command)));
% 已归档的 C++ 输出是独立回归参照，不读取/修改实验 CSV。
model=make_piper_model();
q=deg_to_rad([0;-1.999;1.971;-1.412;22.110;-2.608]);
reference=[-.293558675,-.011209682,.955875330,.051237069; ...
    -.068294484,.997622095,-.009274653,-.000846014; ...
    -.953498384,-.068003667,-.293626180,.168777508;0,0,0,1];
assert(norm(forward_poe_space(model.S_list,q,model.M)-reference,'fro')<3e-9);
initial=[.2;1;-1;.3;-.2;.4]; goal=initial+[.02;-.01;.01;.01;-.01;.01];
target=forward_poe_space(model.S_list,goal,model.M);
solvers={@solve_ik_newton,@solve_ik_dls,@solve_ik_adaptive_dls,@solve_ik_transpose};
for i=1:numel(solvers)
    o=ik_options(); o.max_iterations=3000;
    r=solvers{i}(model.S_list,model.M,target,initial,o);
    if i<4
        assert(r.success && strcmp(r.status,'kConverged'), 'IK方法%d未收敛',i);
    else
        assert(r.position_error<1e-6 && r.orientation_error<1e-5);
        % 转置法可能达到迭代上限；失败也是合法结果，不放宽默认收敛标准。
        assert(r.success || strcmp(r.status,'kMaxIterations'));
    end
end
o=ik_options(); o.max_iterations=0;
r=solve_ik_newton(model.S_list,model.M,target,initial,o);
assert(~r.success && strcmp(r.status,'kMaxIterations') && r.iterations==0);
o=ik_options(); o.q_min=model.q_min; o.q_max=model.q_max;
r=solve_ik_dls(model.S_list,model.M,target,initial,o); assert(r.success);
caught=false; try, solve_ik_newton(model.S_list,eye(4)*2,target,initial); catch, caught=true; end
assert(caught,'非法位姿必须报错');
disp('All C++ alignment tests passed.');
