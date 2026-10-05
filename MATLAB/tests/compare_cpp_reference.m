function compare_cpp_reference(binary_path)
% 调用已编译的 cpp_reference 辅助程序，与 MATLAB 相同输入逐项比较。
% binary_path 是可执行文件，不是日志；编译命令见 MATLAB/README.md。
setup_piper_paths;
assert(isfile(binary_path) && ~contains(binary_path,'"') && ...
    ~contains(binary_path,'$') && ~contains(binary_path,'`'),'可执行文件路径无效');
[status,output]=system(['"',char(binary_path),'"']);
assert(status==0,'C++参考程序运行失败：%s',output);
values=sscanf(output,'%f'); assert(numel(values)==77,'参考输出长度无效');
model=make_piper_model(); q=[.2;1;-1;.3;-.2;.4];
target=forward_poe_space(model.S_list,q+[.02;-.01;.01;.01;-.01;.01],model.M);
reference_T=reshape(values(1:16),4,4)';
assert(norm(target-reference_T,'fro')<1e-10,'FK与C++不一致');
solvers={@solve_ik_newton,@solve_ik_dls,@solve_ik_adaptive_dls,@solve_ik_transpose};
statuses={'kConverged','kMaxIterations','kStalled','kNumericalFailure'};
for i=1:4
    reference=values(17+(i-1)*11:27+(i-1)*11);
    result=solvers{i}(model.S_list,model.M,target,q,ik_options());
    assert(result.success==logical(reference(1)) && result.iterations==reference(2));
    assert(strcmp(result.status,statuses{reference(3)+1}));
    assert(abs(result.position_error-reference(4))<1e-9);
    assert(abs(result.orientation_error-reference(5))<1e-9);
    assert(norm(result.q-reference(6:11))<1e-8,'IK与C++不一致');
end
P=pseudoinverse_svd([1,2,0,0;0,1,3,0],1e-12);
assert(norm(P-reshape(values(61:68),2,4)','fro')<1e-12);
result=solve_cartesian_velocity(diag([1,1,1,1,1,.025]),ones(6,1));
assert(norm(result.q_dot_command-values(69:74))<1e-12);
assert(abs(result.sigma_min-values(75))<1e-12 && ...
    abs(result.speed_scale-values(76))<1e-12 && result.region==values(77));
disp('Actual C++ / MATLAB FK, four IK methods, SVD and velocity comparison passed.');
end
