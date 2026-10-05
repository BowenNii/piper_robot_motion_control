%TEST_MATH 基础数值运算、李群恒等式与错误边界测试。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
addpath(fullfile(fileparts(mfilename('fullpath')),'..', ...
    'algorithm_library','math'));
v=[0.2;-0.4;0.6];
assert(norm(skew(v)+skew(v)','fro')<1e-14);
assert(norm(vee(skew(v))-v)<1e-14);
assert(norm(skew(v)*[1;2;3]-cross(v,[1;2;3]))<1e-14);

rng(42);
for i=1:50
    xi=0.6*randn(6,1);
    T=se3_exp(xi);
    assert(norm(T(4,:)-[0,0,0,1])<1e-12);
    assert(norm(se3_exp(se3_log(T))-T,'fro')<1e-10);
    assert(norm(T*inverse_transform(T)-eye(4),'fro')<1e-10);
    U=se3_exp(0.3*randn(6,1));
    assert(norm(adjoint_se3(T*U)-adjoint_se3(T)*adjoint_se3(U), ...
        'fro')<1e-10);
end
axis=[1;2;3]/sqrt(14);
R_pi=so3_exp((pi-1e-7)*axis);
assert(norm(so3_exp(so3_log(R_pi))-R_pi,'fro')<1e-6);
assert(norm(so3_exp(so3_log(eye(3)))-eye(3),'fro')<1e-12);

assert(isequal(clamp([-2;0;2],[-1;-1;-1],[1;1;1]),[-1;0;1]));
caught=false;
try
    clamp_checked(0,1,-1);
catch
    caught=true;
end
assert(caught,'clamp_checked必须拒绝反置的安全边界');
assert(clamp(0,1,-1)==0,'标准clamp与C++一致，允许交换边界');
disp('All math tests passed.');
