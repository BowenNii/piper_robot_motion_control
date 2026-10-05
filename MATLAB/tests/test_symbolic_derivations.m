%TEST_SYMBOLIC_DERIVATIONS 符号库与数值库的交叉回归测试。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
root=fullfile(fileparts(mfilename('fullpath')),'..');
addpath(fullfile(root,'algorithm_library','math'), ...
    fullfile(root,'algorithm_library','kinematics'), ...
    fullfile(root,'symbolic_derivations','lie_group'), ...
    fullfile(root,'symbolic_derivations','kinematics'));
syms q1 q2 real;

assert(isequal(vee_sym(skew_sym(sym([1;2;3]))),sym([1;2;3])));
assert(isequal(so3_log_sym(sym(eye(3))),sym(zeros(3,1))));
R_pi=so3_exp_sym(sym([1;0;0]),sym(pi));
phi_pi=so3_log_sym(R_pi);
assert(norm(double(so3_exp_sym(phi_pi)-R_pi),'fro')<1e-9);

S1=sym([0;0;1;0;0;0]);
S2=sym([0;0;1;0;-1;0]);
S=[S1,S2];
M=sym([eye(3),[2;0;0];0,0,0,1]);
B=adjoint_se3_sym([eye(3),[-2;0;0];0,0,0,1])*S;
T_sym=forward_poe_space_sym(S,[q1;q2],M);
T_body_sym=forward_poe_body_sym(B,[q1;q2],M);
Js_sym=JacobianSpace_sym(S,[q1;q2]);
Jb_sym=JacobianBody_sym(B,[q1;q2]);
q_values=[0.3;-0.4];
T_num=double(subs(T_sym,[q1,q2],q_values.'));
T_body_num=double(subs(T_body_sym,[q1,q2],q_values.'));
Js_num=double(subs(Js_sym,[q1,q2],q_values.'));
Jb_num=double(subs(Jb_sym,[q1,q2],q_values.'));
assert(norm(T_num-forward_poe_space(double(S),q_values,double(M)),'fro')<1e-10);
assert(norm(T_num-T_body_num,'fro')<1e-10);
assert(norm(Js_num-jacobian_space(double(S),q_values),'fro')<1e-10);
assert(norm(Jb_num-jacobian_body(double(B),q_values),'fro')<1e-10);
assert(norm(Js_num-adjoint_se3(T_num)*Jb_num,'fro')<1e-10);

T_translation=se3_exp_sym(sym([0;0;0;1;2;3]));
assert(isequal(T_translation,sym([eye(3),[1;2;3];0,0,0,1])));
assert(isequal(se3_log_sym(T_translation),sym([0;0;0;1;2;3])));
T_rot=se3_exp_sym(S2,sym(pi)/3);
xi_rot=se3_log_sym(T_rot);
assert(norm(double(se3_exp_sym(xi_rot)-T_rot),'fro')<1e-9);

S_built=make_screw_axes_sym(sym([0,0;0,0;1,0]), ...
    sym([0,1;0,0;0,0]),[],{'revolute','prismatic'});
assert(isequal(S_built(:,1),S1));
assert(isequal(S_built(:,2),sym([0;0;0;1;0;0])));

% 新增与数值 SE(3) 接口对应的符号入口。
T_check=se3_from_rt_sym(sym(eye(3)),sym([1;2;3]));
assert(isequal(T_check*se3_inverse_sym(T_check),sym(eye(4))));
assert(isequal(adjoint_inverse_sym(T_check),adjoint_sym(se3_inverse_sym(T_check))));
assert(isequal(twist_hat_sym(sym([1;2;3;4;5;6])),sym(twist_hat([1;2;3;4;5;6]))));
assert(isequal(little_ad_sym(sym([1;2;3;4;5;6])),sym(little_ad([1;2;3;4;5;6]))));
disp('All symbolic derivation tests passed.');
