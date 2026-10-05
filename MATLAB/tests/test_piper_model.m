%TEST_PIPER_MODEL PiPER URDF模型的零位、旋量、限位与动力学一致性。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
root=fullfile(fileparts(mfilename('fullpath')),'..','algorithm_library');
addpath(fullfile(root,'math'),fullfile(root,'kinematics'), ...
    fullfile(root,'dynamics'),fullfile(root,'robot_model'));
urdf_path='/home/nbw/piper_robot_motion_control/models/urdf/piper_description.urdf';
model=make_piper_model(urdf_path);
assert(isequal(size(model.S_list),[6,6]));
assert(norm(make_revolute_screw_axes(model.w,model.p)-model.S_list,'fro')<1e-12);
assert(isequal(size(model.B_list),[6,6]));
assert(isequal(size(model.Mlist),[4,4,7]));
assert(isequal(size(model.Glist),[6,6,6]));
assert(norm(model.p(:,1)-[0;0;0.123])<1e-12);
assert(norm(model.p(:,2)-[0;0;0.123])<1e-12);
assert(norm(model.w(:,1)-[0;0;1])<1e-12);
assert(abs(model.mass(1)-0.71)<1e-12);
assert(abs(model.mass(2)-1.16)<1e-12);
assert(abs(model.q_min(1)+2.6179938)<1e-10);
assert(abs(model.q_max(2)-3.1415926)<1e-10);
assert(all(model.dq_max==5));

T_home=eye(4);
for i=1:6
    T_home=T_home*model.Mlist(:,:,i);
    assert(abs(norm(model.w(:,i))-1)<1e-12);
    assert(norm(model.S_list(4:6,i)+ ...
        cross(model.w(:,i),model.p(:,i)))<1e-12);
    assert(norm(model.Glist(:,:,i)-model.Glist(:,:,i)','fro')<1e-12);
end
assert(norm(T_home-model.M,'fro')<1e-12);
assert(norm(forward_poe_space(model.S_list,zeros(6,1),model.M)-model.M,'fro')<1e-12);

% 与URDF的关节递推独立比较：每节先乘零位origin，再绕局部axis转q_i。
q=[0.2;1.0;-1.0;0.3;-0.2;0.4];
T_urdf=eye(4);
for i=1:6
    % 本PiPER URDF六个关节的局部axis均为[0;0;1]。
    T_urdf=T_urdf*model.Mlist(:,:,i)* ...
        se3_exp([0;0;q(i);0;0;0]);
end
T_space=forward_poe_space(model.S_list,q,model.M);
T_body=forward_poe_body(model.B_list,q,model.M);
assert(norm(T_space-T_body,'fro')<1e-10);
assert(norm(T_space-T_urdf,'fro')<1e-10);

% RNEA-CRBA惯量关系：dq=0、gravity=0、无外力时tau=M(q)*ddq。
model_no_gravity=model;
model_no_gravity.gravity=zeros(3,1);
ddq=[0.2;-0.1;0.3;0.4;-0.2;0.1];
tau=rnea(q,zeros(6,1),ddq,model_no_gravity);
Mq=crba(q,model_no_gravity);
assert(norm(Mq-Mq','fro')<1e-9);
assert(norm(tau-Mq*ddq)<1e-8);

disp('All PiPER model tests passed.');
