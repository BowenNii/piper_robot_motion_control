%TEST_TRAJECTORY 关节原点、末端位姿、TOPP-RA约束验证。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
root=fullfile(fileparts(mfilename('fullpath')),'..','algorithm_library');
addpath(fullfile(root,'math'),fullfile(root,'dynamics'), ...
    fullfile(root,'kinematics'),fullfile(root,'robot_model'),fullfile(root,'trajectory'));
model=make_piper_model();
q=[0.2;1;-1;0.3;-0.2;0.4];
[points,T_end]=get_joint_points(q,model);
assert(isequal(size(points),[3,7]));
assert(norm(points(:,1)-[0;0;0.123])<1e-12);
assert(norm(points(:,end)-T_end(1:3,4))<1e-12);
assert(norm(T_end-forward_poe_space(model.S_list,q,model.M),'fro')<1e-10);

s=linspace(0,1,21);
q_path=[s;-2*s];
qs=repmat([1;-2],1,numel(s));
qss=zeros(size(qs));
result=topp_ra_parameterize(s,q_path,qs,qss,[1;2],[2;4]);
assert(all(isfinite(result.t)) && all(diff(result.t)>0));
assert(result.sdot(1)==0 && result.sdot(end)==0);
assert(result.max_velocity_violation<1e-8);
assert(result.max_acceleration_violation<1e-8);
assert(abs(result.t(end)-1.5)<1e-6);

% 曲线路径：数值导数与传入的解析dq/ds保持一致。
s=linspace(0,1,101);
q_path=sin(pi*s);
qs=pi*cos(pi*s);
qss=-pi^2*sin(pi*s);
assert(max(abs(gradient(q_path,s)-qs))<0.002);
result=topp_ra_parameterize(s,q_path,qs,qss,2,8);
assert(result.max_velocity_violation<1e-7);
assert(result.max_acceleration_violation<1e-7);
disp('All trajectory tests passed.');
