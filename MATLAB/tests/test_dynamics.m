%TEST_DYNAMICS PiPER标称动力学的随机交叉验证。
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths(true);
root=fullfile(fileparts(mfilename('fullpath')),'..','algorithm_library');
addpath(fullfile(root,'math'),fullfile(root,'dynamics'),fullfile(root,'robot_model'));
model=make_piper_model('/home/nbw/piper_robot_motion_control/models/urdf/piper_description.urdf');
pi_vec=inertial_parameters(model);
rng(43);
for sample=1:10
    q=model.q_min+(model.q_max-model.q_min).*rand(6,1);
    dq=0.5*randn(6,1);
    ddq=randn(6,1);
    tau=rnea(q,dq,ddq,model);
    Y=regressor(q,dq,ddq,model);
    assert(norm(tau-Y*pi_vec)<1e-8);
    Mq=crba(q,model);
    assert(norm(Mq-Mq','fro')<1e-9);
    no_gravity=model; no_gravity.gravity=zeros(3,1);
    assert(norm(rnea(q,zeros(6,1),ddq,no_gravity)-Mq*ddq)<1e-8);
    assert(norm(gravity(q,model)-rnea(q,zeros(6,1),zeros(6,1),model))<1e-9);
end
params=struct('viscous',ones(6,1),'coulomb',2*ones(6,1), ...
    'velocity_scale',0.1*ones(6,1));
assert(all(friction(ones(6,1),params)>0));
assert(all(friction(-ones(6,1),params)<0));
disp('All dynamics tests passed.');
