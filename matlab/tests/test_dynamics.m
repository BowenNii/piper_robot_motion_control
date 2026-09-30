% MATLAB串联开链动力学测试：解析单关节 + PiPER交叉恒等式。
matlab_root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(matlab_root,'algorithm_library','math'));
addpath(fullfile(matlab_root,'algorithm_library','dynamics'));
addpath(fullfile(matlab_root,'algorithm_library','model'));

% 1DOF水平伸出杆，绕y轴旋转：解析重力和惯量可手算。
one.Slist=[0;1;0;0;0;0];
one.Mlist=repmat(eye(4),1,1,2);
one.Glist=spatial_inertia(2,diag([.1,.2,.3]),[.3;0;0]);
one.gravity=[0;0;-9.81];
assert(norm(one.Glist-one.Glist','fro')<1e-12);
assert(abs(gravity(0,one)+2*9.81*.3)<1e-10);
assert(abs(gravity(pi/2,one))<1e-10);
assert(abs(crba(0,one)-(.2+2*.3^2))<1e-10);
one_zero_g=one; one_zero_g.gravity=zeros(3,1);
assert(abs(rnea(0,0,1,one_zero_g)-crba(0,one))<1e-10);
assert(abs(rnea(0,0,0,one_zero_g,[0;1;0;0;0;0])-1)<1e-10);

model=piper_dynamics_model();
M=eye(4);
for i=1:7, M=M*model.Mlist(:,:,i); end
expected_M=[.0871557533126884,0,.996194697167425,.0561352032863624;
            0,1,0,0;
            -.996194697167425,0,.0871557533126884,.21317834408058;
            0,0,0,1];
assert(norm(M-expected_M,'fro')<1e-6);

q=[.4;1.1;-.9;.3;-.4;.6];
dq=[.2;-.1;.3;-.2;.1;-.15];
ddq=[.5;-.4;.1;-.3;.2;-.1];
Mq=crba(q,model);
assert(norm(Mq-Mq','fro')<1e-10);
assert(all(eig(Mq)>0));

zero_g=model; zero_g.gravity=zeros(3,1);
tau_inertia=rnea(q,zeros(6,1),ddq,zero_g);
assert(norm(tau_inertia-Mq*ddq)<1e-9);
assert(norm(gravity(q,model)-rnea(q,zeros(6,1),zeros(6,1),model))<1e-10);

Y=regressor(q,dq,ddq,model);
pi_vec=inertial_parameters(model);
assert(norm(Y*pi_vec-rnea(q,dq,ddq,model))<1e-8);

% 独立参考：Robotics System Toolbox直接读取同一URDF。
if exist('importrobot','file')
    urdf_path=fullfile(fileparts(matlab_root),'models','urdf', ...
                       'piper_description.urdf');
    robot=importrobot(urdf_path);
    robot.DataFormat='column';
    robot.Gravity=[0,0,-9.81];
    assert(norm(rnea(q,dq,ddq,model)-inverseDynamics(robot,q,dq,ddq))<1e-9);
    assert(norm(Mq-massMatrix(robot,q),'fro')<1e-9);
    assert(norm(gravity(q,model)-gravityTorque(robot,q))<1e-9);
    rng(42);
    for sample=1:10
        q_sample=[0;pi/2;-pi/2;0;0;0]+.2*(rand(6,1)-.5);
        dq_sample=rand(6,1)-.5;
        ddq_sample=rand(6,1)-.5;
        assert(norm(rnea(q_sample,dq_sample,ddq_sample,model)- ...
                    inverseDynamics(robot,q_sample,dq_sample,ddq_sample))<1e-9);
        assert(norm(crba(q_sample,model)-massMatrix(robot,q_sample),'fro')<1e-9);
    end
end

params.viscous=.1*ones(6,1);
params.coulomb=.2*ones(6,1);
params.velocity_scale=.01*ones(6,1);
assert(norm(friction(zeros(6,1),params))<1e-12);
assert(all(friction(.2*ones(6,1),params)>0));

disp('All MATLAB dynamics tests passed.');
