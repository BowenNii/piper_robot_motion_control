function model = piper_dynamics_model()
%PIPER_DYNAMICS_MODEL PiPER六轴串联开链的URDF标称动力学模型。
%   输出model字段：Slist(6x6)、Mlist(4x4x7)、Glist(6x6x6)、
%   gravity=[0;0;-9.81]m/s^2。末端取link6；不含夹爪/负载、
%   电机转子惯量、传动与摩擦。固定base_link惯量不参与关节动力学。
%   输入数值来自models/urdf/piper_description.urdf；仅供仿真基准，
%   不能未经参数辨识、传感器校准和安全验证直接用于真机力矩前馈。
%   用法：addpath('matlab/algorithm_library/math', ...
%                 'matlab/algorithm_library/dynamics', ...
%                 'matlab/algorithm_library/model');
%         model=piper_dynamics_model(); tau=rnea(q,dq,ddq,model);

xyz=[0,0,.28503,-.02198,0,8.8259e-5;
     0,0,0,-.25075,0,-.091;
     .123,0,0,0,0,0];
rpy=[0,1.5707963,0,1.5707963,-1.5707963,1.5707963;
     0,-.1357866,0,0,0,0;
     0,-3.1415926,-1.7938494,0,0,0];
mass=[.71,1.16,.5,.38,.383,.007];
com=[.00032,.21852,-.01999,.00013,.00018,-.00003;
     -.00041,-.00861,-.14808,-.00076,-.06104,.00006;
     -.00348,.00126,-.00053,-.00394,-.00227,-.002];
% 每列：[Ixx,Iyy,Izz,Ixy,Ixz,Iyz]，均为URDF质心惯量[kg*m^2]。
Icom=[.00050027,.00111915,.01347608,.00017844,.00172318,.00152112;
      .00040879,.06763251,.00046377,.00018914,.00017936,.00138626;
      .00045802,.067559,.01363321,.00014613,.00171172,.00031152;
      .00000078,.00182916,.00161034,.00000062,.00000259,.00000202;
      .00000626,.00019233,.00000066,.00000067,.00000010,.00000261;
      .00000462,.00000767,.00000162,.00000408,.00000990,.00000002];

n=6;
model.Slist=zeros(6,n);
model.Mlist=repmat(eye(4),1,1,n+1);
model.Glist=zeros(6,6,n);
model.gravity=[0;0;-9.81];
T0i=eye(4);
for i=1:n
    R=rot_z_rad(rpy(3,i))*rot_y_rad(rpy(2,i))*rot_x_rad(rpy(1,i));
    model.Mlist(:,:,i)=make_transform(R,xyz(:,i));
    T0i=T0i*model.Mlist(:,:,i);
    w=T0i(1:3,1:3)*[0;0;1];
    p=T0i(1:3,4);
    model.Slist(:,i)=[w;-cross(w,p)];
    Ic=[Icom(1,i),Icom(4,i),Icom(5,i);
        Icom(4,i),Icom(2,i),Icom(6,i);
        Icom(5,i),Icom(6,i),Icom(3,i)];
    model.Glist(:,:,i)=spatial_inertia(mass(i),Ic,com(:,i));
end
% link6本身即算法末端；若安装夹爪，应另行加入负载惯量和固定TCP。
model.Mlist(:,:,n+1)=eye(4);
end
