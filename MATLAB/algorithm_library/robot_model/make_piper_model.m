function model = make_piper_model(urdf_path)
%MAKE_PIPER_MODEL 从PiPER URDF构建运动学和串联链动力学模型。
%   model=MAKE_PIPER_MODEL(urdf_path)
%   输入：urdf_path 厂商PiPER URDF的文件路径。省略时使用本仓库的 URDF。
%   运动学输出：S_list、B_list (6×6，[omega;v])、M (4×4，link6零位位姿)、
%      q_min、q_max [rad]、dq_max [rad/s]，及关节零位轴向w、轴上一点p。
%   动力学输出：Slist、Mlist (4×4×7)、Glist (6×6×6)、gravity [m/s^2]。
%   Mlist前6页为父link到子link零位变换，第7页为link6到工具的
%   固定变换，默认单位矩阵；如加装夹爪/传感器，须更新工具负载模型。
%   注意：base_link与world间的固定关节必须为单位变换；此函数只支持
%   URDF中joint1...joint6组成的转动串联链。标称惯量不等于辨识后参数。
%   用法：model=make_piper_model('/path/to/piper_description.urdf');
if nargin<1 || isempty(urdf_path)
    repo_dir=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    urdf_path=fullfile(repo_dir,'models','urdf','piper_description.urdf');
end
assert(isfile(urdf_path), '找不到URDF文件：%s',urdf_path);
doc=xmlread(urdf_path);
joint_nodes=doc.getElementsByTagName('joint');
link_nodes=doc.getElementsByTagName('link');
n=6;

base_joint=find_named(joint_nodes,'world_to_base_link');
assert(strcmp(char(base_joint.getAttribute('type')),'fixed'), ...
    'world_to_base_link必须为固定关节');
base_origin=direct_child(base_joint,'origin');
assert(norm(read_triplet(base_origin,'xyz',[0;0;0]))<1e-12 && ...
    norm(read_triplet(base_origin,'rpy',[0;0;0]))<1e-12, ...
    'world到base_link不是单位变换，需显式处理基座外参');

model.name='piper';
model.urdf_path=char(urdf_path);
model.S_list=zeros(6,n);
model.B_list=zeros(6,n);
model.Mlist=zeros(4,4,n+1);
model.Glist=zeros(6,6,n);
model.q_min=zeros(n,1);
model.q_max=zeros(n,1);
model.dq_max=zeros(n,1);
model.effort_max=zeros(n,1);
model.w=zeros(3,n);
model.p=zeros(3,n);
model.mass=zeros(n,1);
model.com=zeros(3,n);
model.I_com=zeros(3,3,n);
model.gravity=[0;0;-9.81];
T=eye(4);
expected_parent='base_link';
for i=1:n
    joint=find_named(joint_nodes,sprintf('joint%d',i));
    assert(strcmp(char(joint.getAttribute('type')),'revolute') && ...
        strcmp(char(direct_child(joint,'parent').getAttribute('link')), ...
        expected_parent), 'joint%d不是预期的转动串联链',i);
    child_name=char(direct_child(joint,'child').getAttribute('link'));
    assert(strcmp(child_name,sprintf('link%d',i)), ...
        'joint%d的子连杆不是link%d',i,i);
    origin=direct_child(joint,'origin');
    xyz=read_triplet(origin,'xyz',[0;0;0]);
    rpy=read_triplet(origin,'rpy',[0;0;0]);
    R=rpy_rotation(rpy);
    T_home=[R,xyz;0,0,0,1];
    model.Mlist(:,:,i)=T_home;
    T=T*T_home;
    axis=read_triplet(direct_child(joint,'axis'),'xyz',[1;0;0]);
    assert(norm(axis)>0, 'joint%d旋转轴为零',i);
    w=T(1:3,1:3)*(axis/norm(axis));
    p=T(1:3,4);
    model.w(:,i)=w;
    model.p(:,i)=p;
    model.S_list(:,i)=[w;-cross(w,p)];
    limit=direct_child(joint,'limit');
    model.q_min(i)=read_number(limit,'lower');
    model.q_max(i)=read_number(limit,'upper');
    model.dq_max(i)=read_number(limit,'velocity');
    model.effort_max(i)=read_number(limit,'effort');

    link=find_named(link_nodes,child_name);
    inertial=direct_child(link,'inertial');
    inertial_origin=direct_child(inertial,'origin');
    c=read_triplet(inertial_origin,'xyz',[0;0;0]);
    R_inertial=rpy_rotation(read_triplet(inertial_origin,'rpy',[0;0;0]));
    mass=read_number(direct_child(inertial,'mass'),'value');
    inertia=direct_child(inertial,'inertia');
    I_inertial=[read_number(inertia,'ixx'),read_number(inertia,'ixy'),read_number(inertia,'ixz'); ...
        read_number(inertia,'ixy'),read_number(inertia,'iyy'),read_number(inertia,'iyz'); ...
        read_number(inertia,'ixz'),read_number(inertia,'iyz'),read_number(inertia,'izz')];
    I_com=R_inertial*I_inertial*R_inertial';
    model.mass(i)=mass;
    model.com(:,i)=c;
    model.I_com(:,:,i)=I_com;
    model.Glist(:,:,i)=spatial_inertia(mass,I_com,c);
    expected_parent=child_name;
end
model.M=T;
model.B_list=adjoint(se3_inverse(T))*model.S_list;
model.Mlist(:,:,n+1)=eye(4);
model.Slist=model.S_list; % dynamics/rnea、crba中的字段名
end

function element=find_named(nodes,name)
element=[];
for index=0:nodes.getLength-1
    candidate=nodes.item(index);
    if strcmp(char(candidate.getAttribute('name')),name)
        element=candidate;
        return;
    end
end
error('URDF缺少名为%s的元素',name);
end

function element=direct_child(parent,tag)
element=[];
children=parent.getChildNodes;
for index=0:children.getLength-1
    candidate=children.item(index);
    if candidate.getNodeType==1 && strcmp(char(candidate.getNodeName),tag)
        element=candidate;
        return;
    end
end
error('URDF元素%s缺少子元素%s',char(parent.getNodeName),tag);
end

function value=read_number(element,name)
text=char(element.getAttribute(name));
value=str2double(text);
assert(~isempty(text) && isfinite(value), 'URDF属性%s不是有限数字',name);
end

function vector=read_triplet(element,name,default)
text=char(element.getAttribute(name));
if isempty(text)
    vector=default;
else
    vector=sscanf(text,'%f');
end
assert(numel(vector)==3 && all(isfinite(vector)), ...
    'URDF属性%s必须包含三个有限数字',name);
end

function R=rpy_rotation(rpy)
roll=rpy(1); pitch=rpy(2); yaw=rpy(3);
cr=cos(roll); sr=sin(roll);
cp=cos(pitch); sp=sin(pitch);
cy=cos(yaw); sy=sin(yaw);
Rx=[1,0,0;0,cr,-sr;0,sr,cr];
Ry=[cp,0,sp;0,1,0;-sp,0,cp];
Rz=[cy,-sy,0;sy,cy,0;0,0,1];
R=Rz*Ry*Rx;
end
