clear; clc;
format long g;
% Craig 改进DH：Ai = Rx(alpha)*Tx(a)*Rz(qi+offset)*Tz(d)
% 基准B：原点p1，方向与URDF base_link一致（右、内、上）。
% 所有H(:,:,i)是零位时 ^B T_i；A0(:,:,i)是 ^{i-1}T_i。
% 不需要 Robotics Toolbox / Symbolic Toolbox。

%% 1. 与当前piper_model.cpp一致的原始URDF参数（每列一个关节）
xyz = [0 0 .28503 -.02198 0 8.8259e-5;
       0 0 0 -.25075 0 -.091;
       .123 0 0 0 0 0];
rpy_raw = [0 1.5707963 0 1.5707963 -1.5707963 1.5707963;
           0 -.1357866 0 0 0 0;
           0 -3.1415926 -1.7938494 0 0 0];
% URDF中的有限位数pi/2、pi带来约1e-8 rad偏差，导致w2的x分量
% 非严格为0。要严格保持用户要求的x1=[1;0;0]和alpha1=-pi/2，
% 明确把这些名义直角恢复为精确值。其余几何参数不变。
% 同时保留原始模型，最后单独报告与C++原始模型的偏差。
rpy = rpy_raw;
rpy(1,[2 4 6]) = pi/2;
rpy(1,5) = -pi/2;
rpy(3,2) = -pi;
n = 6;
[w,p,S,M] = urdf_model(xyz,rpy);
[~,~,S_raw,M_raw] = urdf_model(xyz,rpy_raw);
T_world_B = eye(4); T_world_B(1:3,4) = p(:,1);
pB = p-p(:,1);

%% 2. 公法线建系：zi=wi，xi沿轴i和轴i+1的公法线
% oi取轴i上的公法线起点，不一定是URDF joint原点pi。
H = zeros(4,4,n);
relations = cell(n-1,1);
for i=1:n-1
    [oi,xi,relations{i}] = common_normal(pB(:,i),w(:,i), ...
                                            pB(:,i+1),w(:,i+1));
    zi = w(:,i);
    xi = unit(xi-zi*dot(zi,xi));
    yi = unit(cross(zi,xi));
    xi = unit(cross(yi,zi));
    H(:,:,i) = [xi yi zi oi; 0 0 0 1];
end
% 用户指定的零位关节1坐标系，不能让叉积的符号替换它。
H(:,:,1) = eye(4);
% 末关节没有后继轴：选择link6的x方向，原点选p6。
z6 = w(:,6);
x6 = unit(M(1:3,1)-z6*dot(z6,M(1:3,1)));
y6 = unit(cross(z6,x6));
H(:,:,6) = [x6 y6 z6 pB(:,6); 0 0 0 1];
T_B_DH0 = eye(4);
T_DH6_EE = H(:,:,6) \ (T_world_B \ M);

%% 3. 相邻变换 -> 代数提取 -> 重构检查
alpha=zeros(n,1); a=alpha; d=alpha; offset=alpha; residual=alpha;
A0=zeros(4,4,n);
previous=T_B_DH0;
for i=1:n
    A0(:,:,i)=previous\H(:,:,i);
    A=A0(:,:,i);
    % MATLAB从1开始编号；与C++ extract_modified_dh公式相同。
    alpha(i)=atan2(-A(2,3),A(3,3));
    offset(i)=atan2(-A(1,2),A(1,1));
    a(i)=A(1,4);
    d(i)=-sin(alpha(i))*A(2,4)+cos(alpha(i))*A(3,4);
    residual(i)=norm(A-mdh(alpha(i),a(i),d(i),offset(i)),'fro');
    assert(residual(i)<1e-10,'关节%d不符合改进DH结构',i);
    assert(norm(H(1:3,3,i)-w(:,i))<1e-10,'关节轴方向错误');
    assert(norm(cross(H(1:3,4,i)-pB(:,i),w(:,i)))<1e-10, ...
           'DH原点不在关节轴上');
    previous=H(:,:,i);
end
assert(abs(alpha(2)+pi/2)<1e-10,'alpha1应为-pi/2');
% DH表行i使用alpha(i)=alpha_{i-1}，因此alpha1位于第2行。
DH=table((1:n)',alpha,a,d,offset,residual, 'VariableNames', ...
    {'i','alpha_previous_rad','a_previous_m','d_i_m','theta_offset_rad','error'});
disp(DH);
for i=1:n
    fprintf('\n零位 ^B T_%d：\n',i); disp(H(:,:,i));
    fprintf('零位 ^%d T_%d：\n',i-1,i); disp(A0(:,:,i));
end
disp('T_world_B：'); disp(T_world_B);
disp('T_DH6_EE：'); disp(T_DH6_EE);
disp('空间轴w（world表达）：'); disp(w);
disp('轴上一点p（world表达）：'); disp(p);
disp('POE空间旋量S=[w;-cross(w,p)]：'); disp(S);
disp('末端零位M（link6，不包含额外夹爪TCP）：'); disp(M);
for i=1:n-1
    fprintf('J%d-J%d：%s\n',i,i+1,relations{i});
end

%% 4. 同一q下比较DH、POE、URDF；同时检查DH隐含的空间轴
S_dh=zeros(6,n);
T=T_world_B*T_B_DH0;
for i=1:n
    T=T*mdh(alpha(i),a(i),d(i),offset(i));
    wi=T(1:3,3); pi_axis=T(1:3,4);
    S_dh(:,i)=[wi;-cross(wi,pi_axis)];
end
assert(norm(S_dh-S,'fro')<1e-10,'DH空间轴与POE不一致');

qmin=[-2.6179938;0;-2.9670597;-1.7453292;-1.2217304;-2.0943951];
qmax=[2.6179938;3.1415926;0;1.7453292;1.2217304;2.0943951];
rng(42);
% 零位、逐个关节单独转动、限位端点、随机100组。
single_angles=[.2;.2;-.2;.2;.2;.2];
Q=[zeros(n,1),diag(single_angles),qmin,qmax, ...
   qmin+(qmax-qmin).*rand(n,100)];
errors=zeros(size(Q,2),6);
for k=1:size(Q,2)
    q=Q(:,k);
    Tdh=T_world_B*T_B_DH0;
    for i=1:n
        Tdh=Tdh*mdh(alpha(i),a(i),d(i),q(i)+offset(i));
    end
    Tdh=Tdh*T_DH6_EE;
    Tpoe=poe(S,q,M);
    Turdf=urdf_fk(xyz,rpy,q);
    Traw=poe(S_raw,q,M_raw);
    errors(k,1:2)=pose_error(Tdh,Tpoe);
    errors(k,3:4)=pose_error(Tdh,Turdf);
    errors(k,5:6)=pose_error(Tdh,Traw);
    assert(all(errors(k,1:4)<1e-10),'样本%d交叉验证失败',k);
end
fprintf('\n验证%d组q，最大位置误差(m)/姿态误差(rad)：\n',size(Q,2));
fprintf('DH vs 名义POE：   %.3e / %.3e\n',max(errors(:,1:2),[],1));
fprintf('DH vs 名义URDF：  %.3e / %.3e\n',max(errors(:,3:4),[],1));
fprintf('DH vs 原始C++参数：%.3e / %.3e\n',max(errors(:,5:6),[],1));
% 这里的1e-6仅是直角数值恢复带来的模型差异检查，非真机精度指标。
assert(all(max(errors(:,5:6),[],1)<1e-6),'与原始URDF偏差过大');
disp('[PASS] 改进DH空间旋量、零位及多位形正运动学验证通过。');

%% 局部函数
function [w,p,S,M]=urdf_model(xyz,rpy)
    n=size(xyz,2); w=zeros(3,n); p=w; S=zeros(6,n); T=eye(4);
    for i=1:n
        T=T*origin_transform(xyz(:,i),rpy(:,i));
        w(:,i)=unit(T(1:3,3)); p(:,i)=T(1:3,4);
        S(:,i)=[w(:,i);-cross(w(:,i),p(:,i))];
    end
    M=T;
end
function T=urdf_fk(xyz,rpy,q)
    T=eye(4);
    for i=1:numel(q)
        T=T*origin_transform(xyz(:,i),rpy(:,i))* ...
            [rz(q(i)),zeros(3,1);0 0 0 1];
    end
end
function T=origin_transform(p,rpy)
    R=rz(rpy(3))*ry(rpy(2))*rx(rpy(1));
    T=[R p;0 0 0 1];
end
function [o,x,relation]=common_normal(p,z,pnext,znext)
    z=unit(z); znext=unit(znext); c=cross(z,znext);
    if norm(c)<1e-12
        % 平行轴：投影去掉沿轴分量，公法线起点选当前p。
        o=p; delta=pnext-p; delta=delta-z*dot(z,delta);
        if norm(delta)<1e-12
            [~,j]=min(abs(z)); e=zeros(3,1); e(j)=1;
            x=unit(cross(z,e)); relation='重合（公法线方向人为选择）';
        else
            x=unit(delta); relation='平行';
        end
    else
        % 异面/相交：两轴最近点。保持o严格在当前轴上，不平均原点。
        st=[z,-znext]\(pnext-p);
        o=p+st(1)*z; other=pnext+st(2)*znext;
        x=unit(c); % 符号任选，但后续a、alpha必须与此一致
        if norm(other-o)<1e-10
            relation='相交';
        else
            relation='异面';
        end
    end
end
function A=mdh(alpha,a,d,theta)
    ca=cos(alpha); sa=sin(alpha); ct=cos(theta); st=sin(theta);
    A=[ct -st 0 a;st*ca ct*ca -sa -d*sa; ...
       st*sa ct*sa ca d*ca;0 0 0 1];
end
function T=poe(S,q,M)
    % 用MATLAB通用矩阵指数，避免复用DH实现作为参考。
    T=eye(4);
    for i=1:numel(q)
        hat=[skew(S(1:3,i)),S(4:6,i);0 0 0 0];
        T=T*expm(hat*q(i));
    end
    T=T*M;
end
function e=pose_error(A,B)
    R=A(1:3,1:3)'*B(1:3,1:3);
    sine=.5*norm([R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]);
    cosine=max(-1,min(1,(trace(R)-1)/2));
    e=[norm(A(1:3,4)-B(1:3,4)),atan2(sine,cosine)];
end
function v=unit(v)
    assert(norm(v)>1e-14,'不能单位化零向量'); v=v/norm(v);
end
function K=skew(v)
    K=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end
function R=rx(t)
    R=[1 0 0;0 cos(t) -sin(t);0 sin(t) cos(t)];
end
function R=ry(t)
    R=[cos(t) 0 sin(t);0 1 0;-sin(t) 0 cos(t)];
end
function R=rz(t)
    R=[cos(t) -sin(t) 0;sin(t) cos(t) 0;0 0 1];
end
