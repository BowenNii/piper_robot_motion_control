function result = topp_ra_parameterize(s_grid,q_path,dq_ds,ddq_ds,dq_max,ddq_max)
%TOPP_RA_PARAMETERIZE 固定几何路径的离散TOPP-RA时间参数化。
%   result=TOPP_RA_PARAMETERIZE(s_grid,q_path,dq_ds,ddq_ds,dq_max,ddq_max)
%   输入：s_grid 严格递增N×1；q_path、dq_ds、ddq_ds均为n×N，
%   分别表示q(s)、dq/ds、d²q/ds²；dq_max、ddq_max为正的n维关节上限。
%   两端路径速度固定为0。限制 |dq_i/dt|<=dq_max_i，
%   |d²q_i/dt²|<=ddq_max_i；不含力矩和jerk限制。
%   状态x=(ds/dt)^2，控制u=d²s/dt²，离散动态x_(k+1)=x_k+2*ds*u。
%   后向LP求各节点可达x区间，再前向LP取最大可达x_(k+1)。
%   输出：s、t、q、sdot、dq（节点）、ddq_segment_start/end、sddot_segment、
%   reachable_interval及max_velocity_violation/max_acceleration_violation。
%   依赖Optimization Toolbox的linprog。输入路径导数必须真实、连续。
s=s_grid(:);
N=numel(s);
n=size(q_path,1);
assert(N>=3 && n>=1 && all(isfinite(s)) && all(diff(s)>0) && ...
    isequal(size(q_path),[n,N]) && isequal(size(dq_ds),[n,N]) && ...
    isequal(size(ddq_ds),[n,N]) && ...
    all(isfinite(q_path(:))) && all(isfinite(dq_ds(:))) && ...
    all(isfinite(ddq_ds(:))), '路径或路径导数维度/数值无效');
dq_max=expand_positive(dq_max,n,'dq_max');
ddq_max=expand_positive(ddq_max,n,'ddq_max');

x_cap=Inf(1,N);
for k=1:N
    active=abs(dq_ds(:,k))>1e-12;
    if any(active)
        x_cap(k)=min((dq_max(active)./abs(dq_ds(active,k))).^2);
    end
end
options=optimoptions('linprog','Display','off');
K=zeros(2,N);
K(:,N)=[0;0];
for k=N-1:-1:1
    ds=s(k+1)-s(k);
    [A,b]=segment_inequalities(dq_ds(:,k),ddq_ds(:,k), ...
        dq_ds(:,k+1),ddq_ds(:,k+1),ddq_max,ds);
    lb=[0;K(1,k+1)];
    ub=[x_cap(k);K(2,k+1)];
    [x_min,~,flag_min]=linprog([1;0],A,b,[],[],lb,ub,options);
    [x_max,~,flag_max]=linprog([-1;0],A,b,[],[],lb,ub,options);
    if flag_min<=0 || flag_max<=0 || ~all(isfinite(x_max))
        error('TOPP-RA:NoReachableState', ...
            '节点%d不可达、无界，或路径导数/约束不一致',k);
    end
    K(:,k)=[max(0,x_min(1));max(0,x_max(1))];
end
assert(K(1,1)<=1e-9, 'TOPP-RA:初始零速度不在可达集合内');
x=zeros(1,N);
for k=1:N-1
    ds=s(k+1)-s(k);
    [A,b]=segment_inequalities(dq_ds(:,k),ddq_ds(:,k), ...
        dq_ds(:,k+1),ddq_ds(:,k+1),ddq_max,ds);
    b_y=b-A(:,1)*x(k);
    [x_next,~,flag]=linprog(-1,A(:,2),b_y,[],[], ...
        K(1,k+1),K(2,k+1),options);
    if flag<=0
        error('TOPP-RA:ForwardPass','节点%d前向传播不可行',k);
    end
    x(k+1)=max(0,x_next);
end
x(end)=0; % 终端可达区间被固定为零，仅消除LP的数值残差
sdot=sqrt(x);
t=zeros(1,N);
u=zeros(1,N-1);
ddq_segment_start=zeros(n,N-1);
ddq_segment_end=zeros(n,N-1);
for k=1:N-1
    ds=s(k+1)-s(k);
    denom=sdot(k)+sdot(k+1);
    if denom<=1e-12
        error('TOPP-RA:ZeroSpeedSegment', ...
            '节点%d到%d两端路径速度均为零，有限时间不可行',k,k+1);
    end
    t(k+1)=t(k)+2*ds/denom;
    u(k)=(x(k+1)-x(k))/(2*ds);
    ddq_segment_start(:,k)=ddq_ds(:,k)*x(k)+dq_ds(:,k)*u(k);
    ddq_segment_end(:,k)=ddq_ds(:,k+1)*x(k+1)+dq_ds(:,k+1)*u(k);
end
dq=dq_ds.*sdot;
result.s=s;
result.t=t;
result.q=q_path;
result.sdot=sdot;
result.dq=dq;
result.ddq_segment_start=ddq_segment_start;
result.ddq_segment_end=ddq_segment_end;
result.ddq_segment=ddq_segment_start; % 兼容旧示例：段首加速度
result.sddot_segment=u;
result.reachable_interval=K;
result.max_velocity_violation=max(0,max(abs(dq)-dq_max,[],'all'));
result.max_acceleration_violation=max([0, ...
    max(abs(ddq_segment_start)-ddq_max,[],'all'), ...
    max(abs(ddq_segment_end)-ddq_max,[],'all')]);
end

function [A,b]=segment_inequalities(qs,qss,qs_next,qss_next,amax,ds)
% 设x_k=x，x_(k+1)=y；u=(y-x)/(2ds)。
c=qs/(2*ds);
a=qss-c;
c_next=qs_next/(2*ds);
a_next=-c_next;
b_next=qss_next+c_next;
A=[a,c;-a,-c;a_next,b_next;-a_next,-b_next];
b=repmat(amax,4,1);
end

function value=expand_positive(value,n,name)
if isscalar(value)
    value=repmat(value,n,1);
else
    value=value(:);
end
assert(numel(value)==n && all(isfinite(value)) && all(value>0), ...
    '%s必须为正的n维向量或标量',name);
end
