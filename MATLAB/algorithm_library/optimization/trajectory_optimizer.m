function result = trajectory_optimizer(q_ref,dt,limits,weights)
%TRAJECTORY_OPTIMIZER 固定时间网格上的关节轨迹平滑QP。
%   result=TRAJECTORY_OPTIMIZER(q_ref,dt,limits,weights)
%   输入：q_ref n×N参考关节位置[rad]；dt>0采样周期[s]；
%         limits字段q_min/q_max [rad]、dq_max [rad/s]、
%         ddq_max [rad/s^2]，标量或n维；
%         weights可选tracking(默认1)、acceleration(默认0.01)。
%   输出：q n×N、dq n×(N-1)、ddq n×(N-2)、qp求解信息。
%   约束：首末节点固定；每节位置、速度和加速度满足硬约束。
%   目标：tracking*||q-q_ref||²+acceleration*||D2*q||²。
%   不包含力矩/碰撞/时间最优约束；这些需在后续阶段单独建模。
assert(isnumeric(q_ref) && ismatrix(q_ref) && isreal(q_ref) && ...
    all(isfinite(q_ref(:))) && size(q_ref,1)>0 && size(q_ref,2)>=3, ...
    'q_ref必须为有限实数n×N，N>=3');
assert(isscalar(dt) && isfinite(dt) && dt>0, 'dt必须为正数');
assert(isstruct(limits), 'limits必须为结构体');
if nargin<4 || isempty(weights)
    weights=struct();
end
n=size(q_ref,1); N=size(q_ref,2);
q_min=expand_limit(limits,'q_min',n,-Inf);
q_max=expand_limit(limits,'q_max',n,Inf);
dq_max=expand_limit(limits,'dq_max',n,Inf);
ddq_max=expand_limit(limits,'ddq_max',n,Inf);
assert(all(q_min<=q_max) && all(dq_max>0) && all(ddq_max>0), ...
    '关节限位、速度或加速度上限无效');
tracking=weight_or(weights,'tracking',1);
acceleration=weight_or(weights,'acceleration',0.01);
assert(isfinite(tracking) && tracking>0 && isfinite(acceleration) && ...
    acceleration>=0, '权重必须为非负，且tracking>0');

% MATLAB列主序下x=vec(q)：同一时间节点的n个关节相邻。
D1=kron(diff(eye(N),1,1)/dt,eye(n));
D2=kron(diff(eye(N),2,1)/dt^2,eye(n));
qvec=q_ref(:);
H=2*(tracking*eye(n*N)+acceleration*(D2'*D2));
f=-2*tracking*qvec;
A=[D1;-D1;D2;-D2];
dq_limit=repmat(dq_max,N-1,1);
ddq_limit=repmat(ddq_max,N-2,1);
b=[dq_limit;dq_limit;ddq_limit;ddq_limit];
Aeq=zeros(2*n,n*N);
Aeq(1:n,1:n)=eye(n);
Aeq(n+1:end,end-n+1:end)=eye(n);
beq=[q_ref(:,1);q_ref(:,end)];
problem=struct('H',H,'f',f,'A',A,'b',b,'Aeq',Aeq,'beq',beq, ...
    'lb',repmat(q_min,N,1),'ub',repmat(q_max,N,1),'x0',qvec);
qp=solve_qp(problem);
result.success=qp.success;
result.q=[];result.dq=[];result.ddq=[];
result.qp=qp;
if qp.success
    result.q=reshape(qp.x,n,N);
    result.dq=diff(result.q,1,2)/dt;
    result.ddq=diff(result.q,2,2)/dt^2;
end
end

function value=expand_limit(limits,name,n,default)
if isfield(limits,name)
    value=limits.(name);
else
    value=default;
end
if isscalar(value)
    value=repmat(value,n,1);
else
    value=value(:);
end
assert(numel(value)==n && all(~isnan(value)), '%s必须为标量或n维',name);
end

function value=weight_or(weights,name,default)
if isfield(weights,name)
    value=weights.(name);
else
    value=default;
end
assert(isscalar(value), '%s必须为标量',name);
end
