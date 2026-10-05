function result = solve_ik_poe(S_list,M,target,q_initial,options)
% 空间旋量 6×n、零位/目标位姿、关节初值 -> IKResult 结构体。
% options 同 C++ IKOptions；method 为 newton/dls/adaptive_dls/transpose。
% 输出 q、success、iterations、position_error(m)、orientation_error(rad)、
% status(kConverged/kMaxIterations/kStalled/kNumericalFailure)。
% rotation_error 是旧字段兼容别名。失败的 q 不可作为真机命令。
if nargin<5 || isempty(options), options=struct(); end
assert(isstruct(options) && isscalar(options),'IK options必须为标量结构体');
% 兼容旧字段；同时指定新旧字段时明确拒绝歧义。
aliases={'rotation_tolerance','orientation_tolerance';'step_max','max_joint_step'};
for i=1:size(aliases,1)
    old=aliases{i,1}; new=aliases{i,2};
    if isfield(options,old)
        assert(~isfield(options,new),'不能同时指定新旧字段');
        options.(new)=options.(old); options=rmfield(options,old);
    end
end
o=ik_options(); method='dls';
if isfield(options,'method'), method=char(options.method); options=rmfield(options,'method'); end
assert(any(strcmp(method,{'newton','dls','adaptive_dls','transpose'})),'未知IK方法');
fields=fieldnames(options);
for i=1:numel(fields)
    assert(isfield(o,fields{i}),'未知IK选项：%s',fields{i});
    o.(fields{i})=options.(fields{i});
end
q=q_initial(:); n=numel(q);
assert(n>0 && isequal(size(S_list),[6,n]) && isreal(S_list) && ...
    all(isfinite(S_list(:))) && isreal(q) && all(isfinite(q)),'IK轴或初值无效');
assert(valid_pose(M) && valid_pose(target),'IK: invalid pose');
positive={'position_tolerance','orientation_tolerance','max_joint_step', ...
    'rotation_weight','svd_tolerance','sigma_threshold','lambda_max'};
for i=1:numel(positive)
    validateattributes(o.(positive{i}),{'numeric'},{'real','finite','scalar','positive'});
end
for name={'lambda'}
    validateattributes(o.(name{1}),{'numeric'},{'real','finite','scalar','nonnegative'});
end
for name={'max_iterations','max_backtracks'}
    validateattributes(o.(name{1}),{'numeric'},{'real','finite','scalar','integer','nonnegative'});
end
assert(isempty(o.q_min)==isempty(o.q_max),'IK: invalid bounds dimensions');
if ~isempty(o.q_min)
    o.q_min=o.q_min(:); o.q_max=o.q_max(:);
    assert(numel(o.q_min)==n && numel(o.q_max)==n && ...
        all(isfinite(o.q_min)) && all(isfinite(o.q_max)) && ...
        all(o.q_min<=q) && all(q<=o.q_max),'IK: invalid bounds or initial');
end
D=diag([o.rotation_weight*ones(3,1);ones(3,1)]);
result=struct('success',false,'iterations',0,'position_error',Inf, ...
    'orientation_error',Inf,'rotation_error',Inf,'q',q,'status','kMaxIterations');
for iteration=0:o.max_iterations
    T=forward_poe_space(S_list,q,M);
    [body,pos,rot]=pose_error(T,target);
    result.q=q; result.iterations=iteration; result.position_error=pos;
    result.orientation_error=rot; result.rotation_error=rot;
    if any(~isfinite([body;pos;rot]))
        result.status='kNumericalFailure'; return;
    end
    if pos<=o.position_tolerance && rot<=o.orientation_tolerance
        result.success=true; result.status='kConverged'; return;
    end
    if iteration==o.max_iterations, break; end
    A=D*adjoint_inverse(T)*jacobian_space(S_list,q); b=D*body;
    if strcmp(method,'transpose')
        g=A'*b; denominator=norm(A*g)^2;
        if denominator>0, step=(norm(g)^2/denominator)*g; else, step=zeros(n,1); end
    else
        [U,S,V]=svd(A,'econ'); sigma=diag(S); k=numel(sigma); lambda=0;
        if strcmp(method,'dls'), lambda=o.lambda; end
        if strcmp(method,'adaptive_dls')
            sigma_min=sigma(end); if n<6, sigma_min=0; end
            lambda=o.lambda_max*sqrt(max(0,1-min(1,sigma_min/o.sigma_threshold)^2));
        end
        if lambda>0
            gain=sigma./(sigma.^2+lambda^2);
        else
            gain=zeros(k,1); keep=sigma>o.svd_tolerance; gain(keep)=1./sigma(keep);
        end
        step=V(:,1:k)*diag(gain)*U(:,1:k)'*b;
    end
    if any(~isfinite(step)), result.status='kNumericalFailure'; return; end
    largest=max(abs(step));
    if largest>o.max_joint_step, step=step*(o.max_joint_step/largest); end
    accepted=false; scale=1;
    for backtrack=0:o.max_backtracks
        trial=q+scale*step;
        if ~isempty(o.q_min), trial=clamp(trial,o.q_min,o.q_max); end
        trial_body=pose_error(forward_poe_space(S_list,trial,M),target);
        if all(isfinite(trial_body)) && norm(D*trial_body)^2<norm(b)^2
            q=trial; accepted=true; break;
        end
        scale=scale/2;
    end
    if ~accepted, result.status='kStalled'; return; end
end
end

function valid = valid_pose(T)
valid=isnumeric(T) && isreal(T) && isequal(size(T),[4,4]) && all(isfinite(T(:)));
if ~valid, return; end
R=T(1:3,1:3);
valid=norm(R'*R-eye(3),'fro')<1e-8 && abs(det(R)-1)<1e-8 && ...
    norm(T(4,:)-[0,0,0,1])<1e-8;
end

function [body,position,rotation] = pose_error(T,target)
R=T(1:3,1:3)'*target(1:3,1:3);
s=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
body=se3_log(se3_inverse(T)*target);
position=norm(T(1:3,4)-target(1:3,4));
rotation=atan2(norm(s)/2,clamp((trace(R)-1)/2,-1,1));
end
