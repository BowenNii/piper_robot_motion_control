function result = cartesian_velocity_controller(J, v_command, config)
%CARTESIAN_VELOCITY_CONTROLLER 末端Twist[omega;v] -> 关节速度(rad/s)。
%   使用奇异值分区、末端速度缩放和自适应DLS；仅用于离线仿真。
% 扩展接口：欠驱动时按完整6维任务判定为critical（不同于当前C++）。
% 要比较C++的原始速度求解行为，请使用solve_cartesian_velocity。
%   可选安全约束：config.q_dot_max；以及成组的config.q_current、
%   config.q_min、config.q_max、config.dt；还可加config.q_dot_previous
%   和config.q_ddot_max。约束为离散一步预测，不能替代硬件限位、
%   急停、看门狗或真实笛卡尔位姿反馈闭环。
if nargin < 3 || isempty(config)
    config = struct();
end
if ~isfield(config, 'lambda_max'), config.lambda_max = 0.1; end
if ~isfield(config, 'sigma_safe'), config.sigma_safe = 0.1; end
if ~isfield(config, 'sigma_critical'), config.sigma_critical = 0.01; end
if ~isfield(config, 'minimum_speed_scale'), config.minimum_speed_scale = 0.1; end
if ~isfield(config, 'svd_tolerance'), config.svd_tolerance = 1e-10; end

assert(size(J,1) == 6 && size(J,2) > 0, 'J必须为6xn');
assert(numel(v_command) == 6 && all(isfinite(v_command(:))), 'v_command必须为有限6维向量');
assert(all(isfinite(J(:))), 'J包含非有限数值');
assert(all(structfun(@(x) isnumeric(x) || islogical(x),config)), ...
       '配置字段类型无效');
assert(all(isfinite([config.lambda_max,config.sigma_safe, ...
       config.sigma_critical,config.minimum_speed_scale,config.svd_tolerance])) && ...
       all([numel(config.lambda_max),numel(config.sigma_safe), ...
       numel(config.sigma_critical),numel(config.minimum_speed_scale), ...
       numel(config.svd_tolerance)]==1) && ...
       config.lambda_max >= 0 && config.sigma_safe > config.sigma_critical && ...
       config.sigma_critical > 0 && config.minimum_speed_scale >= 0 && ...
       config.minimum_speed_scale <= 1 && config.svd_tolerance > 0, '配置参数无效');

[U, Sigma, V] = svd(J, 'econ');
sigma = diag(Sigma);
k=numel(sigma); U=U(:,1:k); V=V(:,1:k);
% 少于6列时，全6维任务必然缺失方向。
if size(J,2) < 6
    sigma_min = 0;
else
    sigma_min = min(sigma);
end

if sigma_min >= config.sigma_safe
    region = 'safe';
    speed_scale = 1;
elseif sigma_min <= config.sigma_critical
    region = 'critical';
    speed_scale = config.minimum_speed_scale;
else
    region = 'damped';
    t = (sigma_min-config.sigma_critical) / ...
        (config.sigma_safe-config.sigma_critical);
    h = t^2*(3-2*t); % smoothstep
    speed_scale = config.minimum_speed_scale + ...
        (1-config.minimum_speed_scale)*h;
end

if sigma_min >= config.sigma_safe
    lambda = 0;
else
    lambda = config.lambda_max * ...
        sqrt(max(0, 1-(sigma_min/config.sigma_safe)^2));
end

if lambda > 0
    gain = sigma ./ (sigma.^2 + lambda^2);
else
    gain = zeros(size(sigma));
    valid = sigma > config.svd_tolerance;
    gain(valid) = 1 ./ sigma(valid);
end

v_safe = speed_scale*v_command(:);
q_dot_raw=V*diag(gain)*U'*v_safe;
q_dot_command=q_dot_raw;
n=size(J,2);
lower=-Inf(n,1);
upper=Inf(n,1);
if isfield(config,'q_dot_max')
    vmax=expand_positive(config.q_dot_max,n,'q_dot_max');
    lower=max(lower,-vmax);
    upper=min(upper,vmax);
end
joint_fields={'q_current','q_min','q_max','dt'};
has_joint_fields=cellfun(@(name) isfield(config,name),joint_fields);
if any(has_joint_fields)
    assert(all(has_joint_fields), ...
        '位置约束必须同时提供q_current、q_min、q_max、dt');
    q=config.q_current(:);
    q_min=config.q_min(:);
    q_max=config.q_max(:);
    dt=config.dt;
    assert(numel(q)==n && numel(q_min)==n && numel(q_max)==n && ...
        all(isfinite(q)) && all(isfinite(q_min)) && all(isfinite(q_max)) && ...
        all(q_min<=q) && all(q<=q_max) && ...
        isscalar(dt) && isfinite(dt) && dt>0, ...
        '关节状态、限位或dt无效');
    lower=max(lower,(q_min-q)/dt);
    upper=min(upper,(q_max-q)/dt);
end
if isfield(config,'q_ddot_max') || isfield(config,'q_dot_previous')
    assert(all(has_joint_fields) && isfield(config,'q_ddot_max') && ...
        isfield(config,'q_dot_previous'), ...
        '加速度约束还需要dt、q_dot_previous、q_ddot_max');
    amax=expand_positive(config.q_ddot_max,n,'q_ddot_max');
    q_dot_previous=config.q_dot_previous(:);
    assert(numel(q_dot_previous)==n && all(isfinite(q_dot_previous)), ...
        'q_dot_previous无效');
    lower=max(lower,q_dot_previous-amax*dt);
    upper=min(upper,q_dot_previous+amax*dt);
end
assert(all(lower<=upper+1e-12), ...
    '位置、速度、加速度约束无共同可行区间；应触发上层安全停机');
q_dot_command=min(max(q_dot_command,lower),upper);
result.q_dot_unconstrained=q_dot_raw;
result.q_dot_command=q_dot_command;
result.constraint_active=any(abs(q_dot_command-q_dot_raw)>1e-12);
result.v_achieved=J*q_dot_command;
result.tracking_error=v_safe-result.v_achieved;
result.sigma_min = sigma_min;
result.speed_scale = speed_scale;
result.lambda = lambda;
result.region = region;
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
