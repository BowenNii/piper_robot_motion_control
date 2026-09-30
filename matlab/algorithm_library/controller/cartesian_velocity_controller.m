function result = cartesian_velocity_controller(J, v_command, config)
%CARTESIAN_VELOCITY_CONTROLLER 末端Twist[omega;v] -> 关节速度(rad/s)。
%   使用奇异值分区、末端速度缩放和自适应DLS；仅用于离线仿真。
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
assert(config.lambda_max >= 0 && config.sigma_safe > config.sigma_critical && ...
       config.sigma_critical > 0 && config.minimum_speed_scale >= 0 && ...
       config.minimum_speed_scale <= 1 && config.svd_tolerance > 0, '配置参数无效');

[U, Sigma, V] = svd(J, 'econ');
sigma = diag(Sigma);
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
result.q_dot_command = V*diag(gain)*U'*v_safe;
result.sigma_min = sigma_min;
result.speed_scale = speed_scale;
result.lambda = lambda;
result.region = region;
end
