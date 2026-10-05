function result = solve_cartesian_velocity(J,v_command,config)
% 与 C++ 同名接口：6×n Jacobian、同系 Twist、配置 -> 速度求解结果。
% region=0/1/2 对应 kSafe/kDamped/kCritical；region_name 是辅助文字。
% 不包含位置、速度、加速度限幅，也不执行任何真机通信。
if nargin<3 || isempty(config), config=struct(); end
assert(isstruct(config) && isscalar(config),'config必须为标量结构体');
c=cartesian_velocity_config(); names=fieldnames(config);
for i=1:numel(names)
    assert(isfield(c,names{i}),'未知配置字段：%s',names{i}); c.(names{i})=config.(names{i});
end
validateattributes(J,{'numeric'},{'real','finite','2d','nrows',6,'nonempty'});
v_command=v_command(:);
validateattributes(v_command,{'numeric'},{'real','finite','numel',6});
validateattributes(c.lambda_max,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(c.sigma_safe,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(c.sigma_critical,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(c.minimum_speed_scale,{'numeric'},{'real','finite','scalar','>=',0,'<=',1});
assert(c.sigma_safe>c.sigma_critical,'sigma_safe必须大于sigma_critical');
sigma_min=minimum_singular_value(J);
if sigma_min>=c.sigma_safe
    region=0; scale=1;
elseif sigma_min<=c.sigma_critical
    region=2; scale=c.minimum_speed_scale;
else
    region=1; t=(sigma_min-c.sigma_critical)/(c.sigma_safe-c.sigma_critical);
    scale=c.minimum_speed_scale+(1-c.minimum_speed_scale)*t^2*(3-2*t);
end
[P,lambda]=dls_adaptive(J,c.lambda_max,c.sigma_safe);
labels={'kSafe','kDamped','kCritical'};
result=struct('q_dot_command',P*(scale*v_command),'sigma_min',sigma_min, ...
    'speed_scale',scale,'region',region,'region_name',labels{region+1},'lambda',lambda);
end
