function config = cartesian_velocity_config()
% 与 C++ CartesianVelocityControllerConfig 相同默认值。
config=struct('lambda_max',0.1,'sigma_safe',0.1,'sigma_critical',0.01, ...
    'minimum_speed_scale',0.1);
end
