function report = check_ik_solution(S_list,M,q_rad,T_target,lower_rad,upper_rad,cfg)
%CHECK_IK_SOLUTION 对逆运动学（IK）解的专用验证工具函数，同时检查关节限位。
% 输入：S_list 6×n [omega;v]，M/T_target 4×4，q_rad n个元素；
%       上下限n个元素rad；cfg可省略。
% cfg.position_tolerance_m默认1e-4；rotation_tolerance_rad默认1e-3；
% cfg.limit_tolerance_rad默认0。容差属于实验设置，不代表真机安全认证。
% 输出：ok、T_fk、position_error_m、rotation_error_rad、limits。
% 注意：不求解IK，也不判断路径碰撞；失败求解器仍需在主脚本处理。
if nargin<7, cfg=struct; end
pt=get_value(cfg,'position_tolerance_m',1e-4);
rt=get_value(cfg,'rotation_tolerance_rad',1e-3);
lt=get_value(cfg,'limit_tolerance_rad',0);
validateattributes(pt,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(rt,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(q_rad,{'numeric'},{'real','finite','vector','nonempty'});
assert(check_se3(M).ok && check_se3(T_target).ok,'M和目标位姿必须是有效SE(3)');
limits=check_joint_limits(q_rad(:)',lower_rad,upper_rad,lt);
T_fk=forward_poe_space(S_list,q_rad(:),M);
[p,r]=pose_error(T_fk,T_target);
report=struct('ok',limits.ok && p<=pt && r<=rt,'T_fk',T_fk, ...
    'position_error_m',p,'rotation_error_rad',r,'limits',limits);
end

function v=get_value(cfg,name,default)
v=default;
if isfield(cfg,name), v=cfg.(name); end
end
