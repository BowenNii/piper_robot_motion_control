function report = validate_fk_samples(S_list,M,q_rad,T_reference,cfg)
%VALIDATE_FK_SAMPLES MATLAB独立POE回算，与指定参考位姿逐条比较。
% 输入：S_list 6×n，M 4×4；q_rad N×n；T_reference 4×4×N。
%       所有平移m、角度rad；参考可以是C++或CAN，主脚本明确说明来源。
% cfg.position_tolerance_m、rotation_tolerance_rad必须显式提供。
% 输出：T_matlab、position_error_m、rotation_error_rad、delta_p_m(N×3)、
%       valid_rows与ok。任何数据格式错误直接报错，不跳过、不删行。
% 用法：r=validate_fk_samples(model.S_list,model.M,q,T_cpp,cfg);
% 与CAN一致只证明模型一致性，不代表外部测量的真机精度。
validateattributes(q_rad,{'numeric'},{'real','finite','2d','nonempty'});
assert(check_se3(M).ok,'M必须是有效SE(3)');
assert(isnumeric(T_reference) && isreal(T_reference) ...
    && ndims(T_reference)<=3 && size(T_reference,1)==4 ...
    && size(T_reference,2)==4 && size(T_reference,3)==size(q_rad,1), ...
    '参考位姿必须是4×4×N');
assert(isfield(cfg,'position_tolerance_m') && isfield(cfg,'rotation_tolerance_rad'), ...
    '必须显式设置位置与姿态容差');
pt=cfg.position_tolerance_m; rt=cfg.rotation_tolerance_rad;
validateattributes(pt,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(rt,{'numeric'},{'real','finite','scalar','nonnegative'});
N=size(q_rad,1); poses=zeros(4,4,N); p=zeros(N,1); r=p; dp=zeros(N,3);
for k=1:N
    poses(:,:,k)=forward_poe_space(S_list,q_rad(k,:)',M);
    [p(k),r(k),delta]=pose_error(poses(:,:,k),T_reference(:,:,k));
    dp(k,:)=delta';
end
valid=p<=pt & r<=rt;
report=struct('ok',all(valid),'valid_rows',valid,'T_matlab',poses, ...
    'position_error_m',p,'rotation_error_rad',r,'delta_p_m',dp);
end
