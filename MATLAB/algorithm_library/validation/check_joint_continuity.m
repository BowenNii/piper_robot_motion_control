function report = check_joint_continuity(q_rad,max_step_rad)
%CHECK_JOINT_CONTINUITY 检查相邻关节解/样本的跳变，不自动做角度wrap。
% 输入：q_rad N×n；max_step_rad 标量或n个元素，rad，必须按任务设置。
% 输出：ok、delta_rad、rows、joints；行号是跳变后的采样点。
% 有限转动关节不能随意把2pi跳变当作等价解；本函数使用原始角差。
% 该检查不是速度/加速度限幅，不能替代轨迹安全检查。
validateattributes(q_rad,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(max_step_rad,{'numeric'},{'real','finite','vector','positive'});
threshold=max_step_rad(:)';
assert(isscalar(threshold) || numel(threshold)==size(q_rad,2), ...
    '阈值必须是标量或每个关节一个值');
delta=diff(q_rad,1,1); bad=abs(delta)>threshold;
[rows,joints]=find(bad);
report=struct('ok',~any(bad(:)),'delta_rad',delta, ...
    'rows',rows+1,'joints',joints,'violation_mask',bad);
end
