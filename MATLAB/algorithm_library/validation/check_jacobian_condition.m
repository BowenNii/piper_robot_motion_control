function report = check_jacobian_condition(J,sigma_threshold)
%CHECK_JACOBIAN_CONDITION 计算雅可比奇异值和条件数，雅可比矩阵奇异性评估工具函数。
% 输入：J任务雅可比 m×n；sigma_threshold 正阈值，须由实验显式设置。
% 输出：sigma、sigma_min、condition_number、numerical_rank数值秩、near_singular是否接近奇异。
% 对m>n的矩阵，不能实现所有m维任务，task_full_row_rank=false。
% 六维雅可比混合角/线速度单位：数值取决于尺度，阈值不能跨机器人照搬。
% 可先在主脚本按特征长度缩放J；不要将条件数直接视为碰撞安全判据。
validateattributes(J,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(sigma_threshold,{'numeric'},{'real','finite','scalar','positive'});
s=svd(J); 
tol=max(size(J))*eps(max(s));
r=sum(s>tol); 
c=Inf;
%条件数 = 最大奇异值 / 最小奇异值
if s(end)>tol, c=s(1)/s(end); end
full_row=r==size(J,1);
report=struct('sigma',s,'sigma_min',s(end),'condition_number',c, ...
    'numerical_rank',r,'task_full_row_rank',full_row, ...
    'near_singular',s(end)<=sigma_threshold || ~full_row);
end
