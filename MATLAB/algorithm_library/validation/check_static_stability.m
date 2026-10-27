function report = check_static_stability(q_rad,position_m,max_joint_range_rad,max_position_range_m)
%CHECK_STATIC_STABILITY 检查选定静态窗口内的峰峰变化范围。
% 输入：q_rad N×n，position_m N×3；两个阈值为非负标量rad、m。
% 输出：stable、joint_range_rad 1×n、position_range_m 1×3、sample_count。
% 至少2条样本；只检查已记录精度下的稳定性，不证明机械臂物理静止。
% 窗口选择/最短持续时间留在主脚本，不能拿一个点冒充停稳验证。
validateattributes(q_rad,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(position_m,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(max_joint_range_rad,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(max_position_range_m,{'numeric'},{'real','finite','scalar','nonnegative'});
assert(size(position_m,2)==3 && size(q_rad,1)==size(position_m,1), ...
    '位置必须N×3且与关节角行数相同');
qr=max(q_rad,[],1)-min(q_rad,[],1);
pr=max(position_m,[],1)-min(position_m,[],1);
report=struct('stable',size(q_rad,1)>=2 && all(qr<=max_joint_range_rad) ...
    && all(pr<=max_position_range_m),'joint_range_rad',qr, ...
    'position_range_m',pr,'sample_count',size(q_rad,1));
end
