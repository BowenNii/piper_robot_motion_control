function report = check_joint_limits(q_rad, lower_rad, upper_rad, tolerance_rad)
%CHECK_JOINT_LIMITS 检查N×n关节角，不修改数据、不判断碰撞安全。
% 输入：q_rad N×n [rad]；上下限各n个元素 [rad]。
% tolerance_rad 非负容差[rad]，默认0；应明确记录，不擅自放宽URDF限位。
% 输出：ok、valid_rows、violation_mask；rows/joints列出超限位置；
%       excess_rad为超过原始边界的量，不扣除容差。
if nargin<4, tolerance_rad=0; end
validateattributes(q_rad,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(lower_rad,{'numeric'},{'real','finite','vector'});
validateattributes(upper_rad,{'numeric'},{'real','finite','vector'});
validateattributes(tolerance_rad,{'numeric'},{'real','finite','scalar','nonnegative'});
lo=lower_rad(:)'; hi=upper_rad(:)';
assert(numel(lo)==size(q_rad,2) && numel(hi)==size(q_rad,2), ...
    '上下限元素数必须与关节列数相同');
assert(all(lo<=hi),'下限不能大于上限');
excess=max(max(lo-q_rad,q_rad-hi),0);
bad=excess>tolerance_rad;
[rows,joints]=find(bad);
report=struct('ok',~any(bad(:)),'valid_rows',~any(bad,2), ...
    'violation_mask',bad,'rows',rows,'joints',joints, ...
    'excess_rad',excess(bad));
end
