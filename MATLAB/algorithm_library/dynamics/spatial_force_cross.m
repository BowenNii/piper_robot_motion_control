function ad_star = spatial_force_cross(V)
%SPATIAL_FORCE_CROSS 速度旋量对Wrench的对偶交叉矩阵。
%   输入：V=[omega;v]；输出：ad_star=-ad_V'。
%   因而 ad_star*F 是速度旋量作用于Wrench的交叉项。
%   示例：ad_star=spatial_force_cross([0;0;1;0;0;0]);
assert(numel(V)==6, 'V必须为6维旋量');
ad_star = -little_ad(V(:))';
end
