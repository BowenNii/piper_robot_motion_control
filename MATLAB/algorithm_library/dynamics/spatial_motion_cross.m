function ad = spatial_motion_cross(V)
%SPATIAL_MOTION_CROSS 速度旋量的运动交叉矩阵 ad_V。
%   输入：V=[omega;v]，角速度rad/s、线速度m/s。
%   输出：6x6矩阵，满足 ad_V*U=[V,U]。
%   示例：ad=spatial_motion_cross([0;0;1;0;0;0]);
assert(numel(V)==6, 'V必须为6维旋量');
ad = little_ad(V(:));
end
