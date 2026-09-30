function G = spatial_inertia(m, I_com, c)
%SPATIAL_INERTIA 由质量、质心惯量和质心位置构造6x6空间惯量。
%   G = SPATIAL_INERTIA(m,I_com,c)
%   输入：m [kg]；I_com [kg*m^2]，在质心处、以连杆轴表达；
%         c [m]，质心相对连杆原点的位置，亦以连杆轴表达。
%   输出：G，将连杆旋量[omega;v]映射为动量[角动量;线动量]。
%   公式：G=[I_com+m*[c]x*[c]x', m*[c]x; m*[c]x', m*I3]。
%   示例：G=spatial_inertia(1,diag([.01 .02 .03]),[.1;0;0]);
assert(isscalar(m) && isfinite(m) && m >= 0, '质量m必须为非负有限数');
assert(isequal(size(I_com),[3,3]) && numel(c)==3, '惯量或质心维度错误');
assert(all(isfinite(I_com(:))) && all(isfinite(c(:))), '输入包含非有限数');
c = c(:);
C = skew(c);
G = [I_com+m*C*C', m*C;
     m*C',         m*eye(3)];
end
