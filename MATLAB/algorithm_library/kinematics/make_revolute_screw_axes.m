function S_list = make_revolute_screw_axes(w, p)
%MAKE_REVOLUTE_SCREW_AXES 由旋转轴方向和轴上一点构造空间旋量。
%   S_list = MAKE_REVOLUTE_SCREW_AXES(w, p)
%   输入：w、p 均为 3×n 实数矩阵；第 i 列分别是第 i 个关节在
%         同一空间基坐标系下的轴方向和轴上一点 [m]。n=1 时可传 3×1 向量。
%   输出：S_list 为 6×n，每列 S_i=[omega_i;v_i]，其中
%         omega_i=w_i/norm(w_i)，v_i=-cross(omega_i,p_i)。
%   用法：S_list=make_revolute_screw_axes(w,p);
%   注意：这是旋转关节的空间旋量；关节变量单位为 rad。轴上一点可沿
%         同一轴线任意选取。w 不得为零；p 必须与 w 使用同一坐标系。
if ~isnumeric(w) || ~isnumeric(p) || ~isreal(w) || ~isreal(p) || ...
        size(w,1)~=3 || ~isequal(size(w),size(p)) || ...
        isempty(w) || any(~isfinite(w(:))) || any(~isfinite(p(:)))
    error('w和p必须是同尺寸的有限实数3×n矩阵');
end

axis_norms=sqrt(sum(w.^2,1));
if any(axis_norms==0)
    error('旋转关节的轴方向w不能为零向量');
end

omega=w./axis_norms;
v=-cross(omega,p,1);
S_list=[omega;v];
end
