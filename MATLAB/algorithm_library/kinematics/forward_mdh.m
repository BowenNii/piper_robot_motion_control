function T = forward_mdh(parameters,q,T_base,T_tool)
%FORWARD_MDH 依据改进DH参数表计算串联机械臂正运动学。
%   输入：parameters n×4，每行[alpha_(i-1),a_(i-1),d_i,theta_offset_i]；
%         q n×1关节角[rad]；T_base 基座变换、T_tool 工具变换，可省略。
%   输出：T=T_base*prod_i MDH(alpha,a,d,q_i+offset)*T_tool。
%   注意：本函数仅实现转动关节；PiPER的实际DH表须独立标定和验证。
if nargin<3 || isempty(T_base)
    T_base=eye(4);
end
if nargin<4 || isempty(T_tool)
    T_tool=eye(4);
end
assert(size(parameters,2)==4 && size(parameters,1)==numel(q) && ...
    all(isfinite(parameters(:))) && all(isfinite(q(:))) && ...
    isequal(size(T_base),[4,4]) && isequal(size(T_tool),[4,4]), ...
    '参数维度或数值无效');
q=q(:);
T=T_base;
for i=1:numel(q)
    T=T*mdh_transform(parameters(i,1),parameters(i,2), ...
        parameters(i,3),q(i)+parameters(i,4));
end
T=T*T_tool;
end
