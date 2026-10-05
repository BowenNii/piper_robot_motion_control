function T = forward_poe_space(S_list,q,M)
%FORWARD_POE_SPACE 空间形式POE正运动学。
%   T=FORWARD_POE_SPACE(S_list,q,M)
%   输入：S_list 6×n空间旋量，每列[omega;v]；q n×1关节量[rad或m]；
%         M 4×4零位末端位姿。输出：T 4×4基坐标系下末端位姿。
%   公式：T=exp([S1]q1)...exp([Sn]qn)M。
%   用法：T=forward_poe_space(S_list,q,M);
assert(size(S_list,1)==6 && size(S_list,2)==numel(q), ...
    'S_list必须为6×n，q必须有n个元素');
assert(isequal(size(M),[4,4]) && all(isfinite(M(:))) && ...
    all(isfinite(S_list(:))) && all(isfinite(q(:))), '输入维度或数值无效');
q=q(:);
T=eye(4);
for i=1:numel(q)
    T=T*se3_exp(S_list(:,i)*q(i));
end
T=T*M;
end
