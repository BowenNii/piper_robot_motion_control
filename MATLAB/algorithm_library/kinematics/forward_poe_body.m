function T = forward_poe_body(B_list,q,M)
%FORWARD_POE_BODY 物体形式POE正运动学。
%   T=FORWARD_POE_BODY(B_list,q,M)
%   输入：B_list 6×n物体旋量，每列[omega;v]；q n×1关节量；
%         M 4×4零位末端位姿。输出：T 4×4基坐标系下末端位姿。
%   公式：T=M exp([B1]q1)...exp([Bn]qn)，B_i=Ad_(M^-1)S_i。
assert(size(B_list,1)==6 && size(B_list,2)==numel(q), ...
    'B_list必须为6×n，q必须有n个元素');
assert(isequal(size(M),[4,4]) && all(isfinite(M(:))) && ...
    all(isfinite(B_list(:))) && all(isfinite(q(:))), '输入维度或数值无效');
q=q(:);
T=M;
for i=1:numel(q)
    T=T*se3_exp(B_list(:,i)*q(i));
end
end
