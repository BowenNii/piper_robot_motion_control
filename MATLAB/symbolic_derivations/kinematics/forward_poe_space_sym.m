function T = forward_poe_space_sym(S_list,theta,M)
%FORWARD_POE_SPACE_SYM 空间形式符号正运动学T=prod(exp([S_i]q_i))*M。
%   输入：S_list 6×n，theta n维，M 4×4零位位姿；输出T 4×4。
assert(size(S_list,1)==6 && size(S_list,2)==numel(theta) && ...
    isequal(size(M),[4,4]), 'S_list、theta或M维度错误');
S_list=sym(S_list); theta=sym(theta(:));
T=sym(eye(4));
for i=1:numel(theta)
    T=T*se3_exp_sym(S_list(:,i),theta(i));
end
T=T*sym(M);
end
