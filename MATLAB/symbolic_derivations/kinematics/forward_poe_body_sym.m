function T = forward_poe_body_sym(B_list,theta,M)
%FORWARD_POE_BODY_SYM 物体形式符号正运动学T=M*prod(exp([B_i]q_i))。
%   输入：B_list 6×n，theta n维，M 4×4零位位姿；输出T 4×4。
assert(size(B_list,1)==6 && size(B_list,2)==numel(theta) && ...
    isequal(size(M),[4,4]), 'B_list、theta或M维度错误');
B_list=sym(B_list); theta=sym(theta(:));
T=sym(M);
for i=1:numel(theta)
    T=T*se3_exp_sym(B_list(:,i),theta(i));
end
end
