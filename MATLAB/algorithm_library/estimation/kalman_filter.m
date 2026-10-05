function [x_next,P_next,innovation,K] = kalman_filter(x_prev,P_prev,z,F,Q,H,R,B,u)
%KALMAN_FILTER 线性离散卡尔曼滤波器的一次预测与测量更新。
%   [x_next,P_next,innovation,K] = KALMAN_FILTER(x_prev,P_prev,z,F,Q,H,R)
%   可选末尾B,u，用于状态方程x_k=F*x_{k-1}+B*u+w。
%   输入：x_prev n维状态；P_prev n×n状态协方差；
%         z m维测量（传[]则仅预测）；F n×n状态转移；
%         Q n×n过程噪声协方差；H m×n观测矩阵；R m×m测量噪声协方差。
%   输出：x_next、P_next 后验估计；innovation=z-H*x_pred；K n×m增益。
%   使用Joseph形式更新P，以改善数值对称性。
%   用法：[x,P]=kalman_filter(x,P,z,F,Q,H,R);
if nargin<8 || isempty(B)
    B=[];
end
if nargin<9 || isempty(u)
    u=[];
end
x_prev=x_prev(:);
n=numel(x_prev);
assert(isequal(size(P_prev),[n,n]) && isequal(size(F),[n,n]) && ...
       isequal(size(Q),[n,n]), '状态维度不一致');
assert(all(isfinite(x_prev)) && all(isfinite(P_prev(:))) && ...
       all(isfinite(F(:))) && all(isfinite(Q(:))), '状态输入包含非有限数');
if ~isempty(B)
    assert(size(B,1)==n && size(B,2)==numel(u), 'B/u维度不一致');
end

x_pred=F*x_prev;
if ~isempty(B)
    x_pred=x_pred+B*u(:);
end
P_pred=F*P_prev*F'+Q;

if isempty(z)
    x_next=x_pred;
    P_next=(P_pred+P_pred')/2;
    innovation=[];
    K=zeros(n,0);
    return;
end

z=z(:);
m=numel(z);
assert(isequal(size(H),[m,n]) && isequal(size(R),[m,m]), ...
       '测量维度不一致');
assert(all(isfinite(z)) && all(isfinite(H(:))) && all(isfinite(R(:))), ...
       '测量输入包含非有限数');
innovation=z-H*x_pred;
S=H*P_pred*H'+R;
assert(rcond(S)>1e-14, '创新协方差奇异或严重病态');
K=(P_pred*H')/S; % 右除，避免显式求逆
x_next=x_pred+K*innovation;
I=eye(n);
P_next=(I-K*H)*P_pred*(I-K*H)'+K*R*K';
P_next=(P_next+P_next')/2;
end
