function [output,state] = low_pass_filter(input,alpha,previous_output,cutoff_hz)
% 标准接口与 C++ 一致：input、alpha[0,1]、上次状态 -> output、新状态。
% MATLAB 不能修改传入标量引用，应接收第二输出或用 state=low_pass_filter(...);
% 兼容旧四参数调用 low_pass_filter(x,y_prev,dt,cutoff_hz)。
if nargin==4
    y_prev=alpha; dt=previous_output;
    validateattributes(dt,{'numeric'},{'real','finite','scalar','positive'});
    validateattributes(cutoff_hz,{'numeric'},{'real','finite','scalar','positive'});
    alpha=1-exp(-2*pi*cutoff_hz*dt); previous_output=y_prev;
end
assert(isequal(size(input),size(previous_output)) && all(isfinite(input(:))) && ...
    all(isfinite(previous_output(:))),'滤波输入与状态必须同尺寸且有限');
validateattributes(alpha,{'numeric'},{'real','finite','scalar','>=',0,'<=',1});
output=previous_output+alpha*(input-previous_output); state=output;
end
