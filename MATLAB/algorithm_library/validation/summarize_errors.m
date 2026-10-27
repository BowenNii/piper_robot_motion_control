function stats = summarize_errors(errors,mask)
%SUMMARIZE_ERRORS 对一个非负误差序列统计，不绑定某个实验的结构体字段。
% 输入：errors N×1，单位由调用者决定；mask N×1逻辑，默认全部选中。
% 输出：count、mean、rms、max、max_row；数值单位与输入一致。
% 空选区count=0，其余NaN；选中项NaN/Inf报错，不静默忽略。
% common/error_stats保留原有mm/deg结构体接口，此函数不与其重名。
validateattributes(errors,{'numeric'},{'real','vector'});
errors=errors(:);
if nargin<2, mask=true(size(errors)); end
assert(islogical(mask) && numel(mask)==numel(errors), ...
    'mask必须是与误差等长的逻辑向量');
mask=mask(:); rows=find(mask); x=errors(mask);
assert(all(isfinite(x)) && all(x>=0),'选中误差必须有限且非负');
stats=struct('count',numel(x),'mean',NaN,'rms',NaN,'max',NaN,'max_row',NaN);
if isempty(x), return; end
stats.mean=mean(x);
% 使用norm避免直接平方非常大的有限数时溢出。
stats.rms=norm(x/sqrt(numel(x)));
[stats.max,index]=max(x); stats.max_row=rows(index);
end
