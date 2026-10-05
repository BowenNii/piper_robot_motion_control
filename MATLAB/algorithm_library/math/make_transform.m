function value = make_transform(varargin)
% 旧接口兼容入口；新代码使用 se3_from_rt。
value = se3_from_rt(varargin{:});
end
