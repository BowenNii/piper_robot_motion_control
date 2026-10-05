function value = inverse_transform(varargin)
% 旧接口兼容入口；新代码使用 se3_inverse。
value = se3_inverse(varargin{:});
end
