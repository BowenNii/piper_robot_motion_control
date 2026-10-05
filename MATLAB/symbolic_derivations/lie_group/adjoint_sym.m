function value = adjoint_sym(varargin)
% 与数值/C++命名对应的符号入口；保留旧名称 adjoint_se3_sym。
value=adjoint_se3_sym(varargin{:});
end
