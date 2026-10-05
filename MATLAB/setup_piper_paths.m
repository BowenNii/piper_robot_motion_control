function root = setup_piper_paths(include_symbolic)
% 初始化数值库；输入 true 时也加载符号库。不加载练习和实验目录。
% 用法：addpath('/path/to/repo/MATLAB'); setup_piper_paths;
if nargin<1, include_symbolic=false; end
root=fileparts(mfilename('fullpath'));
modules={'common','math','lie_group','kinematics','robot_model','dynamics', ...
    'control','safety','estimation','optimization','trajectory'};
for i=1:numel(modules)
    addpath(fullfile(root,'algorithm_library',modules{i}));
end
if include_symbolic
    addpath(fullfile(root,'symbolic_derivations','lie_group'), ...
        fullfile(root,'symbolic_derivations','kinematics'));
end
end
