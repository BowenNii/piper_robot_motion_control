function results = run_all_tests(include_symbolic)
%RUN_ALL_TESTS 隔离各测试脚本工作区；缺少工具箱时明确标记SKIP。
if nargin<1, include_symbolic=true; end
root=setup_piper_paths(include_symbolic);
names={'math','kinematics','piper_model','controller','dynamics','estimation', ...
    'optimization','trajectory','cpp_alignment'};
if include_symbolic, names{end+1}='symbolic_derivations'; end
results=struct('name',{},'status',{},'message',{});
for i=1:numel(names)
    name=names{i}; status='PASS'; message='';
    if any(strcmp(name,{'optimization','trajectory'})) && ~license('test','Optimization_Toolbox')
        status='SKIP'; message='缺少Optimization Toolbox';
    elseif strcmp(name,'symbolic_derivations') && ~license('test','Symbolic_Toolbox')
        status='SKIP'; message='缺少Symbolic Math Toolbox';
    else
        try
            run_isolated(fullfile(root,'tests',['test_',name,'.m']));
        catch exception
            status='FAIL'; message=getReport(exception,'extended','hyperlinks','off');
        end
    end
    results(end+1)=struct('name',name,'status',status,'message',message); %#ok<AGROW>
    fprintf('%s: %s %s\n',name,status,message);
end
assert(~any(strcmp({results.status},'FAIL')),'MATLAB回归测试失败，详见上面的错误');
end

function run_isolated(test_path)
run(test_path);
end
