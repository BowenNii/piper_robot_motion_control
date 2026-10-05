function result = solve_qp(problem)
%SOLVE_QP 求解标准凸二次规划：min 0.5*x'*H*x+f'*x。
%   result=SOLVE_QP(problem)
%   必填字段：H n×n对称半正定矩阵，f n×1。
%   可选字段：A,b (A*x<=b)、Aeq,beq (Aeq*x=beq)、lb,ub、x0。
%   输出：x、cost、exitflag、output、max_violation、success。
%   依赖MATLAB Optimization Toolbox的quadprog；本函数负责建模校验，
%   不手写通用QP数值求解器。exitflag>0且约束残差<=1e-7才成功。
assert(isstruct(problem) && isfield(problem,'H') && isfield(problem,'f'), ...
    'problem必须含H和f');
H=problem.H;
f=problem.f(:);
n=numel(f);
assert(n>0 && isequal(size(H),[n,n]) && all(isfinite(H(:))) && ...
    all(isfinite(f)) && norm(H-H','fro')<=1e-10*max(1,norm(H,'fro')), ...
    'H/f维度、有限性或对称性错误');
H=(H+H')/2;
assert(min(eig(H))>=-1e-10*max(1,norm(H,2)), 'H必须半正定');
[A,b]=matrix_constraint(problem,'A','b',n);
[Aeq,beq]=matrix_constraint(problem,'Aeq','beq',n);
lb=field_or(problem,'lb',-Inf(n,1));
ub=field_or(problem,'ub',Inf(n,1));
lb=lb(:); ub=ub(:);
assert(numel(lb)==n && numel(ub)==n && all(lb<=ub) && ...
    all(~isnan(lb)) && all(~isnan(ub)), '上下界无效');
x0=field_or(problem,'x0',[]);
if ~isempty(x0)
    x0=x0(:);
    assert(numel(x0)==n && all(isfinite(x0)), 'x0无效');
end
options=optimoptions('quadprog','Display','off');
[x,cost,exitflag,output]=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
result.x=x;
result.cost=cost;
result.exitflag=exitflag;
result.output=output;
result.success=false;
result.max_violation=Inf;
if ~isempty(x)
    violations=[0;A*x-b;abs(Aeq*x-beq);lb-x;x-ub];
    result.max_violation=max(violations);
    result.success=exitflag>0 && result.max_violation<=1e-7;
end
end

function [A,b]=matrix_constraint(problem,matrix_name,vector_name,n)
A=field_or(problem,matrix_name,zeros(0,n));
b=field_or(problem,vector_name,zeros(0,1));
b=b(:);
assert(size(A,2)==n && size(A,1)==numel(b) && ...
    all(isfinite(A(:))) && all(isfinite(b)), ...
    '%s/%s维度或数值无效',matrix_name,vector_name);
end

function value=field_or(problem,name,default)
if isfield(problem,name)
    value=problem.(name);
else
    value=default;
end
end
