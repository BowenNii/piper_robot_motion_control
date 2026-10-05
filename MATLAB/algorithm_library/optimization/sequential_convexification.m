function result = sequential_convexification(problem,options)
%SEQUENTIAL_CONVEXIFICATION 非线性不等式约束的逐次凸化离线原型。
%   result=SEQUENTIAL_CONVEXIFICATION(problem,options)
%   problem：H,f定义凸二次目标；x0为初值；nonlinear_ineq函数句柄
%   [c,J]=fun(x)，其中c(x)<=0，J=dc/dx (m×n)。可另设A,b、
%   Aeq,beq、lb,ub作为线性约束。options可设max_iterations(50)、
%   trust_radius(0.5)、tolerance(1e-7)、feasibility_tolerance(1e-7)。
%   每步用c(xk)+J(xk)*(x-xk)<=0线性化，叠加信赖域后解QP；
%   仅接受真实非线性约束满足且目标下降的候选点。
%   输出：x、success、iterations、cost、max_violation、message。
%   要求可行初值；无可行初值恢复、松弛变量和收敛保证。勿用于真机。
assert(isstruct(problem) && isfield(problem,'H') && ...
    isfield(problem,'f') && isfield(problem,'x0') && ...
    isfield(problem,'nonlinear_ineq') && ...
    isa(problem.nonlinear_ineq,'function_handle'), 'problem字段不完整');
if nargin<2 || isempty(options)
    options=struct();
end
max_iterations=option_or(options,'max_iterations',50);
trust_radius=option_or(options,'trust_radius',0.5);
tolerance=option_or(options,'tolerance',1e-7);
feas_tol=option_or(options,'feasibility_tolerance',1e-7);
assert(max_iterations>=0 && max_iterations==floor(max_iterations) && ...
    trust_radius>0 && tolerance>0 && feas_tol>0 && ...
    all(isfinite([max_iterations,trust_radius,tolerance,feas_tol])), ...
    'options数值无效');
x=problem.x0(:);
n=numel(x);
assert(n>0 && all(isfinite(x)) && isequal(size(problem.H),[n,n]) && ...
    numel(problem.f)==n, '初值或目标维度不一致');
H=(problem.H+problem.H')/2;
f=problem.f(:);
[c,J]=evaluate_constraints(problem.nonlinear_ineq,x,n);
assert(all(c<=feas_tol), '逐次凸化要求非线性约束可行初值');
result.success=false;
result.iterations=0;
result.message='达到最大迭代数';
for k=1:max_iterations
    subproblem=problem;
    subproblem=rmfield(subproblem,{'x0','nonlinear_ineq'});
    A0=option_or(problem,'A',zeros(0,n));
    b0=option_or(problem,'b',zeros(0,1));
    subproblem.A=[A0;J];
    subproblem.b=[b0(:);-c+J*x];
    lb0=option_or(problem,'lb',-Inf(n,1));
    ub0=option_or(problem,'ub',Inf(n,1));
    subproblem.lb=max(lb0(:),x-trust_radius);
    subproblem.ub=min(ub0(:),x+trust_radius);
    subproblem.x0=x;
    qp=solve_qp(subproblem);
    if ~qp.success
        result.message='线性化QP求解失败';
        break;
    end
    direction=qp.x-x;
    if norm(direction,Inf)<=tolerance
        result.success=true;
        result.message='步长收敛';
        break;
    end
    old_cost=0.5*x'*H*x+f'*x;
    accepted=false;
    scale=1;
    for trial=1:30
        candidate=x+scale*direction;
        [c_new,J_new]=evaluate_constraints(problem.nonlinear_ineq,candidate,n);
        new_cost=0.5*candidate'*H*candidate+f'*candidate;
        if all(c_new<=feas_tol) && new_cost<old_cost-1e-12
            x=candidate;
            c=c_new;
            J=J_new;
            accepted=true;
            break;
        end
        scale=scale/2;
    end
    if ~accepted
        result.message='回溯无法找到可行下降步';
        break;
    end
    result.iterations=k;
end
result.x=x;
result.cost=0.5*x'*H*x+f'*x;
result.max_violation=max([0;c]);
if result.max_violation>feas_tol
    result.success=false;
end
end

function [c,J]=evaluate_constraints(fun,x,n)
[c,J]=fun(x);
c=c(:);
assert(size(J,1)==numel(c) && size(J,2)==n && ...
    all(isfinite(c)) && all(isfinite(J(:))), ...
    'nonlinear_ineq必须返回有限的c和对应m×n Jacobian');
end

function value=option_or(options,name,default)
if isfield(options,name)
    value=options.(name);
else
    value=default;
end
end
