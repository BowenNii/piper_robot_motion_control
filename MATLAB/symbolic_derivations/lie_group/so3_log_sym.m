function [phi,theta,omega] = so3_log_sym(R)
%SO3_LOG_SYM SO(3)主值对数，第一输出为积分旋转向量phi=omega*theta。
%   [phi,theta,omega]=SO3_LOG_SYM(R)，R为3×3符号旋转矩阵。
%   对R=I返回phi=0；对可解析的pi旋转从R-I的零空间取轴；
%   其他情况使用(R-R')/(2sin(theta))，假设0<theta<pi。
%   符号分支依赖变量假设；含未确定角度时不保证全局主值表达。
assert(isequal(size(R),[3,3]), 'R必须为3×3矩阵');
R=sym(R);
if isequal(simplify(R-sym(eye(3))),sym(zeros(3)))
    theta=sym(0);
    omega=sym(zeros(3,1));
    phi=omega;
    return;
end
cos_theta=simplify((trace(R)-1)/2);
theta=acos(cos_theta);
if isequal(cos_theta,sym(-1))
    basis=null(simplify(R-sym(eye(3))));
    assert(~isempty(basis), 'pi旋转的轴无法解析；请补充符号假设');
    omega=simplify(basis(:,1)/sqrt(basis(:,1).'*basis(:,1)));
else
    omega=vee_sym((R-R.')/(2*sin(theta)));
end
phi=simplify(theta*omega);
end
