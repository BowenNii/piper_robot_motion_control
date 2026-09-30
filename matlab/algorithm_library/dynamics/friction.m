function tau_f = friction(dq, params)
%FRICTION 平滑黏滞+库仑摩擦模型，返回阻碍运动的力矩幅值/符号。
%   tau_f=b.*dq+fc.*tanh(dq./velocity_scale)。
%   输入：dq [rad/s]；params.viscous [N*m*s/rad]；
%         params.coulomb [N*m]；params.velocity_scale [rad/s]。
%   输出：tau_f [N*m]，正速度对应正阻力矩；运动方程中通常加tau_f。
%   参数必须经过辨识；URDF没有提供可信的摩擦参数。
%   示例：p.viscous=zeros(6,1); p.coulomb=zeros(6,1);
%         p.velocity_scale=.01*ones(6,1); tau_f=friction(dq,p);
dq=dq(:);
b=params.viscous(:); fc=params.coulomb(:); vs=params.velocity_scale(:);
n=numel(dq);
assert(all([numel(b),numel(fc),numel(vs)]==n), '摩擦参数维度错误');
assert(all(isfinite(dq)) && all(isfinite(b)) && ...
       all(isfinite(fc)) && all(isfinite(vs)) && all(vs>0), '摩擦参数无效');
tau_f=b.*dq+fc.*tanh(dq./vs);
end
