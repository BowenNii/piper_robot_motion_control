function state = joint_state_estimator_step(q_meas,dq_meas,state_prev,dt,noise)
%JOINT_STATE_ESTIMATOR_STEP 融合关节角与可选关节速度的恒速Kalman估计。
%   state = JOINT_STATE_ESTIMATOR_STEP(q_meas,dq_meas,state_prev,dt,noise)
%   输入：q_meas n维关节角[rad]；dq_meas n维关节速度[rad/s]，
%         无速度测量时传[]；state_prev 上次输出，首次传[]；
%         dt 实际采样周期[s]；noise为含以下字段的结构体：
%         sigma_q [rad]、sigma_dq [rad/s]、sigma_acc [rad/s^2]，
%         每项为标量或n维标准差，不是方差。
%   输出：state.q、state.dq、state.x=[q;dq]、state.P协方差、
%         state.innovation测量残差、state.K增益。
%   用法：state=joint_state_estimator_step(q_meas,dq_joint,[],.005,noise);
%   注意：电机转速必须先依据传动比与符号换算为关节rad/s；
%         不能把原始电机转速直接作为dq_meas。本函数只用于离线原型。
q_meas=q_meas(:);
n=numel(q_meas);
assert(n>0 && all(isfinite(q_meas)), 'q_meas必须为有限非空向量');
assert(isscalar(dt) && isfinite(dt) && dt>0, 'dt必须为正数');
if ~isempty(dq_meas)
    dq_meas=dq_meas(:);
    assert(numel(dq_meas)==n && all(isfinite(dq_meas)), ...
           'dq_meas维度错误或含非有限数');
end
assert(isstruct(noise) && all(isfield(noise,{'sigma_q','sigma_dq','sigma_acc'})), ...
       'noise必须含sigma_q、sigma_dq、sigma_acc');
sigma_q=expand_sigma(noise.sigma_q,n,'sigma_q');
sigma_dq=expand_sigma(noise.sigma_dq,n,'sigma_dq');
sigma_acc=expand_sigma(noise.sigma_acc,n,'sigma_acc');

if isempty(state_prev)
    dq_init=zeros(n,1);
    if ~isempty(dq_meas)
        dq_init=dq_meas;
    end
    state.x=[q_meas;dq_init];
    state.P=blkdiag(diag(sigma_q.^2),diag(sigma_dq.^2));
    state.q=q_meas;
    state.dq=dq_init;
    state.innovation=[];
    state.K=[];
    return;
end

F=[eye(n),dt*eye(n);zeros(n),eye(n)];
A=diag(sigma_acc.^2);
Q=[dt^4/4*A,dt^3/2*A;
   dt^3/2*A,dt^2*A];
if isempty(dq_meas)
    z=q_meas;
    H=[eye(n),zeros(n)];
    R=diag(sigma_q.^2);
else
    z=[q_meas;dq_meas];
    H=eye(2*n);
    R=blkdiag(diag(sigma_q.^2),diag(sigma_dq.^2));
end
[state.x,state.P,state.innovation,state.K]=kalman_filter( ...
    state_prev.x,state_prev.P,z,F,Q,H,R);
state.q=state.x(1:n);
state.dq=state.x(n+1:end);
end

function sigma = expand_sigma(value,n,name)
if isscalar(value)
    sigma=repmat(value,n,1);
else
    sigma=value(:);
end
assert(numel(sigma)==n && all(isfinite(sigma)) && all(sigma>0), ...
       '%s必须为正的标量或n维标准差',name);
end
