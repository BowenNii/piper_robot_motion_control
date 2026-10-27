function summary=summarize_static_pose(data,pose_name,cfg)
% 静态位姿精度统计结果；不独立计算POE、不自动判定停稳。
% 输入：load_feedback_csv返回结构；pose_name位形名；cfg可省略。
% cfg.frame_window_s默认0.002；position_recheck_tolerance_m默认1e-11。
% 输出：单行结构，角度均值/范围deg、位置均值mm、偏差统计、跨度。
% 重新核对位置误差列只是算术复核，不等于独立FK或物理精度验证。
if nargin<3, cfg=struct; end
if ~isfield(cfg,'frame_window_s'), cfg.frame_window_s=.002; end
if ~isfield(cfg,'position_recheck_tolerance_m'), cfg.position_recheck_tolerance_m=1e-11; end
validateattributes(cfg.frame_window_s,{'numeric'},{'scalar','finite','nonnegative'});
validateattributes(cfg.position_recheck_tolerance_m,{'numeric'},{'scalar','finite','nonnegative'});
summary=struct('pose',string(pose_name),'sample_count',numel(data.t), ...
    'duration_s',data.t(end)-data.t(1));
q_mean=mean(data.q_deg,1); q_range=max(data.q_deg,[],1)-min(data.q_deg,[],1);
for j=1:size(data.q_rad,2)
    summary.(sprintf('q%d_mean_deg',j))=q_mean(j);
    summary.(sprintf('q%d_range_deg',j))=q_range(j);
end
pf=mean(data.p_feedback,1)*1000; pc=mean(data.p_cpp,1)*1000;
names={'x','y','z'};
for j=1:3
    summary.(['feedback_',names{j},'_mean_mm'])=pf(j);
    summary.(['poe_',names{j},'_mean_mm'])=pc(j);
    summary.(['delta_',names{j},'_mean_mm'])=pc(j)-pf(j);
end
p=summarize_errors(data.position_mm); 
a=summarize_errors(data.rotation_deg);
summary.position_mean_mm=p.mean; 
summary.position_rms_mm=p.rms; 
summary.position_max_mm=p.max;
summary.rotation_mean_deg=a.mean; 
summary.rotation_rms_deg=a.rms; 
summary.rotation_max_deg=a.max;
summary.frame_span_max_ms=max(data.span_ms);
% 保留此前静态实验的1e-6ms数值比较余量；不改变原始数据。
summary.span_over_window_count=nnz(data.span_ms>cfg.frame_window_s*1000+1e-6);
summary.position_recheck_max_mm=max(abs(vecnorm((data.p_cpp-data.p_feedback)*1000,2,2)-data.position_mm));
assert(summary.position_recheck_max_mm<=cfg.position_recheck_tolerance_m*1000, ...
    'CSV位置误差列与位置向量不一致');
end
