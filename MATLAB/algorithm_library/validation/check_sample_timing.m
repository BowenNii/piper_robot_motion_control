function report = check_sample_timing(t_s, frame_span_s, cfg)
% 采样时序质量的专用质检工具函数
% 全面检查时间序列的单调性、采样间隙异常、帧跨度超标三类问题，输出结构化的时序质量报告
% 输入：t_s N×1秒，frame_span_s N×1秒；cfg可省略。
% cfg.max_frame_span_s 默认0.002秒；gap_factor 默认3倍中位采样间隔。
% cfg.max_gap_s 默认Inf；同时满足绝对/相对任一阈值即记录为间隔异常。
% 输出：ok、dt_s、median_dt_s、nonincreasing_rows、gap_rows、span_rows。
% 行号指异常间隔的后一个采样点；不插值、不丢弃样本。
if nargin<3, cfg=struct; end
max_span=field_value(cfg,'max_frame_span_s',0.002);
factor=field_value(cfg,'gap_factor',3);
max_gap=field_value(cfg,'max_gap_s',Inf);
validateattributes(t_s,{'numeric'},{'real','finite','vector','nonempty'});
validateattributes(frame_span_s,{'numeric'},{'real','finite','vector','nonempty'});
validateattributes(max_span,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(factor,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(max_gap,{'numeric'},{'real','scalar','positive','nonnan'});%必须不是NaN（Not a Number非数值）
t_s=t_s(:); 
frame_span_s=frame_span_s(:);
assert(numel(t_s)==numel(frame_span_s),'时间与跨度必须等长');
dt=diff(t_s); %对时间序列做相邻差分，得到每个采样间隔的时长
positive_dt=dt(dt>0);%筛选出所有时间正常递增的间隔
median_dt=NaN;
if ~isempty(positive_dt), median_dt=median(positive_dt); end
gap=dt>max_gap;
if isfinite(median_dt), gap=gap | dt>factor*median_dt; end
report=struct('ok',false,'dt_s',dt,'median_dt_s',median_dt, ...
    'nonincreasing_rows',find(dt<=0)+1,'gap_rows',find(gap)+1, ...
    'span_rows',find(frame_span_s<0 | frame_span_s>max_span), ...
    'max_frame_span_s',max(frame_span_s));
report.ok=isempty(report.nonincreasing_rows) && isempty(report.gap_rows) ...
    && isempty(report.span_rows);
end

function value=field_value(cfg,name,default)
value=default;
if isfield(cfg,name), value=cfg.(name); end
end
