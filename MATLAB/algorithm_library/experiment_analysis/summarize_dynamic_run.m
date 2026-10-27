function [data,summary,phases]=summarize_dynamic_run(data,target_rad,model,cfg)
%SUMMARIZE_DYNAMIC_RUN 分阶段统计动态数据，不计算FK，也不判断跟踪性能。
% 输入：data含t/q_rad/q_deg、position_mm/rotation_deg/span_ms、name/label、
% cpp_position_mm/cpp_rotation_deg；这些比较对象由主脚本明确计算。
% target_rad终态目标1×n；model含q_min/q_max；cfg显式设置：
% change_threshold_deg、motion_padding_s、activity_gap_s、endpoint_window_s、
% frame_window_s、static_range_deg、limit_tolerance_rad、static_baseline_s。
% 输出：补充阶段/终态字段的data、整组summary、三阶段phases结构数组。
% 空阶段统计为NaN；活动段数量是变化间隔启发式，不是完整轨迹分段。

%实数、有限值、一维向量；元素个数必须等于关节数量（和data.q_rad的列数一致）
validateattributes(target_rad,{'numeric'},{'real','finite','vector','numel',size(data.q_rad,2)});
validateattributes(cfg.endpoint_window_s,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(cfg.static_baseline_s,{'numeric'},{'real','finite','scalar','nonnegative'});
thresholds={'change_threshold_deg','motion_padding_s','activity_gap_s', ...
    'frame_window_s','static_range_deg','limit_tolerance_rad'};
for j=1:numel(thresholds)
    validateattributes(cfg.(thresholds{j}),{'numeric'},{'real','finite','scalar','nonnegative'});
end
assert(numel(data.t)>=2 && all(diff(data.t)>0),'动态统计至少两条严格递增时间样本');
% data.phase：和采样点等长的标签向量，每个点对应一个阶段编号：
% 1= 前静态段（运动开始前的静止阶段）
% 2= 运动段（角度持续变化的运动过程）
% 3= 后静态段（运动结束后的静止阶段）
% data.motion_bounds：2 个元素的向量，记录运动段的起始时间、结束时间；
% data.activity_segments：识别到的活动段数量，用来判断数据是否存在多段运动、卡顿断续等异常。
[data.phase,data.motion_bounds,data.activity_segments]=split_phases(data.t,data.q_deg,cfg);
data.target_rad=target_rad(:)';
endpoint=data.t>=data.t(end)-cfg.endpoint_window_s;
%计算终态窗口内每个关节的平均角度作为终态实际位置
data.endpoint_rad=mean(data.q_rad(endpoint,:),1);
%计算终态窗口内每个关节的角度波动范围用来衡量终态抖动程度，数值越大说明终点越不稳
data.endpoint_range_deg=max(data.q_deg(endpoint,:),[],1)-min(data.q_deg(endpoint,:),[],1);
%计算终态平均角度和目标角度的偏差值
data.endpoint_error_deg=(data.endpoint_rad-data.target_rad)*180/pi;
%采样时序质量评估（数据可靠性检查）
timing=check_sample_timing(data.t,data.span_ms/1000, ...
    struct('max_frame_span_s',cfg.frame_window_s,'gap_factor',3));
dt=diff(data.t); %计算相邻采样点时间间隔
data.median_dt_ms=timing.median_dt_s*1000;
data.max_dt_ms=max(dt)*1000; 
data.large_gap_count=numel(timing.gap_rows);
data.over_window_count=numel(timing.span_rows);
%关节限位合规性检查（运行有效性检查）
limits=check_joint_limits(data.q_rad,model.q_min,model.q_max,cfg.limit_tolerance_rad);
data.limit_violation_indices=find(~limits.valid_rows);
data.limit_violation_count=numel(data.limit_violation_indices);
%异常集中告警（问题快速定位）
if data.activity_segments~=1 || data.large_gap_count>0 || data.over_window_count>0 ...
        || data.limit_violation_count>0 || any(data.endpoint_range_deg>cfg.static_range_deg)
    warning('%s：活动段%d，长间隔%d，超窗口%d，限位异常行%d，终态范围%.6fdeg', ...
        data.name,data.activity_segments,data.large_gap_count,data.over_window_count, ...
        data.limit_violation_count,max(data.endpoint_range_deg));
end
%基础误差统计
summary=error_stats(data,true(numel(data.t),1));
%补充运行基本信息
summary.run=data.name; summary.label=data.label; 
summary.samples=numel(data.t);
summary.duration_s=data.t(end); 
summary.motion_start_s=data.motion_bounds(1);
summary.motion_end_s=data.motion_bounds(2); 
summary.motion_duration_s=diff(data.motion_bounds);
%批量补充时序质量指标
fields={'activity_segments','median_dt_ms','max_dt_ms','large_gap_count', ...
    'over_window_count','limit_violation_count'};
for k=1:numel(fields), summary.(fields{k})=data.(fields{k}); end
%补充 C++ 算法误差指标
summary.cpp_position_max_mm=max(data.cpp_position_mm);
summary.cpp_rotation_max_deg=max(data.cpp_rotation_deg);
%补充终态精度指标
summary.endpoint_max_abs_error_deg=max(abs(data.endpoint_error_deg));
summary.endpoint_max_range_deg=max(data.endpoint_range_deg);
%生成分阶段统计数组 `phases`
names=["pre_static","motion_buffered","post_static"];
phases = struct([]);
for k=1:3
    % ① 筛选当前阶段的采样点
    selected = data.phase == k; 
    % ② 计算当前阶段的误差统计
    s = error_stats(data, selected);
    
    % ③ 补充阶段基本信息
    s.run = data.name; 
    s.phase = names(k); 
    s.samples = nnz(selected); 
    ix = find(selected);
    
    % ④ 计算阶段起止时间，空阶段默认NaN
    s.start_s = NaN; s.end_s = NaN; s.duration_s = NaN;
    if ~isempty(ix)
        s.start_s = data.t(ix(1)); 
        s.end_s = data.t(ix(end)); 
        s.duration_s = s.end_s - s.start_s;
    end
    
    % ⑤ 判断静态段时长是否达标
    s.sufficient_static_baseline = k~=2 && ~isempty(ix) && s.duration_s >= cfg.static_baseline_s;
    
    % ⑥ 组装进结构体数组
    if k==1, phases = s; else, phases(k) = s; end
end

end
