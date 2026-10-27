function s = error_stats(r, mask)
% ERROR_STATS 带掩码的误差序列通用统计函数
% 输入：
%   r    - 数据结构体，必须包含字段：
%          position_mm  - 位置偏差序列，单位mm
%          rotation_deg  - 姿态偏差序列，单位°
%          span_ms      - 帧跨度序列，单位ms
%          t            - 相对时间序列，单位s
%   mask - 逻辑筛选向量，与采样点等长；true=纳入统计，false=排除
% 输出：
%   s    - 统计结果结构体，包含均值、RMS、最大值、峰值时间、最大帧跨度
% 
% 设计原则：
%   1. 空阶段返回NaN，不返回0，避免误导
%   2. 峰值时间采用相对时间，便于跨组对比

% 初始化所有字段为NaN
names = {'position_mean_mm','position_rms_mm','position_max_mm','position_peak_t_s', ...
         'rotation_mean_deg','rotation_rms_deg','rotation_max_deg','rotation_peak_t_s', ...
         'frame_span_max_ms'};
for j = 1:numel(names)
    s.(names{j}) = NaN;
end

% 空掩码直接返回，没有数据、无法统计，如果初始化为0，会误导为「这个阶段误差为 0」
if ~any(mask)
    return;
end

% 提取选中范围内的数据
indices = find(mask);
p = r.position_mm(mask);
a = r.rotation_deg(mask);

% 位置误差统计
s.position_mean_mm = mean(p);              % 算术平均值
s.position_rms_mm = sqrt(mean(p.^2));    % 均方根RMS
[s.position_max_mm, ip] = max(p);            % 最大值
s.position_peak_t_s = r.t(indices(ip));        % 峰值对应的相对时间，定位偏差出现的时刻

% 姿态误差统计
s.rotation_mean_deg = mean(a);
s.rotation_rms_deg = sqrt(mean(a.^2));
[s.rotation_max_deg, ia] = max(a);
s.rotation_peak_t_s = r.t(indices(ia));

% 帧跨度统计
s.frame_span_max_ms = max(r.span_ms(mask));

end
