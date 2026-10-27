function [fig,ax]=plot_frame_span(t_s,frame_span_s,threshold_s,cfg)
% 采样帧跨度的专用可视化函数。
% 能直观看到什么时候出现了卡顿、超帧持续了多久、波动大不大；
% 输入：t_s、frame_span_s向量，单位秒；threshold_s非负秒。
% 输出：fig、ax；显示单位固定ms，不据此声称同步采样。
if nargin<4, cfg=struct; end
cfg=plot_config(cfg);
validateattributes(frame_span_s,{'numeric'},{'real','vector','nonnegative'});
validateattributes(threshold_s,{'numeric'},{'real','finite','scalar','nonnegative'});
t=check_plot_data(t_s,frame_span_s(:));
fig=figure('Name','六帧接收跨度','Visible',cfg.visible,'Position',cfg.position);
ax=axes(fig); plot(ax,t,frame_span_s(:)*1000,'LineWidth',cfg.line_width);
yline(ax,threshold_s*1000,'r--','配对窗口','HandleVisibility','off');
% 将阈值纳入纵轴范围，并留白，避免窗口线/文字被顶部裁掉。
values=[frame_span_s(:);threshold_s]*1000;
padding=.05*max(1,max(values)-min(values));
ylim(ax,[max(0,min(values)-padding),max(values)+padding]);
mark_motion_phases(ax,cfg.bounds_s); 
xlabel(ax,'时间 / s'); ylabel(ax,'六帧接收跨度 / ms');
title(ax,cfg.title,'Interpreter','none'); 
apply_plot_style(fig,cfg);
end
