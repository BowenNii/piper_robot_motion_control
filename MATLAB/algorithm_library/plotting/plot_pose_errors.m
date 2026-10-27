function [fig,axes_list]=plot_pose_errors(t_s,position_error_m,rotation_error_rad,cfg)
%PLOT_POSE_ERRORS 绘制已计算的位姿偏差，不相减RPY、不在此计算误差。
% 输入：t_s、position_error_m、rotation_error_rad，各N×1。
% cfg.length_unit默认mm，angle_unit默认deg；bounds_s可选。
% 输出：fig与两个坐标轴；输入偏差必须有限且非负。
if nargin<4, cfg=struct; end
cfg=plot_config(cfg);
validateattributes(position_error_m,{'numeric'},{'vector','nonnegative'});
validateattributes(rotation_error_rad,{'numeric'},{'vector','nonnegative'});
y=[position_error_m(:),rotation_error_rad(:)]; t=check_plot_data(t_s,y);
if strcmp(cfg.length_unit,'mm'), y(:,1)=y(:,1)*1000; end
if strcmp(cfg.angle_unit,'deg'), y(:,2)=y(:,2)*180/pi; end
fig=figure('Name','位姿偏差','Visible',cfg.visible,'Position',cfg.position);
tiledlayout(fig,2,1,'TileSpacing','compact'); axes_list=gobjects(2,1);
labels={['位置偏差 / ',cfg.length_unit],['姿态偏差 / ',cfg.angle_unit]};
for k=1:2
    ax=nexttile; axes_list(k)=ax;
    plot(ax,t,y(:,k),'LineWidth',cfg.line_width);
    mark_motion_phases(ax,cfg.bounds_s); xlabel(ax,'时间 / s'); ylabel(ax,labels{k});
end
sgtitle(fig,cfg.title,'Interpreter','none'); apply_plot_style(fig,cfg);
end
