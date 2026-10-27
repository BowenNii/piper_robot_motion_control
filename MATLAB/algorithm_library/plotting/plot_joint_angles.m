function [fig,axes_list]=plot_joint_angles(t_s,q_rad,cfg)
%PLOT_JOINT_ANGLES 按关节分别绘制角度曲线（不含夹爪单位换算）。
% 输入：t_s N×1秒，q_rad N×n弧度；cfg可省略。
% cfg.angle_unit默认deg，title标题，bounds_s运动边界；
% cfg.target_rad可选1×n终态目标，仅画水平参考线，不表示完整目标轨迹。
% 输出：fig图窗、axes_list n×1坐标轴；不保存、不关闭、不算FK。
if nargin<3, cfg=struct; end
cfg=plot_config(cfg); 
t=check_plot_data(t_s,q_rad); 
n=size(q_rad,2);
scale=1; 
if strcmp(cfg.angle_unit,'deg'), scale=180/pi; end
if isfield(cfg,'target_rad')
    validateattributes(cfg.target_rad,{'numeric'},{'real','finite','vector','numel',n});
end
fig=figure('Name','关节角度','Visible',cfg.visible,'Position',cfg.position);
tiledlayout(fig,ceil(n/2),2,'TileSpacing','compact'); axes_list=gobjects(n,1);
for k=1:n
    ax=nexttile; axes_list(k)=ax;
    plot(ax,t,q_rad(:,k)*scale,'LineWidth',cfg.line_width);
    if isfield(cfg,'target_rad')
        yline(ax,cfg.target_rad(k)*scale,'r:','终态目标','HandleVisibility','off');
    end
    mark_motion_phases(ax,cfg.bounds_s);
    title(ax,sprintf('J%d',k)); xlabel(ax,'时间 / s');
    ylabel(ax,['角度 / ',cfg.angle_unit]);
end
sgtitle(fig,cfg.title,'Interpreter','none'); apply_plot_style(fig,cfg);
end
