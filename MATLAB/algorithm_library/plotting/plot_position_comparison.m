function [fig,axes_list]=plot_position_comparison(t_s,positions_m,labels,cfg)
%PLOT_POSITION_COMPARISON 对比任意数量的末端XYZ序列。
%把不同算法 / 不同来源的末端位置曲线（比如硬件反馈、C++ POE 计算、MATLAB 参考计算）
%放在同一张图里分轴对比，直观验证不同算法的结果一致性、偏差大小。
% 输入：t_s秒；positions_m={p_can,p_cpp,p_matlab}，每个N×3，单位m；
% labels为对应来源名称cell/string；cfg.length_unit默认mm。
% 输出：fig与三个坐标轴；图例来源由主脚本提供，不自动推测。
if nargin<4, cfg=struct; end
cfg=plot_config(cfg); labels=cellstr(string(labels));
assert(iscell(positions_m) && ~isempty(positions_m) ...
    && numel(positions_m)==numel(labels),'位置序列cell与图例必须等长且非空');
for k=1:numel(positions_m)
    t=check_plot_data(t_s,positions_m{k});
    assert(size(positions_m{k},2)==3,'每个位置序列必须N×3');
end
scale=1; if strcmp(cfg.length_unit,'mm'), scale=1000; end
fig=figure('Name','末端位置对比','Visible',cfg.visible,'Position',cfg.position);
tiledlayout(fig,3,1,'TileSpacing','compact'); axes_list=gobjects(3,1);
axis_names={'X','Y','Z'}; styles={'-','--',':','-.'};
for j=1:3
    ax=nexttile; axes_list(j)=ax; hold(ax,'on'); h=gobjects(numel(labels),1);
    for k=1:numel(labels)
        h(k)=plot(ax,t,positions_m{k}(:,j)*scale, ...
            'LineStyle',styles{mod(k-1,4)+1},'LineWidth',cfg.line_width);
    end
    mark_motion_phases(ax,cfg.bounds_s);
    xlabel(ax,'时间 / s'); ylabel(ax,[axis_names{j},' / ',cfg.length_unit]);
    legend(ax,h,labels,'Location',cfg.legend_location,'Interpreter','none');
end
sgtitle(fig,cfg.title,'Interpreter','none'); apply_plot_style(fig,cfg);
end
