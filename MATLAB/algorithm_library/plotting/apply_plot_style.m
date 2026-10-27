function apply_plot_style(fig,cfg)
%APPLY_PLOT_STYLE 统一图窗字体、字号、白背景和网格，不改数据/坐标范围。
% 输入：fig有效figure句柄；cfg可省略，支持font_name、font_size。
% 无输出；不保存、不关闭、不改变全局MATLAB默认设置。
%一、输入参数与配置补全
if nargin<2, cfg=struct; end
cfg=plot_config(cfg);
%二、输入合法性校验
assert(isscalar(fig) && isgraphics(fig,'figure'),'输入必须是有效图窗句柄');
%三、图窗整体设置
set(fig,'Color','w');
%四、全局字体统一设置
set(findall(fig,'-property','FontName'),'FontName',cfg.font_name);
set(findall(fig,'-property','FontSize'),'FontSize',cfg.font_size);
%五、坐标轴样式统一
axes_list=findall(fig,'Type','axes');
for k=1:numel(axes_list)
    grid(axes_list(k),'on'); box(axes_list(k),'on');
end
end
