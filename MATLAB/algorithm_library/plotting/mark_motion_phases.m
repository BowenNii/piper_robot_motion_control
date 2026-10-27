function handles=mark_motion_phases(ax,bounds_s)
%MARK_MOTION_PHASES 在指定坐标轴标注运动区间两条边界线。
%在指定坐标轴的时间轴上，画出两条垂直虚线，标记运动段的起始和结束时刻。
% 输入：ax坐标轴；bounds_s=[开始,结束]秒，必须与图的时间基准一致。
% 空数组或[NaN NaN]表示没有运动，不画线；不自行识别运动阶段。
% 输出：两条ConstantLine句柄；不参与图例，不修改数据。
assert(isscalar(ax) && isgraphics(ax,'axes'),'ax必须是有效坐标轴');
handles=gobjects(0);
if isempty(bounds_s), return; end
validateattributes(bounds_s,{'numeric'},{'real','vector','numel',2});
if all(isnan(bounds_s)), return; end
assert(all(isfinite(bounds_s)) && bounds_s(1)<=bounds_s(2),'边界必须有限且有序');
handles=gobjects(2,1);
for k=1:2
    handles(k)=xline(ax,bounds_s(k),'k--','HandleVisibility','off');
end
end
