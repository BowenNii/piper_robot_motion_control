function t=check_plot_data(t,y)
% 绘图也检查基本形状；不排序、不删行、不插值。
validateattributes(t,{'numeric'},{'real','finite','vector','nonempty'});
validateattributes(y,{'numeric'},{'real','finite','2d','nonempty'});
t=t(:);
assert(size(y,1)==numel(t),'数据必须N×列数，每行对应一个时间');
assert(all(diff(t)>=0),'绘图时间必须非递减；严格递增检查请用validation模块');
end
