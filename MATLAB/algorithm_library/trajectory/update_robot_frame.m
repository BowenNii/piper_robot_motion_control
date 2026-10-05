function update_robot_frame(h,points,T_end)
%UPDATE_ROBOT_FRAME 更新关节位置；仅在传入T_end时更新末端坐标轴。
assert(size(points,1)==3 && size(points,2)>=2 && ...
    all(isfinite(points(:))), 'points必须为有限的3×(n+1)矩阵');
set(h.link,'XData',points(1,:),'YData',points(2,:), ...
    'ZData',points(3,:));
set(h.joint,'XData',points(1,1:end-1), ...
    'YData',points(2,1:end-1),'ZData',points(3,1:end-1));
set(h.end_eff,'XData',points(1,end),'YData',points(2,end), ...
    'ZData',points(3,end));
if isfield(h,'axis')
    assert(nargin>=3 && isequal(size(T_end),[4,4]), ...
        '绘制了末端坐标轴时，更新必须传入T_end');
    p=T_end(1:3,4);
    R=T_end(1:3,1:3);
    scale=0.08;
    for i=1:3
        tip=p+scale*R(:,i);
        set(h.axis(i),'XData',[p(1),tip(1)], ...
            'YData',[p(2),tip(2)],'ZData',[p(3),tip(3)]);
    end
end
end
