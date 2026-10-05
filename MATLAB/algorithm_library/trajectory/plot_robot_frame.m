function h = plot_robot_frame(points,ax_lim,T_end)
%PLOT_ROBOT_FRAME 绘制关节原点与末端；可选绘制真实末端坐标轴。
%   h=plot_robot_frame(points,ax_lim,T_end)
%   points来自get_joint_points；仅传points时不臆造末端姿态。
assert(size(points,1)==3 && size(points,2)>=2 && ...
    all(isfinite(points(:))), 'points必须为有限的3×(n+1)矩阵');
if nargin<2 || isempty(ax_lim)
    span=max(max(points,[],2)-min(points,[],2));
    span=max(span,0.1);
    center=(max(points,[],2)+min(points,[],2))/2;
    ax_lim=[center(1)-span,center(1)+span, ...
        center(2)-span,center(2)+span, ...
        center(3)-span,center(3)+span];
end
assert(numel(ax_lim)==6 && all(isfinite(ax_lim)), 'ax_lim必须为6维有限向量');
hold on; grid on; axis equal; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('机器人关节原点与末端');
axis(ax_lim);
h.link=plot3(points(1,:),points(2,:),points(3,:),'b-','LineWidth',2);
h.joint=scatter3(points(1,1:end-1),points(2,1:end-1), ...
    points(3,1:end-1),50,'r','filled');
h.end_eff=scatter3(points(1,end),points(2,end),points(3,end), ...
    80,'g','filled');
if nargin>=3 && ~isempty(T_end)
    assert(isequal(size(T_end),[4,4]), 'T_end必须为4×4位姿');
    h.axis=draw_end_frame(T_end,0.08);
end
end

function handles=draw_end_frame(T,scale)
p=T(1:3,4);
R=T(1:3,1:3);
colors={'r','g','b'};
handles=gobjects(1,3);
for i=1:3
    tip=p+scale*R(:,i);
    handles(i)=plot3([p(1),tip(1)],[p(2),tip(2)], ...
        [p(3),tip(3)],colors{i},'LineWidth',2);
end
end
