%% run04角度差分估计速度/加速度；只读离线数据，不连接机械臂
% 灰线：原始角度按实际时间戳数值微分，含量化/反馈台阶影响。
% 蓝线：约100ms窗口局部二次拟合的导数，仅用于观察趋势。
% 两者都不是电机直接上报的速度/加速度，不据此认定实际力矩尖峰。
clear;clc;
script_dir=fileparts(mfilename('fullpath'));
repo_dir=fileparts(fileparts(fileparts(fileparts(script_dir))));
root=fullfile(repo_dir,'experiments','project1','2026-10-05_dynamic_fk');
D=readtable(fullfile(root,'data_csv','run04.csv'),'VariableNamingRule','preserve');
t=D.t_s; q=D{:,cellstr(compose('q%d_rad',1:6))}*180/pi;
assert(numel(t)>=21 && all(isfinite(t)) && all(diff(t)>0) && all(isfinite(q),'all'), ...
    '时间必须严格递增，角度必须有限');
v=zeros(size(q)); a=v;
for j=1:6
    v(:,j)=gradient(q(:,j),t);
    a(:,j)=gradient(v(:,j),t);
end
% 实际时间坐标上的局部多项式，避免把采样抖动当作固定dt。
half_window=10; window_s=median(diff(t))*2*half_window;
v_trend=zeros(size(q)); a_trend=v_trend;
for k=1:numel(t)
    first=max(1,min(k-half_window,numel(t)-2*half_window));
    idx=first:first+2*half_window;
    u=t(idx)-t(k);
    coefficients=[ones(size(u)),u,u.^2]\q(idx,:);
    v_trend(k,:)=coefficients(2,:);
    a_trend(k,:)=2*coefficients(3,:);
end
changed=find(any(abs(diff(q))>0.0005,2));
assert(~isempty(changed),'未找到运动');
motion=[max(t(1),t(changed(1))-.1),min(t(end),t(changed(end)+1)+.1)];
fig_dir=fullfile(root,'figures');out_dir=fullfile(root,'analysis');
if ~isfolder(fig_dir),mkdir(fig_dir);end
if ~isfolder(out_dir),mkdir(out_dir);end
visible='off';if usejava('desktop'),visible='on';end
% 相邻区间速度：与原始角度折线每一段的斜率严格对应。
% 相邻角度相等时，该区间估计速度严格为0；不跨平台做中心差分。
v_interval=diff(q)./diff(t);
assert(all(v_interval(diff(q)==0)==0),'平台区间速度应为零');
f=figure('Visible',visible,'Color','w','Position',[70 70 1500 1000]);
tiledlayout(3,2,'TileSpacing','compact');
for j=1:6
    nexttile;stairs(t,[v_interval(:,j);NaN],'b');yline(0,'r:');
    xlim(motion);grid on;title(sprintf('J%d',j));xlabel('时间 / s');
    ylabel('区间估计速度 / (°/s)');set(gca,'FontName','Noto Sans CJK SC');
end
sgtitle({'run04 相邻采样区间速度：不平滑、不跨平台平均', ...
    '每一水平段对应一个采样区间；零值仅表示该区间反馈角没有变化'}, ...
    'FontName','Noto Sans CJK SC');
exportgraphics(f,fullfile(fig_dir,'matlab_run04_velocity_intervals.png'),'Resolution',150);
if strcmp(visible,'off'),close(f);end
f=figure('Visible',visible,'Color','w','Position',[70 70 1400 850]);
tiledlayout(2,1,'TileSpacing','compact');
nexttile;plot(t,q(:,1),'.-');ylabel('J1反馈角 / °');
nexttile;stairs(t,[v_interval(:,1);NaN],'b','LineWidth',1.2);hold on;
yline(0,'r--');ylabel('相邻区间估计速度 / (°/s)');
for ax=findall(f,'Type','axes')'
    xlim(ax,[9.195 9.265]);grid(ax,'on');xlabel(ax,'时间 / s');
    set(ax,'FontName','Noto Sans CJK SC');
end
sgtitle({'run04 J1红圈区间：平台对应零区间速度', ...
    '不是驱动实测速度，不能仅凭零反馈变化认定真实电机停止'}, ...
    'FontName','Noto Sans CJK SC');
exportgraphics(f,fullfile(fig_dir,'matlab_run04_j1_interval_velocity_zoom.png'),'Resolution',150);
if strcmp(visible,'off'),close(f);end
for order=1:2
    if order==1
        raw=v;trend=v_trend;label='估计速度 / (°/s)';kind='velocity';cn='关节速度';
    else
        raw=a;trend=a_trend;label='估计加速度 / (°/s²)';kind='acceleration';cn='关节加速度';
    end
    f=figure('Visible',visible,'Color','w','Position',[70 70 1500 1000]);
    tiledlayout(3,2,'TileSpacing','compact');
    for j=1:6
        nexttile;plot(t,raw(:,j),'Color',[.75 .75 .75]);hold on;
        plot(t,trend(:,j),'b','LineWidth',1.2);yline(0,'k:');
        xlim(motion);grid on;title(sprintf('J%d',j));xlabel('时间 / s');ylabel(label);
        set(gca,'FontName','Noto Sans CJK SC');
        if j==1,legend('原始角度差分估计','约100ms局部拟合趋势','Location','best');end
    end
    sgtitle({['run04 多轴去程：',cn], ...
        '不是驱动实测；原始差分含反馈台阶尖峰，平滑趋势不能用于判定峰值合规'}, ...
        'FontName','Noto Sans CJK SC');
    exportgraphics(f,fullfile(fig_dir,['matlab_run04_',kind,'.png']),'Resolution',150);
    if strcmp(visible,'off'),close(f);end
end
f=figure('Visible',visible,'Color','w','Position',[70 70 1400 950]);
tiledlayout(3,1,'TileSpacing','compact');
nexttile;plot(t,q(:,1),'.-');ylabel('J1角度 / °');
nexttile;plot(t,v(:,1),'Color',[.7 .7 .7]);hold on;plot(t,v_trend(:,1),'b');
ylabel('估计速度 / (°/s)');legend('原始差分','约100ms趋势');
nexttile;plot(t,a(:,1),'Color',[.7 .7 .7]);hold on;plot(t,a_trend(:,1),'b');
ylabel('估计加速度 / (°/s²)');
for ax=findall(f,'Type','axes')'
    xlim(ax,[8.9 9.4]);grid(ax,'on');xlabel(ax,'时间 / s');set(ax,'FontName','Noto Sans CJK SC');
end
sgtitle({'run04 J1局部放大：角度、估计速度与估计加速度', ...
    '原始点保留；100ms拟合会抑制快速变化，不能证明真实运动没有抖动'}, ...
    'FontName','Noto Sans CJK SC');
exportgraphics(f,fullfile(fig_dir,'matlab_run04_j1_derivatives_zoom.png'),'Resolution',150);
if strcmp(visible,'off'),close(f);end
save(fullfile(out_dir,'run04_derivatives.mat'),'t','q','v','a','v_interval','v_trend','a_trend','window_s','motion');
fprintf('已生成速度、加速度、J1局部放大三张图；拟合窗口约%.1fms。\n',window_s*1000);
fprintf('保存于：%s\n',fig_dir);
