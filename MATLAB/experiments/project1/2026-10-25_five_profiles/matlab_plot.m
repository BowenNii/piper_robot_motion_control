%% 五组规划的离线对比：解析速度/加速度，不是真机反馈，不发送指令
clear;clc;
script_dir=fileparts(mfilename('fullpath'));
repo=fileparts(fileparts(fileparts(fileparts(script_dir))));
root=fullfile(repo,'experiments','project1','2026-10-25_five_profiles');
root=fullfile(root,'reference_complete'); % 含C++ FK位姿列的完整参考，旧离线输出不覆盖。
assert(isfile(fullfile(root,'OFFLINE_ONLY.txt')),'生成器未成功完成');
summary=readtable(fullfile(root,'summary.csv'));
figdir=fullfile(root,'figures');if ~isfolder(figdir),mkdir(figdir);end
names={'全程五次：1°/s上限','全程五次：5°/s上限','全程五次：8°/s上限', ...
    '五次速度过渡＋匀速','梯形速度：加速度跳变'};
visible='off';if usejava('desktop'),visible='on';end
Q=cell(5,1);B=cell(5,1);
for k=1:5
    Q{k}=readtable(fullfile(root,sprintf('scheme%d_outbound.csv',k)));
    B{k}=readtable(fullfile(root,sprintf('scheme%d_return.csv',k)));
    D=Q{k};R=B{k};
    shown_name=names{k};
    if k==4 && summary.cruise_time_s(k)<1e-10
        shown_name='五次速度过渡：距离不足，降峰值且无匀速段';
    end
    assert(all(abs(diff(D.t_s)-.02)<1e-10),'采样必须为20ms');
    q=D{:,cellstr(compose('q%d_rad',1:6))};
    v=D{:,cellstr(compose('dq%d_rad_s',1:6))};
    a=D{:,cellstr(compose('ddq%d_rad_s2',1:6))};
    assert(all(isfinite([q,v,a]),'all'));
    assert(max(abs(v),[],'all')*180/pi<=summary.peak_velocity_deg_s(k)+1e-8);
    assert(max(abs(a),[],'all')*180/pi<=summary.peak_acceleration_deg_s2(k)+1e-8);
    assert(max(abs(R{end,cellstr(compose('q%d_rad',1:6))}))<1e-12,'回程未到零位');
    f=figure('Visible',visible,'Color','w','Position',[80 60 1450 1050]);
    tiledlayout(3,2,'TileSpacing','compact');
    for direction=1:2
        if direction==1,Z=D;label='去程';else,Z=R;label='回程（理论起点）';end
        columns={cellstr(compose('q%d_rad',1:6)),cellstr(compose('dq%d_rad_s',1:6)), ...
            cellstr(compose('ddq%d_rad_s2',1:6))};
        units={'角度 / °','速度 / (°/s)','加速度 / (°/s²)'};
        for order=1:3
            nexttile((order-1)*2+direction);
            if k==5 && order==3
                stairs(Z.t_s,Z{:,columns{order}}*180/pi,'LineWidth',1);
            else
                plot(Z.t_s,Z{:,columns{order}}*180/pi,'LineWidth',1);
            end
            grid on;xlabel('时间 / s');ylabel(units{order});title(label);
            set(gca,'FontName','Noto Sans CJK SC');
            if order==1,legend(compose('J%d',1:6),'Location','best');end
        end
    end
    sgtitle({sprintf('方案%d：%s；单程采样时长 %.2fs',k,shown_name,D.t_s(end)), ...
        '离线规划，不含碰撞/力矩验证；方案5加加速度在切换处不连续'},'FontName','Noto Sans CJK SC');
    exportgraphics(f,fullfile(figdir,sprintf('scheme%d_q_v_a.png',k)),'Resolution',130);
    if strcmp(visible,'off'),close(f);end
end
% 同一最大位移轴J6，采用相同物理时间横轴，不能按归一化时间比较速度。
f=figure('Visible',visible,'Color','w','Position',[80 60 1400 1000]);
tiledlayout(3,1,'TileSpacing','compact');
fields={'q6_rad','dq6_rad_s','ddq6_rad_s2'};
labels={'J6角度 / °','J6速度 / (°/s)','J6加速度 / (°/s²)'};
for order=1:3
    nexttile;hold on;
    for k=1:5
        if k==5 && order==3,stairs(Q{k}.t_s,Q{k}.(fields{order})*180/pi,'LineWidth',1.2);
        else,plot(Q{k}.t_s,Q{k}.(fields{order})*180/pi,'LineWidth',1.2);end
    end
    grid on;xlabel('时间 / s');ylabel(labels{order});set(gca,'FontName','Noto Sans CJK SC');
    legend(names,'Location','best');
end
sgtitle('五组去程解析轨迹对比：J6最大位移轴（非真机执行结果）','FontName','Noto Sans CJK SC');
exportgraphics(f,fullfile(figdir,'comparison_j6.png'),'Resolution',130);
if strcmp(visible,'off'),close(f);end
disp(summary);fprintf('已验证10份轨迹并生成6张图：%s\n',figdir);
