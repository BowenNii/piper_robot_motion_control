function compare_five_profiles(root,runs)
%COMPARE_FIVE_PROFILES 五种方案与官方实际运动对照，无机器人通信。
% 输入root实验根目录，runs由analyze_today显式选择的十轮结果组成。
% 20ms速度只是统一网格角度差分；同时保留原始区间差分，不能当作电机真速度。
% 官方按首次可检测角度变化对齐，不是指令发布时刻，不计算其全过程跟随误差。
out=fullfile(root,'comparison_analysis');assert(~isfolder(out),'comparison_analysis已存在，不覆盖');
official=official_data(fullfile(root,'official_movej','run01'));
mkdir(out);figdir=fullfile(out,'figures');mkdir(figdir);
colors=lines(6);labels={'S1 1deg/s','S2 5deg/s','S3 8deg/s','S4 smooth','S5 trapezoid','Official measured'};
summary=table;
for k=1:10
    r=runs{k};loaded=load(fullfile(r.execution_dir,'analysis_complete','analysis_results.mat'),'P','C','M');
    runs{k}.P=loaded.P;runs{k}.C=loaded.C;runs{k}.M=loaded.M;
    row=table(r.meta.scheme,string(r.meta.direction),r.duration_s,max(r.joint_stats.peak_abs_error_rad), ...
        max(abs(r.joint_stats.final_minus_goal_rad)),max(r.span_ms), ...
        max((loaded.C.publish_start_elapsed_s-loaded.C.target_time_s)*1000), ...
        r.bad_status_rows,string(r.execution_dir),'VariableNames',{'scheme','direction','duration_s', ...
        'peak_joint_tracking_rad','endpoint_mean_joint_error_rad','max_frame_span_ms', ...
        'max_pc_lateness_ms','bad_status_rows','execution_dir'});
    summary=[summary;row]; %#ok<AGROW>
end
writetable(summary,fullfile(out,'selected_runs_summary.csv'));
for direction=1:2
    if direction==1,tag='outbound';else,tag='return';end
    selected=runs(direction:2:10);reference=official.legs(direction);
    % 不同方案时长不同：横轴保留实际秒，不用归一化掩盖快慢差异。
    f=base();tiledlayout(3,2);
    for j=1:6
        nexttile;hold on;
        for s=1:5,r=selected{s};mask=r.t>=0&r.t<=r.duration_s;
            plot(r.t(mask),r.q_feedback(mask,j)*180/pi,'Color',colors(s,:));end
        plot(reference.t,reference.q(:,j)*180/pi,'Color',colors(6,:),'LineWidth',1.1);
        axes_label(sprintf('J%d angle / deg',j));
        if j==1,legend(labels,'Location','best','FontSize',8);end
    end
    sgtitle([tag,'：实际关节角；方案按计划起点，官方按检测到运动起点对齐']);
    add_key(f,labels,colors);save_plot(f,figdir,[tag,'_01_angles']);
    f=base();tiledlayout(3,2);
    for j=1:6
        nexttile;hold on;
        for s=1:5,r=selected{s};[t,v]=grid_velocity(r.t,r.q_feedback,r.duration_s);
            stairs(t,v(:,j)*180/pi,'Color',colors(s,:));end
        [t,v]=grid_velocity(reference.t,reference.q,reference.duration_s);
        stairs(t,v(:,j)*180/pi,'Color',colors(6,:));yline(0,'k:');
        axes_label(sprintf('J%d 20ms difference / (deg/s)',j));
        if j==1,legend(labels,'Location','best','FontSize',8);end
    end
    sgtitle([tag,'：统一20ms网格的角度差分；非电机实测，不替代原始差分图']);
    add_key(f,labels,colors);save_plot(f,figdir,[tag,'_02_velocity_20ms']);
    f=base();tiledlayout(3,2);
    for j=1:6
        nexttile;hold on;
        for s=1:5,r=selected{s};m=r.M;mask=m.joint==j;tt=m.timestamp_s(mask)-r.origin_epoch_s;
            active=tt>=0&tt<=r.duration_s;vv=m.current_A(mask);plot(tt(active),vv(active),'Color',colors(s,:));end
        m=official.motors;tt=m.timestamp_s-reference.origin_epoch_s;
        mask=m.joint==j&tt>=0&tt<=reference.duration_s;
        plot(tt(mask),m.current_A(mask),'Color',colors(6,:));axes_label(sprintf('J%d current / A',j));
        if j==1,legend(labels,'Location','best','FontSize',8);end
    end
    sgtitle([tag,'：高速反馈电流；负载/速度不同，不能凭峰值直接判定抖动原因']);
    add_key(f,labels,colors);save_plot(f,figdir,[tag,'_03_current']);
    % 腕部局部细节：各方案中点前后1秒，同一局部时间轴，但非相同全局时刻。
    f=base();tiledlayout(3,2);
    for j=4:6
        nexttile;hold on;
        for s=1:5,r=selected{s};local=r.t-r.duration_s/2;mask=abs(local)<=1;
            center=interp1(r.t,r.q_feedback(:,j),r.duration_s/2);
            stairs(local(mask),(r.q_feedback(mask,j)-center)*180/pi,'Color',colors(s,:));end
        local=reference.t-reference.duration_s/2;mask=abs(local)<=1;
        center=interp1(reference.t,reference.q(:,j),reference.duration_s/2);
        stairs(local(mask),(reference.q(mask,j)-center)*180/pi,'Color',colors(6,:));
        axes_label(sprintf('J%d relative angle / deg',j));xlabel('各自运动中点的相对时间 / s');
        if j==4,legend(labels,'Location','best','FontSize',8);end
        nexttile;hold on;
        for s=1:5,r=selected{s};tt=r.t(1:end-1)-r.duration_s/2;vv=diff(r.q_feedback(:,j))./diff(r.t);
            mask=abs(tt)<=1;stairs(tt(mask),vv(mask)*180/pi,'Color',colors(s,:));end
        tt=reference.t(1:end-1)-reference.duration_s/2;vv=diff(reference.q(:,j))./diff(reference.t);
        mask=abs(tt)<=1;stairs(tt(mask),vv(mask)*180/pi,'Color',colors(6,:));yline(0,'k:');
        axes_label(sprintf('J%d raw interval / (deg/s)',j));xlabel('各自运动中点的相对时间 / s');
    end
    sgtitle([tag,' 腕部原始细节：保留零速/反向；量化尖峰不等于真实机械速度']);
    add_key(f,labels,colors);save_plot(f,figdir,[tag,'_04_wrist_detail']);
    f=base();tiledlayout(3,2);
    for j=1:6,nexttile;hold on;
        for s=1:5,r=selected{s};mask=r.t>=0&r.t<=r.duration_s;
            plot(r.t(mask),r.joint_error(mask,j)*180/pi,'Color',colors(s,:));end
        yline(0,'k:');axes_label(sprintf('J%d tracking error / deg',j));
        if j==1,legend(labels(1:5),'Location','best','FontSize',8);end
    end
    sgtitle([tag,'：反馈角减最近已发目标；官方无内部参考，不加入误差比较']);
    add_key(f,labels(1:5),colors);save_plot(f,figdir,[tag,'_05_tracking_error']);
    % 分开每种方案，避免六条高频差分曲线叠加后难以辨认。
    for kind=1:2
        f=base();f.Position(4)=1800;tiledlayout(6,3);
        for s=1:6
            if s<=5,r=selected{s};time=r.t;q=r.q_feedback;duration=r.duration_s;
            else,time=reference.t;q=reference.q;duration=reference.duration_s;end
            for j=4:6,nexttile;
                if kind==1
                    local=time-duration/2;mask=abs(local)<=.4;
                    center=interp1(time,q(:,j),duration/2);
                    stairs(local(mask),(q(mask,j)-center)*180/pi,'Color',colors(s,:));
                    ylabel('Relative angle / deg');
                else
                    local=time(1:end-1)-duration/2;velocity=diff(q(:,j))./diff(time);
                    mask=abs(local)<=.4;stairs(local(mask),velocity(mask)*180/pi,'Color',colors(s,:));
                    yline(0,'k:');ylabel('Raw difference / (deg/s)');
                end
                title(sprintf('%s J%d',labels{s},j),'Interpreter','none');grid on;
                xlabel('Relative to own motion midpoint / s');
            end
        end
        if kind==1,name='06_wrist_angles_separate';sgtitle([tag,'：腕部原始角度，逐方案分开；中心前后0.4秒']);
        else,name='07_wrist_velocity_separate';sgtitle([tag,'：腕部原始区间差分，保留零速与反向；非真实电机速度']);end
        save_plot(f,figdir,[tag,'_',name]);
    end
end
plot_official(official,out,figdir);
save(fullfile(out,'comparison_results.mat'),'runs','official','summary','-v7.3');
fid=fopen(fullfile(out,'README.md'),'w');assert(fid~=-1);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,['# 明日看图顺序\n\n' ...
    '1. outbound/return_06_wrist_angles_separate和07_wrist_velocity_separate：先逐方案看J4-J6角度台阶、零速和反向，再看04叠加对照。\n' ...
    '2. outbound/return_02_velocity_20ms、03_current：对比实际快慢、电流与官方运动。\n' ...
    '3. outbound/return_05_tracking_error：仅五组有已发参考，不能强加给官方。\n' ...
    '4. official_01至08：官方完整记录、两段裁分、位姿一致性、电流和通信质量。\n' ...
    '5. 每个execution的analysis_complete/figures：12张完整验证图。\n\n' ...
    '统一20ms重采样用线性插值再做区间差分，不是平滑真值，必须结合原始差分图。\n' ...
    '官方时间零点是首次可检测的反馈运动，不是命令时刻，不比较启动延迟。\n' ...
    '官方尾部后续关节变化排除在正式去回程之外，不认定其原因。\n' ...
    '电机速度轴侧/减速比未确认；差分加速度尖峰不认证实际冲击。\n' ...
    '所有位姿均为模型/CAN内部一致性，无外部真值。未自动给出抖动根因或安全认证。\n' ...
    '独立重复次数有限，视频和负载信息需人工补齐，不能据单轮图做统计显著性排名。\n']);
disp(summary);
end

function o=official_data(folder)
d=load_feedback_csv(fullfile(folder,'decoded','feedback_fk.csv'),struct('require_transforms',true,'strict_time',true));
M=readtable(fullfile(folder,'decoded','motors.csv'));
F=readtable(fullfile(folder,'ros_export','feedback.csv'));
S=readtable(fullfile(folder,'ros_export','status.csv'));
% 用反馈变化分组，0.5秒内变化属于同段；首两段经起终点验证后才作为对照。
changed=find(max(abs(diff(d.q_rad)),[],2)>1e-5)+1;
assert(~isempty(changed),'官方记录没有运动');
breaks=[1;find(diff(d.timestamp_s(changed))>.5)+1;numel(changed)+1];
assert(numel(breaks)>=3,'官方缺少去程或回程');
legs=struct([]);bounds=zeros(2,2);goal=[.3,.3,-.3,.3,.3,.5];
for k=1:2
    first=changed(breaks(k));last=changed(breaks(k+1)-1);
    origin=d.timestamp_s(max(1,first-1));finish=d.timestamp_s(last);
    before=max(1,first-1);after=min(size(d.q_rad,1),last+100);
    if k==1,q0=zeros(1,6);q1=goal;else,q0=goal;q1=zeros(1,6);end
    assert(max(abs(d.q_rad(before,:)-q0))<.005 && max(abs(d.q_rad(after,:)-q1))<.005, ...
        '官方裁分端点不符合预期，需要人工检查');
    mask=d.timestamp_s>=origin-.5 & d.timestamp_s<=finish+3;
    legs(k).t=d.timestamp_s(mask)-origin;legs(k).q=d.q_rad(mask,:); %#ok<AGROW>
    legs(k).origin_epoch_s=origin;legs(k).duration_s=finish-origin;
    legs(k).start_q=d.q_rad(before,:);legs(k).end_q=d.q_rad(after,:);
    bounds(k,:)=[origin,finish];
end
% 只对正式两段及间隔做MATLAB独立FK，后续关节变化不混入运动对照。
mask=d.timestamp_s>=bounds(1,1)-3 & d.timestamp_s<=bounds(2,2)+3;
repo=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
model=make_piper_model(fullfile(repo,'models','urdf','piper_description.urdf'));
cross=validate_fk_samples(model.S_list,model.M,d.q_rad(mask,:),d.T_cpp(:,:,mask), ...
    struct('position_tolerance_m',1e-9,'rotation_tolerance_rad',1e-8));
assert(cross.ok,'官方反馈MATLAB/C++ FK交叉验算失败');
Tcan=d.T_can(:,:,mask);N=nnz(mask);pcan=zeros(N,1);rcan=pcan;
for i=1:N,[pcan(i),rcan(i)]=pose_error(cross.T_matlab(:,:,i),Tcan(:,:,i));end
o=struct('folder',folder,'data',d,'motors',M,'ros_feedback',F,'ros_status',S,'legs',legs, ...
    'bounds_epoch_s',bounds,'cross',cross,'cross_time_epoch_s',d.timestamp_s(mask), ...
    'model_can_position_m',pcan,'model_can_rotation_rad',rcan);
end

function plot_official(o,out,figdir)
d=o.data;t=d.timestamp_s-d.timestamp_s(1);M=o.motors;
f=base();tiledlayout(3,2);
for j=1:6,nexttile;plot(t,d.q_deg(:,j));hold on;
    for k=1:2,xline(o.bounds_epoch_s(k,1)-d.timestamp_s(1),'g--');xline(o.bounds_epoch_s(k,2)-d.timestamp_s(1),'r--');end
    axes_label(sprintf('J%d angle / deg',j));end
sgtitle('官方完整CAN记录：绿/红为两段运动边界；后续变化不混入正式对照');
save_plot(f,figdir,'official_01_full_record');
for type=1:3
    f=base();tiledlayout(3,2);
    for j=1:6,nexttile;hold on;
        for k=1:2,l=o.legs(k);
            if type==1,stairs(l.t(1:end-1),diff(l.q(:,j))./diff(l.t)*180/pi);
            elseif type==2
                vel=diff(l.q(:,j))./diff(l.t);tv=(l.t(1:end-1)+l.t(2:end))/2;
                stairs(tv(1:end-1),diff(vel)./diff(tv)*180/pi);
            else,tt=M.timestamp_s-l.origin_epoch_s;mask=M.joint==j&tt>=-.5&tt<=l.duration_s+3;plot(tt(mask),M.motor_velocity_reported(mask));end
        end
        yline(0,'k:');axes_label(sprintf('J%d',j));if j==1,legend('outbound','return');end
    end
    titles={'官方原始区间角速度 / deg/s：保留静止与反向，不等同电机实测', ...
        '官方二次差分加速度 / deg/s^2：量化噪声尖峰不等于真实机械冲击', ...
        '官方电机速度原报告值：轴侧/减速比未确认，不与关节速度直接相减'};
    sgtitle(titles{type});save_plot(f,figdir,sprintf('official_%02d_derivative_motor',type+1));
end
f=base();tiledlayout(2,2);tt=o.cross_time_epoch_s-o.bounds_epoch_s(1,1);
nexttile;plot(tt,o.cross.position_error_m);yline(1e-9,'r:');axes_label('MATLAB/C++ position / m');
nexttile;plot(tt,o.cross.rotation_error_rad);yline(1e-8,'r:');axes_label('MATLAB/C++ rotation / rad');
nexttile;plot(tt,o.model_can_position_m*1000);axes_label('Model/CAN position / mm');
nexttile;plot(tt,o.model_can_rotation_rad*180/pi);axes_label('Model/CAN rotation / deg');
sgtitle('官方反馈FK一致性：无外部定位真值，不计算未知内部参考的跟随误差');
save_plot(f,figdir,'official_05_fk_consistency');
f=base();tiledlayout(3,1);
nexttile;plot(t,d.span_ms);yline(2,'r--');axes_label('Six-frame span / ms');
nexttile;plot(t(2:end),diff(t)*1000);axes_label('CAN group interval / ms');
nexttile;F=o.ros_feedback;tr=F.receive_epoch_s-d.timestamp_s(1);
plot(tr(2:end),diff(tr)*1000);axes_label('ROS bag receive interval / ms');
sgtitle('官方完整记录的通信质量：末尾停更不等同于正式运动段中断');
save_plot(f,figdir,'official_06_timing');
f=base();tiledlayout(2,2);
mask=d.timestamp_s>=o.cross_time_epoch_s(1)&d.timestamp_s<=o.cross_time_epoch_s(end);
pos=reshape(o.cross.T_matlab(1:3,4,:),3,[])';
for j=1:3,nexttile;plot(tt,pos(:,j)*1000);hold on;plot(tt,d.p_feedback(mask,j)*1000,'k:');
    axes_label(sprintf('%s / mm','X'+j-1));legend('MATLAB FK','CAN');end
nexttile;angle=zeros(numel(tt),1);R0=o.cross.T_matlab(1:3,1:3,1);
for i=1:numel(tt)
    R=R0'*o.cross.T_matlab(1:3,1:3,i);
    w=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
    angle(i)=atan2(norm(w),max(-1,min(1,(trace(R)-1)/2)));
end
plot(tt,angle*180/pi);axes_label('Rotation relative to initial pose / deg');
sgtitle('官方正式去回程位姿变化：实际反馈FK，不假设末端直线');
save_plot(f,figdir,'official_07_cartesian');
f=base();tiledlayout(3,2);
for j=1:6,nexttile;hold on;
    for k=1:2,l=o.legs(k);tm=M.timestamp_s-l.origin_epoch_s;
        active=M.joint==j&tm>=-.5&tm<=l.duration_s+3;
        plot(tm(active),M.current_A(active));end
    axes_label(sprintf('J%d current / A',j));if j==1,legend('outbound','return');end
end
sgtitle('官方正式去回程电流：J1-3估算Nm=I*1.18125/4；J4-6=I*0.95844');
save_plot(f,figdir,'official_08_current');
direction=["outbound";"return"];
T=table(direction,o.bounds_epoch_s(:,1),o.bounds_epoch_s(:,2),[o.legs.duration_s]', ...
    'VariableNames',{'direction','detected_start_epoch_s','detected_end_epoch_s','detected_motion_s'});
writetable(T,fullfile(out,'official_motion_windows.csv'));
end

function [t,v]=grid_velocity(time,q,duration)
grid=(0:.02:duration)';qq=interp1(time,q,grid,'linear');
t=grid(1:end-1);v=diff(qq)/.02;
end
function f=base()
f=figure('Visible','off','Color','w','Position',[50 50 1500 1100]);
set(f,'DefaultAxesFontName','Noto Sans CJK SC','DefaultTextFontName','Noto Sans CJK SC');
end
function axes_label(label)
grid on;xlabel('Time / s');ylabel(label);
end
function save_plot(f,folder,name)
exportgraphics(f,fullfile(folder,[name,'.png']),'Resolution',130);close(f);
end
function add_key(f,labels,colors)
% 用图级颜色键，避免跨tile绘制时局部legend被后续tile覆盖。
delete(findall(f,'Type','legend'));
layout=findall(f,'Type','tiledlayout');
if ~isempty(layout),layout(1).OuterPosition=[0 .055 1 .945];end
for k=1:numel(labels)
    annotation(f,'textbox',[.025+(k-1)*.162 .005 .16 .035], ...
        'String',labels{k},'Color',colors(k,:),'EdgeColor','none', ...
        'FontSize',10,'FontWeight','bold','Interpreter','none');
end
end
