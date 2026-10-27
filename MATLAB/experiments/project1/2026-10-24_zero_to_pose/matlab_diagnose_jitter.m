%% 抖动定位：只分析现有记录，不连接ROS，不修改控制参数
clear;clc;
script_dir=fileparts(mfilename('fullpath'));
repo=fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(fullfile(repo,'MATLAB'));setup_piper_paths;
execution=fullfile(repo,'experiments','project1','2026-10-24_zero_to_pose','run01','execution_mwd2hy56');
source=fullfile(execution,'matlab_analysis','analysis_results.mat');
assert(isfile(source),'请先运行本目录matlab_plot.m');
loaded=load(source,'r');r=loaded.r;
assert(r.alignment=="recorded_monotonic_origin" && r.meta.sample_count==3001);
out=fullfile(execution,'matlab_analysis','jitter_diagnostics');if ~isfolder(out),mkdir(out);end
cfg=struct('show_figures',usejava('desktop'),'font','Noto Sans CJK SC');
startup=r.t_feedback>=5&r.t_feedback<=10;
command_start=r.C.target_time_s>=5&r.C.target_time_s<=10;
qcols=cellstr(compose('q%d_rad',1:6));
t=r.t_feedback(startup);q=r.q_feedback(startup,:);e=r.joint_error(startup,:);
direction=sign(r.plan.q_rad(end,:)-r.plan.q_rad(1,:));
peak=zeros(6,1);peak_time=peak;jump=peak;jump_time=peak;hold=peak;backward=peak;cmd_step=peak;
for j=1:6
    [peak(j),i]=max(abs(e(:,j)));peak_time(j)=t(i);
    d=diff(q(:,j));[jump(j),i]=max(abs(d));jump_time(j)=t(i+1);
    changes=find(abs(d)>1e-9)+1;
    boundaries=unique([1;changes;numel(t)]);hold(j)=max(diff(t(boundaries)));
    backward(j)=max([0;-direction(j)*d]);
    cmd_step(j)=max(abs(diff(r.C{command_start,qcols(j)})));
end
startup_stats=table((1:6)',peak,peak_time,jump,jump_time,hold,backward,cmd_step, ...
    'VariableNames',{'joint','peak_abs_error_rad','peak_error_time_s','max_feedback_step_rad', ...
    'max_step_time_s','max_recorded_hold_s','max_reverse_step_rad','max_command_step_rad'});
startup_stats.reverse_steps_over_001rad=zeros(6,1);
for j=1:6,startup_stats.reverse_steps_over_001rad(j)=sum(-direction(j)*diff(q(:,j))>.001);end
startup_stats.first_change_over_001rad_time_s=nan(6,1);
startup_stats.sent_change_at_first_event_deg=nan(6,1);
for j=1:6
    first=find(r.t_feedback>=0 & abs(r.q_feedback(:,j)-r.q_initial_mean(j))>.001,1);
    if ~isempty(first)
        startup_stats.first_change_over_001rad_time_s(j)=r.t_feedback(first);
        startup_stats.sent_change_at_first_event_deg(j)= ...
            (r.q_sent_at_feedback(first,j)-r.plan.q_start(j))*180/pi;
    end
end

% 找实际最大发送间隔，不把它误认为发送前迟到或CAN电机更新时间。
send_dt=diff(r.C.send_elapsed_s);[gap,index]=max(send_dt);index=index+1;
event_time=r.C.send_elapsed_s(index);
before=r.t_feedback>=event_time-.5&r.t_feedback<event_time;
after=r.t_feedback>=event_time&r.t_feedback<=event_time+.5;
receiver_dt=diff(r.J.receive_elapsed_s);[receive_gap,ri]=max(receiver_dt);
between=r.C.send_elapsed_s-r.C.target_time_s-r.C.lateness_s;
event_stats=table(r.C.index(index),event_time,gap*1000, ...
    r.C.lateness_s(index)*1000,between(index)*1000,receive_gap*1000,r.interval_time(ri), ...
    max(r.p_error_mm(before)),max(r.p_error_mm(after)), ...
    max(r.rotation_error_deg(before)),max(r.rotation_error_deg(after)), ...
    'VariableNames',{'command_index','send_return_time_s','max_send_interval_ms', ...
    'pre_send_lateness_ms','precheck_to_send_return_ms','max_receive_interval_ms', ...
    'receive_peak_time_s','position_peak_before_mm','position_peak_after_mm', ...
    'rotation_peak_before_deg','rotation_peak_after_deg'});
window_stats=table;
for k=1:3
    bounds={[5,10],[event_time-.5,event_time+.5],[10,60]};b=bounds{k};
    mask=r.t_feedback>=b(1)&r.t_feedback<=b(2);
    interval=r.interval_time>=b(1)&r.interval_time<=b(2);
    cm=r.C.target_time_s>=b(1)&r.C.target_time_s<=b(2);
    row=table(b(1),b(2),max(abs(r.joint_error(mask,:)),[],'all'), ...
        max(r.p_error_mm(mask)),max(r.rotation_error_deg(mask)), ...
        max(r.dt_receive_ms(interval)),max(r.dt_header_ms(interval)),max(r.C.lateness_s(cm))*1000, ...
        'VariableNames',{'start_s','end_s','joint_peak_rad','position_peak_mm','rotation_peak_deg', ...
        'receive_max_ms','header_max_ms','pre_send_lateness_max_ms'});
    window_stats=[window_stats;row]; %#ok<AGROW>
end
save_analysis_results(out,struct('startup_stats',startup_stats,'event_stats',event_stats, ...
    'window_stats',window_stats),struct('startup_stats',startup_stats,'event_stats',event_stats, ...
    'window_stats',window_stats,'source',source),struct('overwrite',true));

f=new_plot('抖动定位1：启动阶段J4-J6',cfg);tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for j=4:6
    nexttile;plot(r.C.send_elapsed_s,r.C{:,qcols(j)}*180/pi,'b-','LineWidth',1.3);hold on;
    plot(t,q(:,j)*180/pi,'Color',[.9 .35 .05]);grid on;xlim([5,10]);xlabel('指令起点相对时间 / s');ylabel('角度 / °');
    title(sprintf('J%d：计划连续变化与反馈保持/跳变',j));if j==4,legend('发送目标（返回时刻）','反馈（接收时刻）');end
    nexttile;plot(t,e(:,j),'LineWidth',1.0);hold on;yline(0,'k:');
    xline(peak_time(j),'r:');grid on;xlim([5,10]);xlabel('指令起点相对时间 / s');ylabel('反馈 - 最近已发目标 / rad');
    title(sprintf('J%d：峰值 %.6f rad @ %.3fs',j,peak(j),peak_time(j)));
end
sgtitle('5～10秒启动窗口：反馈跳变不等于物理瞬时速度，不能单凭此图定根因');
save_plot(f,out,'jitter_01_startup.png',cfg);

f=new_plot('抖动定位2：最大发送间隔附近',cfg);tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
nexttile;plot(r.C.send_elapsed_s(2:end),send_dt*1000);hold on;yline(20,'k:');ylabel('发送间隔 / ms');title('发送调用返回时刻之差');
nexttile;plot(r.interval_time,r.dt_receive_ms);hold on;plot(r.interval_time,r.dt_header_ms,'--');
ylabel('反馈间隔 / ms');legend('接收间隔','消息时间戳间隔');
nexttile;plot(r.t_feedback,r.joint_error);ylabel('关节偏差 / rad');legend('J1','J2','J3','J4','J5','J6','Location','eastoutside');
nexttile;yyaxis left;plot(r.t_feedback,r.p_error_mm);ylabel('位置偏差 / mm');
yyaxis right;plot(r.t_feedback,r.rotation_error_deg);ylabel('姿态偏差 / °');
axes_handles=findall(f,'Type','axes');
for a=axes_handles',xlim(a,[event_time-.5,event_time+.5]);grid(a,'on');xline(a,event_time,'r:','HandleVisibility','off');xlabel(a,'指令起点相对时间 / s');end
sgtitle(sprintf('最大发送间隔附近 %.6fs：先看时间对应关系，不据此断言因果',event_time));
save_plot(f,out,'jitter_02_timing_event.png',cfg);
assert(height(startup_stats)==6 && height(window_stats)==3);
assert(abs(event_stats.max_send_interval_ms-max(diff(r.C.send_elapsed_s))*1000)<1e-9);
assert(all(startup_stats.max_command_step_rad<.001),'检查计划轨迹是否有意外突跳');
disp(startup_stats);disp(event_stats);disp(window_stats);
fprintf('运动窗口最大反馈间隔与消息时间戳间隔须分开看；接收时刻不等于硬件采样时刻。\n');
fprintf('结果保存：%s\n',out);

function f=new_plot(name,cfg)
visible='off';if cfg.show_figures,visible='on';end
f=figure('Name',name,'Visible',visible,'Color','w','Position',[80,60,1400,950],'Renderer','painters');
set(f,'DefaultAxesFontName',cfg.font,'DefaultTextFontName',cfg.font,'DefaultAxesFontSize',11, ...
    'DefaultTextInterpreter','none','DefaultLegendInterpreter','none');
end
function save_plot(f,out,name,cfg)
drawnow;exportgraphics(f,fullfile(out,name),'Resolution',150);
if ~cfg.show_figures,close(f);end
end
