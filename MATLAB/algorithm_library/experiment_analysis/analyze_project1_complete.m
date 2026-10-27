function r=analyze_project1_complete(execution_dir)
%ANALYZE_PROJECT1_COMPLETE 五组真机单程完整分析，仅读文件，不连接ROS/CAN。
% 输入：execution目录，含summary/commands/feedback/status及decoded/。
% decoded由project1_feedback_export从raw_can.log生成；父目录保存trajectory.csv。
% 输出：新的analysis_complete目录，12张图、统计CSV和MAT；拒绝覆盖。
% 三类误差：跟随、MATLAB/C++交叉验算、MATLAB/CAN一致性，不是外部定位精度。
% 速度/加速度差分是估计；电机侧速度未经减速比确认，独立绘制不冒充关节速度。
execution_dir=char(execution_dir);
repo=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
P=readtable(fullfile(fileparts(execution_dir),'trajectory.csv'),'VariableNamingRule','preserve');
F=readtable(fullfile(execution_dir,'feedback.csv'),'VariableNamingRule','preserve');
C=readtable(fullfile(execution_dir,'commands.csv'),'VariableNamingRule','preserve');
S=readtable(fullfile(execution_dir,'status.csv'),'VariableNamingRule','preserve');
meta=jsondecode(fileread(fullfile(execution_dir,'summary.json')));
decoder=jsondecode(fileread(fullfile(execution_dir,'decoded','decode_summary.json')));
assert(decoder.complete,'解码未完成');
d=load_feedback_csv(fullfile(execution_dir,'decoded','feedback_fk.csv'), ...
    struct('require_transforms',true,'strict_time',true));
M=readtable(fullfile(execution_dir,'decoded','motors.csv'),'VariableNamingRule','preserve');
if height(M)==0,M=array2table(zeros(0,6),'VariableNames',M.Properties.VariableNames);end
K=readtable(fullfile(execution_dir,'decoded','can_commands.csv'),'VariableNamingRule','preserve');
out=fullfile(execution_dir,'analysis_complete');assert(~isfolder(out),'分析目录已存在，请保留旧结果或改名后重跑');
qcols=cellstr(compose('q%d_rad',1:6));vcols=cellstr(compose('dq%d_rad_s',1:6));
acols=cellstr(compose('ddq%d_rad_s2',1:6));
q=P{:,qcols};v=P{:,vcols};a=P{:,acols};
assert(all(isfinite([q,v,a]),'all') && all(diff(P.t_s)>0),'计划数据不合法');
model=make_piper_model(fullfile(repo,'models','urdf','piper_description.urdf'));
% 同一q重新算MATLAB，再与C++独立实现比较，绝不是重复比较同一数组。
cross=validate_fk_samples(model.S_list,model.M,d.q_rad,d.T_cpp, ...
    struct('position_tolerance_m',1e-9,'rotation_tolerance_rad',1e-8));
assert(cross.ok,'反馈MATLAB/C++交叉校验失败');
rr={};for row=1:3,for col=1:3,rr{end+1}=sprintf('poe_r%d%d',row,col);end,end %#ok<AGROW>
Tplan=cpp_columns_to_transforms(P{:,{'poe_x_m','poe_y_m','poe_z_m'}},P{:,rr});
plan_cross=validate_fk_samples(model.S_list,model.M,q,Tplan, ...
    struct('position_tolerance_m',1e-9,'rotation_tolerance_rad',1e-8));
assert(plan_cross.ok,'计划MATLAB/C++交叉校验失败');
% epoch用于CAN，单调时钟用于PC发送与ROS接收；不用拟合轨迹去隐藏延迟。
if isfield(meta,'trajectory_start_epoch_s')
    origin=meta.trajectory_start_epoch_s;alignment='recorded_wall_monotonic_origin';
else
    assert(isfield(meta,'trajectory_start_receive_elapsed_s') && height(F)>0,'没有可追溯的时钟原点');
    origin=median(F.header_stamp_s-F.receive_elapsed_s)+meta.trajectory_start_receive_elapsed_s;
    alignment='legacy_header_receive_clock_relation';
end
t=d.timestamp_s-origin;tm=M.timestamp_s-origin;duration=P.t_s(end);
Tmat=cross.T_matlab;N=numel(t);p_mat=reshape(Tmat(1:3,4,:),3,N)';
v_num=zeros(N,6);a_num=v_num;v_interval=diff(d.q_rad)./diff(t);
for j=1:6,v_num(:,j)=gradient(d.q_rad(:,j),t);a_num(:,j)=gradient(v_num(:,j),t);end
tracking=height(C)>0;
qsent=nan(N,6);eq=qsent;ep=nan(N,1);er=ep;p_sent=nan(N,3);
if tracking
    assert(all(diff(C.send_elapsed_s)>0) && all(isfinite(C{:,qcols}),'all'),'发送记录异常');
    index=C.index+1;
    assert(all(index>=1 & index<=height(P)) && all(diff(index)==1),'发送点序号异常');
    assert(max(abs(C{:,qcols}-q(index,:)),[],'all')<1e-10,'实际发送目标不符合本轮计划');
    ci=interp1(C.send_elapsed_s,(1:height(C))',t,'previous','extrap');
    ci=max(1,min(height(C),ci));
    qsent=C{ci,qcols};eq=d.q_rad-qsent;
    Tsent=Tplan(:,:,index(ci));p_sent=reshape(Tsent(1:3,4,:),3,N)';
    for k=1:N,[ep(k),er(k)]=pose_error(Tmat(:,:,k),Tsent(:,:,k));end
    % 首条指令之前没有已发参考，不能把准备/其他运动画成跟随误差。
    % 只分析本段及其3秒保持，其余原始CAN仍保留在模型/通信图中。
    outside=t<C.send_elapsed_s(1) | t>duration+3;
    eq(outside,:)=NaN;ep(outside)=NaN;er(outside)=NaN;
end
dp_can=p_mat-d.p_feedback;ep_can=zeros(N,1);er_can=ep_can;
for k=1:N,[ep_can(k),er_can(k)]=pose_error(Tmat(:,:,k),d.T_can(:,:,k));end
% 独立CAN日志可能覆盖其他操作：终点统计不取整份日志最后一段。
observation_end=min(max(t),duration+3);
tail=t>=observation_end-.5 & t<=observation_end;motion=t>=0 & t<=duration;
assert(any(motion),'CAN记录未覆盖运动时间，请检查时钟原点');
% 统计按当前计划阶段标注；失败的执行不伪装成完整到位实验。
if ~ismember('phase',P.Properties.VariableNames) || all(P.phase==0)
    phase_plan=2*(P.t_s>=duration/2);
else,phase_plan=P.phase;end
phase=interp1(P.t_s,phase_plan,max(0,min(duration,t)),'previous');phase(t<0 | t>duration)=3;
joint=(1:6)';peak_abs_error_rad=nan(6,1);rms_error_rad=peak_abs_error_rad;
final_minus_goal_rad=mean(d.q_rad(tail,:),1)'-q(end,:)';
tail_range_rad=(max(d.q_rad(tail,:),[],1)-min(d.q_rad(tail,:),[],1))';
if tracking
    peak_abs_error_rad=max(abs(eq(motion,:)),[],1,'omitnan')';rms_error_rad=sqrt(mean(eq(motion,:).^2,1,'omitnan'))';
end
joint_stats=table(joint,peak_abs_error_rad,rms_error_rad,final_minus_goal_rad,tail_range_rad);
phase_stats=table;
for k=0:3
    mask=t>=0 & t<=duration+3 & phase==k;
    if any(mask) && tracking
        row=table(k,nnz(mask),max(ep(mask),[],'omitnan')*1000,sqrt(mean(ep(mask).^2,'omitnan'))*1000, ...
            max(er(mask),[],'omitnan')*180/pi,'VariableNames',{'phase','samples','position_peak_mm','position_rms_mm','rotation_peak_deg'});
        phase_stats=[phase_stats;row]; %#ok<AGROW>
    end
end
[endpoint_p,endpoint_r]=pose_error(forward_poe_space(model.S_list,mean(d.q_rad(tail,:),1)',model.M),Tplan(:,:,end));
pose_stats=table(endpoint_p*1000,endpoint_r*180/pi,max(ep_can)*1000,max(er_can)*180/pi, ...
    max(cross.position_error_m),max(cross.rotation_error_rad), ...
    'VariableNames',{'endpoint_model_position_mm','endpoint_model_rotation_deg','model_can_peak_mm', ...
    'model_can_peak_deg','matlab_cpp_position_m','matlab_cpp_rotation_rad'});
mkdir(out);figdir=fullfile(out,'figures');mkdir(figdir);
writetable(joint_stats,fullfile(out,'joint_stats.csv'));writetable(pose_stats,fullfile(out,'pose_stats.csv'));
if ~isempty(phase_stats),writetable(phase_stats,fullfile(out,'phase_stats.csv'));end
timing_stats=interval_stats('can_six_frame_groups',d.timestamp_s);
for j=1:6,timing_stats=[timing_stats;interval_stats(sprintf('can_motor_J%d',j),M.timestamp_s(M.joint==j))];end %#ok<AGROW>
if tracking,timing_stats=[timing_stats;interval_stats('pc_publish_returns',C.send_elapsed_s)];end
if height(F)>1,timing_stats=[timing_stats;interval_stats('ros_feedback_receive',F.receive_elapsed_s)];end
writetable(timing_stats,fullfile(out,'timing_stats.csv'));
% 所有图的灰线/估计值保留原始台阶影响，不用平滑掩盖异常。
f=figure_base();tiledlayout(3,2);
for j=1:6,nexttile;plot(P.t_s,q(:,j)*180/pi,'b');hold on;
    plot(t,d.q_rad(:,j)*180/pi,'Color',[.9 .35 .05]);
    if tracking,stairs(C.send_elapsed_s,C{:,qcols{j}}*180/pi,'k:');end
    format_axis(sprintf('J%d角度 / °',j));end
sgtitle('01 关节位置：蓝=计划，橙=CAN角度，黑点线=已发目标');savefig_png(f,figdir,'01_joint_position');
f=figure_base();tiledlayout(3,2);
for j=1:6,nexttile;plot(t,eq(:,j)*180/pi);yline(0,'k:');format_axis(sprintf('J%d跟随误差 / °',j));end
sgtitle('02 CAN角减最近已发目标角；缺少发送记录则不计算');savefig_png(f,figdir,'02_joint_error');
f=figure_base();tiledlayout(3,2);
for j=1:6,nexttile;plot(P.t_s,v(:,j)*180/pi,'b','LineWidth',1.3);hold on;
    stairs(t,[v_interval(:,j);NaN]*180/pi,'Color',[.7 .7 .7]);format_axis(sprintf('J%d估计速度 / (°/s)',j));end
sgtitle('03 蓝=规划关节速度，灰=反馈角相邻区间差分（不是电机实测）');savefig_png(f,figdir,'03_joint_velocity');
f=figure_base();tiledlayout(3,2);
for j=1:6,nexttile;plot(t,a_num(:,j)*180/pi,'Color',[.7 .7 .7]);hold on;
    plot(P.t_s,a(:,j)*180/pi,'b','LineWidth',1.3);format_axis(sprintf('J%d加速度 / (°/s²)',j));end
sgtitle('04 蓝=解析规划加速度，灰=角度二次差分；灰色尖峰不能认证真实力矩');savefig_png(f,figdir,'04_joint_acceleration');
f=figure_base();tiledlayout(3,2);
for j=1:6,nexttile;mask=M.joint==j;plot(tm(mask),M.motor_velocity_reported(mask));format_axis(sprintf('J%d电机速度原报告值',j));end
sgtitle('05 高速CAN速度：SDK字段缩放后rad/s，轴侧/减速比未确认，不与关节规划直接相减');savefig_png(f,figdir,'05_motor_velocity');
f=figure_base();tiledlayout(3,2);
for j=1:6,nexttile;mask=M.joint==j;yyaxis left;plot(tm(mask),M.current_A(mask));ylabel('电流 / A');
    yyaxis right;plot(tm(mask),M.torque_estimated_Nm(mask));ylabel('估算力矩 / Nm');format_axis(sprintf('J%d：厂家公式，非独立力矩测量',j));end
sgtitle('06 电流与估算力矩：两者按常数换算，不是两份独立证据');savefig_png(f,figdir,'06_current_torque');
f=figure_base();tiledlayout(2,2);
for j=1:3,nexttile;plot(P.t_s,reshape(Tplan(j,4,:),[],1)*1000,'b');hold on;
    plot(t,p_mat(:,j)*1000,'Color',[.9 .35 .05]);plot(t,d.p_feedback(:,j)*1000,'k:');
    format_axis(sprintf('%s / mm','X'+j-1));end
nexttile;plan_angle=zeros(height(P),1);feedback_angle=zeros(N,1);can_angle=zeros(N,1);
R0=Tplan(1:3,1:3,1);
for k=1:height(P),plan_angle(k)=rotation_angle(R0'*Tplan(1:3,1:3,k));end
for k=1:N
    feedback_angle(k)=rotation_angle(R0'*Tmat(1:3,1:3,k));
    can_angle(k)=rotation_angle(R0'*d.T_can(1:3,1:3,k));
end
plot(P.t_s,plan_angle*180/pi,'b');hold on;plot(t,feedback_angle*180/pi,'Color',[.9 .35 .05]);plot(t,can_angle*180/pi,'k:');
format_axis('相对同一初始姿态的SO(3)角 / °');
sgtitle('07 位姿变化：蓝=规划FK，橙=反馈角MATLAB FK，黑=CAN；不要求直线');savefig_png(f,figdir,'07_cartesian_position');
f=figure_base();tiledlayout(2,1);
nexttile;plot(t,ep*1000);format_axis('位置跟随误差 / mm');
nexttile;plot(t,er*180/pi);format_axis('姿态跟随误差 / °');
sgtitle('08 反馈角FK对最近已发目标FK；非外部实测定位精度');savefig_png(f,figdir,'08_pose_tracking_error');
f=figure_base();tiledlayout(2,1);
nexttile;plot(t,cross.position_error_m);format_axis('MATLAB/C++位置差 / m');yline(1e-9,'r:');
nexttile;plot(t,cross.rotation_error_rad);format_axis('MATLAB/C++姿态差 / rad');yline(1e-8,'r:');
sgtitle('09 同一反馈关节角的独立FK数值一致性');savefig_png(f,figdir,'09_matlab_cpp');
f=figure_base();tiledlayout(3,1);
nexttile;plot(t,dp_can*1000);format_axis('MATLAB-CAN XYZ差 / mm');legend('X','Y','Z');
nexttile;plot(t,ep_can*1000);format_axis('MATLAB/CAN位置差 / mm');
nexttile;plot(t,er_can*180/pi);format_axis('MATLAB/CAN姿态差 / °');
sgtitle('10 模型与CAN位姿一致性：CAN不是外部独立真值');savefig_png(f,figdir,'10_matlab_can');
f=figure_base();tiledlayout(3,1);
nexttile;plot(t,d.span_ms);hold on;yline(2,'r--','报告阈值2ms');format_axis('六帧接收跨度 / ms');
nexttile;plot(t(2:end),diff(t)*1000);format_axis('六帧组间隔 / ms');
nexttile;hold on;for j=1:6,tt=tm(M.joint==j);plot(tt(2:end),diff(tt)*1000);end
format_axis('各电机高速帧间隔 / ms');
sgtitle(sprintf('11 六帧跨度非硬件同步精度；>2ms=%d，丢弃组=%d，宽窗口50ms',decoder.span_over_2ms,decoder.discarded_groups));savefig_png(f,figdir,'11_can_timing');
if height(S)>0
    bad=S.arm_status~=0 | S.err_status~=0 | status_flag(S.any_angle_limit) | status_flag(S.any_communication_error);
else,bad=[];end
f=figure_base();f.Position(4)=1300;tiledlayout(5,1);
nexttile;if tracking,plot(C.send_elapsed_s(2:end),diff(C.send_elapsed_s)*1000);end
yline(20,'k:');format_axis('PC发布返回间隔 / ms');
nexttile;if tracking,plot(C.send_elapsed_s,C.lateness_s*1000);end;format_axis('发布前迟到 / ms');
nexttile;if height(F)>1,plot(F.receive_elapsed_s(2:end)-meta.trajectory_start_receive_elapsed_s,diff(F.receive_elapsed_s)*1000);end
format_axis('ROS反馈接收间隔 / ms');
nexttile;plot(P.t_s,P.weighted_sigma_min);format_axis('加权最小奇异值（诊断）');
nexttile;
if height(S)>0,stairs(S.receive_elapsed_s-meta.trajectory_start_receive_elapsed_s,double(bad));end
ylim([-.1 1.1]);yticks([0 1]);format_axis('ROS故障/限位/通信异常标记');
sgtitle('12 时序与奇异性；PC发布时刻不是CAN线上发送时刻');savefig_png(f,figdir,'12_command_timing_sigma');
% 单独保留状态异常表，不把不存在的报警画成正常曲线。
if height(S)>0
    writetable(S(bad,:),fullfile(out,'abnormal_ros_status.csv'));
end
r=struct('execution_dir',execution_dir,'meta',meta,'decoder',decoder,'alignment',alignment,'origin_epoch_s',origin, ...
    't',t,'q_feedback',d.q_rad,'q_sent',qsent,'joint_error',eq,'position_error_m',ep,'rotation_error_rad',er, ...
    'matlab_can_position_m',ep_can,'matlab_can_rotation_rad',er_can,'span_ms',d.span_ms, ...
    'joint_stats',joint_stats,'pose_stats',pose_stats,'phase_stats',phase_stats,'timing_stats',timing_stats,'bad_status_rows',nnz(bad), ...
    'wire_commands_available',height(K)>0,'execution_success',meta.success,'duration_s',duration);
save(fullfile(out,'analysis_results.mat'),'r','P','M','C','S','v_num','a_num','v_interval');
disp(joint_stats);disp(pose_stats);
fprintf('执行success=%d，状态异常%d条，CAN线上目标记录可用=%d。\n',meta.success,nnz(bad),height(K)>0);
fprintf('已保存12张图和统计；模型位姿不是实测精度：%s\n',out);
end

function f=figure_base()
f=figure('Visible','off','Color','w','Position',[60 60 1500 1050]);
set(f,'DefaultAxesFontName','Noto Sans CJK SC','DefaultTextFontName','Noto Sans CJK SC');
end
function format_axis(label)
grid on;xlabel('相对轨迹起点时间 / s');ylabel(label);
end
function savefig_png(f,folder,name)
exportgraphics(f,fullfile(folder,[name,'.png']),'Resolution',130);close(f);
end
function angle=rotation_angle(R)
w=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
angle=atan2(norm(w),max(-1,min(1,(trace(R)-1)/2)));
end
function flags=status_flag(values)
if isnumeric(values) || islogical(values),flags=values~=0;return;end
text=lower(string(values));
assert(all(ismember(text,["true","false","0","1"])),'状态布尔字段有未知值');
flags=text=="true" | text=="1";
end
function row=interval_stats(name,time)
dt=diff(time)*1000;
if isempty(dt),middle=NaN;peak=NaN;else,middle=median(dt);peak=max(dt);end
row=table(string(name),numel(time),middle,peak,'VariableNames',{'stream','samples','median_interval_ms','max_interval_ms'});
end
