function result=analyze_project1_closeout(root,cfg)
%ANALYZE_PROJECT1_CLOSEOUT Project1收尾：质量→路径→关节→响应→时域误差。
% 仅离线。外部精度/TCP/硬实时未验证。A/B只有终点，不虚构内部参考q(t)。
% cfg可填joint_tolerance_deg/position_tolerance_mm/orientation_tolerance_deg。
% 默认空值只报告性能，不把事后选择的数字写成预先规定的验收标准。
if nargin<2,cfg=struct;end
for key={'joint_tolerance_deg','position_tolerance_mm','orientation_tolerance_deg'}
    if ~isfield(cfg,key{1}),cfg.(key{1})=[];end
    if ~isempty(cfg.(key{1})),validateattributes(cfg.(key{1}),{'numeric'},{'scalar','finite','positive'});end
end
base=fullfile(root,'project1_closeout');out=fullfile(base,'matlab');
assert(~isfolder(out),'已有输出不覆盖，请先归档生成目录');
D=jsondecode(fileread(fullfile(base,'inventory.json')));records=D.records;
% 清单里的旧绝对路径保留作历史记录，读取时按当前仓库重定位，不改写清单。
for k=1:numel(records)
    [~,execution_name]=fileparts(records(k).execution_dir);
    [~,decoded_name]=fileparts(records(k).decoded_dir);
    records(k).execution_dir=fullfile(root,records(k).group,records(k).run_name,execution_name);
    records(k).decoded_dir=fullfile(base,'decoded',decoded_name);
end
model=make_piper_model;N=numel(records);cache=cell(N,1);response=[];
mkdir(out);mkdir(fullfile(out,'figures'));mkdir(fullfile(out,'traces'));
quality=readtable(fullfile(base,'decode_comparison.csv'));
for k=1:N
    fprintf('收尾分析 %d/%d\n',k,N);r=records(k);man=r.manifest;meta=r.execution_summary;
    f=load_feedback_csv(fullfile(r.decoded_dir,'feedback_fk.csv'),struct('require_transforms',true,'strict_time',true));
    cmd=readtable(fullfile(r.execution_dir,'commands.csv'));ros=readtable(fullfile(r.execution_dir,'feedback.csv'));
    start=meta.trajectory_start_epoch_s;t=f.timestamp_s-start;ct=cmd.publish_return_epoch_s-start;
    qs=man.q_start_rad(:)';qg=man.q_goal_rad(:)';qc=cmd{:,cellstr(compose('q%d_rad',1:6))};
    active=find(abs(qg-qs)>1e-6);duration=man.duration_s;
    ix=unique(round(linspace(1,numel(t),101)));
    cross=validate_fk_samples(model.S_list,model.M,f.q_rad(ix,:),f.T_cpp(:,:,ix), ...
        struct('position_tolerance_m',1e-9,'rotation_tolerance_rad',1e-7));assert(cross.ok);
    tail=t>=duration+2&t<=duration+3;assert(nnz(tail)>20);
    qend=mean(f.q_rad(tail,:),1);Tgoal=forward_poe_space(model.S_list,qg',model.M);
    Tend=forward_poe_space(model.S_list,qend',model.M);[endp,endr]=pose_error(Tend,Tgoal);
    for j=active
        mask=t>=ct(1);tt=t(mask)-ct(1);q=f.q_rad(mask,j);
        progress=(q-qs(j))/(qg(j)-qs(j));
        onset=sustained_first(tt,progress>=.01,.02);
        t99=first_time(tt,progress>=.99);
        inband=abs(q-qg(j))<=.01*abs(qg(j)-qs(j));
        lastbad=find(~inband,1,'last');
        if isempty(lastbad),settled=tt(1);
        elseif lastbad<numel(tt)&&tt(end)-tt(lastbad+1)>=.5,settled=tt(lastbad+1);
        else,settled=NaN;end
        response=[response;struct('record',k,'group',string(r.group),'mode',string(man.mode), ...
            'direction',string(man.direction),'joint',j,'onset_1pct_s',onset, ...
            'arrival_99pct_s',t99,'settled_1pct_band_s',settled)]; %#ok<AGROW>
    end
    late=(cmd.publish_start_elapsed_s-cmd.target_time_s)*1000;
    span=f.span_ms;
    qrow=struct('record',k,'group',string(r.group),'run_name',string(r.run_name), ...
        'span_max_ms',max(span),'span_p95_ms',prctile_local(span,.95), ...
        'span_over2_fraction',mean(span>2),'pc_lateness_max_ms',max(late), ...
        'ros_receive_gap_max_ms',max(diff(ros.receive_elapsed_s))*1000, ...
        'endpoint_joint_max_deg',rad2deg(max(abs(qend-qg))), ...
        'endpoint_model_position_mm',endp*1000,'endpoint_model_rotation_deg',rad2deg(endr), ...
        'fk_crosscheck_max_m',max(cross.position_error_m));
    z=struct('t',t,'q',f.q_rad,'p',f.p_cpp*1000,'T',f.T_cpp,'ct',ct,'qc',qc, ...
        'active',active,'row',qrow,'duration',duration,'qs',qs,'qg',qg,'tr',[], ...
        'path_forward',[],'path_reverse',[],'refP',[],'lag_candidate',NaN);
    if strcmp(man.mode,'stream')
        z.refT=fk_sequence(model,qc);z.refP=squeeze(z.refT(1:3,4,:))'*1000;
        mask=t>=ct(1)&t<=duration;z.tr=t(mask);z.qtrace=f.q_rad(mask,:);z.Ttrace=f.T_cpp(:,:,mask);
        z.ptrace=f.p_cpp(mask,:)*1000;
        z.path_forward=project1_path_distance(z.ptrace,z.refP);
        z.path_reverse=project1_path_distance(z.refP,z.ptrace);
        % 去程20%-80%时间窗拟合一个共同延迟，仅沿时间轴平移。不能逐轴各自挑最优值。
        fit=z.tr>=.2*duration&z.tr<=.8*duration;
        candidates=(0:.001:.3)';score=zeros(size(candidates));
        for h=1:numel(candidates)
            ref=interp1(ct,qc,z.tr(fit)-candidates(h),'linear');
            delta=z.qtrace(fit,active)-ref(:,active);score(h)=mean(delta.^2,'all');
        end
        [~,best]=min(score);z.lag_candidate=candidates(best);z.lag_boundary=best==numel(candidates);
    end
    cache{k}=z;
end
% 延迟只用同组去程拟合，再原样应用到返程作独立检验；不替代原始误差。
for k=1:N
    r=records(k);z=cache{k};row=z.row;
    if ~strcmp(r.manifest.mode,'stream'),continue;end
    source=find(arrayfun(@(x)strcmp(x.group,r.group)&&strcmp(x.manifest.mode,'stream')&& ...
        strcmp(x.manifest.direction,'outbound'),records));assert(isscalar(source));
    lag=cache{source}.lag_candidate;z.lag_used=lag;z.lag_boundary=cache{source}.lag_boundary;
    [z.jerror,z.pe,z.re]=errors(z,0);
    valid=z.tr>=z.ct(1)+lag;z.aligned_t=z.tr(valid);
    [je,pe,re]=errors(z,lag);z.jaligned=je(valid,:);z.paligned=pe(valid);z.raligned=re(valid);
    rawj=max(abs(z.jerror),[],2);alignedj=max(abs(z.jaligned),[],2);
    jm=project1_trace_metrics(z.tr,rawj,cfg.joint_tolerance_deg);
    pm=project1_trace_metrics(z.tr,z.pe,cfg.position_tolerance_mm);
    rm=project1_trace_metrics(z.tr,z.re,cfg.orientation_tolerance_deg);
    am=project1_trace_metrics(z.aligned_t,alignedj,[]);
    ap=project1_trace_metrics(z.aligned_t,z.paligned,[]);
    ar=project1_trace_metrics(z.aligned_t,z.raligned,[]);
    row.stream_joint_peak_deg=jm.peak;row.stream_joint_rms_deg=jm.rms;
    row.stream_position_peak_mm=pm.peak;row.stream_position_rms_mm=pm.rms;
    row.stream_rotation_peak_deg=rm.peak;row.stream_rotation_rms_deg=rm.rms;
    row.path_forward_peak_mm=max(z.path_forward);row.path_forward_rms_mm=sqrt(mean(z.path_forward.^2));
    row.path_reverse_peak_mm=max(z.path_reverse);
    row.lag_from_outbound_ms=lag*1000;row.lag_search_boundary=z.lag_boundary;
    row.aligned_joint_rms_deg=am.rms;row.aligned_position_rms_mm=ap.rms;row.aligned_rotation_rms_deg=ar.rms;
    row.joint_over_fraction=jm.over_fraction;row.joint_longest_over_s=jm.longest_over_s;
    row.position_over_fraction=pm.over_fraction;row.position_longest_over_s=pm.longest_over_s;
    row.rotation_over_fraction=rm.over_fraction;row.rotation_longest_over_s=rm.longest_over_s;
    % 去程仅用拟合窗外数据验残差，返程全程作为未参与拟合的数据。
    holdout=valid;
    if strcmp(r.manifest.direction,'outbound'),holdout=holdout&~(z.tr>=.2*z.duration&z.tr<=.8*z.duration);end
    row.raw_holdout_joint_rms_deg=sqrt(mean(max(abs(z.jerror(holdout,:)),[],2).^2));
    row.aligned_holdout_joint_rms_deg=sqrt(mean(max(abs(je(holdout,:)),[],2).^2));
    z.row=row;cache{k}=z;
    trace=array2table([z.tr,z.jerror,z.pe,z.re,z.path_forward], ...
        'VariableNames',[{'time_s'},cellstr(compose('joint%d_error_deg',1:6)), ...
        {'model_position_error_mm','model_rotation_error_deg','path_distance_mm'}]);
    writetable(trace,fullfile(out,'traces',sprintf('%02d_stream_errors.csv',k)));
end
% 不适用字段用NaN，而不是把A/B不存在的过程参考误差填成零。
streamfields=fieldnames(cache{find(cellfun(@(z)~isempty(z.tr),cache),1)}.row);
for k=1:N
    row=cache{k}.row;
    for j=1:numel(streamfields),if ~isfield(row,streamfields{j}),row.(streamfields{j})=NaN;end;end
    row=orderfields(row,streamfields);cache{k}.row=row;
end
result=struct2table(vertcat_rows(cache));response=struct2table(response);
writetable(result,fullfile(out,'run_metrics.csv'));writetable(response,fullfile(out,'response_metrics.csv'));
save(fullfile(out,'analysis_results.mat'),'result','response','cfg');
draw_figures(records,cache,D.selected_record_numbers,quality,result,response,out);
fprintf('完成：%s\n',out);
end

function rows=vertcat_rows(cache)
rows=cache{1}.row;for k=2:numel(cache),rows(end+1)=cache{k}.row;end
end
function [je,pe,re]=errors(z,lag)
query=z.tr-lag;ind=interp1(z.ct,(1:numel(z.ct))',max(z.ct(1),query),'previous');
je=rad2deg(z.qtrace-z.qc(ind,:));pe=zeros(size(query));re=pe;
for h=1:numel(query),[p,r]=pose_error(z.Ttrace(:,:,h),z.refT(:,:,ind(h)));pe(h)=p*1000;re(h)=rad2deg(r);end
end
function poses=fk_sequence(model,q)
poses=zeros(4,4,size(q,1));for h=1:size(q,1),poses(:,:,h)=forward_poe_space(model.S_list,q(h,:)',model.M);end
end
function value=first_time(t,mask)
k=find(mask,1);if isempty(k),value=NaN;else,value=t(k);end
end
function value=sustained_first(t,mask,hold_s)
value=NaN;start=[];
for k=1:numel(t)
    if mask(k)
        if isempty(start),start=k;end
        if t(k)-t(start)>=hold_s,value=t(start);return;end
    else,start=[];end
end
end
function value=prctile_local(x,fraction)
x=sort(x);value=x(max(1,ceil(numel(x)*fraction)));
end

function draw_figures(records,cache,selected,quality,result,response,out)
dirs={'outbound','return'};modes={'once','repeat','stream'};names={'A 单次终点','B 重复固定终点','C 连续轨迹'};
f=fig;tl=tiledlayout(f,4,1,'TileSpacing','compact');
ax=nexttile(tl);bar(ax,[quality.old_max_span_ms,quality.new_max_span_ms]);ylabel(ax,'最大组跨度 / ms');key(f,{'旧组装','按周期组装'},{[.1 .4 .8],[.9 .3 .1]});
ax=nexttile(tl);bar(ax,[quality.old_over_2ms./quality.old_samples,quality.new_over_2ms_fraction]*100);ylabel(ax,'超2ms组比例 / %');
ax=nexttile(tl);bar(ax,result.pc_lateness_max_ms);ylabel(ax,'最大发布迟到 / ms');yline(ax,20,'r--');
ax=nexttile(tl);bar(ax,result.ros_receive_gap_max_ms);ylabel(ax,'最大ROS接收间隔 / ms');xlabel(ax,'记录编号');
title(tl,'01 数据质量：全部34段，保留修正前后对照');finish(f,out,'01_data_quality');
for d=1:2
    kc=pick(records,selected,'multi_abc','stream',dirs{d});z=cache{kc};
    f=fig;tl=tiledlayout(f,2,2,'TileSpacing','compact');
    ax=nexttile(tl);plot3(ax,z.refP(:,1),z.refP(:,2),z.refP(:,3),'k--','LineWidth',1.5);hold(ax,'on');
    plot3(ax,z.ptrace(:,1),z.ptrace(:,2),z.ptrace(:,3),'b');xlabel(ax,'X / mm');ylabel(ax,'Y / mm');zlabel(ax,'Z / mm');axis(ax,'equal');grid(ax,'on');view(ax,3);ax.Tag='spatial';
    pairs=[1 2;1 3;2 3];labels={'X','Y','Z'};
    for j=1:3
        ax=nexttile(tl);p=pairs(j,:);plot(ax,z.refP(:,p(1)),z.refP(:,p(2)),'k--','LineWidth',1.5);hold(ax,'on');
        plot(ax,z.ptrace(:,p(1)),z.ptrace(:,p(2)),'b');axis(ax,'equal');grid(ax,'on');xlabel(ax,[labels{p(1)} ' / mm']);ylabel(ax,[labels{p(2)} ' / mm']);ax.Tag='spatial';
    end
    key(f,{'C参考路径','反馈角回算路径'},{[0 0 0],[0 0 .8]});
    title(tl,sprintf('%02d 六轴C %s：link6空间路径，不是外部实测',d+1,dirs{d}),'Interpreter','none');
    finish(f,out,sprintf('%02d_C_path_%s',d+1,dirs{d}));
    f=fig;tl=tiledlayout(f,6,3,'TileSpacing','compact','Padding','compact');
    for j=1:6
        for m=1:3
            k=pick(records,selected,'multi_abc',modes{m},dirs{d});v=cache{k};ax=nexttile(tl);
            plot(ax,v.t,rad2deg(v.q(:,j)),'b');hold(ax,'on');
            if m==3,stairs(ax,v.ct,rad2deg(v.qc(:,j)),'k--');else,yline(ax,rad2deg(v.qg(j)),'k--');end
            ylabel(ax,sprintf('J%d / deg',j));grid(ax,'on');xlim(ax,[0 13.76]);
            if j==1,title(ax,names{m});end
            if j==6,xlabel(ax,'时间 / s');end
        end
    end
    key(f,{'反馈关节角','A/B终点或C发布目标'},{[0 0 .8],[0 0 0]});title(tl,['六轴关节曲线 ' dirs{d}]);
    finish(f,out,sprintf('%02d_joint_angles_%s',d+3,dirs{d}));
end
% A/B路径没有规划中间参考，只画实际回算路径，C作为另一运动方案对照。
f=fig;tl=tiledlayout(f,1,2);
colors={[0 .4 .8],[.8 .3 .1],[.1 .6 .2]};
for d=1:2
    ax=nexttile(tl);hold(ax,'on');
    for m=1:3
        k=pick(records,selected,'multi_abc',modes{m},dirs{d});v=cache{k};mask=v.t>=0&v.t<=v.duration;
        plot3(ax,v.p(mask,1),v.p(mask,2),v.p(mask,3),'Color',colors{m});
    end
    axis(ax,'equal');grid(ax,'on');view(ax,3);xlabel(ax,'X / mm');ylabel(ax,'Y / mm');zlabel(ax,'Z / mm');title(ax,dirs{d});ax.Tag='spatial';
end
key(f,{'A回算路径','B回算路径','C回算路径'},colors);title(tl,'06 A/B仅约束终点，不要求与C走相同空间路径');finish(f,out,'06_ABC_executed_paths');
f=fig;tl=tiledlayout(f,3,1,'TileSpacing','compact');
sel=ismember(response.record,selected)&response.group=="multi_abc";R=response(sel,:);
for j=1:3
    ax=nexttile(tl);hold(ax,'on');
    fields={'onset_1pct_s','arrival_99pct_s','settled_1pct_band_s'};
    labels={'1%行程起动响应 / s','99%行程到达 / s','1%行程带稳定 / s'};
    for m=1:3,mask=R.mode==string(modes{m});scatter(ax,R.joint(mask)+.1*(m-2),R.(fields{j})(mask),45,colors{m},'filled');end
    xlabel(ax,'关节编号（去返程均显示）');ylabel(ax,labels{j});grid(ax,'on');xticks(ax,1:6);
end
key(f,names,colors);title(tl,'07 六轴响应：1%行程为描述性定义，不是任务验收容差');finish(f,out,'07_response_times');
f=fig;tl=tiledlayout(f,6,2,'TileSpacing','compact','Padding','compact');
for j=1:6
    for d=1:2
        k=pick(records,selected,'multi_abc','stream',dirs{d});z=cache{k};ax=nexttile(tl);
        plot(ax,z.tr,z.jerror(:,j),'b');hold(ax,'on');plot(ax,z.aligned_t,z.jaligned(:,j),'Color',[.8 .3 .1]);
        ylabel(ax,sprintf('J%d误差 / deg',j));grid(ax,'on');title(ax,dirs{d});
    end
end
key(f,{'原始时间轴','固定延迟对齐（非替代验收）'},{[0 0 .8],[.8 .3 .1]});
title(tl,'08 六轴C关节跟踪误差；去程拟合延迟，原样用于返程');finish(f,out,'08_C_joint_tracking');
f=fig;tl=tiledlayout(f,3,2,'TileSpacing','compact');
for d=1:2
    k=pick(records,selected,'multi_abc','stream',dirs{d});z=cache{k};
    for row=1:3
        ax=nexttile(tl,(row-1)*2+d);
        if row==1,y=z.pe;ya=z.paligned;label='模型位置误差 / mm';
        elseif row==2,y=z.re;ya=z.raligned;label='模型姿态误差 / deg';
        else,y=z.path_forward;ya=[];label='到参考折线距离 / mm';end
        plot(ax,z.tr,y,'b');hold(ax,'on');if ~isempty(ya),plot(ax,z.aligned_t,ya,'Color',[.8 .3 .1]);end
        ylabel(ax,label);xlabel(ax,'时间 / s');grid(ax,'on');title(ax,dirs{d});
    end
end
key(f,{'原始误差/空间距离','延迟对齐误差'},{[0 0 .8],[.8 .3 .1]});
title(tl,'09 时间跟随误差与空间路径偏差分开评价');finish(f,out,'09_C_pose_and_path_errors');
f=fig;tl=tiledlayout(f,2,1);
stream=ismember(result.record,selected)&~isnan(result.lag_from_outbound_ms);V=result(stream,:);
ax=nexttile(tl);bar(ax,V.record,V.lag_from_outbound_ms);ylabel(ax,'去程拟合延迟 / ms');xlabel(ax,'C记录编号（返程复用去程值）');grid(ax,'on');
ax=nexttile(tl);bar(ax,V.record,[V.raw_holdout_joint_rms_deg,V.aligned_holdout_joint_rms_deg]);ylabel(ax,'未参与拟合数据关节误差RMS / deg');xlabel(ax,'记录编号');grid(ax,'on');
key(f,{'原始误差','固定延迟对齐'},{[.1 .4 .8],[.9 .3 .1]});title(tl,'10 延迟是事后估计，不代表实时补偿或固件响应保证');finish(f,out,'10_delay_validation');
end
function k=pick(records,selected,group,mode,direction)
k=find(arrayfun(@(r)strcmp(r.group,group)&&strcmp(r.manifest.mode,mode)&& ...
    strcmp(r.manifest.direction,direction)&&ismember(r.record,selected),records));assert(isscalar(k));
end
function f=fig
f=figure('Visible','off','Color','w','Position',[20 20 1600 1100]);
set(f,'DefaultAxesFontName','Noto Sans CJK SC','DefaultTextFontName','Noto Sans CJK SC');
end
function key(f,labels,colors)
tl=findall(f,'Type','tiledlayout');tl.OuterPosition=[0 .055 1 .945];
n=numel(labels);width=.94/n;
for k=1:n
    annotation(f,'textbox',[.03+(k-1)*width .005 width .04],'String',['━━ ' labels{k}], ...
        'Color',colors{k},'EdgeColor','none','FontSize',11,'Interpreter','none');
end
end
function finish(f,out,name)
% 布局重排后固定完整数据范围，防止axis equal在狭窄tile里裁掉路径端点。
drawnow;
axeslist=findall(f,'Type','axes','Tag','spatial');
for a=axeslist'
    lines=findall(a,'Type','line');x=[];y=[];z=[];
    for h=lines',x=[x,h.XData(:)'];y=[y,h.YData(:)'];z=[z,h.ZData(:)'];end %#ok<AGROW>
    xlim(a,padded_limits(x));ylim(a,padded_limits(y));
    if ~isempty(z),zlim(a,padded_limits(z));end
end
objects=findall(f,'-property','DataTipTemplate');
for k=1:numel(objects)
    entries=objects(k).DataTipTemplate.DataTipRows;
    for j=1:numel(entries),entries(j).Format='%.9g';end
end
savefig(f,fullfile(out,'figures',[name '.fig']),'compact');
exportgraphics(f,fullfile(out,'figures',[name '.png']),'Resolution',130);close(f);
end
function limits=padded_limits(x)
lo=min(x);hi=max(x);pad=max(.01,.08*(hi-lo));limits=[lo-pad hi+pad];
end
