%% 汇总已完成分析的真实轮次；失败轮也保留，不凭样本少就宣称更平稳。
clear;clc;
here=fileparts(mfilename('fullpath'));repo=fileparts(fileparts(fileparts(fileparts(here))));
root=fullfile(repo,'experiments','project1','2026-10-25_five_profiles');
files=dir(fullfile(root,'runs','**','analysis_complete','analysis_results.mat'));
assert(~isempty(files),'尚无完整分析记录');
rows=table;
for k=1:numel(files)
    loaded=load(fullfile(files(k).folder,files(k).name),'r');r=loaded.r;
    row=table(r.meta.scheme,string(r.meta.direction),r.execution_success,r.duration_s, ...
        max(r.joint_stats.peak_abs_error_rad),max(r.joint_stats.tail_range_rad), ...
        r.pose_stats.endpoint_model_position_mm,r.pose_stats.endpoint_model_rotation_deg, ...
        max(r.span_ms),r.decoder.span_over_2ms,r.decoder.discarded_groups,r.bad_status_rows, ...
        string(r.execution_dir),'VariableNames',{'scheme','direction','success','duration_s', ...
        'joint_peak_error_rad','tail_range_rad','endpoint_model_mm','endpoint_model_deg', ...
        'max_six_frame_span_ms','span_over_2ms','discarded_groups','bad_status_rows','execution_dir'});
    rows=[rows;row]; %#ok<AGROW>
end
rows=sortrows(rows,{'scheme','direction'});disp(rows);
destination=fullfile(root,['comparison_',char(datetime('now','Format','yyyyMMdd_HHmmss')),'.csv']);
assert(~isfile(destination),'输出已存在');writetable(rows,destination);
fprintf('汇总不是安全合格判定。视频现象需人工补充；失败/中断轮不能直接参与完整性能排名。\n%s\n',destination);
