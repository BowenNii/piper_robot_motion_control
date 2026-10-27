function results=analyze_today()
%ANALYZE_TODAY 今日十段有效实验+官方对照的一键离线分析入口。
% 先读这里的有效轮次表，再读analyze_project1_complete和compare_five_profiles。
% 不连接机器人。原始CSV/CAN不修改；已有单轮分析直接加载，不覆盖。
% 首次运行前需用project1_feedback_export解码每段raw_can.log。
here=fileparts(mfilename('fullpath'));
repo=fileparts(fileparts(fileparts(fileparts(here))));
addpath(fullfile(repo,'MATLAB'));setup_piper_paths;
root=fullfile(repo,'experiments','project1','2026-10-25_five_profiles');
selected={ ...
    'scheme1_out02','execution_e9ciw07w';'scheme1_return02','execution_nct6klzu'; ...
    'scheme2_out03','execution_x39j4vgp';'scheme2_return04','execution_ih9ohuf6'; ...
    'scheme3_out02','execution_upj6z36c';'scheme3_return03','execution_r5f94lp4'; ...
    'scheme4_out02','execution_h91zbohu';'scheme4_return02','execution_yt0krvy5'; ...
    'scheme5_out02','execution_777zkwpd';'scheme5_return02','execution_pgf7c1lm'};
results=cell(10,1);
for k=1:10
    execution=fullfile(root,'runs',selected{k,1},selected{k,2});
    %拼接出每组实验的原始采集数据目录：实验根目录 / runs / 轮次名称 / 执行实例ID
    meta=jsondecode(fileread(fullfile(execution,'summary.json')));
    assert(meta.success,'选中记录未成功：%s',execution);
    fprintf('\n分析 %d/10：%s\n',k,selected{k,1});
    saved=fullfile(execution,'analysis_complete','analysis_results.mat');
    if isfile(saved),loaded=load(saved,'r');results{k}=loaded.r;
    else,results{k}=analyze_project1_complete(execution);end
end
compare_five_profiles(root,results);
fprintf('\n完成。先查看comparison_analysis/README.md和figures。\n');
end
