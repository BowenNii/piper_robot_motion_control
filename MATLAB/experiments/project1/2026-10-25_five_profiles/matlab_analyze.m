%% 真机完整分析入口；与matlab_plot.m（只画计划）区分。
clear;clc;
here=fileparts(mfilename('fullpath'));
repo=fileparts(fileparts(fileparts(fileparts(here))));
addpath(fullfile(repo,'MATLAB'));setup_piper_paths;
% 填入一轮执行目录；留空时使用本实验runs下最新一轮（失败轮也如实显示）。
execution_dir='';
if isempty(execution_dir)
    files=dir(fullfile(repo,'experiments','project1','2026-10-25_five_profiles','runs','**','summary.json'));
    assert(~isempty(files),'尚无新实验真实记录；请先完成prepare/execute/analyze');
    [~,idx]=max([files.datenum]);execution_dir=files(idx).folder;
end
r=analyze_project1_complete(execution_dir);
