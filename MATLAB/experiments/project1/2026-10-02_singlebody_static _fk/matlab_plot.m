%% 实验1：单个位形静态FK一致性（只统计与绘图，不独立重建POE）
clear; clc; close all;
script_dir=fileparts(mfilename('fullpath'));
repo_dir=fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(fullfile(repo_dir,'MATLAB')); setup_piper_paths;
root_dir=fullfile(repo_dir,'experiments','project1','2026-10-02_singlebody_static _fk');

%% 1. 读取与统计；原始CSV始终只读
data=load_feedback_csv(fullfile(root_dir,'data.csv'));
% 此归档部分坐标保留小数位较少，给算术复核留1e-10m余量，不改数据。
summary=summarize_static_pose(data,'static01',struct('position_recheck_tolerance_m',1e-10));
% 时间/跨度质量报告保留在data.timing；不隐式删除异常样本。
if ~data.timing.ok, warning('时间或六帧跨度存在异常，请查看data.timing'); end

%% 2. 绘图；本实验仍只显示，由使用者自行保存
cfg_plot=struct('title','六关节角度反馈');
plot_joint_angles(data.t,data.q_rad,cfg_plot);
cfg_plot.title='末端位置：CAN反馈与C++ POE';
plot_position_comparison(data.t,{data.p_feedback,data.p_cpp}, ...
    {'CAN反馈','C++ POE'},cfg_plot);
cfg_plot.title='C++ POE与CAN模型一致性偏差';
plot_pose_errors(data.t,data.position_mm/1000,data.rotation_deg*pi/180,cfg_plot);

%% 3. 输出统计；模型一致性不代表外部测量的实际定位精度
fprintf('有效数据组数：%d\n',summary.sample_count);
fprintf('位置偏差：均值 %.6f mm，RMS %.6f mm，最大 %.6f mm\n', ...
    summary.position_mean_mm,summary.position_rms_mm,summary.position_max_mm);
fprintf('姿态偏差：均值 %.6f deg，RMS %.6f deg，最大 %.6f deg\n', ...
    summary.rotation_mean_deg,summary.rotation_rms_deg,summary.rotation_max_deg);
