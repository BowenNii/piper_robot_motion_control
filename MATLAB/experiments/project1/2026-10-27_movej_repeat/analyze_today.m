function results=analyze_today(cfg)
%ANALYZE_TODAY Project1收尾离线入口，不连接ROS/CAN，不修改原始记录。
% 先运行tools/ros2/prepare_project1_closeout.py，再运行本入口。
% 输出project1_closeout/matlab：10张PNG+可交互FIG，统计表与逐时刻误差。
% 已有输出不覆盖。重跑前归档生成的matlab目录，不动原始日志。
% cfg可选：joint_tolerance_deg、position_tolerance_mm、orientation_tolerance_deg。
% 默认读取config/project1_analysis.json（null只报告性能）。显式cfg覆盖默认配置。
here=fileparts(mfilename('fullpath'));
repo=fileparts(fileparts(fileparts(fileparts(here))));
addpath(fullfile(repo,'MATLAB'));setup_piper_paths;
root=fullfile(repo,'experiments','project1','2026-10-27_movej_repeat');
if nargin<1,cfg=load_project1_analysis_config(fullfile(repo,'config','project1_analysis.json'));end
results=analyze_project1_closeout(root,cfg);
end
