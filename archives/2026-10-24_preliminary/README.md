# 2026-10-24前置实验归档

主实验目录与MATLAB实验目录中只保留2026-10-24_zero_to_pose。
本归档不是删除，三个前置实验原始数据完整保留，两个实验级MATLAB目录随之移动。

数据位置：本目录/experiments/project1下的offline_csv_action、real_ik_z1mm、real_ik_z2mm日期目录。
MATLAB位置：本目录/MATLAB/experiments/project1下的offline_csv_action、real_ik_z2mm日期目录。
z1mm没有独立MATLAB目录，只有静态原始rosbag。

已同步修改两个归档绘图脚本与MATLAB回归测试的工作路径，公共分析库仍在仓库MATLAB目录，
因为最后一轮正式实验也要使用它们。执行归档绘图入口会将派生结果保存至归档内，不重建主目录。

JSON清单、原始日志和既有MAT中保存的旧绝对路径是历史记录，不改写；
它们不意味着数据仍位于旧目录。分析只使用归档后的实际路径。

需要恢复时，先确认主目录中不存在同名文件夹，再将对应数据与MATLAB目录移回。
同时恢复两个归档脚本的路径推导及MATLAB/tests中的归档路径；不要直接覆盖已有实验。

日常入口始终是仓库MATLAB/experiments/project1/2026-10-24_zero_to_pose/matlab_plot.m。
