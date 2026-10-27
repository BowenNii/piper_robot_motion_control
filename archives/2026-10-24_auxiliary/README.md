# 2026-10-24辅助文件清理恢复包

auxiliary.tar.gz保存以下可重复生成/已替代的内容，移出活动目录前已核对归档可读：

- experiments/project1/2026-10-24_zero_to_pose/offline_preview：精确零位离线样例，不是真机反馈。
- MATLAB/experiments/project1/2026-10-24_zero_to_pose/matlab_preview.m：旧独立计划预览入口。
- experiments/project1/2026-10-24_offline_csv_action/run01/figures：旧模拟实验派生图片。
- experiments/project1/2026-10-24_offline_csv_action/run01/analysis：旧模拟实验派生统计/MAT。

清理后保留的正式入口是zero_to_pose/matlab_plot.m。该入口会复核实际计划，
并分析真实执行记录。原预览脚本需要恢复后方可使用，不能作为日常入口。
模拟实验的原始CSV/summary保留，原matlab_plot.m可重新生成其图表，测试不依赖被清理的图表。

恢复：先查看tar -tzf的清单。确认目标文件不存在或已备份后，在仓库根目录执行
tar -xzf archives/2026-10-24_auxiliary/auxiliary.tar.gz --keep-old-files。
不要使用无保护的解包覆盖已有修改。被移出的辅助文件也放入系统回收站，可从原位置恢复。
Python的三个.pyc缓存可自动再生成，无需恢复。

后续三个前置实验及对应MATLAB目录已移至archives/2026-10-24_preliminary。
本恢复包保留清理当时的历史目录结构；若不希望主目录重新出现旧日期目录，
请解包到独立恢复目录，再按新归档位置放回，不要直接在仓库根目录原位解包。

没有清理真机原始记录、两轮真机分析结果、正式C++/Python/MATLAB代码、测试或1mm静态rosbag。
