# 通用绘图模块

主脚本负责数据来源、算法计算、阶段识别和保存计划；本模块仅负责展示。
所有输入长度 **m**、角度 **rad**、时间 **s**，默认显示 **mm、deg、s**。
数据每行一个采样点。不删数据、不插值、不计算FK、不隐式保存或关闭图窗。

## 函数清单

| 函数 | 输入 | 输出 |
|---|---|---|
| `plot_joint_angles` | t、N×n关节角rad、cfg | 图窗、n个坐标轴 |
| `plot_position_comparison` | t、位置序列cell（各N×3 m）、来源标签、cfg | 图窗、XYZ坐标轴 |
| `plot_pose_errors` | t、位置偏差m、姿态偏差rad、cfg | 图窗、两个坐标轴 |
| `plot_frame_span` | t、跨度s、窗口阈值s、cfg | 图窗、坐标轴 |
| `mark_motion_phases` | 坐标轴、开始/结束时间s | 两条边界线句柄 |
| `apply_plot_style` | 图窗、cfg | 无；修改本图格式 |
| `save_experiment_figure` | 图窗、目录、文件名、保存cfg | 完整输出路径 |

`private/` 是内部默认配置和形状检查，主脚本不需要调用或重复阅读。

## 主脚本调用例子

```matlab
setup_piper_paths;
% t_s、q_rad等由本次实验提供，不在绘图函数内读取文件。
cfg_plot = struct('title','run04 六轴关节角', ...
                 'visible','on','angle_unit','deg', ...
                 'bounds_s',[8.59,11.97]);
[fig,axes_list] = plot_joint_angles(t_s,q_rad,cfg_plot);

cfg_save = struct('resolution',150,'overwrite',false);
save_experiment_figure(fig,figures_dir,'matlab_六关节角度.png',cfg_save);
% 是否关闭由主脚本决定：close(fig);

cfg_plot.title = '末端XYZ：CAN、C++与MATLAB对比';
fig = plot_position_comparison(t_s,{p_can_m,p_cpp_m,p_matlab_m}, ...
    {'CAN反馈','C++ POE','MATLAB POE'},cfg_plot);

cfg_plot.title = 'MATLAB POE与CAN位姿偏差';
fig = plot_pose_errors(t_s,position_error_m,rotation_error_rad,cfg_plot);
fig = plot_frame_span(t_s,frame_span_s,0.002,cfg_plot);
```

## 参数配置

绘图cfg均可省略；支持：

- `visible`：`'on'`（默认）或`'off'`，控制是否显示图窗。
- `title`：总标题，默认空；明确说明本次实验/对比对象。
- `font_name`、`font_size`、`line_width`：默认Noto Sans CJK SC、11、1.2。
- `position`：图窗位置/大小，默认`[60,60,1120,780]`。
- `angle_unit`：`'deg'`或`'rad'`，只改变显示，不改变输入rad约定。
- `length_unit`：`'mm'`或`'m'`，只改变显示，不改变输入m约定。
- `bounds_s`：可选运动边界，时间基准必须与t一致；空或两个NaN不标线。
- `legend_location`：位置对比图默认`'northeast'`，使用固定位置便于批量导出。
- `target_rad`：仅关节图使用，每关节一个终态目标，不是目标轨迹。

保存配置独立于绘图配置：`resolution=150`，`overwrite=false`。
无扩展名默认PNG；支持PNG/JPG/JPEG/PDF/FIG。已有同名文件默认拒绝覆盖，
重新出图可换名或显式设置`overwrite=true`。目录不存在会创建。

自定义局部放大可在主脚本使用 `xlim(axes_list(1),[开始,结束])`。
专项多位形汇总图暂保留在实验脚本，不为缩短主代码而过度封装。
三个现有实验脚本已接入本模块：实验1使用通用时序图，实验2保留专项布局并复用格式工具，
实验3复用关节角/XYZ图、边界标注和保存工具。归档实验图片未因重构而覆盖。

## 测试

```matlab
run('tests/test_plotting.m'); % 测试图片保存到独立临时目录
run_all_tests(false);        % 含绘图模块的全套数值测试
```
