# 实验数据处理模块

本模块连接 CSV、验证算法和统计结果。输入单位统一为 **m、rad、s**。
主脚本仍负责实验路径、阈值、比较来源、独立算法计算、异常处理及专项图布局。
模块不控制机械臂，不修改输入 CSV，不默默筛选数据。

## 函数清单

| 函数 | 输入 | 输出与用途 |
|---|---|---|
| `load_feedback_csv` | CSV路径、可选cfg | 统一数据结构；调用格式/时间检查 |
| `feedback_rpy_to_transforms` | N×3位置m、N×3 RPY rad | CAN位姿4×4×N，RzRyRx约定 |
| `cpp_columns_to_transforms` | N×3位置m、N×9旋转元素 | C++位姿4×4×N，检查SE(3) |
| `summarize_static_pose` | 统一数据结构、位形名称、可选cfg | 单个位形统计结构，含位置偏差列复核 |
| `summarize_dynamic_run` | 已算偏差的数据、终态目标、模型、cfg | 阶段/终态信息、整组与分阶段统计 |
| `save_analysis_results` | 输出目录、统计表结构、MAT变量结构、保存cfg | CSV与MAT路径，默认拒绝覆盖 |

## 1. 数据读取：实验脚本不再重复查列

```matlab
setup_piper_paths;
data = load_feedback_csv(csv_path); % 静态模式
% 常用字段：
% data.table       原始表（原值与列名保留）
% data.t           相对时间N×1，首点为0，单位s
% data.timestamp_s 原绝对时间戳s
% data.q_rad       N×6关节角rad
% data.q_deg       仅为统计/分段准备的deg版本
% data.p_feedback  CAN位置N×3 m
% data.p_cpp       C++ POE位置N×3 m
% data.position_mm / rotation_deg / span_ms：CSV中的偏差及跨度显示单位版本
% data.timing      时间质量报告，不自动删行
```

默认要求关节、XYZ、时间、跨度和偏差列。静态模式允许重复时间但禁止倒退。
动态模式使用 `struct('require_transforms',true,'strict_time',true)`：
要求至少两条样本、严格递增时间，以及 RPY 和 C++ 旋转元素；额外提供 `T_can/T_cpp`。
时间戳关系、负跨度/偏差、NaN/Inf、缺列属于输入错误，直接报错。
六帧超窗口或长间隔属于质量报告，由主脚本决定警告、停止还是显式筛选。

## 2. 位姿组装：不把数据转换冒充独立FK

```matlab
T_can = feedback_rpy_to_transforms(p_can_m,rpy_rad);
% RPY列顺序roll、pitch、yaw；R=Rz(yaw)*Ry(pitch)*Rx(roll)
T_cpp = cpp_columns_to_transforms(p_cpp_m,rotation_rows);
% rotation_rows列顺序r11,r12,r13,r21,r22,r23,r31,r32,r33
```

位置和姿态必须具有相同基坐标系与末端参考点。没有单位或坐标信息时，函数不会猜测。

## 3. 静态统计

```matlab
cfg_static = struct('frame_window_s',0.002, ...
                    'position_recheck_tolerance_m',1e-11);
row = summarize_static_pose(data,'pose01',cfg_static);
```

输出角度均值/范围deg、位置均值mm、偏差均值/RMS/最大值、跨度和样本数。
位置偏差列复核只比较两位置向量之差，不重建POE。
各位形分别统计，不能把多个静态文件直接拼成连续轨迹。
实验1归档CSV部分坐标小数位较少，算术复核容差在主脚本显式取1e-10m；
实验2仍取1e-11m。两者都不改变CSV，不将舍入差当作机械臂误差。

## 4. 动态统计：独立算法比较仍在主脚本

先在主脚本从 `q_rad` 计算 MATLAB POE，并明确与 C++、CAN比较，
将得到的 `position_mm`、`rotation_deg`、`cpp_position_mm`、`cpp_rotation_deg`
放入 data，再调用：

```matlab
[data,row,phase_rows] = summarize_dynamic_run(data,target_rad,model,cfg);
```

cfg需要显式设置：变化检测阈值deg、缓冲s、活动间隔s、终态窗口s、
六帧窗口s、终态角度范围deg、限位比较容差rad、最短静态基线s。
现有动态实验将原来的 `0.001deg` 限位比较余量写入 `limit_tolerance_rad`，
不修改URDF，也不删除三条原有异常样本。

`split_phases` 保留用户原函数：阶段2是第一处至最后一处变化的缓冲包络，
可能覆盖中途停顿；activity_segments只是依据变化间隔计数的启发式结果。
空阶段统计NaN，不能当作零偏差。没有时间对齐的目标轨迹时不计算跟踪误差。
CAN位姿比较是模型一致性，不是外部测量的物理定位精度。

## 5. 保存与复现

```matlab
summary = struct2table(row);
tables = struct('summary',summary); % 字段名对应summary.csv
variables = struct('summary',summary,'cfg',cfg_static);
paths = save_analysis_results(analysis_dir,tables,variables);
% 如明确要重算覆盖：第四个参数struct('overwrite',true)
```

默认MAT文件名 `analysis_results.mat`，可用 `cfg.mat_name` 指定。
MAT保存variables的字段，方便 `load` 直接拿到summary/cfg。
如果字段是cell数组，用 `struct('results',{results})` 避免意外生成结构数组。
保存前检查所有目标是否存在；CSV/MAT不是操作系统原子事务。

## 三个实验的现有行为

- 实验1：读取→静态统计→通用绘图→命令窗口结果；不自动保存图。
- 实验2：十个文件分别统计→保存原有summary.csv/MAT→专项总览与详细图；不自动保存图。
- 实验3：完整数据检查→独立POE交叉验证→动态统计→保存CSV/MAT→绘图保存PNG。
- 重复运行实验2/3时，主脚本明确开启覆盖其生成的统计输出；实验3也明确覆盖同名PNG。
- 日志、输入CSV、照片、报告和C++代码不修改。

建议阅读顺序：实验主脚本→本说明→按需查看load/位姿转换/统计函数。
`private`绘图工具和通用检查不必每次重读。

## 测试

```matlab
run('tests/test_experiment_analysis.m');
run_all_tests(false);
```

## 连续路径IK分析补充

analyze_ik_trajectory(csv_path,urdf_path,cfg)

- 输入：C++连续路径IK的CSV、PiPER URDF、MATLAB/C++交叉验算容差；兼容旧75列几何格式和新94列时间格式。
- 输出：r（逐点数据与配置）、summary（单行汇总table）、points（逐点复核table）。
- 检查：必需列/有限值、连续index与s、配置一致性、关节限位及相邻增量，
  并独立重算POE、目标SE(3)插值、残差和加权雅可比最小奇异值。
- 不写文件、不删除异常样本、不运行第二套IK，不把s当时间。
- 存在t_s时，复核等间隔时间、五次时间缩放、六轴速度与加速度差分及自设离线上限；输出r.t、r.dq_rad_s、r.ddq_rad_s2和峰值统计。首尾是单边差分估计，不强制置零。
- 用法见 MATLAB/experiments/project1/2026-10-18_ik_trajectory/matlab_plot.m。
- 回归测试 MATLAB/tests/test_ik_trajectory_analysis.m 使用已归档的默认10秒501点CSV；
  若刻意更换默认数据，应同步更新该固定案例的预期值。
