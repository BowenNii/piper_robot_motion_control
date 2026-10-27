# 实验验证函数（Project1 / Project2）

数学约定：长度 **m**，角度 **rad**，时间 **s**；旋量按 `[omega;v]`。
关节序列是 **N×n（每行一个采样点）**；单个IK解可为列向量。
位姿必须使用相同基坐标系和相同末端参考点。

初始化：在 MATLAB 中打开仓库的 `MATLAB` 文件夹，运行 `setup_piper_paths`。
函数只报告或计算，不控制真机、不改日志、不删除异常行。
无效维度/参数通常直接报错；格式检查 `check_feedback_data` 返回错误说明。
质量报告 `ok=false` 不会自动停止，主脚本自行决定是否继续以及选哪些样本。

## 函数清单

| 函数 | 输入 | 主要输出 | 用途 |
|---|---|---|---|
| `check_feedback_data` | table、必需数值列名 | ok、messages、缺列、无效行 | 共用格式检查 |
| `check_sample_timing` | 时间、六帧跨度、配置 | 异常行、dt、中位间隔、最大跨度 | 共用时间质量检查 |
| `check_joint_limits` | N×n角度、上下限、可选容差 | 超限行/关节/超限量、valid_rows | 共用限位检查 |
| `check_se3` | 4×4位姿、可选数值容差 | 正交误差、行列式、末行误差、ok | 共用位姿结构检查 |
| `pose_error` | actual与reference位姿 | 距离m、转角rad、有符号位置差 | 共用偏差计算 |
| `summarize_errors` | 非负误差序列、可选逻辑mask | count、mean、rms、max、max_row | 通用统计，单位不变 |
| `validate_fk_samples` | S、M、N×n角度、4×4×N参考位姿、容差 | MATLAB位姿、逐条偏差、valid_rows | Project1独立POE交叉验证 |
| `check_static_stability` | 窗口内角度、XYZ、范围阈值 | 各列峰峰范围、stable | Project1静态窗口检查 |
| `check_ik_solution` | S、M、解q、目标、限位、配置 | FK位姿、残差、限位、ok | Project2解回代检查 |
| `check_joint_continuity` | 角度序列、最大相邻变化量 | 跳变行/关节、delta_rad | Project2解跳变检查 |
| `check_jacobian_condition` | 任务雅可比、最小奇异值阈值 | 奇异值、条件数、秩、风险 | Project2奇异性诊断 |
| `check_target_reached` | 反馈/目标位姿、位置/姿态容差 | reached、偏差 | Project2单条反馈终点判定 |

已有 `common/error_stats.m` 的结构体统计接口保留，不创建同名函数；
已有 `common/split_phases.m` 继续用于动态分段，不在此复制。

## Project1：主脚本保留实验流程

```matlab
data = readtable(csv_path); % csv_path由实验脚本定义
names = compose('q%d_rad',1:6);
required = [{'t_s','frame_span_s'},cellstr(names)];
format = check_feedback_data(data,required);
if ~format.ok
    error('%s',strjoin(format.messages,'；'));
end

cfg_time = struct('max_frame_span_s',0.002,'gap_factor',3);
timing = check_sample_timing(data.t_s,data.frame_span_s,cfg_time);
model = make_piper_model;
q = data{:,cellstr(names)};
limits = check_joint_limits(q,model.q_min,model.q_max);
% 查看timing、limits报告；不要隐式删除异常数据。
```

独立FK验证时，主脚本从CSV构造 `T_cpp(:,:,k)` 或 `T_can(:,:,k)`；
使用CAN RPY时旋转为 `Rz(yaw)*Ry(pitch)*Rx(roll)`，输入必须是rad。
这部分保留在实验脚本，让参考数据来源可追溯。

```matlab
cfg_fk = struct('position_tolerance_m',1e-9, ...
                'rotation_tolerance_rad',1e-8);
% T_cpp需由本次CSV明确构造，不能省略其来源。
cross = validate_fk_samples(model.S_list,model.M,q,T_cpp,cfg_fk);
stats_mm = summarize_errors(cross.position_error_m*1000);
```

上例容差用于MATLAB/C++相同模型数值比较，不是CAN/真机精度验收标准。

## Project2：先求解，再独立验收解

```matlab
% q_solution与T_target由主脚本的IK流程提供。
cfg_ik = struct('position_tolerance_m',1e-4, ...
                'rotation_tolerance_rad',1e-3, ...
                'limit_tolerance_rad',0);
result = check_ik_solution(model.S_list,model.M,q_solution,T_target, ...
                          model.q_min,model.q_max,cfg_ik);
% 求解器失败、不可达目标等情况仍在主脚本明确处理。
```

`check_joint_continuity` 的阈值必须依据相邻目标间距/采样周期设置，
不是直接套用关节最大速度。有限转动关节不自动做2pi角度wrap。
六维雅可比混合线/角速度单位，奇异阈值取决于尺度，必要时先按特征长度缩放。
`check_target_reached` 只判断一个反馈；持续到位/停稳须另选时间窗口验证。

## 阅读与测试

先看主脚本如何配置阈值、解释报告，再按需要读对应函数；不用每次重读所有检查。

```matlab
run('tests/test_validation.m'); % 独立测试
run_all_tests(false);          % 全部数值库回归测试
```

这些模块不验证路径碰撞、制动能力或硬件急停；CAN末端反馈通常不是外部独立测量。
数据合格、模型一致、真机定位精度与安全是不同层次，不能相互替代。
