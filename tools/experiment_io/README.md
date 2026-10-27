# 实验文件输出工具

## 连续路径IK的CSV导出

头文件：include/experiment_io/ik_trajectory_csv.hpp。
write_ik_trajectory_csv 接收完整采样结果、模型、IKOptions、起终角度、相邻增量阈值及IkTrajectoryTiming，
独占创建94列CSV。只有所有点及数值速度/加速度检查通过后应用才调用；不覆盖、不自动创建目录。
原75列保持顺序，追加t_s、ds_dt、d2s_dt2、六轴dq和ddq及4个时间/离线约束配置。
s仍为路径进度，t_s才是时间。每行含六轴q、六轴有符号delta_q、目标/回代位姿、
残差、加权最小奇异值、迭代次数和重复的实验配置。
第一行delta_q是起始解相对q_start的变化，默认0；其他行为相邻解差。
终端无参数模式不写文件；参数模式示例：

```bash
./build/debug/ik_trajectory_validation /tmp/new_trajectory.csv 10
```

默认10秒501点；时长可由第二个参数指定，采样间隔为时长/500，不是固定控制频率。
导出器不运行IK、不计算导数；时间缩放和差分由core完成。导数为数值估计，首尾用单边公式。
0.1 rad/s与0.1 rad/s²为人工选择的离线上限，不是厂家参数；不保证点间或真机安全。
磁盘错误可能留下不完整输出，必须检查退出码。

CMake 库名：piper_experiment_io。当前使用 Ubuntu/Linux 文件接口，不依赖 ROS。
数学计算在 core；CAN 解码在 hal；本目录只保存已经计算好的实验结果。

| 编号 | 名称 | 输入 | 输出 |
| --- | --- | --- | --- |
| IO-01 | FkValidationResult | 时间(s)、六轴角(rad)、反馈原始RPY(rad)、反馈与POE位姿、误差 | 一组统一结果结构，不做计算 |
| IO-02 | FkCsvWriter(path) | 尚不存在的CSV文件路径 | 独占创建文件并写表头；父目录必须已存在 |
| IO-03 | write(result) | 一组 FkValidationResult | 写一行29列CSV；成功行数加1 |
| IO-04 | close() | 无 | 检查刷盘/关闭结果；成功后不允许再write |
| IO-05 | row_count() | 无 | 已成功写入的行数，不含表头 |

头文件位于 include/experiment_io/；实现位于 src/fk_csv_writer.cpp。

## 使用步骤

1. 在主程序中完成解码、RPY转矩阵、POE计算和误差计算。
2. 填入 FkValidationResult。T_feedback/T_poe 使用相同基坐标系和末端参考点。
3. 构造 writer，再逐行调用 write。
4. 正常结束显式 close 后才报告成功。析构只负责不抛异常地释放文件句柄。

```cpp
#include "experiment_io/fk_csv_writer.hpp"
experiment_io::FkValidationResult result;
// 用本次样本填入 result 的各成员，不能将默认零值当作采集结果。
experiment_io::FkCsvWriter writer("/tmp/new_fk_result.csv");
writer.write(result);
writer.close();
```

## CSV约定

29列顺序保持原 project1 约定：3个时间字段、6个关节角、3个反馈XYZ、
3个反馈RPY、3个POE XYZ、9个POE旋转矩阵元素（逐行）、2个位姿误差。
单位固定为 s、rad、m；使用小数点和17位有效数字。
终端显示的 mm/deg 与CSV的 m/rad 不同，不能混用。

write 检查有限值、非负时间/跨度/误差、SE(3)矩阵结构；
不重新解码、不重新计算POE、不验证误差与矩阵是否一致，也不核对原始RPY与矩阵是否一致。
这些关联由主程序负责，保留原始RPY可以避免通过矩阵反求RPY造成分支变化。
时间顺序、2ms配对规则由日志流程负责，writer 不跨行判断同步性。

## 异常与重新运行

已有同名文件（含符号链接）一律拒绝覆盖，使用系统 O_EXCL 防止检查/创建之间的竞争。
不自动建目录，不自动复制至归档目录。重新运行请指定新文件名。
输入格式错误抛 invalid_argument，文件错误抛 runtime_error；主程序必须捕获并以非零码结束。
磁盘失败可能留下不完整文件：保留供检查，不自动删除，也不能当作成功实验。
批处理不是多文件原子事务，中途失败时前面成功的文件可能已经存在。

## 与旧结果比较

新 pose_error 使用 atan2 计算相对旋转角，改善近零角度的数值稳定性；
旧版使用 acos(trace)。误差列最后几位可能不同，不代表模型或单位改变。
ErrorAccumulator 使用缩放平方和和更高精度累计，统计最后几位也可能变化。
原实验归档不会自动更新。测试见 tests/unit/test_fk_csv_writer.cpp。

## 本次验证记录（2026-10-07）

- Debug、Release 都编译成功；两种构建的17项CTest全部通过。
- 单日志、十日志批处理、已有输出拒绝覆盖、缺参数/缺目录/空日志都做了命令行测试。
- 用原程序与新程序分别回放16份已有日志，总计40380行；所有样本数和29列表头一致。
- 27个时间/关节/反馈/POE字段逐项完全一致。
- 位置误差差值最大约 1.09e-19 m；旋转误差差值最大约 2.25e-9 rad，来自近零角数值计算调整。
- 回放输出写入临时目录，未覆盖已有实验日志、CSV、报告或图片。

在仓库根目录运行测试：

```bash
cmake --preset debug
cmake --build --preset debug-build
ctest --preset debug-test --output-on-failure
cmake --preset release
cmake --build --preset release-build
ctest --test-dir build/release --output-on-failure
```
