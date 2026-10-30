# PiPER Robot Motion Control

C++17算法实现、MATLAB独立验算与PiPER真机实验。当前完成Project1基础运动学与执行验证；Project2～4是后续方向，目录存在不代表功能完成。

## 项目状态

| 项目 | 内容 | 当前状态 |
| --- | --- | --- |
| Project1 | FK、Jacobian、数值IK、时间参数化、真机位置执行与反馈分析 | 已有四阶段实验和收尾报告；外部绝对精度、抓取、硬实时未验证 |
| Project2 | 动力学建模、参数辨识与模型控制 | C++主体尚未实现，下一阶段开展 |
| Project3 | 约束轨迹优化 | 预留接口/示例，尚未完成 |
| Project4 | 力感知与笛卡尔阻抗 | 预留接口/示例，尚未完成 |

详见[实现状态与边界](docs/PROJECT_STATUS.md)。MATLAB数值原型通过测试，不代表同名C++模块或真机功能已经实现。

## 目录与依赖

```text
core/          通用C++算法，不依赖ROS、HAL或实验文件读写
hal/           CAN字节解码与离线帧组装，不直接控制机械臂
apps/          程序入口，组合算法、HAL和实验I/O
tools/         ROS适配、日志缓冲、离线导出
MATLAB/        独立验算、数值/符号算法、实验绘图与测试
models/        URDF和网格资源
experiments/   正式实验数据、结果、报告
tests/         C++与Python测试，硬件测试不自动执行
config/        显式读取的离线分析JSON，不是硬件运行配置
docs/          跨实验技术说明
templates/     实验文档模板，MATLAB/template另放代码模板
build/         本地生成目录，已忽略，不提交
```

依赖方向：`apps → core / hal / experiment_io`，ROS适配位于`tools/ros2`。当前C++模型使用源于URDF的标称常量，编辑URDF不会自动同步C++；修改模型必须做跨语言复核。

## 环境

- CMake ≥3.21、Ninja、C++17编译器、Eigen3。
- Python ≥3.10：默认离线Python测试，不需要ROS。
- MATLAB：本机验证R2024b；优化/符号测试缺少工具箱时明确SKIP。
- ROS2 Jazzy：只在使用ROS适配工具时需要，基础构建与CTest不连接机器人。

Ubuntu可按需安装：`sudo apt install build-essential cmake ninja-build libeigen3-dev python3`，本次不自动执行安装。

## 日常构建与测试

在仓库根目录运行：

```bash
cmake --preset debug
cmake --build --preset debug-build
ctest --preset debug-test
```

默认构建Project1和离线测试，不构建未完成的Project2～4示例，不注册真机动作或自动使能。

带34段保留数据的收尾核查：

```bash
cmake --preset project1
cmake --build --preset project1-build
ctest --preset project1-test
```

使用独立`build/project1`。数据缺失明确失败，不以跳过代替通过。发布构建对应`release`、`release-build`、`release-test`。
编译数据库在`build/<preset>/compile_commands.json`。目前没有install打包规则。

| CMake开关 | 默认 | 用途 |
| --- | --- | --- |
| `BUILD_TESTING` | ON | 构建/注册离线测试 |
| `PIPER_ENABLE_OFFLINE_PYTHON_TESTS` | ON | 纯Python假节点、保护与配置测试，不启动ROS |
| `PIPER_ENABLE_EXPERIMENT_DATA_TESTS` | OFF | 数据回归，project1预设显式ON |
| `PIPER_BUILD_PLACEHOLDER_APPS` | OFF | 可选编译未完成历史示例，不代表对应任务已实现 |

机器专属覆盖放`CMakeUserPresets.json`，不提交。VS Code的本地任务与离线IK调试已配置在`.vscode`，该目录仍按现有规则忽略；共享构建以CMake预设为准。调试需要C/C++扩展及GDB。

## Project1使用入口

先读[实验索引](experiments/project1/README.md)与[最终收尾报告](experiments/project1/2026-10-27_movej_repeat/project1_closeout/REPORT.md)。
四阶段为静态FK、动态FK、精简离线IK和最终真机验证。

```bash
./build/debug/ik_trajectory_validation
```

以上只读离线验证，不连接机器人、不写文件。导出必须指定不存在的文件：

```bash
./build/debug/ik_trajectory_validation /tmp/piper_ik_new.csv 10
```

ROS适配限制见[工具索引](tools/ros2/README.md)。历史`--wide-preview`、`--relative-target`、`--endpoint-joint-target`不是当前C++ CLI支持的功能。

## MATLAB与配置

```matlab
addpath('/home/nbw/piper_robot_motion_control/MATLAB'); % 换成实际仓库路径
results = run_all_tests(false); % 数值与保留数据测试
% run_all_tests(true) 额外测试符号模块，缺少工具箱标注SKIP
```

进入`MATLAB/experiments/project1/2026-10-27_movej_repeat`，运行`view_project1_figure(2)`即可打开可读数值的路径图，不必重跑分析。

`analyze_today`默认读取[分析JSON](config/project1_analysis.json)。容差null表示未设任务验收要求，不代表零误差；显式cfg参数覆盖默认JSON。配置只影响重新分析，不影响真机增益、速度、限位或停止，详见[配置说明](config/README.md)。已有结果拒绝覆盖。

## 历史与复现边界

当前工作树没有`archives`目录。历史内容曾在整理时移出，但不能再宣称仍可从本仓库恢复；恢复需要独立备份。四阶段正式数据与报告仍保留。

旧解码对照已保存于正式结果的`decode_comparison.csv`。重新导出可用`--output-dir`指定新目录，默认读取保存的对照表，不依赖缺失归档；缺少旧统计时明确写NaN，不伪造数据。参见[实验说明](experiments/project1/2026-10-27_movej_repeat/README.md)。

## 技术文档与限制

- [POE数值IK](docs/ik_methods.md)
- [DH推导（未完成草稿）](docs/piper_dh_derivation.md)
- [CAN解码与组装](hal/README.md)
- [MATLAB数学约定](MATLAB/README.md)

反馈角回算、厂家内部位姿和模型一致性不是独立外部精度。TCP、带载、全工作区、抓取和硬实时需要单独验证。软件门控不等于硬件急停，离线测试通过不是安全放行。

收尾后先核对`git status`和差异，再决定提交或打标签。本配置不自动提交、推送或操作真机。
