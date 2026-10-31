# Python离线分析入口

从2026-10-30起，Project1正式分析和Project2新增数值原型/绘图以这里为主。
C++核心算法及ROS执行仍保留原职责；本目录没有ROS/CAN依赖，不使能、不发送运动命令。
MATLAB保留为历史复现与交叉验证基准，不删除、不继续强制维护重复绘图。

本次交付与核查结果见[MIGRATION_REPORT.md](MIGRATION_REPORT.md)，工程化结构见[ARCHITECTURE.md](ARCHITECTURE.md)，Project2公式阅读见[PROJECT2_READING.md](PROJECT2_READING.md)。
后续工程化重构与结果不变核查见[REFACTOR_REPORT.md](REFACTOR_REPORT.md)。

## 明天从哪里读起

1. `scripts/analyze_project1.py`：分析入口，先读完整处理流程。
2. `src/piper_analysis/experiments/project1/pipeline.py`：只看流程顺序。
3. `data_io/project1.py`、`common/records.py`：数据读取与带单位的结构。
4. `experiments/project1/record.py`、`stream.py`：单段和C轨迹分析。
5. `validation/statistics.py`、`path.py`、`response.py`、`alignment.py`：可复用指标定义。
6. `plotting/closeout.py`及`paths.py`、`errors.py`：绘图组织与具体图形。
7. 进入Project2后按[阅读路线](PROJECT2_READING.md)阅读`robot_model/`、`dynamics/`等。

以上第3～7项均相对`analysis/src/piper_analysis/`。原来几个大文件已拆分，根目录同名小文件只保留兼容导入，不是主要阅读入口。

## 环境

依赖：Python>=3.10，NumPy>=1.24，SciPy>=1.10，Matplotlib>=3.6。
本机实际验证Python3.12 / NumPy1.26.4 / SciPy1.11.4 / Matplotlib3.6.3。
没有修改系统包或ROS环境。系统已有科学计算依赖，本机使用：

```bash
cd /home/nbw/piper_robot_motion_control
python3 -m venv --without-pip --system-site-packages .venv-analysis
.venv-analysis/bin/python -m unittest discover -s analysis/tests -v
```

`--without-pip`是本机缺少ensurepip时的可用办法，不要求sudo安装。虚拟环境已忽略，不提交Git。
全新机器如果没有这些依赖，在具备venv/pip的环境中用`python -m pip install -e ./analysis`安装。
不要向ROS系统Python执行升级依赖命令。脚本自动定位src，直接运行不要求先pip安装。

## Project1：全部四阶段

```bash
.venv-analysis/bin/python analysis/scripts/analyze_project1.py
```

默认输出在`experiments/project1/2026-10-27_movej_repeat/project1_closeout/python/`。
本次已经生成，重复运行会拒绝覆盖。需要重跑时显式指定一个不存在的新目录：

```bash
.venv-analysis/bin/python analysis/scripts/analyze_project1.py --output /tmp/piper_project1_python_new
```

输出内容：

- `run_metrics.csv`、`response_metrics.csv`：34段分析与关节响应。
- `matlab_comparison.csv`：逐字段跨语言对照，不用MATLAB结果参与Python计算。
- `traces/`：10段C轨迹的逐点误差、对齐后数据。
- `figures/`：按原01～10主题生成的PNG和可缩放SVG。
- `plot_data/`：安全数值NPZ（禁止pickle）与元数据，可重建交互图。
- `history/static_fk/`：10个位形明细、总览和汇总表。
- `history/dynamic_fk/`：5组运动曲线、分阶段和全程统计。
- `history/ik_trajectory/`：6张图、逐点核对和统计。
- `provenance.json`、历史`source_hashes.json`：输入摘要、模型版本和配置。

数值对应关系：

- `T`使用`(N,4,4)`；MATLAB为`(4,4,N)`。
- 关节/位置内部rad、m；图与表按字段转换成deg、mm。
- A/B只比较终点与实际路径，不编造固件内部时间参考。
- C按实际发布返回时刻做零阶保持；延迟拟合仍用线性插值，评价仍用原始ZOH参考。
- 延迟仅用同组去程20%～80%拟合，返程不重新调参；原始误差始终保留。
- `trace_metrics`是左端保持时间加权RMS；空间路径RMS与历史统计保持原有样本等权定义。
- 未设验收限值仍为NaN，不把本轮统计峰值当验收线。
- 历史近常数误差的峰值时刻对浮点舍入敏感，历史对照不要求峰值时刻相同；其余汇总数值逐字段核对。
- 原始CAN、CSV、旧MATLAB图和报告不改。反馈角回算link6轨迹不是外部实测TCP精度。

### 交互查看具体数值

```bash
.venv-analysis/bin/python analysis/scripts/view_project1.py 9
```

在有桌面的终端运行；1～10对应原十张图。工具栏放大、平移、保存，底部显示鼠标坐标。
点击曲线可显示最近数据点精确值。精确逐点数据也在traces CSV中。
服务器无桌面时用PNG/SVG，不把交互窗口未显示解释为计算失败。

## Project2：已准备的离线函数

| 模块 | 函数 | 用途/边界 |
| --- | --- | --- |
| robot_model / kinematics / validation | load_piper、forward、pose_error | 模型、FK与误差分别维护；默认末端link6 |
| dynamics/spatial.py、spatial_inertia.py | 空间运算与空间惯量 | [角;线]约定 |
| robot_model/payload.py | with_payload | 显式负载合并，返回新模型，不猜夹爪参数 |
| dynamics/rnea.py、crba.py、terms.py | 逆/正动力学、惯量矩阵、重力与速度偏置 | 不含摩擦与转子惯量 |
| dynamics/friction.py | friction | 参数必须另行辨识 |
| identification/ | 惯性参数、回归矩阵、堆叠与拟合 | 离线使用，报告秩，不保证参数物理可行 |
| control/computed_torque.py | computed_torque | 纯数值控制律，未做真机保护与验证 |
| signals/ | 重采样、离线滤波与导数 | 使用未来数据，不能直接放入实时控制环 |
| dynamics/batch.py、experiments/project2/pipeline.py | 批量预测与流程组织 | 文件适配、指标和绘图另外分类 |

模型载荷约定：`Ftip=[力矩;力]`表示所需抵抗的工具负载，正Ftip增加RNEA驱动力矩。
如果是环境施加给机器人的外力，输入其相反数。该约定与现有MATLAB递推结果一致。

### 先运行不需要真机的演示

```bash
.venv-analysis/bin/python analysis/scripts/project2_demo.py
```

本次已生成`experiments/project2/nominal_demo_python/`：合成轨迹CSV、六轴力矩分解图与说明。
重跑时用`--output`指定新目录。合成数据不是实验数据，不证明项目2完成。

### 后续真实数据分析格式

```bash
.venv-analysis/bin/python analysis/scripts/analyze_project2.py /path/to/states.csv --output /tmp/piper_project2_analysis_new
```

CSV必须明确列出：`time_s`、`q1_rad..q6_rad`、`dq1_rad_s..dq6_rad_s`、`ddq1_rad_s2..ddq6_rad_s2`。
可选`tau1_Nm..tau6_Nm`必须已核对单位、符号、传动侧；包含这些列时必须显式加`--measured-torque-verified`。
不把ROS effort或电流静默视为实测关节力矩，不从旧Project1数据凭空生成可靠加速度。
输出模型预测与残差；残差包含摩擦、模型误差等，不等于外力实测值。
正式参数辨识仍需独立拟合/验证实验、负载模型及力矩定义确认。

## 验证

```bash
.venv-analysis/bin/python -m unittest discover -s analysis/tests -v
.venv-analysis/bin/python tests/integration/test_python_migration.py -v
.venv-analysis/bin/python tests/integration/test_python_refactor.py -v
cmake --preset project1
cmake --build --preset project1-build
ctest --preset project1-test
```

Project1预设启用分析测试；普通debug保持无科学计算依赖的旧测试范围。
`analysis/tests/fixtures/matlab_dynamics_reference.csv`来自本仓库MATLAB与同一URDF的12组确定性状态。
可选生成器`export_matlab_dynamics_reference.m`仅用于重新建立基准，不是日常Python依赖。
单摆解析解、势能梯度、RNEA/CRBA一致性、回归一致性与输入异常另行测试。
