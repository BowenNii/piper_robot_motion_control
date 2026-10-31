# Python工程结构与依赖边界

本目录借鉴官方ROS驱动的职责分离，并尽量对应C++算法分类，但不把离线算法包装成ROS节点。
只创建已有实现的模块，不创建空的优化/安全/控制框架。

## 目录职责

| 目录 | 放什么 | 不放什么 |
| --- | --- | --- |
| common | 数值输入检查、序列数据类型、默认仓库路径 | 实验指标与硬件参数 |
| lie_group | SE(3)、旋量指数/对数、伴随变换 | URDF/CSV读取 |
| robot_model | 模型数据结构、URDF适配、显式负载合并 | 实验处理与绘图 |
| kinematics | POE正运动学、位姿序列、空间/物体雅可比 | 读取某次实验文件 |
| dynamics | 空间运算、惯量、RNEA/CRBA、重力、摩擦、批量预测 | 控制律、参数拟合和文件读写 |
| identification | 惯性参数、回归矩阵、堆叠和拟合 | 绘图与硬件发送 |
| control | 纯数值计算力矩律 | CAN/ROS接口、使能或安全承诺 |
| signals | 重采样、离线滤波、导数估计 | 偷改输入CSV或掩盖丢包 |
| validation | 位姿/路径/时序/响应/延迟/力矩残差/基准比较 | 文件保存和报告排版 |
| data_io | CSV、配置、反馈适配、来源摘要、绘图缓存 | 动力学和绘图 |
| plotting | 路径、关节、响应、误差、质量和力矩图 | 重新运行FK/RNEA或重新定义误差 |
| experiments | 组合各模块的Project1/2流程与特定结果导出 | 可复用数学公式的唯一实现 |
| scripts | 命令行参数、调用入口、结束提示 | 实验循环、模型算法和画图细节 |

Python不需要`.hpp/.cpp`分离。紧密相关的小函数可以放在同一文件，例如SE(3)运算放在`lie_group/transforms.py`。
不为了“文件越多越工程化”而进一步拆分几行函数。

## Project1调用链

```text
scripts/analyze_project1.py
    experiments/project1/pipeline.py          总流程
        data_io/project1.py                  清单重定位、CSV -> FeedbackRecord
        experiments/project1/record.py       单段FK核对、终点/响应统计
        experiments/project1/stream.py       C路径与去程/返程延迟验证
            validation/response.py           描述性响应
            validation/alignment.py          延迟拟合、ZOH参考误差
            validation/statistics.py         时间加权RMS等
        experiments/project1/export.py       结果文件和来源记录
        plotting/closeout.py                  十图组织
            quality.py / paths.py / joints.py / response.py / errors.py
```

`experiments/project1/history.py`只组织`static_fk.py`、`dynamic_fk.py`、`ik_trajectory.py`。
历史指标定义保留原样，历史绘图另放`plotting/history.py`。

## Project2调用链

```text
scripts/analyze_project2.py
    experiments/project2/pipeline.py
        data_io/joint_states.py -> JointStateSeries
        robot_model/urdf.py
        dynamics/batch.py -> rnea.py / crba.py / terms.py
        validation/torque.py
        experiments/project2/export.py
        plotting/torque.py
```

`scripts/project2_demo.py`只解析参数；合成演示在`experiments/project2/demo.py`。
模型默认到link6，不自动包含夹爪。真机控制接口仍属于仓库原有`tools/ros2`，不混进离线包。

## 数据约定

- `RobotModel`：`robot_model/types.py`，保存模型而不读取文件。
- `FeedbackRecord`：`common/records.py`，一段ABC数据，字段名显式包含单位。
- `JointStateSeries`：`common/records.py`，Project2状态及可选、已确认的关节力矩。
- 上述冻结数据类只阻止字段重新赋值，不会让内部NumPy数组自动只读；分析函数不得原地修改输入数组。
- 分析结果仍保留字段明确的字典，以兼容已有CSV/NPZ和交互图；没有强行引入复杂继承框架。

## 兼容与测试

根目录的`robot.py`、`metrics.py`、`io.py`、`project1.py`、`project2.py`、`history.py`只是旧导入转发层。
`dynamics/__init__.py`中的旧辨识/控制名称是惰性转发，算法实现不在其中。
新脚本和新测试直接导入领域模块，不继续依赖兼容入口。

测试分类：

- `analysis/tests/test_dynamics.py`：原动力学、标称模型、控制律与CSV保护测试。
- `analysis/tests/test_validation.py`：统计、路径、时间、配置和信号测试。
- `analysis/tests/test_kinematics.py`：雅可比与FK小增量交叉检查。
- `analysis/tests/test_architecture.py`：依赖方向、纯算法导入、兼容一致性、响应/延迟辅助函数。
- `tests/integration/test_python_migration.py`：正式结果、原始数据摘要及交互重建。
- `tests/integration/test_python_refactor.py`：临时目录完整重跑，比较旧版17张CSV和44份NPZ，以及Project2演示。

不能仅凭文件数增加判断工程化完成；依赖边界和结果回归才是约束。
