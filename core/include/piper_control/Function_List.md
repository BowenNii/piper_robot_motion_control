# Function List

## common / constants

* kDof：机器人自由度数量常量
* kPi：圆周率常量
* kEpsilon：数值计算精度阈值
* kSmallAngle：小角度判断阈值

---

## math

* 【MATH-01】 clamp：数值 → 指定范围内的限幅值
* 【MATH-02】 trim_small：输入矩阵 → 微小元素置零后的矩阵
* 【MATH-03】 deg_to_rad：角度 degree → 角度 rad
* 【MATH-04】 rad_to_deg：角度 rad → 角度 degree

---

## lie_group / so3

* 【SO3-01】 skew：3维向量 → 3×3反对称矩阵
* 【SO3-02】 vee：3×3反对称矩阵 → 3维向量
* 【SO3-03】 rot_x：旋转角度 θ → 绕 X 轴的 3×3 旋转矩阵
* 【SO3-04】 rot_y：旋转角度 θ → 绕 Y 轴的 3×3 旋转矩阵
* 【SO3-05】 rot_z：旋转角度 θ → 绕 Z 轴的 3×3 旋转矩阵
* 【SO3-06】 so3_exp：3维旋转向量 → 3×3 SO(3) 旋转矩阵
* 【SO3-07】 so3_log：3×3 SO(3) 旋转矩阵 → 3维旋转向量
* 【SO3-08】 rot_axis_angle：旋转轴 + 旋转角度 → 3×3 SO(3) 旋转矩阵

---

## lie_group / se3

* 【SE3-01】 twist_hat：6维 Twist 旋量 → 4×4 se(3) 矩阵
* 【SE3-02】 se3_from_rt：3×3 旋转矩阵 + 3维平移向量 → 4×4 SE(3) 齐次变换矩阵
* 【SE3-03】 se3_inverse：4×4 SE(3) 齐次变换矩阵 → 4×4 逆齐次变换矩阵
* 【SE3-04】 adjoint：4×4 SE(3) 齐次变换矩阵 → 6×6 SE(3) 伴随矩阵
* 【SE3-05】 adjoint_inverse：4×4 SE(3) 齐次变换矩阵 → 6×6 逆伴随矩阵
* 【SE3-06】 little_ad：6维 Twist → 6×6 little adjoint 矩阵
* 【SE3-07】 se3_exp：6维 Twist 坐标 → 4×4 SE(3) 齐次变换矩阵
* 【SE3-08】 se3_log：4×4 SE(3) 齐次变换矩阵 → 6维 Twist 坐标

---

## lie_group / screw

* 【SCREW-01】 make_revolute_screw_axis：旋转轴方向 + 轴上一点 → 6维旋转关节 Screw Axis
* 【SCREW-02】 make_prismatic_screw_axis：移动方向 → 6维移动关节 Screw Axis
* 【SCREW-03】 screw_exp：6维 Screw Axis + 关节变量 θ → 4×4 SE(3) 运动变换矩阵

---

## lie_group / wrench

* 【WRENCH-01】 transform_wrench：Wrench + SE(3) 坐标变换 → 变换后的 Wrench

---

## kinematics / forward_kinematics

* 【FK-01】 forward_poe_space：空间 Screw Axis 矩阵 + 关节位置 + Home 位姿 → 末端 SE(3) 位姿
* 【FK-02】 forward_poe_body：Body Screw Axis 矩阵 + 关节位置 + Home 位姿 → 末端 SE(3) 位姿

---

## kinematics / jacobian

* 【JAC-01】 jacobian_space：空间 Screw Axis 矩阵 + 关节位置 → 6×n 空间 Jacobian
* 【JAC-02】 jacobian_body：Body Screw Axis 矩阵 + 关节位置 → 6×n Body Jacobian
* 【JAC-03】 twist_from_space_jacobian：空间 Jacobian + 关节速度 → 6维空间 Twist
* 【JAC-04】 twist_from_body_jacobian：Body Jacobian + 关节速度 → 6维 Body Twist

---

## kinematics / singularity

* 【SING-01】 singular_values：Jacobian → 奇异值向量
* 【SING-02】 minimum_singular_value：Jacobian → 最小奇异值
* 【SING-03】 condition_number：Jacobian → 条件数
* 【SING-04】 manipulability：Jacobian → 可操作度
* 【SING-05】 pseudoinverse_svd：Jacobian → SVD 伪逆
* 【SING-06】 dls_pseudoinverse：Jacobian + 阻尼系数 λ → DLS 伪逆
* 【SING-07】 dls_adaptive：Jacobian + 最大阻尼系数 λ_max + 奇异值阈值 σ_threshold → 自适应 DLS 伪逆

---

## kinematics / ik_solver

* 【IK-01】 solve_ik_newton：机器人运动学模型 + 目标位姿 + 初始关节位置 → Newton 逆运动学结果
* 【IK-02】 solve_ik_dls：机器人运动学模型 + 目标位姿 + 初始关节位置 → DLS 逆运动学结果
* 【IK-03】 solve_ik_adaptive_dls：POE模型 + 目标位姿 + 初值 → 自适应阻尼逆运动学结果
* 【IK-04】 solve_ik_transpose：POE模型 + 目标位姿 + 初值 → 雅可比转置迭代逆运动学结果

---

## robot_model

## robot_model

* 【MODEL-01】 make_piper_model：PiPER URDF 的 6 个关节零位参数（`origin xyz/rpy`、局部 `axis`、位置/速度限位）→ `PiperModel`
  - `S_list`：零位时相对 `world == base_link` 表达的 6×6 空间旋量轴矩阵；
  - `M`：零位时末端 `link6` 相对 `base_link` 的 SE(3) 位姿；
  - `B_list`：相对零位末端 `link6` 坐标系表达的 6×6 身体旋量轴矩阵，满足 \(B = \operatorname{Ad}_{M^{-1}}S\)；
  - `q_min`、`q_max`、`dq_max`：从 URDF 读取的关节位置与速度限制。

---

## dynamics / spatial

* 【DYN-02】 spatial：空间代数基础运算 → 空间代数计算结果

## dynamics / spatial_inertia

* 【DYN-01】 spatial_inertia：刚体质量 + 质心惯量 + 质心位置 → 6×6 空间惯量矩阵

## dynamics / rnea

* 【DYN-03】 rnea：机器人状态 + 动力学参数 → 关节力矩

## dynamics / crba

* 【DYN-04】 crba：机器人状态 + 动力学参数 → 关节空间惯量矩阵

## dynamics / gravity

* 【DYN-05】 gravity：机器人关节位置 + 动力学参数 → 重力力矩

## dynamics / friction

* 【DYN-06】 friction：关节速度 + 摩擦参数 → 摩擦力矩

## dynamics / regressor

* 【DYN-07】 regressor：机器人状态 → 动力学回归矩阵

---

## control / pd_controller

* 【CTRL-01】 pd_controller：期望关节状态 + 当前关节状态 → PD 控制力矩

## control / gravity_compensation

* 【CTRL-02】 gravity_compensation：机器人状态 + 动力学参数 → 重力补偿力矩

## control / computed_torque

* 【CTRL-03】 computed_torque：期望状态 + 当前状态 + 动力学模型 → 计算力矩控制指令

## control / joint_impedance

* 【CTRL-04】 joint_impedance：期望关节状态 + 当前关节状态 → 关节阻抗控制力矩

## control / cartesian_impedance

* 【CTRL-05】 cartesian_impedance：期望末端位姿 + 当前末端状态 → 笛卡尔空间阻抗控制力矩

## control / cartesian_velocity_controller   
* 【CTRL-06】 cartesian_velocity_controller：Jacobian + 目标末端 Twist + 奇异性配置 → 经自适应 DLS 与奇异降速处理后的关节速度命令、最小奇异值、速度缩放系数和奇异性状态


---

## trajectory / se3_trajectory

* 【TRAJ-01】 interpolate_se3：起始 SE(3) 位姿 + 目标 SE(3) 位姿 + 插值系数 → 插值 SE(3) 位姿

---

## optimization / qp_problem

* 【OPT-01】 qp_problem：优化变量 + 目标函数 + 约束 → QP 问题

## optimization / trajectory_optimizer

* 【OPT-02】 trajectory_optimizer：机器人模型 + 轨迹约束 → 优化后轨迹

## optimization / sequential_convexification

* 【OPT-03】 sequential_convexification：非线性动力学优化问题 → 逐次凸化优化轨迹

---

## estimation / low_pass_filter

* 【EST-01】 low_pass_filter：输入信号 + 滤波参数 + 上一时刻状态 → 滤波后信号

## estimation / kalman_filter

* 【EST-02】 kalman_filter：系统状态 + 观测数据 → 估计状态

## estimation / joint_state_estimator

* 【EST-03】 estimate_joint_velocity：当前关节位置 + 上一时刻关节位置 + 采样周期 → 关节速度估计

---

## safety / command_limiter

* 【SAFE-01】 limit_joint_velocity：关节速度指令 + 最大关节速度 → 限幅后的关节速度

## safety / joint_limits

* 【SAFE-02】 check_joint_limits：N×n 关节角序列(rad) + n维上下限(rad) + 容差 → 全部通过标志、逐行通过标志、超限幅度矩阵及超限行/关节索引。只检查位置，不裁剪输入，不检查碰撞。

## safety / watchdog

* 【SAFE-03】 watchdog：控制周期 / 通信状态 → Watchdog 状态

---

## 本次补充：common / error_statistics（已实现）

* 【STAT-01】 ErrorAccumulator::add：单个非负有限误差 → 累计样本数、和、缩放平方和、最大值；不保留全部样本。单位由调用者保持一致。
* ErrorAccumulator::statistics：已累计数据 → count、mean、rms、max；无样本时 count=0，其余 NaN。
* ErrorAccumulator::reset：无输入 → 清空统计。负数/NaN/Inf 不会进入累计。

## validation / pose_error（已实现）

* 【VAL-01】 check_se3：矩阵 + 数值容差（默认 1e-8） → 是否为有限4×4齐次位姿、旋转正交误差、det(R)、末行误差。不修正输入矩阵。
* 【VAL-02】 pose_error：actual、reference 两个 SE(3) → position_m、rotation_rad、delta_p_m。位置差为 actual-reference；转角范围 [0,pi]，不直接相减 RPY。

## validation / kinematics_validation（已实现）

* 【VAL-03】 check_ik_solution：6×n 空间旋量 S、零位 M、候选关节角 q、目标位姿、上下限、IkValidationConfig → FK回代位姿、位姿残差、关节限位报告、综合通过标志。不是逆解求解器；还需调用者检查求解器成功状态。
* 【VAL-04】 check_joint_continuity：N×n 角度序列 + 每关节变化阈值（或一个共用阈值，rad） → 相邻有符号差 (N-1)×n、跳变索引和幅度。不做2pi绕回，不替代速度/加速度检查。

所有报告中的行号与关节号均从0开始。N×n表示“一行一个样本，一列一个关节”。
输入维度/有限值/配置不合法时抛 std::invalid_argument；合法输入但验证未通过时返回 ok=false。
check_se3 是例外：非法矩阵返回报告，非法容差才抛异常。

### 阅读与使用顺序

1. 先看各头文件中的输入输出说明；再看同名 cpp 的计算步骤。
2. 阅读 tests/unit/test_validation_modules.cpp，查看正常输入与拒绝输入示例。
3. 最后看 apps/project1/fk_validation.cpp：它保留实验业务流程，调用位姿误差和统计模块。

示例（变量 q、model、target 已准备好）：

```cpp
#include "piper_control/validation/kinematics_validation.hpp"
#include "piper_control/common/error_statistics.hpp"
const auto report = piper_control::check_ik_solution(
    model.S_list, model.M, q, target, model.q_min, model.q_max);
// report.ok 同时要求 FK 回代误差满足阈值、角度不超限。
piper_control::ErrorAccumulator stats;
stats.add(report.error.position_m);
const auto summary = stats.statistics(); // mean/rms/max 都是 m。
```

### 文件输出与反馈诊断的位置

文件输出不放入数学 core：见 tools/experiment_io/README.md。
CAN 帧组装与丢弃原因不放入数学 core：见 hal/README.md。
这些验证函数是工程检查，不是硬件急停或认证安全控制器。
上面既有列表仍含 dynamics、estimation、watchdog 等占位模块，不能将列表中的全部条目视为已实现功能。

## trajectory / time_parameterization（2026-10-19补充）

* quintic_time_scaling(t, duration)：当前时间与总时长（s）→ PathTimeState，含路径进度s、ds_dt（1/s）、d2s_dt2（1/s²）。五次时间缩放，起终点路径速度及加速度为0；拒绝越界时间、非正时长和非有限输入。
* differentiate_joint_samples(q, dt)：N×n等时间采样角度与采样间隔（s）→ JointTimeDerivatives，velocity（rad/s）和acceleration（rad/s²），均为N×n。至少4行，内点中心差分，首尾单边差分，不强制置零。差分会放大误差，不代表真机测量或点间约束保证。

示例入口：apps/project1/ik_trajectory_validation.cpp；测试：tests/unit/test_time_parameterization.cpp。
小角度SO(3)/SE(3)映射使用级数保留微小旋转，避免平滑启停时的旋转截断与IK停滞。
