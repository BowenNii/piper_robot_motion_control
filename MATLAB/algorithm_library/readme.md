# MATLAB 函数使用清单：普通算法与符号推导

## 1. 使用入口与关键约定

### 1.1 初始化

在 MATLAB 命令窗口运行（换电脑时只修改仓库路径）：

```matlab
addpath('/home/nbw/piper_robot_motion_control/MATLAB');
setup_piper_paths;       % 加载普通算法
% setup_piper_paths(true); % 需要符号推导时使用这一行
```

不要对整个 MATLAB 目录使用 genpath：实验、模板和练习存在同名脚本。每次按需打开实验脚本，不将所有实验目录加入路径。

| 位置 | 使用目的 |
| --- | --- |
| algorithm_library | 普通数值算法、检查、统计和绘图工具 |
| symbolic_derivations | 含符号变量的公式推导，数值代入后可核对普通算法 |
| MATLAB/experiments | 本次实验的路径、阈值、比较来源、流程和专项图布局 |
| 仓库根目录experiments | 原始log、CSV、报告、照片和结果图 |
| MATLAB/tests | 函数测试，不是真机操作程序 |

原则：**通用工具抽出去，业务流程留外面**。主脚本不能为了缩短而隐藏比较对象、数据来源或验收标准。局部函数和 private 工具不列为独立公共接口。

### 1.2 尺寸与单位

- n：关节数，PiPER为6；N：采样条数。单次关节量通常n×1，实验关节序列N×n（每行一个时刻）。
- 角度rad、长度m、时间s、质量kg、力矩N·m；展示层可转deg/mm，但不能把显示单位直接送进算法。
- R为3×3旋转矩阵；p为3×1平移；T为4×4齐次位姿。多时刻位姿4×4×N。
- 右手系、列向量；T_ab将b系坐标转换到a系。位姿比较必须使用相同基坐标系与末端参考点。
- Twist为[omega;v]，Wrench为[torque;force]；雅可比前三行为角速度，后三行为线速度。
- so3_exp(phi)的phi已含转角；se3_exp(xi)输入积分旋量；screw_exp(S,q)才把轴与关节变量分开输入。
- RPY顺序roll、pitch、yaw，R=rot_z(yaw)*rot_y(pitch)*rot_x(roll)。姿态偏差用相对旋转角，不直接相减RPY。
- 表中cfg/options是结构体，使用 `cfg.字段名=值` 配置；result/report是结果结构体，通过 `result.字段名` 读取。

### 1.3 应该先看哪些模块

Project1优先：robot_model → lie_group/kinematics → experiment_analysis → validation → plotting。
Project2再看：IK、Jacobian、奇异性、IK回代与连续性检查。动力学、力控、优化按项目需要再看，不必一次读完。

## 2. 普通函数：algorithm_library

### common：常量、统计与运动分段

| 编号 | 函数与输入 | 输出及用途 |
| --- | --- | --- |
| COMMON-01 | `c=piper_constants()`，无需输入 | 常量结构体：kDof、kPi、kEpsilon、kSmallAngle。 |
| COMMON-02 | `s=error_stats(r,mask)`；r 含 position_mm、rotation_deg、span_ms、t，mask 为 N×1 逻辑向量 | 位置/姿态均值、RMS、最大值、峰值时刻及最大帧跨度；单位 mm、deg、ms、s。空选择返回 NaN。 |
| COMMON-03 | `[phase,bounds,segments]=split_phases(t,q_deg,cfg)`；时间 N×1 s、角度 N×n deg | phase：1前静态/2运动缓冲/3后静态；bounds：原始运动起止 s；segments：按变化间隔推断的活动段数。 |

`split_phases` 必须设置 `cfg.change_threshold_deg`（相邻角度变化阈值，不是速度阈值）、`motion_padding_s`（运动包络缓冲）、`activity_gap_s`（活动段间隔）。无变化返回全1、[NaN NaN]、0。运动缓冲段可能包含中间停顿，不能当成精确运动学分段。

### math、lie_group：基础运算与李群

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| MATH-01 | `clamp(value,lower,upper)` | 数值/数组及同尺寸或标量上下界 | 限幅值；上下界反置时交换，与 C++ 一致；安全参数检查用 `clamp_checked`。 |
| MATH-02 | `matEqual(A,B,tol)` | 两矩阵、绝对容差 | 布尔值，逐元素近似相等。 |
| SO3-01 | `skew(w)` | 3 维向量 | 3×3 反对称矩阵，满足 `skew(w)*v=cross(w,v)`。 |
| SO3-02 | `vee(W)` | 3×3 反对称矩阵 | 3 维向量。 |
| SO3-03 | `rot_x(theta)` / `rot_y(theta)` / `rot_z(theta)` | 转角 `[rad]` | 对应轴的 3×3 旋转矩阵。 |
| SO3-04 | `so3_exp(phi)` | 3 维积分旋转向量 `[rad]` | 3×3 旋转矩阵。 |
| SO3-05 | `so3_log(R)` | 3×3 旋转矩阵 | 3 维主值旋转向量 `[rad]`。 |
| SE3-01 | `twist_hat(xi)` | 6 维 `[ω;v]` | 4×4 se(3) 矩阵。 |
| SE3-02 | `se3_from_rt(R,p)` | 3×3 旋转、3 维平移 `[m]` | 4×4 齐次位姿。 |
| SE3-03 | `se3_inverse(T)` | 4×4 位姿 | 4×4 逆位姿。 |
| SE3-04 | `adjoint(T)` | 4×4 位姿 | 6×6 大伴随矩阵。 |
| SE3-05 | `little_ad(xi)` | 6 维旋量 | 6×6 小伴随矩阵。 |
| SE3-06 | `se3_exp(xi)` | 6 维积分旋量 | 4×4 SE(3) 位姿。 |
| SE3-07 | `se3_log(T)` | 4×4 SE(3) 位姿 | 6 维主值积分旋量。 |

补充常用入口：

| 编号 | 函数与输入 | 输出及用途 |
| --- | --- | --- |
| MATH-03 | `deg_to_rad(angle)` / `rad_to_deg(angle)`；标量或数组 | 同尺寸角度换算；degree→rad / rad→degree。 |
| MATH-04 | `trim_small(A,eps)`；矩阵、可选阈值默认1e-10 | 小元素置零后的矩阵，不修改原输入。 |
| MATH-05 | `clamp_checked(value,lo,hi)` | 限幅值；反置边界报错，而不是交换。 |
| MATH-06 | `rotation_angle(R)`；3×3相对旋转矩阵 | 主值转角 rad，适合两姿态的相对旋转比较。 |
| SO3-06 | `rot_axis_angle(axis,theta)`；非零3维轴、转角 rad | 3×3旋转矩阵；数值轴会归一化。 |
| SE3-08 | `adjoint_inverse(T)`；4×4位姿 | 6×6的 Ad(T⁻¹)。 |
| SCREW-02 | `make_revolute_screw_axis(omega,point)`；3维轴、轴上一点 m | 6×1转动轴，归一化方向后计算 [ω;−ω×p]。 |
| SCREW-03 | `make_prismatic_screw_axis(direction)`；非零3维移动方向 | 6×1移动轴 [0;单位方向]。 |
| SCREW-04 | `screw_exp(S,q)`；6×1旋量轴、关节量 | 4×4单关节变换；转动量 rad，移动量 m。 |

### robot_model、kinematics：PiPER 模型与运动学

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| MODEL-01 | `make_piper_model(urdf_path)` | PiPER URDF 路径；可省略以使用仓库内默认路径 | `model`：`S_list/B_list/M`、关节限位、`Mlist/Glist/gravity` 等。标称值来自 URDF。 |
| SCREW-01 | `make_revolute_screw_axes(w,p)` | 同一空间基坐标系下的轴方向 `w` 和轴上一点 `p`，均为 3×n | 6×n 空间旋量；逐列归一化 `w`，计算 `S_i=[ω_i;-cross(ω_i,p_i)]`。 |
| FK-01 | `forward_poe_space(S_list,q,M)` | 6×n 空间轴、n 维关节量、零位位姿 | 4×4 末端位姿 `T=∏exp([S_i]q_i)M`。 |
| FK-02 | `forward_poe_body(B_list,q,M)` | 6×n 物体轴、关节量、零位位姿 | 4×4 末端位姿 `T=M∏exp([B_i]q_i)`。 |
| JAC-01 | `jacobian_space(S_list,q)` | 空间轴、关节量 | 6×n 空间雅可比，`V_s=J_s*dq`。 |
| JAC-02 | `jacobian_body(B_list,q)` | 物体轴、关节量 | 6×n 物体雅可比，`V_b=J_b*dq`。 |
| SING-01 | `jacobian_singular_values(J)` | 有限实 Jacobian | 从大到小的奇异值。 |
| SING-02 | `jacobian_condition_number(J)` | Jacobian | 条件数；秩亏时为 `Inf`。混合角/线速度时需先考虑尺度。 |
| SING-03 | `manipulability(J)` | m×n Jacobian | 奇异值乘积；`m>n` 时只是子空间体积，不是完整任务空间体积。 |
| SING-04 | `pseudoinverse_svd(J,tol)` | Jacobian、可选绝对阈值 | 截断 SVD 伪逆。 |
| SING-05 | `dls_pseudoinverse(J,lambda)` | Jacobian、阻尼 `lambda>=0` | DLS 伪逆；`lambda=0` 回到 SVD 伪逆。 |
| SING-06 | `adaptive_dls_pseudoinverse(J,lambda_max,sigma_safe)` | Jacobian、最大阻尼、安全阈值 | `[J_pinv,lambda,sigma_min]`；按奇异值调阻尼。 |
| IK-01 | `solve_ik_poe(S_list,M,T_target,q_initial,options)` | 模型、目标 4×4 位姿、关节初值及可选方法/限位/容差 | `result`：`q/success/iterations/position_error/orientation_error/status`；失败的 `q` 不得下发真机。 |
| MDH-01 | `mdh_transform(alpha_prev,a_prev,d_i,theta_i)` | 改进 DH 四参数，角度 rad、长度 m | 单节 4×4 变换；本轮未修改。 |
| MDH-02 | `forward_mdh(parameters,q,T_base,T_tool)` | n×4 改进 DH 表、关节量、可选基座/工具变换 | 末端 4×4 位姿；PiPER 参数表仍需独立核对。 |

运动学的标准入口与 IK：

| 编号 | 函数与输入 | 输出及用途 |
| --- | --- | --- |
| JAC-03 | `twist_from_space_jacobian(J,dq)` / `twist_from_body_jacobian(J,dq)`；6×n雅可比、n×1速度 | 6×1 Twist，分别在空间/物体坐标系表达。 |
| SING-07 | `singular_values(J)` / `minimum_singular_value(J)` / `condition_number(J)` | 奇异值向量 / 最小奇异值 / 条件数。 |
| SING-08 | `[J_pinv,lambda,sigma_min]=dls_adaptive(J,lambda_max,sigma_threshold)` | n×m伪逆、实际阻尼、最小奇异值；是推荐的标准命名。 |
| IK-02 | `options=ik_options()` | 默认 IK 参数结构体。 |
| IK-03 | `solve_ik_newton(S,M,target,q0,options)` | Newton/SVD 迭代结果结构体。 |
| IK-04 | `solve_ik_dls(S,M,target,q0,options)` | 固定阻尼 DLS 结果结构体。 |
| IK-05 | `solve_ik_adaptive_dls(S,M,target,q0,options)` | 自适应阻尼 DLS 结果结构体。 |
| IK-06 | `solve_ik_transpose(S,M,target,q0,options)` | 雅可比转置迭代结果结构体。 |

四种 IK 输入均为：S 6×n空间轴、M/target 4×4位姿、q0 n×1初值、可选 options。输出 `q/success/iterations/position_error/orientation_error/status`。先设置 `options.q_min/q_max`，再检查 success；失败解不应下发真机。迭代失败不等于证明目标不可达。

默认 options：max_iterations=100、position_tolerance=1e-6 m、orientation_tolerance=1e-6 rad、lambda=1e-3、max_joint_step=0.2、rotation_weight=0.2、svd_tolerance=1e-10、sigma_threshold=0.05、lambda_max=0.05、max_backtracks=20。步长限制按 max(abs(dq))，不是向量范数；阈值应按任务设置。

`forward_mdh` 参数表每行 [alpha_prev,a_prev,d_i,theta_offset]，角度 rad、长度 m，仅实现转动关节。保留原函数，不把未经核对的厂家参数表当作已验证模型。

### dynamics：空间刚体动力学

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| DYN-01 | `spatial_inertia(m,I_com,c)` | 质量 `[kg]`、质心惯量 `[kg·m²]`、质心位置 `[m]` | 6×6 连杆空间惯量。 |
| DYN-02 | `spatial_motion_cross(V)` / `spatial_force_cross(V)` | 6 维速度旋量 | 6×6 运动/力对偶交叉矩阵。 |
| DYN-03 | `chain_transforms(model,q)` | 串联链模型、关节量 | `[A,Xup]`，供 RNEA/CRBA 使用的运动子空间与父子变换。 |
| DYN-04 | `rnea(q,dq,ddq,model,Ftip)` | 关节位置/速度/加速度、模型、可选末端外力 | n 维逆动力学力矩 `[N·m]`；不含摩擦/转子惯量。 |
| DYN-05 | `crba(q,model)` | 关节量、模型 | n×n 关节惯量矩阵。 |
| DYN-06 | `gravity(q,model)` | 关节量、模型 | n 维重力力矩 `[N·m]`。 |
| DYN-07 | `friction(dq,params)` | 关节速度、黏滞/库仑摩擦参数 | n 维摩擦模型力矩；参数须辨识。 |
| DYN-08 | `regressor(q,dq,ddq,model)` | 关节状态和模型 | n×(10n) 动力学回归矩阵 `Y`。 |
| DYN-09 | `inertial_parameters(model)` | 含 `Glist` 的模型 | 10n 维惯性参数，与 `regressor` 满足 `tau≈Y*pi`。 |

### control：离线控制原型

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| CTRL-01 | `joint_pd(q_des,dq_des,q,dq,kp,kd)` | 期望/当前关节位置速度、力矩增益 | n 维 PD 力矩。 |
| CTRL-02 | `gravity_compensation(q,inverse_dynamics_fn)` | 关节量、逆动力学函数句柄 | n 维重力补偿力矩。 |
| CTRL-03 | `computed_torque(q_des,dq_des,ddq_des,q,dq,kp,kd,inverse_dynamics_fn)` | 期望/当前状态、加速度反馈增益、逆动力学句柄 | n 维计算力矩命令；`kp/kd` 是加速度反馈增益。 |
| CTRL-04 | `joint_impedance(q_des,dq_des,q,dq,K,D,tau_ff)` | 关节状态、刚度/阻尼、可选前馈 | n 维关节阻抗力矩。 |
| CTRL-05 | `cartesian_impedance(T_des,V_des,T,V,J_body,K,D,wrench_ff)` | 末端目标/当前位姿和同坐标系 Twist、物体雅可比、增益 | `[tau,wrench_cmd]`；力矩映射 `tau=J_body'*wrench_cmd`。 |
| CTRL-06 | `cartesian_velocity_controller(J,v_command,config)` | 6×n Jacobian、6 维目标 Twist、可选奇异/关节安全参数 | `result.q_dot_command`、奇异分区、实际 Twist 等；不是真机完整闭环。 |
| CTRL-07 | `force_controller(wrench_des,wrench_meas,Kp,Ki,integral_prev,dt,limit)` | 期望/实测 Wrench、PI 增益、积分状态/周期/限幅 | `[delta_pose,integral_next]`；接触方向和符号须实验确认。 |

### estimation：离线状态估计

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| EST-01 | `low_pass_filter(input,alpha,previous_output)` | 当前值、系数 `[0,1]`、上次状态 | `[output,state]`；C++ 接口以引用更新状态，MATLAB 必须接收返回值。旧四参数调用仍兼容。 |
| EST-02 | `estimate_joint_velocity(q,q_prev,dt)` | 两次关节角 `[rad]`、实际周期 `[s]` | 后向差分关节速度 `[rad/s]`。 |
| EST-03 | `kalman_filter(x_prev,P_prev,z,F,Q,H,R,B,u)` | 上次状态/协方差、观测、离散模型及噪声；`B,u` 可选 | `[x_next,P_next,innovation,K]`；`z=[]` 时只预测。 |
| EST-04 | `joint_state_estimator_step(q_meas,dq_meas,state_prev,dt,noise)` | 关节角、可选关节速度、上次状态、周期及标准差结构体 | `state.q/dq/P` 等；电机转速须先换算为关节 rad/s。 |

### optimization、trajectory：离线约束优化与可视化

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| OPT-01 | `solve_qp(problem)` | `H,f` 必需，`A,b,Aeq,beq,lb,ub,x0` 可选 | QP 结果 `x/cost/success/max_violation`；依赖 `quadprog`。 |
| OPT-02 | `trajectory_optimizer(q_ref,dt,limits,weights)` | n×N 参考位置、周期、位置/速度/加速度限位、权重 | 平滑 QP 轨迹 `q/dq/ddq`；时间网格固定。 |
| OPT-03 | `sequential_convexification(problem,options)` | 二次目标、**可行初值**及非线性约束函数 `[c,J]=fun(x)` | 逐次线性化结果；仅离线原型，不保证全局最优。 |
| TRAJ-01 | `topp_ra_parameterize(s_grid,q_path,dq_ds,ddq_ds,dq_max,ddq_max)` | 路径及一、二阶真实路径导数；关节速度/加速度上限 | 时间轴、路径速度、关节速度及分段加速度；离散节点约束，依赖 `linprog`。 |
| TRAJ-02 | `get_joint_points(q,model)` | 关节量、含 `Mlist/Slist` 的模型 | `[points,T_end]`，URDF关节原点 3×(n+1) 与末端位姿。 |
| TRAJ-03 | `plot_robot_frame(points,ax_lim,T_end)` | 关节点、可选绘图范围和末端位姿 | 绘图句柄；不传 `T_end` 时不画末端坐标轴。 |
| TRAJ-04 | `update_robot_frame(h,points,T_end)` | 已有句柄、新关节点、必要时新末端位姿 | 更新动画，无返回值。 |
| 示例 | `trajectory/TOPP_RA.m` | 直接 `run(...)` | 演示脚本，不是通用函数或真机指令生成器。 |


### 标准速度入口、插值和限幅

| 编号 | 函数与输入 | 输出及用途 |
| --- | --- | --- |
| CTRL-08 | `config=cartesian_velocity_config()` | 默认 lambda_max=.1、sigma_safe=.1、sigma_critical=.01、minimum_speed_scale=.1。 |
| CTRL-09 | `result=solve_cartesian_velocity(J,V,config)`；6×n J、6×1 Twist、可选配置 | q_dot_command、sigma_min、speed_scale、region及辅助字段；奇异降速与自适应阻尼。 |
| EST-05 | `y=low_pass_filter_hz(x,previous,dt,cutoff_hz)` | 同尺寸滤波值；dt s、截止频率 Hz。下一次调用需传回 y。 |
| TRAJ-05 | `T=interpolate_se3(T0,T1,s)`；两个4×4位姿、插值系数 | 4×4插值位姿，s限制到[0,1]。 |
| SAFE-01 | `limited=limit_joint_velocity(dq,dq_max)`；n维速度及正上限 rad/s | 限幅后的同维速度。不是碰撞检测、硬件急停或看门狗。 |

`cartesian_velocity_controller` 是扩展接口，额外支持位置/速度/加速度约束，不能与标准接口混为一谈。其可选字段以该函数头部说明为准。控制输出是离线计算结果，不会自动发送 CAN。

### validation：数据与算法验证

关节序列为 N×n，每行一条样本；批量位姿为4×4×N。除标注外，长度 m、角度 rad、时间 s。参数格式错误通常报错，质量报告中的 ok=false 由主脚本处理，不自动删除样本。

| 编号 | 函数与输入 | 输出及用途 |
| --- | --- | --- |
| VAL-01 | `report=check_feedback_data(data,required_columns)`；table、必需数值列名 | ok、messages、缺列及无效行；检查表格式、数值列和 NaN/Inf。 |
| VAL-02 | `report=check_sample_timing(t_s,frame_span_s,cfg)`；两个N维序列、可选配置 | dt、时间异常行、长间隔行、超跨度行、中位间隔、最大跨度。 |
| VAL-03 | `report=check_joint_limits(q_rad,lower_rad,upper_rad,tolerance_rad)`；N×n角度、n维限位、可选容差默认0 | ok、valid_rows、超限行/关节/超限量。 |
| VAL-04 | `report=check_se3(T,tolerance)`；4×4位姿、可选容差默认1e-8 | ok、旋转正交误差、行列式及末行误差。 |
| VAL-05 | `[position_m,rotation_rad,delta_p_m]=pose_error(T_actual,T_reference)` | 位置距离 m、相对转角 rad、有符号位置差 actual−reference（3×1）。 |
| VAL-06 | `stats=summarize_errors(errors,mask)`；非负误差向量、可选逻辑mask | count、mean、rms、max、max_row；输入单位不变，空选择为NaN。 |
| VAL-07 | `report=validate_fk_samples(S_list,M,q_rad,T_reference,cfg)`；6×n轴、零位、N×n角、参考4×4×N | 独立 MATLAB POE 位姿 T_matlab、逐条位置/姿态偏差、delta_p_m、valid_rows、ok。 |
| VAL-08 | `report=check_static_stability(q_rad,position_m,max_joint_range_rad,max_position_range_m)`；至少2条角度/XYZ、峰峰范围阈值 | 各列变化范围及 stable；判断所选窗口是否稳定。 |
| VAL-09 | `report=check_ik_solution(S_list,M,q_rad,T_target,lower_rad,upper_rad,cfg)`；单个IK解和模型 | T_fk、位置/姿态残差、限位报告、ok；回代验收，不求解IK。 |
| VAL-10 | `report=check_joint_continuity(q_rad,max_step_rad)`；角度序列、相邻变化上限 | delta_rad、跳变行/关节；不自动做2π角度绕回。 |
| VAL-11 | `report=check_jacobian_condition(J,sigma_threshold)`；m×n任务雅可比、正阈值 | 奇异值、条件数、秩、task_full_row_rank、near_singular。 |
| VAL-12 | `report=check_target_reached(T_feedback,T_target,position_tolerance_m,rotation_tolerance_rad)` | reached、位姿偏差；只判断单条反馈，不保证持续到位或停稳。 |

常用配置：

| 调用位置 | cfg字段 | 默认/要求 |
| --- | --- | --- |
| check_sample_timing | max_frame_span_s、gap_factor、max_gap_s | .002 s、3、Inf；接收跨度不是同步采样证明。 |
| validate_fk_samples | position_tolerance_m、rotation_tolerance_rad | 必须显式设置；MATLAB/C++数值容差与CAN模型偏差容差分开。 |
| check_ik_solution | position_tolerance_m、rotation_tolerance_rad、limit_tolerance_rad | 1e-4 m、1e-3 rad、0 rad。 |

六维雅可比混合线/角速度尺度，奇异阈值不能直接跨机器人照搬。数据合格、模型一致、外部定位精度与安全是不同概念。

### experiment_analysis：CSV转换与实验统计

本模块读取的是 C++ 已导出的 CSV，不直接接收 CAN，也不解码原始 log。主脚本仍负责比较对象、独立算法、阈值和异常处理。

| 编号 | 函数与输入 | 输出及用途 |
| --- | --- | --- |
| ANALYSIS-01 | `data=load_feedback_csv(path,cfg)`；CSV路径、可选配置 | 原表、相对时间、关节角、CAN/C++位置、偏差、跨度及时间质量报告；完整模式还含T_can/T_cpp。 |
| ANALYSIS-02 | `T=feedback_rpy_to_transforms(position_m,rpy_rad)`；N×3 XYZ和RPY | 4×4×N CAN位姿，RPY列顺序 roll/pitch/yaw，R=RzRyRx。不是独立FK。 |
| ANALYSIS-03 | `T=cpp_columns_to_transforms(position_m,rotation_rows)`；N×3位置、N×9旋转元素 | 4×4×N C++位姿并检查SE(3)；旋转列为r11,r12,r13,r21,r22,r23,r31,r32,r33。 |
| ANALYSIS-04 | `summary=summarize_static_pose(data,pose_name,cfg)` | 单个位形样本数、关节均值/范围deg、位置均值mm、偏差统计、帧跨度及位置偏差列复核。 |
| ANALYSIS-05 | `[data,summary,phases]=summarize_dynamic_run(data,target_rad,model,cfg)` | 补充运动阶段/终态信息的data、整组统计、三阶段统计；不计算FK。 |
| ANALYSIS-06 | `paths=save_analysis_results(folder,tables,variables,cfg)` | 保存CSV与MAT并返回完整路径；默认拒绝覆盖，缺失输出目录会创建。 |

`load_feedback_csv` 常用字段：
- table：原始表，不修改输入文件。
- t：首点为0的N×1相对时间 s；timestamp_s：原绝对时间。
- q_rad/q_deg：N×6关节角。
- p_feedback/p_cpp：N×3位置 m。
- position_mm/rotation_deg/span_ms：偏差与跨度的显示单位版本。
- timing：时间质量报告。
- T_can/T_cpp：仅 require_transforms=true 时组装。

默认 cfg 为 require_transforms=false、strict_time=false。动态分析建议两项均 true：要求RPY和旋转元素、至少2条严格递增样本。缺列、非有限值、时间倒退、负跨度/偏差会报错；不自动排序或丢行。

静态 cfg：frame_window_s=.002、position_recheck_tolerance_m=1e-11。实验1归档坐标有舍入，主脚本取1e-10 m。位置列复核只算两位置差，不是POE交叉验算。

动态输入 data 还需 name、label、cpp_position_mm、cpp_rotation_deg；position_mm/rotation_deg 的比较来源由主脚本明确设置。model需q_min/q_max。cfg必须显式包含：

| 字段 | 意义/单位 |
| --- | --- |
| change_threshold_deg | 相邻关节角变化检测阈值 deg |
| motion_padding_s / activity_gap_s | 运动包络缓冲 / 活动间隔 s |
| endpoint_window_s | 终态统计窗口 s |
| frame_window_s | 六帧接收跨度窗口 s |
| static_range_deg | 终态角度峰峰范围阈值 deg |
| limit_tolerance_rad | 限位比较数值余量 rad |
| static_baseline_s | 静态基线最短时长 s |

保存：tables 为“字段名→table”的标量结构体，例如 struct('summary',summary_table) 输出 summary.csv；variables 的各字段存进 analysis_results.mat。cfg.overwrite 默认false；cfg.mat_name 可改MAT名称。显式重算才设置 overwrite=true；多文件保存不是原子事务。

### plotting：通用绘图

输入仍是 m、rad、s，默认显示 mm、deg、s；每行一个样本。不计算FK、不删行、不隐式保存或关闭图窗。

| 编号 | 函数与输入 | 输出及用途 |
| --- | --- | --- |
| PLOT-01 | `[fig,axes_list]=plot_joint_angles(t_s,q_rad,cfg)`；N×1时间、N×n角度 | 图窗和n个坐标轴：各关节角随时间曲线。 |
| PLOT-02 | `[fig,axes_list]=plot_position_comparison(t_s,positions_m,labels,cfg)`；位置序列cell，各N×3，及来源标签 | 图窗和XYZ坐标轴，多来源末端位置对比。 |
| PLOT-03 | `[fig,axes_list]=plot_pose_errors(t_s,position_error_m,rotation_error_rad,cfg)` | 图窗和两个坐标轴，位置/姿态偏差曲线。 |
| PLOT-04 | `[fig,ax]=plot_frame_span(t_s,frame_span_s,threshold_s,cfg)` | 接收跨度曲线及阈值线，默认显示ms。 |
| PLOT-05 | `handles=mark_motion_phases(ax,bounds_s)`；坐标轴、起止时间 | 两条运动边界线句柄；空或[NaN NaN]不标线。 |
| PLOT-06 | `apply_plot_style(fig,cfg)` | 无返回值，修改指定图窗字体、线宽等样式。 |
| PLOT-07 | `path=save_experiment_figure(fig,folder,name,cfg)` | 保存图片并返回完整路径；支持png/jpg/jpeg/pdf/fig，无后缀默认png。 |

绘图 cfg 可省略：

| 字段 | 默认/意义 |
| --- | --- |
| title / visible | 空总标题 / 'on'；隐藏图窗用'off' |
| font_name / font_size / line_width | Noto Sans CJK SC / 11 / 1.2 |
| position | [60 60 1120 780]，图窗位置与尺寸 |
| angle_unit / length_unit | 'deg' / 'mm'；也支持'rad' / 'm'，只改显示 |
| bounds_s | 空；标线时间基准必须与t一致 |
| legend_location | 'northeast' |
| target_rad | 仅关节图使用，n个终态目标，不是连续目标轨迹 |

保存 cfg 独立：resolution=150、overwrite=false。目录会创建；同名文件默认报错，明确重算才开启覆盖。plotting/private中的内部函数不直接调用。

### 兼容旧名称

旧脚本仍可调用这些别名；新脚本优先用右侧标准名称，参数和输出沿用标准函数：

| 旧名称 | 标准名称 |
| --- | --- |
| rot_x_rad / rot_y_rad / rot_z_rad | rot_x / rot_y / rot_z |
| adjoint_se3 | adjoint |
| inverse_transform | se3_inverse |
| make_transform | se3_from_rt |
| twist_skew | twist_hat |
| jacobian_singular_values | singular_values |
| jacobian_condition_number | condition_number |
| adaptive_dls_pseudoinverse | dls_adaptive |

## 3. 符号函数：symbolic_derivations

符号计算依赖 Symbolic Math Toolbox。`syms q1 q2 real` 后用 `subs(表达式,[q1,q2],[数值1,数值2])` 代入，再用 `double(...)` 与数值库交叉验证。符号 `Log` 使用主值分支；当变量的符号或取值范围尚未确定时，应补 `assume` 条件，不要假设表达式自动在所有分支成立。完整 PiPER 六轴符号动力学不在本库范围内。

### lie_group：符号李群与旋量

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| SYM-SO3-01 | `skew_sym(w)` / `vee_sym(W)` | 3 维向量 / 3×3 反对称矩阵，可含 `sym` | 对应符号反对称矩阵 / 向量。 |
| SYM-SO3-02 | `rot_x_sym(theta)` / `rot_y_sym(theta)` / `rot_z_sym(theta)` | 符号或数值角度 `[rad]` | 3×3 符号旋转矩阵。 |
| SYM-SO3-03 | `so3_exp_sym(phi)` 或 `so3_exp_sym(w,theta)` | 积分旋转向量，或单位轴 + 有符号转角 | 3×3 符号旋转矩阵；符号单位轴条件由调用者保证。 |
| SYM-SO3-04 | `[phi,theta,omega]=so3_log_sym(R)` | 3×3 符号旋转矩阵 | 首输出 `phi=omega*theta` 与普通版一致；单位矩阵返回零。 |
| SYM-SE3-01 | `adjoint_se3_sym(T)` | 4×4 符号位姿 | 6×6 符号伴随矩阵。 |
| SYM-SE3-02 | `se3_exp_sym(xi)` 或 `se3_exp_sym(S,theta)` | 6 维积分旋量，或单位旋量 + 关节量 | 4×4 符号位姿；直接推关节变量建议用两参数形式。 |
| SYM-SE3-03 | `se3_log_sym(T)` | 4×4 符号位姿 | 6 维主值积分旋量，与普通 `se3_log` 输出约定相同。 |
| SYM-SCREW-01 | `make_screw_axes_sym(w,p,h_in,joint_types)` | 3×n 轴方向、轴上一点/移动方向、可选螺距与关节类型 | 6×n 符号旋量；含未定符号轴时必须显式指定类型。 |

补充对应的符号变换接口：

| 编号 | 函数与输入 | 输出及用途 |
| --- | --- | --- |
| SYM-SE3-04 | `twist_hat_sym(xi)`；6×1符号旋量 | 4×4符号se(3)矩阵。 |
| SYM-SE3-05 | `se3_from_rt_sym(R,p)`；3×3符号旋转、3×1位置 | 4×4符号位姿。 |
| SYM-SE3-06 | `se3_inverse_sym(T)`；4×4符号位姿 | 4×4逆位姿。 |
| SYM-SE3-07 | `adjoint_sym(T)` / `adjoint_inverse_sym(T)` | 6×6符号Ad(T) / Ad(T⁻¹)。 |
| SYM-SE3-08 | `little_ad_sym(xi)`；6×1符号旋量 | 6×6符号小伴随矩阵。 |

符号层兼容名：`rotx_sym/roty_sym/rotz_sym` 对应 `rot_x_sym/rot_y_sym/rot_z_sym`；`adjoint_se3_sym` 对应 `adjoint_sym`；`JacobianSpace_sym/JacobianBody_sym` 对应小写 `jacobian_space_sym/jacobian_body_sym`。注意Linux文件名大小写，建议新代码使用小写标准入口。

`make_screw_axes_sym` 的 joint_types 是每关节一个的cell/string数组：'revolute'或'prismatic'。转动轴w和移动方向必须是单位向量；含未确定符号轴时显式传类型。螺距h_in默认0，单位m/rad；移动关节w必须为零，p列表示移动方向而非轴上一点。该函数不自动归一化任意符号方向。

### kinematics：符号 POE 与雅可比

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| SYM-FK-01 | `forward_poe_space_sym(S_list,theta,M)` | 6×n 空间轴、n 维符号关节量、零位位姿 | 4×4 符号空间 POE 末端位姿。 |
| SYM-FK-02 | `forward_poe_body_sym(B_list,theta,M)` | 6×n 物体轴、符号关节量、零位位姿 | 4×4 符号物体 POE 末端位姿。 |
| SYM-JAC-01 | `jacobian_space_sym(S_list,theta)` | 空间轴、符号关节量 | 6×n 符号空间雅可比。 |
| SYM-JAC-02 | `jacobian_body_sym(B_list,theta)` | 物体轴、符号关节量 | 6×n 符号物体雅可比。 |

### 符号推导与数值代入示例

```matlab
syms q1 q2 real
S = sym([0 0; 0 0; 1 1; 0 0; 0 -1; 0 0]);
M = sym([eye(3), [2;0;0]; 0 0 0 1]);
T_sym = forward_poe_space_sym(S, [q1;q2], M);
J_sym = jacobian_space_sym(S, [q1;q2]);
q_sample = [0.3; -0.4];
T_sample = double(subs(T_sym, [q1,q2], q_sample.'));
J_sample = double(subs(J_sym, [q1,q2], q_sample.'));
assert(norm(T_sample - forward_poe_space(double(S),q_sample,double(M)),'fro') < 1e-10)
assert(norm(J_sample - jacobian_space(double(S),q_sample),'fro') < 1e-10)
```


## 4. 常用调用示例

### 4.1 从模型计算正运动学

```matlab
model = make_piper_model();
q = deg_to_rad([0;2;-2;0;20;0]); % 六轴角度由deg转rad
T = forward_poe_space(model.S_list,q,model.M);
p_m = T(1:3,4);                % 末端位置m
R = T(1:3,1:3);               % 末端旋转矩阵
J = jacobian_space(model.S_list,q);
```

model常用字段：S_list/B_list（6×6空间/物体轴）、M（零位末端位姿）、q_min/q_max/dq_max（限位）。动力学另用Mlist/Glist/gravity等，均为URDF标称参数。关系：B_list=adjoint_inverse(M)*S_list。

### 4.2 CSV → 独立MATLAB POE → 与C++比较 → 绘图

先给csv_path指定真实输入文件；此示例不修改CSV，不自动保存图片。

```matlab
data = load_feedback_csv(csv_path, ...
    struct('require_transforms',true,'strict_time',true));
model = make_piper_model();
cfg_fk = struct('position_tolerance_m',1e-9, ...
                'rotation_tolerance_rad',1e-8);
cross = validate_fk_samples(model.S_list,model.M, ...
    data.q_rad,data.T_cpp,cfg_fk);
stats = summarize_errors(cross.position_error_m); % 仍是m
fig = plot_pose_errors(data.t,cross.position_error_m, ...
    cross.rotation_error_rad,struct('title','MATLAB / C++ POE交叉验算'));
% 如需保存，显式调用；同名图片默认拒绝覆盖：
% save_experiment_figure(fig,figures_dir,'matlab_CPP交叉验算.png');
```

这是相同模型的数值算法比较。若参考改为data.T_can，则是在比较模型与CAN反馈，容差须另定，不能照搬上面的数值精度阈值。恢复CSV矩阵、绘图、两位置相减都不等于独立POE计算。

### 4.3 IK求解与回代检查

```matlab
model = make_piper_model();
q0 = zeros(6,1);
T_target = forward_poe_space(model.S_list, ...
    deg_to_rad([5;5;-5;2;10;2]),model.M); % 演示目标
options = ik_options();
options.q_min = model.q_min;
options.q_max = model.q_max;
result = solve_ik_dls(model.S_list,model.M,T_target,q0,options);
if ~result.success
    error('IK未收敛，不使用失败解控制真机');
end
report = check_ik_solution(model.S_list,model.M,result.q, ...
    T_target,model.q_min,model.q_max);
disp(report);
```

验收通过也不保证运动路径无碰撞。这里仅离线计算，不发送机械臂命令。

## 5. 测试、依赖与使用边界

```matlab
addpath('/home/nbw/piper_robot_motion_control/MATLAB');
results = run_all_tests(false); % 普通算法，含validation/plotting/experiment_analysis
% results = run_all_tests(true); % 额外运行符号测试
```

基础数学和运动学使用数值MATLAB；符号函数依赖Symbolic Math Toolbox；
quadprog/linprog相关优化和TOPP-RA依赖Optimization Toolbox。总测试对缺少工具箱的组显示SKIP，SKIP不代表已验证通过。可单独运行MATLAB/tests中的test_*.m。

当前C++核心运动学与MATLAB有对应标准接口；动力学、控制、估计和优化的对应实现状态不能只凭同名推断。实际跨语言测试入口与编译方式见 [MATLAB总体说明](../README.md)。本清单不会自动运行C++或改变实验归档。

实验1/2现有脚本主要读取C++结果、统计和绘图，不能因此声称独立重建了MATLAB POE；实验3主脚本已显式计算MATLAB POE，并分别与C++和CAN比较。

CAN末端反馈通常来自厂家内部运动学，不是外部独立测量；一致性偏差不是实际定位精度。六帧接收跨度小于2ms不保证同时采样。URDF中的几何、惯量和限位是标称值，夹爪负载、摩擦、安装偏差仍需核对。

所有验证、绘图和控制原型都不替代硬件急停、碰撞检测、制动与通信看门狗。原始log不修改；重算结果需明确输出目录和覆盖策略。

进一步的模块说明：[validation](validation/README.md)、[experiment_analysis](experiment_analysis/README.md)、[plotting](plotting/README.md)。

## 连续路径IK数据分析（2026-10-18补充）

`[r,summary,points]=analyze_ik_trajectory(csv_path,urdf_path,cfg)`：
C++路径CSV和URDF → 逐点绘图数据、整体统计表、逐点复核表。
独立回算MATLAB POE、SE(3)目标插值、关节增量与加权奇异值，拒绝不一致输入。
不重新求IK，不写文件，不把路径进度s当时间。
2026-10-19补充：含t_s时自动识别时间轨迹；检查等间隔时间和五次s(t)，
根据q独立差分复核dq、ddq，并按CSV的自设离线上限检查数值峰值。
输出r.t、r.dq_rad_s、r.ddq_rad_s2，summary增加时长、峰值和端点差分误差。
首尾使用单边差分，不强制置零；这不是测量速度或真机安全认证。
调用示例见 MATLAB/experiments/project1/2026-10-18_ik_trajectory/matlab_plot.m。
