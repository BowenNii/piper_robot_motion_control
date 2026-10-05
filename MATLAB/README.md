# MATLAB 算法库与 C++ 对齐说明

## 1. 统一数学约定

- 右手坐标系、列向量；`T_ab` 将 b 系坐标转换到 a 系：`p_a=T_ab*p_b`。
- 旋转矩阵为主动旋转；角度 rad、长度 m、时间 s、质量 kg、力矩 N·m。
- Twist 固定为 `[omega;v]`，Wrench 固定为 `[torque;force]`；Jacobians 前三行是角速度。
- `so3_exp(phi)` 的 phi 已含转角；`se3_exp(xi)` 的 xi 是积分旋量。`screw_exp(S,q)` 才是轴与关节量分开输入。
- 空间 POE：`T=exp(S1*q1)...exp(Sn*qn)*M`；物体轴：`B_list=adjoint_inverse(M)*S_list`。
- URDF/CAN RPY 构造旋转矩阵：`R=rot_z(yaw)*rot_y(pitch)*rot_x(roll)`。POE 本身不是 RPY 内旋/外旋算法。
- IK 输入统一空间轴；内部将空间 Jacobian 转为物体 Jacobian。比较姿态用相对旋转角，不直接相减 RPY。
- 接口/数学意义对齐不要求浮点结果逐位相等；小角度与接近 pi 的数值处理允许合理误差。

## 2. 分类与使用入口

`algorithm_library/` 按 C++ 分类：common、math、lie_group、kinematics、robot_model、dynamics、control、safety、estimation、optimization、trajectory。`controller` 已改为 `control`，`model` 已改为 `robot_model`，李群函数从 math 移入 lie_group。

`symbolic_derivations/lie_group` 与 `kinematics` 是符号推导层；`tests/` 是回归/交叉验证；`experiments/` 仅保存 MATLAB 实验脚本。原始日志、CSV、报告和结果图仍在仓库根目录的 `experiments/`。

```matlab
addpath('/home/nbw/piper_robot_motion_control/MATLAB');
setup_piper_paths;        % 普通数值函数
% setup_piper_paths(true) % 同时加载符号函数
model = make_piper_model();
q = zeros(6,1);
T = forward_poe_space(model.S_list,q,model.M);
options = ik_options();
options.q_min = model.q_min; options.q_max = model.q_max;
result = solve_ik_dls(model.S_list,model.M,T,q,options);
if ~result.success, error('IK失败，不使用result.q下发真机'); end
```

不要对整个 MATLAB 文件夹使用 `genpath`：练习、模板与实验脚本可能存在同名文件。旧脚本也应先调用 `setup_piper_paths`，不能再只 addpath(math)。

## 3. 实现状态与差异

| 内容 | 对齐状态 |
| --- | --- |
| SO(3)/SE(3)、旋量、POE、Jacobian、SVD/DLS、四种 IK | 与当前 C++ 已实现算法、接口顺序和关键参数对齐 |
| 速度求解、SE(3) 插值、关节速度限幅 | 标准入口与 C++ 一致 |
| PiPER 模型 | 六个核心字段一致；MATLAB 额外从 URDF 读取质量/惯量等用于离线原型 |
| 动力学、PD/力矩/阻抗、Kalman、优化、低通与关节估计 | 当前 C++ 多为 TODO；MATLAB 是已实现的独立原型，不能称为已完成 C++ 交叉验证 |
| 改进 DH | 原有函数保留，本次不重新推导或修改参数表 |

`solve_cartesian_velocity` 是标准速度接口；`cartesian_velocity_controller` 是 MATLAB 扩展接口，额外支持一步位置/速度/加速度约束。扩展接口将少于六列的 Jacobian 按完整六维任务判定为欠驱动；当前 C++ 标准入口仅取 SVD 最后一个奇异值，因此二者在欠驱动情况下不等价。不能据此认定完整六维任务安全。

`clamp` 与 C++ 一样交换反置边界；真正检查安全边界应使用 `clamp_checked` 或先验证限位。这些函数不提供硬件急停、碰撞检测或 CAN 看门狗。

## 4. 新增/标准入口清单

| 分类 | 函数与输入 | 输出 |
| --- | --- | --- |
| common | `piper_constants()` | kDof、kPi、kEpsilon、kSmallAngle |
| math | `deg_to_rad(angle)` / `rad_to_deg(angle)` | 同尺寸角度转换 |
| math | `trim_small(A,eps=1e-10)` | 小元素置零后的矩阵，不改变原输入 |
| math | `clamp_checked(value,lo,hi)` | 限幅结果；边界反置报错 |
| lie_group | `rot_axis_angle(axis,theta)` | 3×3 R；轴归一化，theta rad |
| lie_group | `adjoint_inverse(T)` | 6×6 Ad(T^-1) |
| lie_group | `make_revolute_screw_axis(w,p)` | 单轴 6×1 S；p m |
| lie_group | `make_prismatic_screw_axis(direction)` | 单轴 6×1 S；关节量 m |
| lie_group | `screw_exp(S,q)` | 单关节 4×4 变换 |
| kinematics | `twist_from_space_jacobian(J,dq)` / body 同名入口 | 6×1 同系 Twist |
| kinematics | `singular_values(J)` / `minimum_singular_value(J)` / `condition_number(J)` | 奇异值向量、最小奇异值、条件数 |
| kinematics | `dls_adaptive(J,lambda_max,sigma_threshold)` | [伪逆、实际阻尼、最小奇异值] |
| kinematics | `ik_options()` | 与 C++ IKOptions 一致的默认结构体 |
| kinematics | `solve_ik_newton(S,M,target,q0,options)` | IKResult |
| kinematics | `solve_ik_dls(...)` / `solve_ik_adaptive_dls(...)` / `solve_ik_transpose(...)` | 相同 IKResult |
| control | `cartesian_velocity_config()` | 默认速度配置 |
| control | `solve_cartesian_velocity(J,V,config)` | q_dot_command、sigma_min、speed_scale、region(0/1/2)、辅助 region_name/lambda |
| trajectory | `interpolate_se3(T0,T1,s)` | 4×4 位姿，s 限制到 [0,1] |
| safety | `limit_joint_velocity(dq,dq_max)` | 同维限幅后速度 rad/s |
| estimation | `low_pass_filter_hz(x,previous,dt,cutoff)` | 保留 Hz 配置的 MATLAB 扩展低通 |
| symbolic | `rot_x_sym/rot_y_sym/rot_z_sym`、`adjoint_sym`、`jacobian_space_sym/jacobian_body_sym` | 与数值函数对应的符号结果 |

符号层还提供 `se3_from_rt_sym(R,p)`、`se3_inverse_sym(T)`、`adjoint_inverse_sym(T)`、`twist_hat_sym(xi)`、`little_ad_sym(xi)`，输入输出维度与同名数值函数一致；旋转合法性/变量实数假设由使用者保证。

### IK 关键约定

默认：max_iterations=100、position_tolerance=1e-6 m、orientation_tolerance=1e-6 rad、lambda=1e-3、max_joint_step=0.2、rotation_weight=0.2、svd_tolerance=1e-10、sigma_threshold=0.05、lambda_max=0.05、max_backtracks=20。

步长限制是 `max(abs(dq))`，不是 dq 的二范数；回溯比较加权物体误差。自适应阻尼采用平方根律；普通 SVD 伪逆使用绝对阈值。结果 status 为 kConverged / kMaxIterations / kStalled / kNumericalFailure；只有 success=true 才可继续使用该解。达到迭代上限不证明目标不可达。

### 旧接口兼容

`adjoint_se3→adjoint`、`inverse_transform→se3_inverse`、`make_transform→se3_from_rt`、`twist_skew→twist_hat`、`rot_*_rad→rot_*`、`jacobian_singular_values→singular_values`、`jacobian_condition_number→condition_number`、`adaptive_dls_pseudoinverse→dls_adaptive`。

IK 的旧字段 rotation_tolerance、step_max 分别映射到 orientation_tolerance、max_joint_step；不能同时提供新旧字段。rotation_error 保留为 orientation_error 别名。四参数旧低通仍可调用，但建议改用 low_pass_filter_hz。兼容不等于保持旧数值行为：阻尼、SVD阈值、IK权重/步长已按 C++ 统一。

## 5. 测试与实验脚本

```matlab
addpath('/home/nbw/piper_robot_motion_control/MATLAB');
results = run_all_tests;
```

数值测试覆盖 math、kinematics、robot_model、control、dynamics、estimation、optimization、trajectory 和新增接口；符号测试独立验证代入后的结果。缺少 Optimization/Symbolic 工具箱时对应测试明确显示 SKIP。

实际 C++ 库交叉验证（先在仓库根目录编译）：

```bash
cmake --preset debug
cmake --build --preset debug-build
g++ -std=c++17 -Icore/include -I/usr/include/eigen3 MATLAB/tests/cpp_reference.cpp build/debug/libpiper_control.a -o /tmp/piper_cpp_reference
```

```matlab
addpath('/home/nbw/piper_robot_motion_control/MATLAB/tests');
compare_cpp_reference('/tmp/piper_cpp_reference');
```

这会以相同输入比较实际 C++ / MATLAB 的 FK、四种 IK（包含失败状态）、矩形矩阵伪逆与速度求解；不会发送 CAN，也不修改实验数据。参考例的转置法在100次内不一定收敛，测试核对的正是两端状态和结果是否一致。

两个实验的 matlab_plot.m 已使用仓库相对定位读取根目录 experiments 下的数据，修正移动脚本后的路径。旧归档的图表、CSV和报告未重算；MATLAB 的数学对齐测试不改变原来实验结论或替代外部定位精度测量。

---

## 6. 完整普通/符号函数清单

## 一、普通数值函数：`algorithm_library/`

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

## 二、符号函数：`symbolic_derivations/`

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

### kinematics：符号 POE 与雅可比

| 编号 | 函数 | 输入 | 输出及用途 |
| --- | --- | --- | --- |
| SYM-FK-01 | `forward_poe_space_sym(S_list,theta,M)` | 6×n 空间轴、n 维符号关节量、零位位姿 | 4×4 符号空间 POE 末端位姿。 |
| SYM-FK-02 | `forward_poe_body_sym(B_list,theta,M)` | 6×n 物体轴、符号关节量、零位位姿 | 4×4 符号物体 POE 末端位姿。 |
| SYM-JAC-01 | `jacobian_space_sym(S_list,theta)` | 空间轴、符号关节量 | 6×n 符号空间雅可比。 |
| SYM-JAC-02 | `jacobian_body_sym(B_list,theta)` | 物体轴、符号关节量 | 6×n 符号物体雅可比。 |

## 最小符号推导示例

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

## 测试与使用边界

测试脚本位于 `../tests/test_*.m`。建议依次运行 `test_math.m`、`test_kinematics.m`、`test_piper_model.m`、`test_dynamics.m`、`test_controller.m`、`test_estimation.m`、`test_optimization.m`、`test_trajectory.m`、`test_symbolic_derivations.m`。符号测试会将表达式代入数值，与普通函数交叉核对。

URDF 里的质量、惯量和关节限位是**标称值**；安装偏差、夹爪/工具负载、电机转子、摩擦和通信延迟还需辨识或真机核对。TOPP-RA 是离散路径参数化，区间内部和硬件执行需留安全裕量；控制函数也不提供硬件急停、CAN 看门狗或真实位姿反馈闭环。
