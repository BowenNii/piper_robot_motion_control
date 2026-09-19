# Function List

## common/constants

- kDof：机器人自由度数量常量
- kPi：圆周率常量
- kEpsilon：数值计算精度阈值
- kSmallAngle：小角度判断阈值

---

## math

- 【MATH-01】 clamp：数值 → 指定范围内的限幅值
- 【MATH-02】 trim_small：输入矩阵 → 微小元素置零后的矩阵
- 【MATH-03】 deg_to_rad：角度 degree → 角度 rad
- 【MATH-04】 rad_to_deg：角度 rad → 角度 degree

---

## lie_group / so3

- 【SO3-01】 skew：3维向量 → 3×3反对称矩阵
- 【SO3-02】 vee：3×3反对称矩阵 → 3维向量
- 【SO3-03】 rot_x：旋转角度 θ → 绕X轴的3×3旋转矩阵
- 【SO3-04】 rot_y：旋转角度 θ → 绕Y轴的3×3旋转矩阵
- 【SO3-05】 rot_z：旋转角度 θ → 绕Z轴的3×3旋转矩阵
- 【SO3-06】 so3_exp：3维旋转向量 → 3×3 SO(3)旋转矩阵
- 【SO3-07】 so3_log：3×3 SO(3)旋转矩阵 → 3维旋转向量
- 【SO3-08】 rot_axis_angle：旋转轴 + 旋转角度 → 3×3 SO(3)旋转矩阵

---

## lie_group / se3

- 【SE3-01】 twist_hat：6维Twist旋量 → 4×4 se(3)反对称矩阵
- 【SE3-02】 se3_from_rt：3×3旋转矩阵 + 3维平移向量 → 4×4 SE(3)齐次变换矩阵
- 【SE3-03】 se3_inverse：4×4 SE(3)齐次变换矩阵 → 4×4逆齐次变换矩阵
- 【SE3-04】 adjoint：4×4 SE(3)齐次变换矩阵 → 6×6伴随矩阵
- 【SE3-05】 adjoint_inverse：4×4 SE(3)齐次变换矩阵 → 6×6逆伴随矩阵
- 【SE3-06】 little_ad：6维Twist → 6×6 little adjoint矩阵
- 【SE3-07】 se3_exp：6维Twist坐标 → 4×4 SE(3)齐次变换矩阵
- 【SE3-08】 se3_log：4×4 SE(3)齐次变换矩阵 → 6维Twist坐标

---

## lie_group / screw

- 【SCREW-01】 make_revolute_screw_axis：旋转轴方向 + 轴上一点 → 6维旋转关节Screw Axis
- 【SCREW-02】 make_prismatic_screw_axis：移动方向 → 6维移动关节Screw Axis
- 【SCREW-03】 screw_exp：6维Screw Axis + 关节变量 θ → 4×4 SE(3)运动变换矩阵

---

## lie_group / wrench

- 【WRENCH-01】 transform_wrench：Wrench + SE(3)坐标变换 → 变换后的Wrench

---

## kinematics / forward_kinematics

- 【FK-01】 forward_poe：空间Screw Axis矩阵 + 关节位置 + Home位姿 → 末端SE(3)位姿

---

## kinematics / jacobian

- 【JAC-01】 jacobian_space：空间Screw Axis矩阵 + 关节位置 → 6×n空间Jacobian
- 【JAC-02】 jacobian_body：本体Screw Axis矩阵 + 关节位置 → 6×n本体Jacobian
- 【JAC-03】 twist_from_space_jacobian：空间Jacobian + 关节速度 → 6维空间Twist
- 【JAC-04】 twist_from_body_jacobian：本体Jacobian + 关节速度 → 6维本体Twist

---

## kinematics / singularity

- 【SING-01】 singular_values：Jacobian → 奇异值向量
- 【SING-02】 minimum_singular_value：Jacobian → 最小奇异值
- 【SING-03】 condition_number：Jacobian → 条件数
- 【SING-04】 manipulability：Jacobian → 可操作度
- 【SING-05】 pseudoinverse_svd：Jacobian → SVD伪逆
- 【SING-06】 dls_pseudoinverse：Jacobian + 阻尼系数 λ → DLS伪逆

---

## kinematics / ik_solver

- 【IK-01】 solve_ik_newton：机器人运动学模型 + 目标位姿 + 初始关节位置 → Newton逆运动学结果
- 【IK-02】 solve_ik_dls：机器人运动学模型 + 目标位姿 + 初始关节位置 → DLS逆运动学结果

---

## robot_model

- 【MODEL-01】 make_piper_model：PiPER机器人参数 → PiPER机器人模型

---

## dynamics

- 【DYN-01】 spatial_inertia：刚体质量 + 质心惯量 + 质心位置 → 6×6空间惯量矩阵
- 【DYN-02】 spatial：空间代数基础运算 → 空间代数计算结果
- 【DYN-03】 rnea：机器人状态 + 动力学参数 → 关节力矩
- 【DYN-04】 crba：机器人状态 + 动力学参数 → 关节空间惯量矩阵
- 【DYN-05】 gravity：机器人关节位置 + 动力学参数 → 重力力矩
- 【DYN-06】 friction：关节速度 + 摩擦参数 → 摩擦力矩
- 【DYN-07】 regressor：机器人状态 → 动力学回归矩阵

---

## control

- 【CTRL-01】 pd_controller：期望关节状态 + 当前关节状态 → PD控制力矩
- 【CTRL-02】 gravity_compensation：机器人状态 + 动力学参数 → 重力补偿力矩
- 【CTRL-03】 computed_torque：期望状态 + 当前状态 + 动力学模型 → 计算力矩控制指令
- 【CTRL-04】 joint_impedance：期望关节状态 + 当前关节状态 → 关节阻抗控制力矩
- 【CTRL-05】 cartesian_impedance：期望末端位姿 + 当前末端状态 → 笛卡尔空间阻抗控制力矩

---

## trajectory

- 【TRAJ-01】 interpolate_se3：起始SE(3)位姿 + 目标SE(3)位姿 + 插值系数 → 插值位姿

---

## optimization

- 【OPT-01】 qp_problem：优化变量 + 目标函数 + 约束 → QP问题
- 【OPT-02】 trajectory_optimizer：机器人模型 + 轨迹约束 → 优化后轨迹
- 【OPT-03】 sequential_convexification：非线性动力学优化问题 → 逐次凸化优化轨迹

---

## estimation

- 【EST-01】 low_pass_filter：输入信号 + 滤波参数 + 上一时刻状态 → 滤波后信号
- 【EST-02】 kalman_filter：系统状态 + 观测数据 → 估计状态
- 【EST-03】 estimate_joint_velocity：当前关节位置 + 上一时刻关节位置 + 采样周期 → 关节速度估计

---

## safety

- 【SAFE-01】 limit_joint_velocity：关节速度指令 + 最大关节速度 → 限幅后的关节速度
- 【SAFE-02】 joint_limits：机器人关节状态 → 关节限制检查结果
- 【SAFE-03】 watchdog：控制周期/通信状态 → Watchdog状态