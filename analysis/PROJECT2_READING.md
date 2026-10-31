# Project2 Python阅读路线

建议先把这些数值函数读懂，再设计真机实验。它们尚未与真实力矩执行接口连接。

## 1. 模型和坐标系

以下路径相对`analysis/src/piper_analysis/`。先读`robot_model/types.py`的数据结构，再读`robot_model/urdf.py:load_piper`：逐个读取joint1～joint6的父子变换、旋转轴、质量、质心和惯量。
URDF的惯性坐标系旋转先转换到link轴；再通过平行轴关系构造空间惯量。
默认只到link6，不包含夹爪或负载。空间惯量在`dynamics/spatial_inertia.py`，负载合并在`robot_model/payload.py:with_payload`。

数组约定：q为`(6,)`，轨迹为`(N,6)`，Mlist为`(7,4,4)`，Glist为`(6,6,6)`。
虽然Glist三个维度刚好都等于6，其语义仍是`Glist[link,row,column]`，不能按MATLAB维度理解。
内部单位：m、rad、s、kg、N·m，空间向量为`[角;线]`。

## 2. RNEA为何分两遍

先读`dynamics/spatial.py:chain_transforms`，再读`dynamics/rnea.py:rnea`，理解：

- A：在每个link坐标系表达的关节运动轴。
- X：把父link的运动向量转换到当前link。
- V：当前link的空间速度。
- acc：空间加速度；基座初始化为负重力，把重力纳入递推。

前向递推：

```text
V_i = X_i V_parent + A_i dq_i
acc_i = X_i acc_parent + A_i ddq_i + cross_motion(V_i) A_i dq_i
```

反向递推：

```text
F_i = X_child.T F_child + G_i acc_i + cross_force(V_i) G_i V_i
tau_i = A_i.T F_i
```

不要把空间加速度各分量直接等同于普通XYZ二阶导数；两者与表达坐标系及空间向量定义有关。
当前RNEA不含摩擦和电机转子惯量。

## 3. 再理解动力学分解

```text
tau = M(q) ddq + c(q,dq) + g(q)
```

- `dynamics/crba.py`独立计算质量矩阵M，不只是换个名字重复调用RNEA。
- `dynamics/terms.py:gravity`把速度和加速度置零。
- 同文件的`velocity_bias`把重力和关节加速度置零，得到c向量，不返回唯一的C矩阵。
- 同文件的`forward_dynamics`解线性方程求ddq，不显式求逆矩阵。
- `dynamics/friction.py`需要另行提供已辨识参数，默认模型不假设摩擦为真机真实值。

运行`project2_demo.py`后，观察六轴总力矩、重力、速度项、惯性项的区别。
这些是合成轨迹的模型计算，不是电机反馈。

## 4. 最后读参数辨识与控制

`identification/regressor.py:regressor`构造Y，使无外力、无摩擦模型满足`tau=Y*pi`。
每个link的pi是`[m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]`，I是link原点惯量，不是质心惯量。
`stack_regressors`按样本、再按关节堆叠，必须与`tau.reshape(-1)`顺序相同。

`identification/parameters.py`负责惯性参数排列；`identification/fitting.py:fit_parameters`会报告矩阵秩与拟合残差。六轴完整60个标称惯性参数不一定可独立辨识；
拟合残差小不代表参数真实或物理可行，也不能替代独立验证数据。

`control/computed_torque.py`只计算一个理想控制律的输出，不负责限幅、故障处理和实时执行。
不要直接把这个函数接到MIT/CAN真机接口。

## 5. 真实数据的前置工作

1. 明确夹爪/工具的实际负载参数和坐标系。
2. 核对电流/力矩单位、正负号、传动侧与时间戳，不能仅凭字段叫effort就确认其意义。
3. 保留原始数据，再显式选择重采样与滤波参数；不要覆盖原始信号。
4. 区分电机速度与关节速度，差分加速度与实测量。
5. 用多姿态静态数据先检查重力，再考虑动态项和摩擦辨识。

`signals/filtering.py`与`signals/derivatives.py`的双向滤波与SG导数使用未来数据，只能用于离线分析。
模型残差不是直接的末端外力，也不能作为安全停止装置。
