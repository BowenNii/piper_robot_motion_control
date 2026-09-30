# MATLAB 串联开链动力学参考库

本目录是数值算法原型，不调用 MATLAB 的 `inverseDynamics` 或 `massMatrix` 来实现 RNEA/CRBA；这两个工具箱函数只在测试中充当独立对照。C++ `core/dynamics` 目前多数仍是 TODO，不能将本目录误认为已完成 C++ 移植。

## 坐标与单位

- 旋量采用 `[角速度; 线速度]`，Wrench 采用 `[力矩; 力]`。
- 关节量单位分别为 rad、rad/s、rad/s²、N·m。
- `model.Slist` 为零位基坐标系的 6×n 空间关节轴；`model.Mlist(:,:,1:n)` 为零位的父连杆到子连杆变换；最后一页为末连杆到工具坐标系的固定变换。
- `model.Glist(:,:,i)` 为第 i 连杆坐标系下的空间惯量；`model.gravity` 为基坐标系重力加速度。
- 当前算法仅支持串联开链；不包含摩擦、转子惯量、柔性传动或夹爪负载，除非调用者显式扩展模型。

## 快速使用

```matlab
addpath('matlab/algorithm_library/math', ...
        'matlab/algorithm_library/dynamics', ...
        'matlab/algorithm_library/model');

model = piper_dynamics_model();
q = [0; pi/2; -pi/2; 0; 0; 0];
dq = zeros(6,1);
ddq = ones(6,1);

tau = rnea(q, dq, ddq, model);       % 逆动力学力矩，N*m
Mq = crba(q, model);                 % 关节空间惯量矩阵
tau_g = gravity(q, model);          % 重力力矩
Y = regressor(q, dq, ddq, model);  % 6x60 惯性参数回归矩阵
pi_vec = inertial_parameters(model); % 60x1 参数向量
assert(norm(Y*pi_vec - tau) < 1e-8);
```

`piper_dynamics_model` 中的质量、质心和惯量是 URDF 标称值。用于真机前馈前，还需核对关节力矩/电流换算、负载与夹爪惯量，并完成参数辨识和安全验证。`friction` 单独计算黏滞与平滑库仑摩擦补偿项，不会自动加入 `rnea`。

测试：在 MATLAB 运行 `run('matlab/tests/test_dynamics.m')`。
