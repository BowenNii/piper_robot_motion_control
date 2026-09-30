# POE数值逆运动学：实现与验证

## 阅读顺序

先读 `ik_solver.hpp` 的公共数学约定和选项，再读 `.cpp` 的
`error`、`solve`，最后阅读 `test_ik_solver.cpp`。
`IKResult.success` 只有同时满足位置和姿态容差时才为真。
`iterations` 是已接受的关节更新次数；初值即解时为0。

## 四种已实现方法

误差 e=Log(T(q)^-1 Td)^vee，与身体雅可比 Jb=Ad(T^-1)Js 配套。
旋量顺序为[旋转;平移]。用 D=diag(wR I3,I3) 得到 A=DJb、b=De。
旋转权重默认0.2只是任务尺度选择，不是通用最优参数。

1. 牛顿–拉夫森：dq=A^+b，采用截断SVD伪逆，无需显式矩阵求逆。
2. 固定DLS：最小化 ||A dq-b||²+lambda²||dq||²。
   每个奇异方向增益 sigma/(sigma²+lambda²)；lambda=0时退化为截断伪逆。
3. 自适应DLS：根据加权Jacobian的最小奇异值，在0到lambda_max间调整阻尼。
   六维任务少于六关节时按缺失方向处理。较大的阻尼可能显著减慢收敛。
4. Jacobian转置：g=A^T b，eta=||g||²/||Ag||²，dq=eta*g。
   eta来自局部线性误差模型；不声称Jb是大误差SE(3)对数残差的精确导数。

所有方法执行方向保持的最大关节步长限制，再回溯试探；只接受
加权对数误差下降的步。指定限位时对候选q投影。这是简单有界局部搜索，
不是QP/SQP求解器，在边界可能停滞。返回失败不等于数学上无解。
位置停止判据是||p-pd||，姿态判据是相对旋转角，均在返回q上计算。

## PiPER调用

```cpp
const auto model = make_piper_model();
IKOptions options;
options.q_min = model.q_min;
options.q_max = model.q_max;
options.max_iterations = 300;
const auto result = solve_ik_dls(
    model.S_list, model.M, target, q_initial, options);
if (result.success) {
    // result.q是终点解；仍需另外规划轨迹，不能把迭代步当真机指令。
}
```

## 验证

测试不用assert作为通过判据，Release也会执行：

- 有解析答案的一自由度移动关节，验证四种算法和秩不足情况。
- 初值即目标、不可达方向、限位阻挡、零迭代预算、非法位姿、NaN配置。
- PiPER使用固定随机种子生成限位内目标，初值在目标附近扰动。
- 对成功结果重新计算FK，分别检查位置与旋转矩阵误差以及限位。
- 输出每个方法的局部收敛数量；这不是全工作空间成功率。

目标由FK生成只能证明模型内部一致性，不能证明真实机械臂标定准确。
同一末端位姿可能对应不同q，所以不把关节向量相等作为PiPER通过条件。

## 其他方法（尚未实现）

- 解析法/几何法：针对机器人构型推导解分支，速度快；需要专门处理退化、
  分支和关节限位，不能把其他六轴机器人的公式直接套到PiPER。
- Levenberg–Marquardt：结合实际下降表现调整阻尼/信赖域。
  本项目按sigma调阻尼的自适应DLS不应直接称为完整LM。
- QP/SQP约束逆解：把关节限位、任务优先级等显式写成约束。
- 多初值重启：外层策略，可套在Newton/DLS上，提高找到分支的机会，
  不保证找到全部解，也不能凭有限次失败证明不可达。
- CCD/FABRIK：常用于位置链求解；完整末端姿态及实际关节约束需额外处理。

当前主线先学Newton和DLS，转置法用于对比，约束优化后续再扩展。

数学参考：[Modern Robotics 6.2](https://modernrobotics.northwestern.edu/nu-gm-book-resource/6-2-numerical-inverse-kinematics-part-2-of-2/)。
