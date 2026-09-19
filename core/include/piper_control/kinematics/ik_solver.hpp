#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

struct IKOptions
{
    int max_iterations = 100;

    double position_tolerance = 1e-6;

    double orientation_tolerance = 1e-6;

    double lambda = 1e-3;

    double max_joint_step = 0.2;
};

struct IKResult
{
    bool success = false;

    int iterations = 0;

    double position_error = 0.0;

    double orientation_error = 0.0;

    VecXd q;
};

/*【IK-01】
 * @brief 使用 Newton-Raphson 方法求解逆运动学。
 *
 * @param S_list：6×n 空间螺旋轴矩阵。
 * @param M：home configuration 下的末端齐次变换矩阵。
 * @param target：目标末端齐次变换矩阵。
 * @param q_initial：n×1 初始关节位置。
 * @param options：IK 求解参数。
 *
 * @return：IKResult，包含求解状态、迭代次数、误差和关节结果。
 */
IKResult solve_ik_newton(
    const MatXd& S_list,
    const Mat4& M,
    const Mat4& target,
    const VecXd& q_initial,
    const IKOptions& options = {});

/*【IK-02】
 * @brief 使用阻尼最小二乘法求解逆运动学。
 *
 * @param S_list：6×n 空间螺旋轴矩阵。
 * @param M：home configuration 下的末端齐次变换矩阵。
 * @param target：目标末端齐次变换矩阵。
 * @param q_initial：n×1 初始关节位置。
 * @param options：IK 求解参数。
 *
 * @return：IKResult，包含求解状态、迭代次数、误差和关节结果。
 */
IKResult solve_ik_dls(
    const MatXd& S_list,
    const Mat4& M,
    const Mat4& target,
    const VecXd& q_initial,
    const IKOptions& options = {});

}  // namespace piper_control