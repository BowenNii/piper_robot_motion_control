#include "piper_control/kinematics/jacobian.hpp"

#include <cassert>

#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/screw.hpp"

namespace piper_control
{

// ============================================================
// 【JAC-01】 jacobian_space
//
// @brief
// 根据 Space Screw Axis 计算 Space Jacobian。
//
// 数学定义：
//
//     J_s(:,i)
//     = Ad_{e^[S_1]q_1 ... e^[S_{i-1}]q_{i-1}} S_i
//
// 因此：
//
//     J_s(q)
//     = [ S_1,
//         Ad_{e^[S_1]q_1} S_2,
//         Ad_{e^[S_1]q_1]e^[S_2]q_2} S_3,
//         ... ]
//
// 并满足：
//
//     V_s = J_s(q) q_dot
//
// 坐标系：
//     所有 Jacobian 列向量均在 Space Frame 中表达。
// ============================================================

[[nodiscard]] MatXd jacobian_space(
    const MatXd& S_list,
    const VecXd& q)
{
    // Space Screw Axis 必须为 6×n。
    assert(S_list.rows() == 6);

    // Screw Axis 数量必须与关节变量数量一致。
    assert(S_list.cols() == q.size());

    const Eigen::Index n = q.size();

    // Space Jacobian 为 6×n。
    MatXd J = MatXd::Zero(6, n);

    // T 表示当前已经经过的关节变换：
    //
    //     T_i = exp([S_1]q_1) ... exp([S_i]q_i)
    //
    // 初始时还没有经过任何关节，因此：
    //
    //     T = I
    Mat4 T = Mat4::Identity();

    for (Eigen::Index i = 0; i < n; ++i)
    {
        // 第 i 个 Space Jacobian 列：
        //
        //     J_s(:,i)
        //     = Ad_T S_i
        //
        // 对于第一个关节：
        //
        //     T = I
        //
        // 因此：
        //
        //     J_s(:,1) = S_1
        J.col(i) = adjoint(T) * S_list.col(i);

        // 更新已经经过的关节变换：
        //
        //     T <- T exp([S_i]q_i)
        //
        // 用于下一列 Jacobian。
        T = T * screw_exp(S_list.col(i), q(i));
    }

    return J;
}


// ============================================================
// 【JAC-02】 jacobian_body
//
// @brief
// 根据 Body Screw Axis 计算 Body Jacobian。
//
// 数学定义：
//
//     J_b(:,i)
//     = Ad_{e^(-[B_{i+1}]q_{i+1}) ... e^(-[B_n]q_n)} B_i
//
// 因此计算时从最后一个关节向第一个关节反向递推。
//
// 最后一个关节：
//
//     J_b(:,n) = B_n
//
// 再向前递推：
//
//     J_b(:,i)
//     = Ad_T B_i
//
// 其中 T 保存：
//
//     T = e^(-[B_{i+1}]q_{i+1}) ... e^(-[B_n]q_n)
//
// 注意这里必须使用负指数，因为需要的是后续关节
// 运动的逆变换。
// ============================================================

[[nodiscard]] MatXd jacobian_body(
    const MatXd& B_list,
    const VecXd& q)
{
    // Body Screw Axis 必须为 6×n。
    assert(B_list.rows() == 6);

    // Screw Axis 数量必须与关节变量数量一致。
    assert(B_list.cols() == q.size());

    const Eigen::Index n = q.size();

    // Body Jacobian 为 6×n。
    MatXd J = MatXd::Zero(6, n);

    // 从末端 Body Frame 开始，因此初始变换为单位阵。
    Mat4 T = Mat4::Identity();

    // 从最后一个关节向第一个关节反向递推。
    for (Eigen::Index i = n - 1; i >= 0; --i)
    {
        // 根据：
        //
        //     J_b(:,i)
        //     = Ad_T B_i
        //
        // 此时 T 已经包含第 i+1 到 n 个关节的逆变换。
        J.col(i) = adjoint(T) * B_list.col(i);

        // 累积当前关节的逆运动：
        //
        //     T <- T exp(-[B_i]q_i)
        //
        // 这样下一次循环计算第 i-1 列时：
        //
        //     T
        //     = exp(-[B_i]q_i)
        //       ... 
        //       exp(-[B_n]q_n)
        //
        // 从而得到正确的 Body Jacobian 列。
        T = T * screw_exp(B_list.col(i), -q(i));
    }

    return J;
}


// ============================================================
// 【JAC-03】 twist_from_space_jacobian
//
// @brief
// 根据 Space Jacobian 和关节速度计算 Space Twist。
//
// 数学定义：
//
//     V_s = J_s(q) q_dot
//
// 其中：
//     V_s : Space Twist
//     J_s : Space Jacobian
//     q_dot : 关节速度
//
// 坐标系：
//     输出 Twist 在 Space Frame 中表达。
// ============================================================

[[nodiscard]] Vec6 twist_from_space_jacobian(
    const MatXd& J_space,
    const VecXd& q_dot)
{
    // Jacobian 的列数必须与关节速度维度一致。
    assert(J_space.cols() == q_dot.size());

    // 直接执行：
    //
    //     V_s = J_s q_dot
    //
    return J_space * q_dot;
}


// ============================================================
// 【JAC-04】 twist_from_body_jacobian
//
// @brief
// 根据 Body Jacobian 和关节速度计算 Body Twist。
//
// 数学定义：
//
//     V_b = J_b(q) q_dot
//
// 其中：
//     V_b : Body Twist
//     J_b : Body Jacobian
//     q_dot : 关节速度
//
// 坐标系：
//     输出 Twist 在 Body / End-Effector Frame 中表达。
// ============================================================

[[nodiscard]] Vec6 twist_from_body_jacobian(
    const MatXd& J_body,
    const VecXd& q_dot)
{
    // Jacobian 的列数必须与关节速度维度一致。
    assert(J_body.cols() == q_dot.size());

    // 直接执行：
    //
    //     V_b = J_b q_dot
    //
    return J_body * q_dot;
}

} // namespace piper_control