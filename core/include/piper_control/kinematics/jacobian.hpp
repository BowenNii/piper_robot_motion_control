#ifndef PIPER_CONTROL_KINEMATICS_JACOBIAN_HPP
#define PIPER_CONTROL_KINEMATICS_JACOBIAN_HPP

#include "piper_control/common/types.hpp"

namespace piper_control
{

// ============================================================
// Jacobian
//
// 本模块实现机器人 Jacobian 的 POE 表达，包括：
//     1. Space Jacobian
//     2. Body Jacobian
//     3. 根据 Space Jacobian 计算 Space Twist
//     4. 根据 Body Jacobian 计算 Body Twist
//
// Twist 统一采用 6×1 向量：
//
//     V = [ omega ]
//         [   v   ]
//
// 其中：
//     omega : 角速度
//     v     : 线速度
//
// Jacobian 满足：
//
//     V_s = J_s(q) q_dot
//
//     V_b = J_b(q) q_dot
//
// Space Jacobian 与 Body Jacobian 描述的是同一个末端
// Twist 在不同坐标系下的表达。
// ============================================================


// ============================================================
// 【JAC-01】 jacobian_space
//
// @brief
// 根据 Space Screw Axis 计算 Space Jacobian。
//
// 核心数学定义：
//
//     J_s(q)
//     = [ S_1,
//         Ad_{e^[S_1]q_1} S_2,
//         Ad_{e^[S_1]q_1}e^[S_2]q_2 S_3,
//         ... ]
//
// 第 i 列为：
//
//     J_s(:,i)
//     = Ad_{e^[S_1]q_1 ... e^[S_{i-1}]q_{i-1}} S_i
//
// 当 i = 1 时：
//
//     J_s(:,1) = S_1
//
// 关节速度与 Space Twist 的关系：
//
//     V_s = J_s(q) q_dot
//
// 输入：
//     S_list : 6×n Space Screw Axis 矩阵
//     q      : n×1 关节位置向量
//
// 输出：
//     J_s    : 6×n Space Jacobian
//
// 坐标系：
//     Jacobian 每一列均在 Space Frame 中表达。
//
// 注意：
//     S_list 必须具有 6 行。
//     S_list 的列数必须与 q 的维度一致。
// ============================================================

[[nodiscard]] MatXd jacobian_space(
    const MatXd& S_list,
    const VecXd& q);


// ============================================================
// 【JAC-02】 jacobian_body
//
// @brief
// 根据 Body Screw Axis 计算 Body Jacobian。
//
// 核心数学定义：
//
//     J_b(q)
//     = [ Ad_{e^(-[B_2]q_2)...e^(-[B_n]q_n)} B_1,
//         Ad_{e^(-[B_2]q_2)...e^(-[B_n]q_n)} B_2,
//         ...,
//         Ad_{e^(-[B_n]q_n)} B_{n-1},
//         B_n ]
//
// 更一般地，第 i 列为：
//
//     J_b(:,i)
//     = Ad_{e^(-[B_{i+1}]q_{i+1}) ... e^(-[B_n]q_n)} B_i
//
// 因此 Body Jacobian 通常采用从末端到基座的反向递推计算。
//
// 关节速度与 Body Twist 的关系：
//
//     V_b = J_b(q) q_dot
//
// 输入：
//     B_list : 6×n Body Screw Axis 矩阵
//     q      : n×1 关节位置向量
//
// 输出：
//     J_b    : 6×n Body Jacobian
//
// 坐标系：
//     Jacobian 每一列均在 Body / End-Effector Frame 中表达。
//
// 注意：
//     B_list 必须具有 6 行。
//     B_list 的列数必须与 q 的维度一致。
//     反向递推时使用负指数，即逆变换：
//
//         exp(-[B_i]q_i)
//
//     不能使用正指数。
// ============================================================

[[nodiscard]] MatXd jacobian_body(
    const MatXd& B_list,
    const VecXd& q);


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
//     J_s : 6×n Space Jacobian
//     q_dot : n×1 关节速度
//     V_s : 6×1 Space Twist
//
// 输入：
//     J_space : 6×n Space Jacobian
//     q_dot   : n×1 关节速度
//
// 输出：
//     V_space : 6×1 Space Twist
//
// 坐标系：
//     输出 Twist 在 Space Frame 中表达。
// ============================================================

[[nodiscard]] Vec6 twist_from_space_jacobian(
    const MatXd& J_space,
    const VecXd& q_dot);


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
//     J_b : 6×n Body Jacobian
//     q_dot : n×1 关节速度
//     V_b : 6×1 Body Twist
//
// 输入：
//     J_body : 6×n Body Jacobian
//     q_dot  : n×1 关节速度
//
// 输出：
//     V_body : 6×1 Body Twist
//
// 坐标系：
//     输出 Twist 在 Body / End-Effector Frame 中表达。
// ============================================================

[[nodiscard]] Vec6 twist_from_body_jacobian(
    const MatXd& J_body,
    const VecXd& q_dot);

} // namespace piper_control

#endif // PIPER_CONTROL_KINEMATICS_JACOBIAN_HPP