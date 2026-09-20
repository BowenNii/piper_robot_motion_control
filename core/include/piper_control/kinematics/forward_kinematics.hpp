#ifndef PIPER_CONTROL_KINEMATICS_FORWARD_KINEMATICS_HPP
#define PIPER_CONTROL_KINEMATICS_FORWARD_KINEMATICS_HPP

#include "piper_control/common/types.hpp"

namespace piper_control
{

// ============================================================
// Forward Kinematics
//
// 本模块使用 Product of Exponentials (POE) 表达机器人正运动学。
// 提供 Space Form 和 Body Form 两种标准 POE 表达方式。
//
// Space Form:
//     T(q) = exp([S_1]q_1) ... exp([S_n]q_n) M
//
// Body Form:
//     T(q) = M exp([B_1]q_1) ... exp([B_n]q_n)
//
// 其中：
//     S_i : 第 i 个关节在 Space Frame 中表达的 Screw Axis
//     B_i : 第 i 个关节在 Body Frame 中表达的 Screw Axis
//     q_i : 第 i 个关节变量
//     M   : 机器人 Home Configuration
//     T(q): 当前关节位置下的末端 SE(3) 位姿
//
// 对于同一个机器人模型：
//
//     B_i = Ad_{M^{-1}} S_i
//
// 因此 Space Form 和 Body Form 在相同 q 下应得到相同的
// End-Effector Pose。
//
// 坐标系约定：
//     Space Form:
//         Screw Axis 在 Space Frame 中表达。
//
//     Body Form:
//         Screw Axis 在 Body / End-Effector Frame 中表达。
//
// 输出 T 表示：
//     End-Effector Frame 相对于 Space Frame 的位姿。
// ============================================================


// ============================================================
// 【FK-01】 forward_poe_space
//
// @brief
// 使用 Space Form POE 计算机器人末端位姿。
//
// 核心数学定义：
//
//     T(q) = exp([S_1]q_1)
//          · exp([S_2]q_2)
//          · ...
//          · exp([S_n]q_n)
//          · M
//
// 其中：
//     S_i : Space Frame 下的第 i 个 Screw Axis
//     q_i : 第 i 个关节变量
//     M   : Home Configuration
//
// 输入：
//     S_list : 6×n Space Screw Axis 矩阵
//     q      : n×1 关节位置向量
//     M      : 4×4 Home Configuration
//
// 输出：
//     T      : 4×4 当前末端 SE(3) 位姿
//
// 使用方法：
//     给定机器人在 Space Frame 下的 Screw Axis、
//     当前关节位置 q 和 Home Configuration M，
//     调用该函数计算末端位姿。
//
// 注意：
//     S_list 必须具有 6 行。
//     S_list 的列数必须与 q 的维度一致。
// ============================================================

[[nodiscard]] Mat4 forward_poe_space(
    const MatXd& S_list,
    const VecXd& q,
    const Mat4& M);


// ============================================================
// 【FK-02】 forward_poe_body
//
// @brief
// 使用 Body Form POE 计算机器人末端位姿。
//
// 核心数学定义：
//
//     T(q) = M
//          · exp([B_1]q_1)
//          · exp([B_2]q_2)
//          · ...
//          · exp([B_n]q_n)
//
// 其中：
//     B_i : Body Frame 下的第 i 个 Screw Axis
//     q_i : 第 i 个关节变量
//     M   : Home Configuration
//
// Space Screw Axis 与 Body Screw Axis 的关系：
//
//     B_i = Ad_{M^{-1}} S_i
//
// 因此理论上：
//
//     forward_poe_space(S_list, q, M)
//         =
//     forward_poe_body(B_list, q, M)
//
// 输入：
//     B_list : 6×n Body Screw Axis 矩阵
//     q      : n×1 关节位置向量
//     M      : 4×4 Home Configuration
//
// 输出：
//     T      : 4×4 当前末端 SE(3) 位姿
//
// 使用方法：
//     给定机器人在 Body Frame 下的 Screw Axis、
//     当前关节位置 q 和 Home Configuration M，
//     调用该函数计算末端位姿。
//
// 注意：
//     B_list 必须具有 6 行。
//     B_list 的列数必须与 q 的维度一致。
//     Body POE 中指数映射的乘法顺序必须保持不变。
// ============================================================

[[nodiscard]] Mat4 forward_poe_body(
    const MatXd& B_list,
    const VecXd& q,
    const Mat4& M);

} // namespace piper_control

#endif // PIPER_CONTROL_KINEMATICS_FORWARD_KINEMATICS_HPP