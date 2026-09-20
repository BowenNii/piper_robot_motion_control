#include <cassert>

#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/screw.hpp"

namespace piper_control
{

// ============================================================
// 【FK-01】 forward_poe_space
//
// @brief
// 使用 Product of Exponentials (POE) 的 Space Form 计算机器人末端位姿。
//
// 数学定义：
//
//     T(q) = exp([S_1] q_1) exp([S_2] q_2) ... exp([S_n] q_n) M
//
// 其中：
//     S_i : 第 i 个关节在 Space Frame 下定义的 Screw Axis
//     q_i : 第 i 个关节变量
//     M   : 机器人零位（Home Configuration）下的末端位姿
//     T(q): 当前关节位置 q 下的末端 SE(3) 位姿
//
// 这里 screw_exp(S_i, q_i) 计算：
//
//     exp([S_i] q_i)
//
// 最终得到：
//
//     T = e^[S_1]q_1 ... e^[S_n]q_n M
//
// 坐标系约定：
//     S_list 的每一列都是在 Space Frame 中表达的 Screw Axis。
//     输出 T 表示 End-Effector Frame 相对于 Space Frame 的位姿。
//
// 输入：
//     S_list : 6×n Space Screw Axis 矩阵
//     q      : n×1 关节位置向量
//     M      : 4×4 Home Configuration
//
// 输出：
//     T      : 4×4 当前末端 SE(3) 位姿
//
// 注意：
//     S_list.cols() 必须与 q.size() 一致。
//     S_list 必须具有 6 行。
// ============================================================

[[nodiscard]] Mat4 forward_poe_space(
    const MatXd& S_list,
    const VecXd& q,
    const Mat4& M)
{
    // Space Screw Axis 必须为 6×n。
    assert(S_list.rows() == 6);

    // Screw Axis 数量必须与关节变量数量一致。
    assert(S_list.cols() == q.size());

    // 从单位变换开始，依次左乘每个关节的指数映射。
    Mat4 T = Mat4::Identity();

    for (Eigen::Index i = 0; i < q.size(); ++i)
    {
        // 计算：
        //
        //     exp([S_i] q_i)
        //
        // 并按照 POE Space Form：
        //
        //     T = T exp([S_i] q_i)
        //
        // 最终得到：
        //
        //     T = exp([S_1]q_1) ... exp([S_n]q_n)
        T = T * screw_exp(S_list.col(i), q(i));
    }

    // 右乘 Home Configuration：
    //
    //     T(q) = exp([S_1]q_1) ... exp([S_n]q_n) M
    //
    return T * M;
}


// ============================================================
// 【FK-02】 forward_poe_body
//
// @brief
// 使用 Product of Exponentials (POE) 的 Body Form 计算机器人末端位姿。
//
// 数学定义：
//
//     T(q) = M exp([B_1] q_1) exp([B_2] q_2) ... exp([B_n] q_n)
//
// 其中：
//     B_i : 第 i 个关节在 Body Frame 下定义的 Screw Axis
//     q_i : 第 i 个关节变量
//     M   : 机器人零位（Home Configuration）下的末端位姿
//     T(q): 当前关节位置 q 下的末端 SE(3) 位姿
//
// Body Screw Axis 与 Space Screw Axis 的关系为：
//
//     B_i = Ad_{M^{-1}} S_i
//
// 因此，对于同一个机器人模型和同一个关节位置 q，
// Space Form 与 Body Form 应得到相同的末端位姿：
//
//     forward_poe_space(S, q, M)
//         =
//     forward_poe_body(B, q, M)
//
// 坐标系约定：
//     B_list 的每一列都是在 Body Frame 中表达的 Screw Axis。
//     输出 T 表示 End-Effector Frame 相对于 Space Frame 的位姿。
//
// 输入：
//     B_list : 6×n Body Screw Axis 矩阵
//     q      : n×1 关节位置向量
//     M      : 4×4 Home Configuration
//
// 输出：
//     T      : 4×4 当前末端 SE(3) 位姿
//
// 注意：
//     B_list.cols() 必须与 q.size() 一致。
//     B_list 必须具有 6 行。
//     Body POE 中指数映射的乘法顺序不能改变。
// ============================================================

[[nodiscard]] Mat4 forward_poe_body(
    const MatXd& B_list,
    const VecXd& q,
    const Mat4& M)
{
    // Body Screw Axis 必须为 6×n。
    assert(B_list.rows() == 6);

    // Screw Axis 数量必须与关节变量数量一致。
    assert(B_list.cols() == q.size());

    // Body Form 从 Home Configuration M 开始。
    Mat4 T = M;

    for (Eigen::Index i = 0; i < q.size(); ++i)
    {
        // 根据 Body POE：
        //
        //     T(q) = M exp([B_1]q_1) ... exp([B_n]q_n)
        //
        // 因此这里依次右乘：
        //
        //     exp([B_i]q_i)
        T = T * screw_exp(B_list.col(i), q(i));
    }

    return T;
}

} // namespace piper_control