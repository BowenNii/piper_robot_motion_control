#include "piper_control/lie_group/screw.hpp"

#include <cassert>

#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/se3.hpp"

namespace piper_control
{

// ============================================================
// 【SCREW-01】 make_revolute_screw_axis
//
// @brief
// 构造旋转关节的单位 Screw Axis。
//
// 数学定义：
//
//     S = [ omega ]
//         [   v   ]
//
// 对于旋转轴上一点 q：
//
//     v = -omega × q
//
// 输入的 omega 不要求已经归一化，函数内部首先计算：
//
//     omega_hat = omega / ||omega||
//
// 最终：
//
//     S = [ omega_hat             ]
//         [ -omega_hat × point   ]
//
// 坐标系：
//     omega 和 point 必须在同一个参考坐标系中表达。
// ============================================================

[[nodiscard]] Twist make_revolute_screw_axis(
    const Vec3& omega,
    const Vec3& point)
{
    // 旋转轴方向不能为零向量。
    const double norm = omega.norm();

    assert(norm > kEpsilon);

    // 将旋转轴方向归一化。
    const Vec3 unit_omega = omega / norm;

    // 根据 Screw Axis 定义：
    //
    //     v = -omega × q
    //
    // 这里 omega 使用单位化后的旋转轴方向。
    const Vec3 v = -unit_omega.cross(point);

    Twist S;

    // Angular component.
    S.head<3>() = unit_omega;

    // Linear component.
    S.tail<3>() = v;

    return S;
}


// ============================================================
// 【SCREW-02】 make_prismatic_screw_axis
//
// @brief
// 构造移动关节的单位 Screw Axis。
//
// 数学定义：
//
//     S = [ 0 ]
//         [ v ]
//
// 其中：
//
//     ||v|| = 1
//
// 输入 direction 可以是任意非零长度向量，
// 函数内部进行单位化。
// ============================================================

[[nodiscard]] Twist make_prismatic_screw_axis(
    const Vec3& direction)
{
    // 移动方向不能为零向量。
    const double norm = direction.norm();

    assert(norm > kEpsilon);

    // 初始化为零：
//  angular component = 0
//  linear component  = direction / ||direction||
    Twist S = Twist::Zero();

    S.tail<3>() = direction / norm;

    return S;
}


// ============================================================
// 【SCREW-03】 screw_exp
//
// @brief
// 计算 Screw Axis 的指数映射。
//
// 数学定义：
//
//     T(theta) = exp([S] theta)
//
// 等价于先构造：
//
//     V = S theta
//
// 再计算：
//
//     T = exp([V])
//
// 即：
//
//     T = se3_exp(S theta)
//
// 对于旋转 Screw Axis：
//     theta 表示旋转角度。
//
// 对于移动 Screw Axis：
//     theta 表示沿 Screw Axis 的位移。
// ============================================================

[[nodiscard]] Mat4 screw_exp(
    const Twist& S,
    double theta)
{
    // 根据：
    //
    //     exp([S]theta)
    //
    // 由于 se3_exp() 的输入是 6×1 Twist 坐标，
    // 因此直接计算：
    //
    //     S_theta = S * theta
    //
    // 然后进行 SE(3) 指数映射。
    return se3_exp(S * theta);
}

} // namespace piper_control