#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/**
 * @brief 根据旋转轴 omega 和轴上一点 q 创建旋转关节 screw axis.
 *
 * S = [omega; v]
 * v = -omega × q
 *
 * omega 不要求单位化，函数内部会单位化。
 */
Twist make_revolute_screw_axis(
    const Vec3& omega,
    const Vec3& point);

/**
 * @brief 创建移动关节 screw axis.
 *
 * S = [0; v]
 */
Twist make_prismatic_screw_axis(
    const Vec3& direction);

/**
 * @brief exp([S] * theta)
 *
 * S 应为单位 screw axis.
 */
Mat4 screw_exp(
    const Twist& S,
    double theta);

}  // namespace piper_control