#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【TRAJ-01】
 * @brief 计算两个 SE(3) 位姿之间的指数插值。
 *
 * @param T0：4×4 起始 SE(3) 位姿。
 * @param T1：4×4 目标 SE(3) 位姿。
 * @param s：插值系数，通常取 [0,1]。
 *
 * @return：4×4 插值后的 SE(3) 位姿。
 */
Mat4 interpolate_se3(
    const Mat4& T0,
    const Mat4& T1,
    double s);

}  // namespace piper_control